/*
 * Copyright (C) GJ van Dam
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/rl_soaring/rl_soaring.c"
 * @author GJ van Dam
 * Obstacle avoidance method using RL and the aerodynamic interaction between obstacle and quadrotor
 */
// Include header file
#ifndef rl_soaring_H
#include "modules/rl_soaring/rl_soaring.h"
#endif

// Include standard libraries
#include <stdio.h>
#include "std.h"
#include <string.h>
#include <errno.h>

// Include telemetry headers
#include "subsystems/datalink/telemetry.h"

// Include modules which will be used to retrieve measurements
#include "state.h"
#include "subsystems/imu.h"
#include "subsystems/actuators.h"
#include "boards/bebop/actuators.h"
#include "subsystems/gps.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "filters/low_pass_filter.h"
#include "subsystems/gps/gps_datalink.h"
#include "generated/flight_plan.h" // for waypoint reference pointers
#include "modules/ctrl/follow_me.h"

// Include other paparazzi modules
#include "generated/modules.h"
#include "subsystems/datalink/datalink.h" // dl_buffer


// Declaration of global variables
float rl_exploration_rate = 0.5;
float rl_gamma = 0.95;
float rl_alpha = 0.85;

// Declaration of local variables
struct timeval currentTime; // can be overwritten during functions calls
struct timeval lastcallTime; // has to stay constant for period of module
struct timeval nowcallTime; // has to stay same for period of module
int module_period = 1;// in seconds
static int32_t start_time_seconds = 0;
static int32_t start_time_milliseconds = 0;
static int32_t episode_start_time_seconds = 0;
static int32_t episode_start_time_milliseconds = 0;

int16_t rl_teleported;
struct UtmCoor_f teleport_pos;
// teleport_pos.east = 0;
// teleport_pos.north = 0;
// teleport_pos.alt = 0;


static int32_t episode = 0;
static int32_t timestep = 0;
static int32_t time_rl = 0;
static int32_t episode_time_rl = 0;

// Episode variables
static int32_t max_episode_time = 120000; // in seconds

// RL variables
float min_follow_dist = -5;
float desired_accuracy = 0.5; // accuracy at which the discretised states are created
#define ACTION_SIZE_1 5  // increase decrease or no change in groundspeed
#define STATE_SIZE_1 32
#define STATE_SIZE_2 32

static rl_state state1;
static rl_state state2;
static int action1;
static int action2;

static uint16_t action_space[ACTION_SIZE_1];
static float Q[STATE_SIZE_1][STATE_SIZE_2][ACTION_SIZE_1];

// Gives action to be performed based on current state
static uint16_t current_policy[STATE_SIZE_1][STATE_SIZE_2] = {};
static int rl_episode_timeout; // not used
static int rl_episode_boatcrash;
static int rl_episode_beyond_wp;
static int rl_episode_started;

// static void rl_soaring_send_message_down(char *_request, char *_parameters);
static uint16_t rl_soaring_get_action(rl_state state);
static void rl_soaring_state_estimator(void);
static void rl_soaring_perform_action(uint16_t action);
static float random_float_in_range(float min, float max);
static void send_rl_variables(struct transport_tx *trans, struct link_device *dev);
static void send_teleport_me(struct transport_tx *trans, struct link_device *dev);

void print_pos(void){
	struct UtmCoor_f *po_Utm = stateGetPositionUtm_f();
	printf("Current utm position is given by: %f %f %f\n", po_Utm->east, po_Utm->north, po_Utm->alt);
}

float idx_to_dist(int idx){
	if (idx > STATE_SIZE_1 || idx < 0){
	}
    return (min_follow_dist + idx*desired_accuracy);
}

int dist_to_idx(float dist){
	int idx = floor((dist - min_follow_dist)/desired_accuracy);
	if (idx > STATE_SIZE_1 || idx < 0){
	    idx = -1;
	}
	return idx;
}

// sends all the messages through the pprzlink which are update in rl_soaring_update_measurements
static void send_rl_variables(struct transport_tx *trans, struct link_device *dev){
    // When prompted, return all the telemetry variables
	float old_dist = idx_to_dist(state1.dist_wp_idx_old);
	float curr_dist = idx_to_dist(state1.dist_wp_idx);
    pprz_msg_send_RL_SOARING(trans, dev, AC_ID, &timestep, &episode, &old_dist, &curr_dist);
}

// Function to send teleport message so that the ocaml simulation can reset the position
static void send_teleport_me(struct transport_tx *trans, struct link_device *dev){
	int16_t teleported;
	if (rl_teleported){
		teleported = 1;
	} else {
		teleported = 0;
	}
	pprz_msg_send_RL_TELEPORT(trans, dev, AC_ID, &teleported, &teleport_pos.east, &teleport_pos.north, &teleport_pos.alt);
}

/** Initialization function **/
void rl_soaring_init(void) {
    // Register telemetery function
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RL_SOARING, send_rl_variables);
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RL_TELEPORT, send_teleport_me);
}



/** Function called when the module is started, performs the following functions:
 * */
void rl_soaring_start(void){
    // Initialise discrete state space
	for (int i=0; i<STATE_SIZE_1; i++){
		for (int j=0; j<STATE_SIZE_2; j++){
			if (idx_to_dist(i)>0){
			    current_policy[i][j] = 0; // set the current policy to do nothing
			}
			else{
			    current_policy[i][j] = 4;
			}
			for (int k=0; k<ACTION_SIZE_1; k++){
		        Q[i][j][k] = 0;  // set all the initial qvalues to 0
		    }
		}
	}

	// Initialise action space
	for (int i=0; i<ACTION_SIZE_1; i++){
		action_space[i] = i;
	}

    // Set start time in seconds
    gettimeofday(&currentTime, NULL);
    start_time_seconds = currentTime.tv_sec;
    start_time_milliseconds = currentTime.tv_usec;
}


/* Function called at the start of each episode
 */
void rl_soaring_start_episode(){
    // Set intervention & fail states to false
    rl_episode_beyond_wp = false; // in case we fly beyond wp2
    rl_episode_boatcrash = false; // in case we crash the boat
    rl_episode_timeout = false;
    rl_episode_started = true;

    // Increase episode counter and save start time
    episode++;
    gettimeofday(&currentTime, NULL);
    episode_start_time_seconds = currentTime.tv_sec;
    episode_start_time_milliseconds = currentTime.tv_usec;


    // Reset state as well to current position
    // Obtain state, so follow me distances
    int location = follow_me_set_wp();
    if (location == -1){
    	rl_episode_boatcrash = true;
    } else if (location == 2){
    	rl_episode_beyond_wp = true;
    }

    // Obtain index of state based on location
    int idx_old = dist_to_idx(dist_wp_follow_old);
    int idx = dist_to_idx(dist_wp_follow);
    state1.dist_wp_idx_old = state2.dist_wp_idx_old;
    state1.dist_wp_idx = state2.dist_wp_idx;
    state2.dist_wp_idx_old = idx_old;
    state2.dist_wp_idx = idx;

    // Set action to one (no-action) (for the state prev_action)
    action1 = 2;
    action2 = 2;
}

// Function which resets the agent to its initial position
void rl_reset_agent(void);
void rl_reset_agent(void){

    follow_me_set_wp();

	teleport_pos.east = wp_follow_utm.x;
	teleport_pos.north = wp_follow_utm.y;
	teleport_pos.alt = wp_follow_utm.z;

	stateSetPositionUtm_f(&teleport_pos);

	printf("Teleport initiated, changing location\n");
	rl_teleported = true;


}

// Not used at all for now ///////
void rl_soaring_end_episode(void){
	if (rl_episode_beyond_wp){
		printf("Ended because beyond wp\n");
	    rl_episode_beyond_wp = false;
	}
	if (rl_episode_boatcrash){
		printf("Ended because boatcrash\n");
        rl_episode_boatcrash = false;
	}
	if (rl_episode_timeout){
		printf("Ended because of timeout\n");
	    rl_episode_timeout = false;
	}
	rl_episode_started = false;
    rl_reset_agent();
}



// Updates all the mesaurements that were required to send down to the ground segment
void rl_soaring_update_measurements(void){
    // Update timestep
    timestep++;

    // Get time in milliseconds since the measurement has started
    gettimeofday(&currentTime, NULL);
    time_rl = (currentTime.tv_sec - start_time_seconds) * 1000 + (currentTime.tv_usec) / 1000 - start_time_milliseconds / 1000;
    if(episode > 0) {
        episode_time_rl = (currentTime.tv_sec - episode_start_time_seconds) * 1000 + (currentTime.tv_usec) / 1000 -
                          episode_start_time_milliseconds / 1000;
        if (episode_time_rl > max_episode_time){
        	rl_episode_timeout = true;
        }
    }

    // Obtain state, so follow me distances
    int location = follow_me_set_wp();

    if (location == -1){
    	rl_episode_boatcrash = true;
    } else if (location == 2){
    	rl_episode_beyond_wp = true;
    }

    // Obtain index of state based on location
    int idx_old = dist_to_idx(dist_wp_follow_old);
    int idx = dist_to_idx(dist_wp_follow);
    state1.dist_wp_idx_old = state2.dist_wp_idx_old;
    state1.dist_wp_idx = state2.dist_wp_idx;
    state2.dist_wp_idx_old = idx_old;
    state2.dist_wp_idx = idx;
}



void rl_soaring_state_estimator(void){
    // Estimates the states (maybe wind later on)
}


void rl_navigation(void){
	// Standart navigational loop
	NavGotoWaypoint(WP_FOLLOW2);
	NavVerticalAltitudeMode(follow_me_height, 0.);
}


int rl_episode_stop_condition(){
	if (rl_episode_timeout || rl_episode_boatcrash || rl_episode_beyond_wp){
		return 1;
	}
	else{
		return 0;
	}
}

float calc_reward(rl_state state1, rl_state state2);
float calc_reward(rl_state state1, rl_state state2){
	float closer_to_wp = fabs(idx_to_dist(state1.dist_wp_idx)) - fabs(idx_to_dist(state2.dist_wp_idx));
	return closer_to_wp;
}

void update_q_value(rl_state state1, rl_state state2, float reward, int action1, int action2);
void update_q_value(rl_state state1, rl_state state2, float reward, int action1, int action2){
	float predicted_q_value = Q[state1.dist_wp_idx][state1.dist_wp_idx_old][action1];
    float target = reward + rl_gamma * Q[state2.dist_wp_idx][state2.dist_wp_idx_old][action2];
    Q[state1.dist_wp_idx][state1.dist_wp_idx_old][action1] = Q[state1.dist_wp_idx][state1.dist_wp_idx_old][action1] + rl_alpha * (target - predicted_q_value);
}


void update_policy(void);
void update_policy(void){
	for (int i=0; i<STATE_SIZE_1; i++){ // moves through the first state
	    for (int j=0; j<STATE_SIZE_2; j++){ // moves through the second state
	    	// Now we need to find the maximum Q value of all the actions
	    	int max_index = 0;
	    	int maximum = 0;
	    	for (int k=0; k<ACTION_SIZE_1; k++){
	    		if (Q[i][j][k] > maximum){
	    			maximum = Q[i][j][k];
	    			max_index = k;
	    		}
	    	// Update the current policy accordingly
	        current_policy[i][j] = max_index;
	    	}
	    }
	}
}


/*
 * Function periodic, the heartbeat of the module
 */
int rl_soaring_call(void) {
    gettimeofday(&nowcallTime, NULL);
	// The start of this module is called constantly by the fact that the intiialisation of the module created the periodique call
	// Update measurements
	rl_soaring_update_measurements();

	// Get state
	rl_soaring_state_estimator();

	// Check whether the episode has ended
	if (rl_episode_stop_condition()){
		 rl_episode_started = false;
		 rl_soaring_end_episode();
	}
	if ((nowcallTime.tv_sec - lastcallTime.tv_sec) > module_period){
		lastcallTime = nowcallTime;


		// Check whether we are in an episode already
		if (rl_episode_started){
			action2 = rl_soaring_get_action(state2);
			float reward = calc_reward(state1, state2);
			update_q_value(state1, state2, reward, action1, action2);
			update_policy();
			action1 = action2;
			rl_soaring_perform_action(action1);
		}
		else{
		   // If the episode has not been then start a new episode
		   printf("\n\n\nStarting a new episode.. \n");
		   rl_soaring_start_episode();
		}
	}
	// Standart navigational loop
    rl_navigation();
    return 1;
}


// Get either an action from the policy or a random action depending on the current epsilon greed
uint16_t rl_soaring_get_action(rl_state state){
    uint16_t action;
    float epsilon = random_float_in_range(0,1);
    if (epsilon<rl_exploration_rate){
        action = action_space[rand() % ACTION_SIZE_1];
    } else{
        action = current_policy[state.dist_wp_idx][state.dist_wp_idx_old];
    }
    return action;
}


// Function which performs the action which is required
void rl_soaring_perform_action(uint16_t action){
    // If conditions depending on chosen_action
	// Action 1, increase ground speed
	if (action == 0){
		v_ctl_auto_groundspeed_setpoint += 0.5;
	} else if (action == 1){
		v_ctl_auto_groundspeed_setpoint += 0.1;
	} else if (action == 2){	// Action 2, same ground speed
	} else if (action == 3){	// Action 3, decrease ground speed
		v_ctl_auto_groundspeed_setpoint -= 0.1;
	} else if (action == 4){
		v_ctl_auto_groundspeed_setpoint -= 0.5;
	}
}



/** Function called when the module is stopped, performs the following functions:
 * */
void rl_soaring_stop(void){
    // Reset episode counter
    episode = 0;
}


/*
 *  Generate random float within a specified range
 */

float random_float_in_range(float min, float max)
{
    float range = (max - min);
    float div = RAND_MAX / range;
    return min + (rand() / div);
}
