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

#include "math/pprz_algebra_int.h"


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
#include <Ivy/ivy.h>
//#include <Ivy/ivyglibloop.h>

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

static int32_t start_time_seconds = 0;
static int32_t start_time_milliseconds = 0;
static int32_t episode_start_time_seconds = 0;

static int32_t episode = 0;
static int32_t timestep = 0;
static int32_t time_rl = 0;
static int32_t episode_time_rl = 0;

// Episode variables
static int32_t max_episode_time = 120000; // in seconds

// RL variables
float desired_accuracy = 10; // accuracy at which the discretised states are created


// Need to have struct for array because otherwise we have 2 possibilities for state 6 for example, 2x3 and 3x2
// NOTE: USE ODD VALUE SO THAT WINDOW CAN BE CENTERED AROUND 0.
// EVEN VALUES HAVE NOT BEEN TESTED!!
#define state_size_lateral 3
#define state_size_longitudinal 1
#define state_size_height 3
#define action_size_lateral 3  // hardcoded as well, as it is the same
#define action_size_longitudinal 1
#define action_size_height 3


float state_accuracy = 50; // accuracy at which states are seperated between each other

// Create variables for reference of flying window
float lateral_offset_reference;
float follow_me_distance_reference;
float follow_me_height_reference;


static float Q[state_size_lateral][state_size_height][action_size_lateral][action_size_height];

// Gives action to be performed based on current state
// Inputs are the current state and output the desired state
static struct Int8Vect3 current_policy[state_size_lateral][state_size_height] = {};

static struct Int8Vect3 state1;
static struct Int8Vect3 state2;
static struct Int8Vect3 current_state;
static struct Int8Vect3 desired_state;
static struct Int8Vect3 action1;
static struct Int8Vect3 action2;
float reward;

static uint8_t rl_episode_timeout; // not used
static uint8_t rl_episode_boatcrash;
static uint8_t rl_episode_out_of_window;
static uint8_t rl_episode_beyond_wp;
static uint8_t rl_episode_target_reached;
static uint8_t rl_episode_started;
int rl_started = false;

// static void rl_soaring_send_message_down(char *_request, char *_parameters);
static struct Int8Vect3 rl_soaring_get_action(struct Int8Vect3 state);
float calc_reward(void);

static void rl_soaring_perform_action(struct Int8Vect3 action);
static float random_float_in_range(float min, float max);
static void send_rl_variables(struct transport_tx *trans, struct link_device *dev);
void update_policy(void);
void update_q_value(struct Int8Vect3, struct Int8Vect3, float, struct Int8Vect3, struct Int8Vect3);


struct Int8Vect3 state_to_idx(void);
struct Int8Vect3 state_to_idx(){
	// Obtain current offsets
    struct FloatVect3 current_attitude = compute_state();

    printf("Current state has been computed as (%f, %f %f)\n", current_attitude.x, current_attitude.y, current_attitude.z);
    int8_t lateral_state = round((current_attitude.x - lateral_offset_reference)/state_accuracy);
    int8_t forward_state = round((current_attitude.y - follow_me_distance_reference)/state_accuracy);
    int8_t height_state = round((current_attitude.z - follow_me_height_reference)/state_accuracy);

    if ((abs(ceil(lateral_state)) > state_size_lateral/2.) || (abs(ceil(height_state)) > state_size_height/2) ){
    	rl_episode_out_of_window = 0;
    	// printf("Out of window because %d > %d || %d > %d\n", abs(ceil(lateral_state)), state_size_lateral/2, abs(ceil(height_state)), state_size_height/2);
    }

    printf("The reference window is centered at %f %f %f\n", lateral_offset_reference, follow_me_distance_reference, follow_me_height_reference);
    printf("Based on this the following indices have been obtained (%d %d %d)\n", lateral_state, forward_state, height_state);

    struct Int8Vect3 idx;
    idx.x = lateral_state;
    idx.y = forward_state; // not used at the moment
    idx.z = height_state;
    return idx;
}

struct FloatVect3 idx_to_state(struct Int8Vect3);
struct FloatVect3 idx_to_state(struct Int8Vect3 idx){
	printf("Performing idx_to_state calculation for idx (%d %d %d)\n", idx.x, idx.y, idx.z);
    struct FloatVect3 actual_state;
    actual_state.x = lateral_offset_reference + idx.x*state_accuracy;
    actual_state.y = follow_me_distance_reference + idx.y*state_accuracy;
    actual_state.z = follow_me_height_reference + idx.z*state_accuracy;
    printf("Based on this the state is given by (%f %f %f)\n\n", actual_state.x, actual_state.y, actual_state.z);
    return actual_state;
}


// Function that loads an existing Q file in order to obtain the Q values for each corresponding action
static void rl_load_Q_file(void){
	// FILE *f = fopen("/home/chris/paparazzi/sw/airborne/modules/rl_soaring/rl_Q.csv", "r");
	// if (f == NULL){
	    for (int i=0; i<state_size_lateral; i++){
		    for (int j=0; j<state_size_height; j++){
			    for (int k=0; k<action_size_lateral; k++){
				    for (int l=0; l<action_size_height; l++){
				    	Q[i][j][k][l] = 0;
				    }
			    }
		    }
	    }
	    return;
	/* }

	char line[1024]; // assumption that we have a maximum of 1024 characters per line which should be sufficient
	if (fgets(line, 1024, f) == NULL){
		printf("Error obtaining first line in load Qfile\n"); // remove first line which is only header
	}
	char *token;
	token = strtok(line, ",");
	episode = atoll(token);
	for (int i=0; i<STATE_SIZE_1; i++){
		// Obtain the first line which is under format State1.1 Q1.1.1, Q1.1.2, Q 1.1.3, Q 1.1.4, Q1.1.15, Q 1.2.1
	    if (fgets(line, 1024, f) == NULL){
	    	printf("Error obtaining line in load Qfile\n");
	    }
	    token = strtok(line, ",");  // This takes the first cell of the current line which is simply the state
	    int j = 0;
	    int k = 0;
	    while (j<STATE_SIZE_2){
	        token = strtok(NULL, ",");  // this gives the next cell
	        Q[i][j][k] = atof(token);
	        k++;
	        if (k==ACTION_SIZE_1){
	        	k=0;
	        	j++;
	        }
	    }
	} */

}

static void rl_write_Q_file(void){
	FILE *f = fopen("/home/chris/paparazzi/sw/airborne/modules/rl_soaring/rl_Q.csv", "w");
	if (f == NULL){
	    printf("Error opening Q table file for writing!\n");
	    return;
	}

	// First print the header consisting of only the possible states2
	fprintf(f, "Episodes trained, %d\n", episode);
	fprintf(f, "State.x, State.z, Action.x, Action.y, Qvalue\n");
	for (int i=0; i<state_size_lateral; i++){
		for (int j=0; j<state_size_height; j++){
			for (int k=0; k<action_size_lateral; k++){
				for (int l=0; l<action_size_height; l++){
					fprintf(f, "%d,%d,%d,%d,%f\n", i, j, k, l, Q[i][j][k][l]);
				}
			}
		}
	}

	// Close the file
	fclose(f);
}


// sends all the messages through the pprzlink which are update in rl_soaring_update_measurements
static void send_rl_variables(struct transport_tx *trans, struct link_device *dev){
     // When prompted, return all the telemetry variables
	uint timestamp = 0;
	uint episodes = 0;
	float old_distance = 0;
	float current_distance = 0;
	float foo = 0;
    pprz_msg_send_RL_SOARING(trans, dev, AC_ID, &timestamp, &episodes, &old_distance, &current_distance, &foo);

}


/** Initialization function **/
void rl_soaring_init(void) {
	// Create a random seed based on current time
	gettimeofday(&currentTime, NULL);
	srand(currentTime.tv_sec);

    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_RL_SOARING, send_rl_variables);
}



/** Function called when the module is started, performs the following functions:
 * */
void rl_soaring_start(void){
	if (!rl_started){
		// Initialise window reference
	    lateral_offset_reference = lateral_offset;
		follow_me_distance_reference = follow_me_distance;
		follow_me_height_reference = follow_me_height;

        // Load Q values and generate current policy
		rl_load_Q_file();

		// Update policy based on these Q values
		update_policy();

		// Set start time in seconds
		gettimeofday(&currentTime, NULL);
		start_time_seconds = currentTime.tv_sec;
		start_time_milliseconds = currentTime.tv_usec;
		rl_started = true;
	}
	else{
		printf("Rl soaring_start has not been executed. Start already..\n");
	}
}


/* Function called at the start of each episode
 */
void rl_soaring_start_episode(){
    // Set intervention & fail states to false
    rl_episode_beyond_wp = false; // in case we fly beyond wp2
    rl_episode_boatcrash = false; // in case we crash the boat
    rl_episode_target_reached = false;
    rl_episode_out_of_window = false;
    rl_episode_timeout = false; // in case we have a timeout
    rl_episode_started = true;

    // Increase episode counter and save start time
    episode++;
    gettimeofday(&currentTime, NULL);
    episode_start_time_seconds = currentTime.tv_sec;

    state1 = current_state;
    printf("State 1 has been set to %d %d %d\n", state1.x, state1.y, state1.z);
}


// Not used at all for now ///////
void rl_soaring_end_episode(void){
	if (rl_episode_target_reached){
		printf("Ended because target has been reached\n");
		rl_episode_target_reached = false;
	}
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
	if (rl_episode_out_of_window){
		printf("Ended because out of window\n");
		rl_episode_out_of_window = false;
	}

	// Calculate end state of an episode and obtain the new desired action
	state2 = current_state;
	printf("State 2 has been set to (%d %d %d)\n", state2.x, state2.y, state2.z);
	action2 = rl_soaring_get_action(state2);
	printf("Action 2 set to %d %d %d\n", action2.x, action2.y, action2.z);

	// Calculate reward based on beginning and end state and update the policies accordingly
	reward = calc_reward();
	update_q_value(state1, state2, reward, action1, action2);
	update_policy();
	action1 = action2;
	printf("Action 1 set to %d %d %d\n", action1.x, action1.y, action1.z);
	rl_soaring_perform_action(action1);

	printf("Episode %d ended. Total flying time: %d.\n", episode, episode_time_rl);
	rl_episode_started = false;


}



// Updates all the mesaurements that are required for the follow me module
// This means the state is estimated based on the newly set waypoint of the follow_me module
void rl_soaring_update_measurements(void){
    // Update timestep
    timestep++;
    // Get time in milliseconds since the measurement has started
    gettimeofday(&currentTime, NULL);
    time_rl = (currentTime.tv_sec - start_time_seconds);
    if(episode > 0) {
        episode_time_rl = (currentTime.tv_sec - episode_start_time_seconds) ;
        if (episode_time_rl > max_episode_time){
        	rl_episode_timeout = true;
        }
    }

    // In order to calculate state first calculate location of current waypoints
    follow_me_compute_wp();

    // Convert these states to an idx
    current_state = state_to_idx();
}



void rl_navigation(void);
void rl_navigation(void){
	compute_follow_distances();

    // Loop through controllers
	follow_me_throttle_pid();

	// Standart navigational loop
	NavGotoWaypoint(WP__FOLLOW2);
    NavVerticalAltitudeMode(follow_me_altitude, 0.);
}


int rl_episode_stop_condition(void);
int rl_episode_stop_condition(void){
	// if (rl_episode_timeout || rl_episode_boatcrash || rl_episode_beyond_wp || rl_episode_out_of_window || rl_episode_target_reached){
	if (rl_episode_target_reached){
		return 1;
	}
	else{
		return 0;
	}
}

float calc_reward(void){
	return 0;
}

void update_q_value(struct Int8Vect3 s1, struct Int8Vect3 s2, float r, struct Int8Vect3 a1, struct Int8Vect3 a2){
	float predicted_q_value = Q[s1.x][s1.z][a1.x][a1.z];
    float target = r + rl_gamma * Q[s2.x][s2.z][a2.x][a2.z];
    Q[s1.x][s1.z][a1.x][a1.z] = Q[s1.x][s1.z][a1.x][a1.z] + rl_alpha * (target - predicted_q_value);
}


void update_policy(void){
	for (int i=0; i<state_size_lateral; i++){ // moves through the lateral state
	    for (int j=0; j<state_size_height; j++){ // moves through the height state
	    	// Now we need to find the maximum Q value of all the actions
	    	int max_index_lateral = 0;
	    	int max_index_height = 0;
	    	int maximum = 0;
	    	for (int k=0; k<action_size_lateral; k++){
	    		for (int l=0; l<action_size_height; l++){
					if (Q[i][j][k][l] > maximum){
						maximum = Q[i][j][k][l];
						max_index_lateral = k;
						max_index_height = l;
					}
	    		}
	    	}
	    	// Update the current policy accordingly
	    	struct Int8Vect3 desired_action;
	    	desired_action.x = max_index_lateral;
	    	desired_action.y = 0; // not used
	    	desired_action.z = max_index_height;
	        current_policy[i][j] = desired_action;
	    }
	}
}


/*
 * Function periodic, the heartbeat of the module
 */
int rl_soaring_call(void) {
	if (!rl_started){
		rl_soaring_start();
	}
	// Backup Q file after 10 episodes
	if ((episode % 2 == 0) && (episode != 0)){
		 rl_write_Q_file();
	}

    gettimeofday(&nowcallTime, NULL);

	// The start of this module is called constantly by the fact that the intiialisation of the module created the periodique call
	// Update measurements
	rl_soaring_update_measurements();



	// If we have reached our target the episode ends
	// if ((current_state.x == desired_state.x) && (current_state.y == desired_state.y) && (current_state.z == desired_state.z)){
	if ((fabs(dist_wp_follow.x) < 4) && (fabs(dist_wp_follow.y) < 4)){
	    rl_episode_target_reached = 1;
	    printf("Target has been reached and dist.wp_follow is given by: (%f %f %f)\n", dist_wp_follow.x, dist_wp_follow.y, dist_wp_follow.z);
	} else {
		printf("Target has not been reached because dist.wp_follow is given by: (%f %f %f)\n", dist_wp_follow.x, dist_wp_follow.y, dist_wp_follow.z);
	}

	// Check whether the episode has ended
	if (rl_episode_stop_condition()){
		 rl_episode_started = false;
		 rl_soaring_end_episode();
	}

	// Check whether we are in an episode already
	if (!rl_episode_started){
	   // If the episode has not been then start a new episode
	   printf("\n\n\nStarting a new episode.. \n");
	   rl_soaring_start_episode();
	}

	// Standart navigational loop
    rl_navigation();
    return 1;
}


// Get either an action from the policy or a random action depending on the current epsilon greed
struct Int8Vect3 rl_soaring_get_action(struct Int8Vect3 actual_state){
    struct Int8Vect3 action;
    float epsilon = random_float_in_range(0,1);
    if (epsilon<rl_exploration_rate){
        action.x = rand() % action_size_lateral - (action_size_lateral-1)/2;
        action.y = rand() % action_size_longitudinal - (action_size_longitudinal-1)/2;
        action.z = rand() % action_size_height - (action_size_height-1)/2;
        printf("Obtaining random action (%d %d %d)\n", action.x, action.y, action.z);
    } else{
    	printf("Obtaining predefined action (%d %d %d)\n", action.x, action.y, action.z);
        action = current_policy[actual_state.x][actual_state.z];
    }
    return action;
}


// Function which performs the action which is required
void rl_soaring_perform_action(struct Int8Vect3 action){
	struct FloatVect3 carr = idx_to_state(action);
	desired_state = action;
	lateral_offset = carr.x;
    // follow_me_distance = carrc.y;
    follow_me_height = carr.z;
    printf("Follow me height has been set to %d\n", follow_me_height);
}



/** Function called when the module is stopped, performs the following functions:
 * */
void rl_soaring_stop(void){
    // Reset episode counter
    // episode = 0;
    // rl_started = false;
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
