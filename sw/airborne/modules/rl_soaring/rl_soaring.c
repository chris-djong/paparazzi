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
#include <unistd.h>

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
float desired_accuracy = 3; // the uav has reached its state in case all direction are lower than the desired accuracy


// Need to have struct for array because otherwise we have 2 possibilities for state 6 for example, 2x3 and 3x2
// NOTE: USE ODD VALUE SO THAT WINDOW CAN BE CENTERED AROUND 0.
// NOT WORKING ON EVEN VALUES --> TODO: CREATE ASSERTION
#define state_size_lateral 3
#define state_size_longitudinal 1
#define state_size_height 1
#define action_size_lateral 3   // hardcoded as well, as it is the same
#define action_size_longitudinal 1
#define action_size_height 1


uint8_t state_accuracy = 7; // accuracy at which states are seperated between each other

uint8_t simulating_counter = 0;

// Create variables for reference of flying window
float lateral_offset_reference;
float follow_me_distance_reference;
float follow_me_height_reference;


static float Q[state_size_lateral][state_size_longitudinal][state_size_height][action_size_lateral][action_size_longitudinal][action_size_height];

// Gives action to be performed based on current state
// Inputs are the current state and output the desired state
static struct Int8Vect3 current_policy[state_size_longitudinal][state_size_lateral][state_size_height] = {};

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
static float rl_episode_integrated_reward;
static uint32_t rl_episode_amount_iterations;
int rl_started = false;

// static void rl_soaring_send_message_down(char *_request, char *_parameters);
static struct Int8Vect3 rl_soaring_get_action(struct Int8Vect3 state);
float calc_reward(void);

static void rl_soaring_perform_action(struct Int8Vect3 action);
static float random_float_in_range(float min, float max);
static void send_rl_variables(struct transport_tx *trans, struct link_device *dev);
void update_policy(void);
void update_q_value(struct Int8Vect3, struct Int8Vect3, float, struct Int8Vect3, struct Int8Vect3);

uint8_t average_distance_size = 15;
struct FloatVect3 average_distance;
struct FloatVect3 AverageDistance(struct FloatVect3);
// Calculates the average distance based on the previous average distance
// It should be noted that this function is incorrect in case we have not enough measurements (at least average_distance_size measurements are required)
// Used for the calculation on when an epiosde is terminated
struct FloatVect3 AverageDistance(struct FloatVect3 distance){
	struct FloatVect3 average;
	average.x = (average_distance.x * (average_distance_size-1) + distance.x) / (average_distance_size);
	average.y = (average_distance.y * (average_distance_size-1) + distance.y) / (average_distance_size);
	average.z = (average_distance.z * (average_distance_size-1) + distance.z) / (average_distance_size);
    return average;
}

struct Int8Vect3 state_to_idx(void);
struct Int8Vect3 state_to_idx(){
	// Obtain current offsets
    struct FloatVect3 current_attitude = compute_state();

    int8_t lateral_state = round((current_attitude.x - lateral_offset_reference)/state_accuracy);
    int8_t forward_state = round((current_attitude.y - follow_me_distance_reference)/state_accuracy);
    int8_t height_state = round((current_attitude.z - follow_me_height_reference)/state_accuracy);

    if ((abs(ceil(lateral_state)) > state_size_lateral/2.) || (abs(ceil(height_state)) > state_size_height/2) ){
    	rl_episode_out_of_window = 0;
    }

    struct Int8Vect3 idx;
    idx.x = lateral_state;
    idx.y = forward_state; // not used at the moment
    idx.z = height_state;
    return idx;
}

struct FloatVect3 idx_to_state(struct Int8Vect3);
struct FloatVect3 idx_to_state(struct Int8Vect3 idx){
    struct FloatVect3 actual_state;
    actual_state.x = lateral_offset_reference + idx.x*state_accuracy;
    actual_state.y = follow_me_distance_reference + idx.y*state_accuracy;
    actual_state.z = follow_me_height_reference + idx.z*state_accuracy;
    return actual_state;
}


static void rl_write_Q_file(void);

// Function that loads an existing Q file in order to obtain the Q values for each corresponding action
static void rl_load_Q_file(void){
	// Open the File
	char filename[150];
	sprintf(filename, "/home/chris/paparazzi/sw/airborne/modules/rl_soaring/rl_Q_%d_%d_%d_%d_%d_%d_%d.csv", state_size_lateral, state_size_longitudinal, state_size_height, action_size_lateral, action_size_longitudinal, action_size_height, state_accuracy);
    // sprintf(filename, "/data/ftp/internal_000/rl_soaring/rl_Q_%d_%d_%d_%d_%d_%d_%d.csv", state_size_lateral, state_size_longitudinal, state_size_height, action_size_lateral, action_size_longitudinal, action_size_height, state_accuracy);
	if (access(filename, F_OK) == -1){
		printf("File does not exist. Creating new file.\n");
		/* Uncomment this text in the desired Q table does not exist yet */
		for (int i=0; i<state_size_lateral; i++){
			for (int j=0; j<state_size_longitudinal; j++){
				for (int k=0; k<state_size_height; k++){
					for (int l=0; l<action_size_lateral; l++){
						for (int m=0; m<action_size_longitudinal; m++){
							for (int n=0; n<action_size_height; n++){
								Q[i][j][k][l][m][n] = 0;
							}
						}
					}
				}
			}
		}
		rl_write_Q_file();
	}

	FILE *f = fopen(filename, "r");

	// Create a char for each line
	char line[1024]; // Assumption that we have a maximum of 1024 character in each line
	// Store the first line of of f in the line char
	if (fgets(line, 1024, f) == NULL){
		printf("Error obtaining first line in load Qfile\n"); // remove first line which is only header
	}
	// Create a token to store each cell and split the line into the different tokens
	char *token;
	// In case a line is given as an argument it just takes the first index.
	// In order to obtain the subsequent delimiters instead of line a NULL pointer is used
	token = strtok(line, ",");
	token = strtok(NULL, ","); // Obtain seconds cell of first line where amount of episodes trained is stored
	// A to long integer. Converts a string to an integer
	episode = atol(token);
	if (fgets(line, 1024, f) == NULL){
		printf("Error obtaining second line\n"); // Obtain line with header State.x, State.z, Action.x, Action.z, Qvalue
	}

	// Now we can loop through the subsequent lines
	while (fgets(line, 1024, f) != NULL){
		// Obtain the states
		token = strtok(line, ",");
		uint8_t state_x = atol(token);
		token = strtok(NULL, ",");
		uint8_t state_y = atol(token);
		token = strtok(NULL, ",");
		uint8_t state_z = atol(token);

		// Obtain the actions
		token = strtok(NULL, ",");
		uint8_t action_x = atol(token);
		token = strtok(NULL, ",");
		uint8_t action_y = atol(token);
		token = strtok(NULL, ",");
		uint8_t action_z = atol(token);

		// Store the Q value
		token = strtok(NULL, ",");
		float Q_value = atof(token);
		Q[state_x][state_y][state_z][action_x][action_y][action_z] = Q_value;
	}
	// Close the file
	fclose(f);
}

static void rl_write_Q_file(void){
	char filename[150];
	sprintf(filename, "/home/chris/paparazzi/sw/airborne/modules/rl_soaring/rl_Q_%d_%d_%d_%d_%d_%d_%d.csv", state_size_lateral, state_size_longitudinal, state_size_height, action_size_lateral, action_size_longitudinal, action_size_height, state_accuracy);
    // sprintf(filename, "/data/ftp/internal_000/rl_soaring/rl_Q_%d_%d_%d_%d_%d_%d_%d.csv", state_size_lateral, state_size_longitudinal, state_size_height, action_size_lateral, action_size_longitudinal, action_size_height, state_accuracy);

	FILE *f = fopen(filename, "w");
	if (f == NULL){
	    printf("Error opening Q table file for writing!\n");
	    return;
	}

	// First print the header consisting of only the possible states2
	fprintf(f, "Episodes trained, %d\n", episode);
	fprintf(f, "State.x, State.y, State.z, Action.x, Action.y, Action.z, Qvalue\n");
	for (int i=0; i<state_size_lateral; i++){
		for (int j=0; j<state_size_longitudinal; j++){
		    for (int k=0; k<state_size_height; k++){
			    for (int l=0; l<action_size_lateral; l++){
				    for (int m=0; m<action_size_longitudinal; m++){
				        for (int n=0; n<action_size_height; n++){
					        fprintf(f, "%d,%d,%d,%d,%d,%d,%f\n", i, j, k, l, m, n, Q[i][j][k][l][m][n]);\
				        }
				    }
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
	int32_t timestamp = 0;
	int32_t episodes = 0;
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
}


// Not used at all for now
void rl_soaring_end_episode(void){
	if (rl_episode_target_reached){
		rl_episode_target_reached = false;
	}
	if (rl_episode_beyond_wp){
	    rl_episode_beyond_wp = false;
	}
	if (rl_episode_boatcrash){
        rl_episode_boatcrash = false;
	}
	if (rl_episode_timeout){
	    rl_episode_timeout = false;
	}
	if (rl_episode_out_of_window){
		rl_episode_out_of_window = false;
	}

	// Calculate end state of an episode and obtain the new desired action
	state2 = current_state;
	action2 = rl_soaring_get_action(state2);

	// Calculate reward based on beginning and end state and update the policies accordingly
	reward = calc_reward();
	update_q_value(state1, state2, reward, action1, action2);
	update_policy();
	action1 = action2;
	rl_soaring_perform_action(action1);
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

    // Loop through controllers
	follow_me_throttle_pid();
	follow_me_roll_pid();

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
    float rew = rl_episode_integrated_reward/rl_episode_amount_iterations;
    rl_episode_integrated_reward = 0;
    rl_episode_amount_iterations = 0;
	return rew;
}

void update_q_value(struct Int8Vect3 s1, struct Int8Vect3 s2, float r, struct Int8Vect3 a1, struct Int8Vect3 a2){
	float predicted_q_value = Q[s1.x][s1.y][s1.z][a1.x][a1.y][a1.z];
    float target = r + rl_gamma * Q[s2.x][s2.y][s2.z][a2.x][a2.y][a2.z];
    Q[s1.x][s1.y][s1.z][a1.x][a1.y][a1.z] = Q[s1.x][s1.y][s1.z][a1.x][a1.y][a1.z] + rl_alpha * (target - predicted_q_value);
}


void update_policy(void){
	for (int i=0; i<state_size_lateral; i++){ // moves through the lateral state
		for (int j=0; j<state_size_longitudinal; j++){
			for (int k=0; k<state_size_height; k++){ // moves through the height state
				// Now we need to find the maximum Q value of all the actions
				int max_index_lateral = 0;
				int max_index_longitudinal = 0;
				int max_index_height = 0;
				int maximum = 0;
				for (int l=0; l<action_size_lateral; l++){
					for (int m=0; m<action_size_longitudinal; m++){
						for (int n=0; n<action_size_height; n++){
							if (Q[i][j][k][l][m][n] > maximum){
								maximum = Q[i][j][k][l][m][n];
								max_index_lateral = l;
								max_index_longitudinal = m;
								max_index_height = n;
							}
						}
					}
				}
				// Update the current policy accordingly
				struct Int8Vect3 desired_action;
				desired_action.x = max_index_lateral - (action_size_lateral - 1)/2; // max_index is between 0 and state_size_lateral*2, we want however also negative distances
				desired_action.y = max_index_longitudinal - (action_size_longitudinal - 1)/2;
				desired_action.z = max_index_height - (action_size_height - 1)/2;
				current_policy[i][j][k] = desired_action;
			}
	    }
	}
}


void rl_soaring_integrate_reward(void);
void rl_soaring_integrate_reward(void){
	// uint16_t current_throttle = 100 * autopilot.throttle / MAX_PPRZ;
	float speed = stateGetHorizontalSpeedNorm_f();
	float current_kinetic = 0.5*speed*speed;
	struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();
	float current_potential = 9.81*pos_Utm->alt;
	rl_episode_amount_iterations++;
	rl_episode_integrated_reward += current_kinetic + current_potential;
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
	compute_follow_distances();


	// The start of this module is called constantly by the fact that the intiialisation of the module created the periodique call
	// Update measurements
	rl_soaring_update_measurements();

	// Calculates current energy consumption
	rl_soaring_integrate_reward();



	// If we have reached our target the episode ends
	average_distance = AverageDistance(dist_wp_follow);
	simulating_counter++;
	// if ((fabs(average_distance.x) < desired_accuracy) && (fabs(average_distance.y) < desired_accuracy)){
	if (simulating_counter == 50){
	    rl_episode_target_reached = 1;
	    average_distance.x = 10;
	    average_distance.y = 10;
	    average_distance.z = 10;
	    simulating_counter = 0;
	}

	// Check whether the episode has ended
	if (rl_episode_stop_condition()){
		 rl_episode_started = false;
		 rl_soaring_end_episode();
	}

	// Check whether we are in an episode already
	if (!rl_episode_started){
	   // If the episode has not been then start a new episode
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
    } else{
        action = current_policy[actual_state.x][actual_state.y][actual_state.z];
    }
    return action;
}


// Function which performs the action which is required
void rl_soaring_perform_action(struct Int8Vect3 action){
	struct FloatVect3 carr = idx_to_state(action);
	desired_state = action;
	lateral_offset = carr.x;
    follow_me_distance = carr.y;
    follow_me_height = carr.z;
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
