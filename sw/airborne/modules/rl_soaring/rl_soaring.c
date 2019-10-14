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
#include "firmwares/rotorcraft/stabilization.h"
#include "filters/low_pass_filter.h"
#include "subsystems/gps/gps_datalink.h"

// Include other paparazzi modules
#include "generated/modules.h"
#include "subsystems/datalink/datalink.h" // dl_buffer

// Set pre-processor constants
#define rl_soaring_LOG TRUE
#define rl_soaring_TELEMETRY TRUE

// Declaration of global variables
float rl_soaring_filter_cutoff = 3.0;
float rl_soaring_termination_dist = 0.05;
float rl_soaring_descend_speed = -0.3;
float rl_wall_heading = 237.0;
int rl_soaring_policy_received = false;
float rl_exploration_rate = 0.0;
int rl_autostart = false;
int rl_exploring_starts = true;
int rl_exploring_starts_frozen = false;

// Declaration of local variables
static int32_t number_of_variables = 0;
struct timeval currentTime;
static int32_t start_time_seconds = 0;
static int32_t start_time_milliseconds = 0;
static int32_t episode_start_time_seconds = 0;
static int32_t episode_start_time_milliseconds = 0;

static int32_t episode = 0;
static int32_t timestep = 0;
static int32_t time_rl = 0;
static int32_t episode_time_rl = 0;

// Episode variables
static int32_t max_episode_length = 10000;

// RL variables
#define POLICY_SIZE_1 9
#define POLICY_SIZE_2 3

static uint16_t prev_action;
static rl_state current_state;
static uint16_t action_policy;
static uint16_t action_chosen;
static uint16_t action_performed;
static float discr_state_F_ext_bounds[POLICY_SIZE_1] = {-1.05, -0.95, -0.85, -0.75, -0.65, -0.55, -0.45, -0.35, -0.25}; // Bebop 1

static uint16_t action_space[POLICY_SIZE_2] = {1, 2, 3};
static uint16_t current_policy[POLICY_SIZE_1][POLICY_SIZE_2] = {};

static int rl_episode_fail = false;
static int rl_episode_timeout = false;

static void rl_soaring_send_message_down(char *_request, char *_parameters);
static uint16_t rl_soaring_get_action(rl_state state);
static void rl_soaring_state_estimator(void);
static void rl_soaring_perform_action(uint16_t action);
static int rl_soaring_check_crash(void);
static float randn (float mu, float sigma);
static float random_float_in_range(float min, float max);
static void send_rl_variables(struct transport_tx *trans, struct link_device *dev);
static void send_rl_variables(struct transport_tx *trans, struct link_device *dev){
    // When prompted, return all the telemetry variables
    pprz_msg_send_rl_soaring(trans, dev, AC_ID,
                                        &timestep, &time_rl,
                                        &body_acceleration_u, &body_acceleration_v, &body_acceleration_w,
                                        &body_speed_u, &body_speed_v, &body_speed_w, &gps_vertical_speed,
                                        &enu_position_x, &enu_position_y, &enu_position_z,
                                        &body_rate_p, &body_rate_q, &body_rate_r,
                                        &body_attitude_phi, &body_attitude_theta, &body_attitude_psi,
                                        &motor_speed_nw, &motor_speed_ne, &motor_speed_se, &motor_speed_sw,
                                        &F_ext_onboard_est, &F_ext_rolling_mean);
}

/** Initialization function **/
void rl_soaring_init(void) {
    // Register telemetery function
    register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_rl_soaring, send_rl_variables);
}

/** Function called when the module is started, performs the following functions:
 * -> Create log file
 * -> Open log file
 * */
void rl_soaring_start(void){
    // HERE WAS INITIAL VARIABLES OF STATES ETC

    // Set start time in seconds
    gettimeofday(&currentTime, NULL);
    start_time_seconds = currentTime.tv_sec;
    start_time_milliseconds = currentTime.tv_usec;
}

void rl_soaring_update_measurements(void){
    // Update timestep
    timestep++;

    // Get time in milliseconds since the measurement has started
    gettimeofday(&currentTime, NULL);
    time_rl = (currentTime.tv_sec - start_time_seconds) * 1000 + (currentTime.tv_usec) / 1000 - start_time_milliseconds / 1000;
    if( episode > 0) {
        episode_time_rl = (currentTime.tv_sec - episode_start_time_seconds) * 1000 + (currentTime.tv_usec) / 1000 -
                          episode_start_time_milliseconds / 1000;
    }

    // HERE WAS UPDATE OF ALL THE STATES
}



/* Function called at the start of each episode
 */
void rl_soaring_start_episode(){
    // Set intervention & fail states to false
    rl_episode_fail = false;

    // Increase episode counter and save start time
    episode = episode+1;
    gettimeofday(&currentTime, NULL);
    episode_start_time_seconds = currentTime.tv_sec;
    episode_start_time_milliseconds = currentTime.tv_usec;

    // Set action to one (no-action) (for the state prev_action)
    action_chosen = 1;
    action_performed = 1;

    // Determine exploring starts height
    if(rl_exploring_starts){
        rl_exploring_starts_frozen = true;
        rl_starting_height = random_float_in_range(0.1, 0.45);
    } else {
        rl_exploring_starts_frozen = false;
        rl_starting_height = 1.0;
    }

    // Print to the console
    if(rl_exploring_starts) {
        printf("New episode started, episode number %d, frozen till %.2fm\n", episode, rl_starting_height);
    } else {
        printf("New episode started, episode number %d\n", episode);
    }
}

/*
 * Function periodic, the heartbeat of the module
 */
void rl_soaring_periodic(void) {
    double random_double;

    // Update measurements
    rl_soaring_update_measurements();

    // Get state
    rl_soaring_state_estimator();

    if((flight_status >= 50) && (flight_status < 60) && (rl_episode_fail == false)){

        // Check for episode timeout
        if((rl_intervention_hover == false) && (rl_intervention_save == false) && (rl_episode_timeout == false)) {
            if (episode_time_rl > max_episode_length) {
                rl_episode_timeout = true;
                printf("Episode timeout at %d seconds\n", (int) episode_time_rl / 1000);
            }
        }

        // Check for end of exploring starts freeze


        // Only pick and perform an action if we're not yet in another action (check with booleans)
            // Prepare next timestep, get next action from policy

            // Check whether we explore or not
            // Chose action based on exploration or exploitation

            // Check safety of action and need to abort
            // iChose the action that was performed

            // Perform action
    }
}

void rl_soaring_end_episode(){
    rl_episode_fail = false;
    rl_episode_timeout = false;
    printf("Episode ended\n");
}

void rl_soaring_state_estimator(void){
    // Re estimate states
}

/*
 *  Function used to check if the quadrotor is 'crashed' and the episode should be aborted
 */
int rl_soaring_check_crash(void){
    // Check whether we are in window
}

// Obtain a new action
uint16_t rl_soaring_get_action(rl_state current_state){
    uint16_t action;

   // Get action from policy
    action = current_policy[current_state.discr_F_ext][current_state.discr_prev_action];
    return action;
}

void rl_soaring_perform_action(uint16_t action){
    // If conditions depending on chosen_action
}

// Here there were a lot of function with all the actions

int rl_soaring_fail(void){
    return rl_episode_fail;
}

int rl_soaring_timeout(void){
    return rl_episode_timeout;
}

// Here there were function which set the start of an action to false (in case they terminate)

void rl_soaring_turn_on(void){
    rl_soaring_rl_soaring_periodic_status = MODULES_START;
}

void rl_soaring_turn_off(void){
    rl_soaring_rl_soaring_periodic_status = MODULES_STOP;
}

/** Function called when the module is stopped, performs the following functions:
 * -> Close log file
 * */
void rl_soaring_stop(void){
    // Reset episode counter
    episode = 0;
}

/** Functon used to set and store the flight status:
 * */
void rl_soaring_flight_status(int status){
    flight_status = status;
}


/*
 *  FUnction used to generate a random number from normal distribution
 */
float randn (float mu, float sigma)
{
  float U1, U2, W, mult;
  static float X1, X2;
  static int call = 0;

  if (call == 1)
    {
      call = 0;
      return (mu + sigma * (float) X2);
    }

  do
    {
      U1 = -1 + ((float) rand () / RAND_MAX) * 2;
      U2 = -1 + ((float) rand () / RAND_MAX) * 2;
      W = pow (U1, 2) + pow (U2, 2);
    }
  while (W >= 1 || W == 0);

  mult = sqrt ((-2 * log (W)) / W);
  X1 = U1 * mult;
  X2 = U2 * mult;

  call = !call;

  return (mu + sigma * (float) X1);
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

/*
 *  Request new policy
 */

void rl_soaring_request_new_policy(void){
    char request[100] = "";
    char parameters[100] = "";

    // Start waiting for new policy
    rl_soaring_policy_received = false;

    // Send message
    strcpy(request, "request_new_policy");
    sprintf(parameters, "%s %d", rl_soaring_run_filename, episode);
    rl_soaring_send_message_down(request, parameters);

    printf("Waiting for new policy\n");
}

/*
 *  Function to send a message down
 */

void rl_soaring_send_message_down(char *_request, char *_parameters){
    uint16_t nb_request;
    uint16_t nb_parameters;
    struct pprzlink_msg msg;

    // Calculate length of strings
    nb_request = strlen(_request);
    nb_parameters = strlen(_parameters);

    // Setup message struct
    msg.trans = &(DefaultChannel).trans_tx;
    msg.dev = &(DefaultDevice).device;
    msg.sender_id = AC_ID;
    msg.receiver_id = 0;
    msg.component_id = 0;

    pprzlink_msg_v2_send_RL_TRAINING_DOWN(&msg, nb_request, _request, nb_parameters, _parameters);
}

/*
 * Parse the uplink messages, receive and implement new policy
 */
void rl_soaring_parse_uplink(void){
//    uint16_t ac_id;
    uint16_t index_1;
    uint16_t index_2;
    uint16_t value;
    uint16_t old_policy;

    // Get request and parameters from datalink buffer
//    ac_id = pprzlink_get_DL_RL_TRAINING_UP_ac_id(dl_buffer);
    index_1 = pprzlink_get_DL_RL_TRAINING_UP_index_1(dl_buffer);
    index_2 = pprzlink_get_DL_RL_TRAINING_UP_index_2(dl_buffer);
    value = pprzlink_get_DL_RL_TRAINING_UP_value(dl_buffer);

    // Update policy
    old_policy = current_policy[index_1][index_2];
    current_policy[index_1][index_2] = value;

    printf("Updated policy of state [%u][%u]: %u -> %u\n", index_1, index_2, old_policy, value);

    // Check if this was the last update
    if(index_1 == (POLICY_SIZE_1-1) && index_2 == (POLICY_SIZE_2-1)){
        printf("Policy completely updated\n");
        rl_soaring_policy_received = true;
    }

}


