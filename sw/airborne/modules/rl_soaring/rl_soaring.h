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
 * @file "modules/rl_obstacle_avoidance/rl_obstacle_avoidance.h"
 * @author GJ van Dam
 * Obstacle avoidance method using RL and the aerodynamic interaction between obstacle and quadrotor
 */

#ifndef RL_SOARING_H
#define RL_SOARING_H

#include "std.h"

// Define data structures
typedef struct {
    char name[30];
    char type[10];
    char format[5];
    void *pointer;
} rl_variable;

#define RL_ROLLING_WINDOW_SIZE 25
#define DL_RL_TRAINING_UP 77

typedef struct{
//    float F_ext_rolling[RL_ROLLING_WINDOW_SIZE];
    int discr_F_ext;
    int discr_prev_action;

} rl_state;

// Variables
extern float rl_soaring_termination_dist;
extern float rl_soaring_descend_speed;
extern float rl_wall_heading;
extern int rl_soaring_policy_received;
extern int set_estimated_accel_bias;
extern int set_estimated_k_over_m;
extern float rl_exploration_rate;
extern int rl_autostart;
extern int rl_exploring_starts;

// Functions
extern void rl_soaring_init(void);
extern void rl_soaring_start(void);
extern void rl_soaring_periodic(void);
extern void rl_soaring_stop(void);
extern void rl_soaring_turn_on(void);
extern void rl_soaring_turn_off(void);

extern void rl_obstace_avoidance_new_log_file(void);
extern int rl_soaring_est_accel_bias(void);
extern void rl_soaring_est_k_over_m(void);
extern void rl_soaring_update_measurements(void);
extern void rl_soaring_flight_status(int);

extern int rl_soaring_hover(void);
extern int rl_soaring_save(void);
extern int rl_soaring_fail(void);
extern int rl_soaring_timeout(void);
extern void rl_soaring_hover_concluded(void);
extern void rl_soaring_save_concluded(void);

extern void rl_soaring_parse_uplink(void);
extern void rl_soaring_start_episode(void);
extern void rl_soaring_end_episode(void);

extern void rl_soaring_request_new_policy(void);
//extern void send_rl_variables(struct transport_tx *trans, struct link_device *dev);

#endif

