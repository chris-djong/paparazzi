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

extern int rl_teleported;

typedef struct{
    // states
	int dist_wp_idx;
	int dist_wp_idx_old;
} rl_state;



// Functions
extern void rl_soaring_init(void);
extern void rl_soaring_start(void);
extern int rl_soaring_call(void);
extern void rl_soaring_stop(void);

extern void rl_soaring_update_measurements(void);


extern void rl_soaring_start_episode(void);
extern void rl_soaring_end_episode(void);

#endif

