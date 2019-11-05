/*
 * Copyright (C) 2019 Freek van Tienen <freek.v.tienen@gmail.com>
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
 * @file "modules/ctrl/follow_me.h"
 * @author Freek van Tienen <freek.v.tienen@gmail.com>
 * Control a rotorcraft to follow at a defined distance from the target
 */

#ifndef FOLLOW_ME_H
#define FOLLOW_ME_H

#include "std.h"
// #include "firmware"
#include "firmwares/fixedwing/guidance/guidance_v_n.h"


// Variables required for settings and file logger
extern uint8_t follow_me_distance;
extern uint8_t follow_me_height;
extern float follow_me_heading;
extern float desired_ground_speed;
extern float actual_ground_speed;
extern float dist_wp_follow;
extern float dist_wp_follow_old;
int8_t follow_me_location;
struct Int32Vect3 wp_ground_utm;


extern float ground_speed_diff_igain;
extern float ground_speed_diff_pgain;
extern float ground_speed_diff_dgain;

/** init function */
extern void follow_me_init(void);

// Starting function in order to initialsie parameters such as follow_me_height
extern void follow_me_startup(void);

// Function called before each follow_me_call
extern int follow_me_pre_call(void);

/** on receiving a GROUND_GPS message
 */
extern void follow_me_parse_ground_gps(uint8_t *buf);

// run function
extern int follow_me_call(void);

extern int follow_me_set_wp(void);

#endif

