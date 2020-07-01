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
//#include "firmware"
//#include "firmwares/fixedwing/guidance/guidance_v_n.h"


/************************************************
  Variables used by the settings
*************************************************/

extern int16_t follow_me_distance;
extern int16_t follow_me_distance_2;
extern int16_t stdby_distance;
extern int16_t follow_me_height;
extern float follow_me_altitude;
extern uint16_t follow_me_region;
extern float follow_me_heading;
extern int8_t lateral_offset;

extern float airspeed_igain;
extern float airspeed_pgain;
extern float airspeed_dgain;
extern uint8_t average_speed_size;
extern uint8_t follow_me_use_magnetometer;

extern float roll_enable; // when this x distance is exceeded the roll PID is enabled
extern float roll_disable; // when the x distance is lower the roll PID is disabled again
extern float roll_limit; // maximum and minimum allowable change in desired_roll_angle compared to the desired value by the controller -> 0.2 is around 10 degree
extern float roll_pgain;

float pitch_enable; // when this y distance is exceeded the pitch PID is enabled
float pitch_disable; // when the y distance is lower the pitch PID is disabled again
float pitch_pgain;
float pitch_dgain;
float pitch_igain;
float pitch_limit;

/************************************************
  Variables used by internal file logger
*************************************************/

extern uint8_t stationary_ground;
extern float ground_speed;
extern struct FloatVect3 dist_wp_follow;
extern struct FloatVect3 dist_wp_follow2;
extern struct FloatVect3 wp_follow_enu;
extern struct UtmCoor_f ground_utm;
extern uint8_t follow_me_roll; // boolean variable used to overwrite h_ctl_roll_setpoint in stab_adaptive and stab_attitude
extern uint8_t follow_me_pitch;


/************************************************
  Function declarations
*************************************************/

/** init function */
extern void follow_me_init(void);

// Starting function in order to initialsie parameters such as follow_me_height
extern void follow_me_startup(void);

// Function which is called once in each block which is not the follow me block
extern void follow_me_stop(void);

/** on receiving a GROUND_GPS message
 */
extern void follow_me_parse_ground_gps(uint8_t *buf);

// run function
extern int follow_me_call(void);

extern void compute_follow_distances(void);

extern void follow_me_disable_roll(void);
extern void follow_me_enable_roll(void);
extern void follow_me_disable_pitch(void);
extern void follow_me_enable_pitch(void);

extern void follow_me_throttle_loop(void);
extern void follow_me_roll_loop(void);
extern void follow_me_pitch_loop(void);

// Used by telemetry for reception of ground gps
//extern void follow_me_set_wp(void);

// Used by flight plan
void follow_me_soar_here(void);

struct FloatVect3 compute_state(void);
extern void follow_me_compute_wp(void);


#endif

