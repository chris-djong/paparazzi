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

extern float follow_me_distance;
extern uint8_t follow_me_height;
extern uint16_t follow_me_region;
extern float follow_me_heading;
extern float lateral_offset;

extern float airspeed_igain;
extern float airspeed_pgain;
extern float airspeed_dgain;

extern float roll_diff_igain;
extern float roll_diff_pgain;
extern float roll_diff_dgain;
extern float roll_enable;
extern float roll_disable;
extern uint8_t follow_me_roll;

/************************************************
  Variables used by internal file logger
*************************************************/

extern float ground_speed;
extern float actual_enu_speed;
extern struct FloatVect3 dist_wp_follow;
extern struct FloatVect3 wp_follow_enu;
extern int fix_mode;
extern struct UtmCoor_f ground_utm;




/************************************************
  Variables used by RL_MODULE
*************************************************/

//extern struct FloatVect3 dist_wp_follow_old;
//extern struct FloatVect3 wp_follow_utm;


//extern float dist_wp_follow_y_min;
//extern float dist_wp_follow_y_max;
// For settings



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

// Used by telemetry for reception of ground gps
extern void follow_me_set_wp(void);

// Used by flight plan
void follow_me_soar_here(void);


#endif

