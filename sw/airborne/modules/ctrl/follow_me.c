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
 * @file "modules/ctrl/follow_me.c"
 * @author rotorcraft Freek van Tienen <freek.v.tienen@gmail.com>
 * @author fixedwing Chris de Jong <djchris261@hotmail.com>
 * Control a FIXEDWING to follow at a defined distance from the target
 *
*/ 

#include <stdio.h>

#include "follow_me.h"
#include "state.h"

#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/navigation/common_nav.h"
#include "generated/modules.h"
#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/guidance/guidance_v_n.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "generated/flight_plan.h" // for waypoint reference pointers
#include <Ivy/ivy.h> // for go to block
#include "subsystems/datalink/telemetry.h"

#ifdef RL_SOARING_H
#include "modules/rl_soaring/rl_soaring.h"
#define HAND_RL_SIZE 20 // the amount of average_follow_me_distance that need to be below the threshold in order to hand control over to RL
int8_t hand_rl[HAND_RL_SIZE] = {0};
float hand_rl_threshold = 1;
int8_t hand_rl_idx = 0; // the index value that needs to be modified
#endif

/*********************************
  Parameters for follow_me module
*********************************/

// Waypoint parameters
float follow_me_distance = 20; // distance from which the follow me points are created
uint8_t stdby_distance = 40; // based on nav_radius + 10
uint8_t follow_me_height = 10;
uint16_t follow_me_region = 200;
float follow_me_heading = 0;
uint8_t dist_follow2 = 250;

// Roll PID
float roll_enable = 2; // when this x distance is exceeded the roll PID is enabled
float roll_disable = 0.5; // when the x distance is lower the roll PID is disabled again
float roll_diff_limit = 0.2; // maximum and minimum allowable change in desired_roll_angle compared to the desired value by the controller -> 0.2 is around 10 degree
float roll_diff_pgain = 0.006;
float roll_diff_igain = 0.0;
float roll_diff_dgain = 0.11;
float roll_diff_sum_err = 0.0;

// Throttle PID
float ground_speed_diff_sum_err = 0.0;
float ground_speed_diff_limit = 1.5; // maximum and minimum allowable change in ground speed compared to desired value from gps

// Energy control defines FW_V_CTL_ENERGY_H
#ifdef FW_V_CTL_ENERGY_H
float ground_speed_diff_pgain = 0.6;
float ground_speed_diff_igain = 0.02;
float ground_speed_diff_dgain = 0.01;
#else
// New control does not define FW_V_CTL_ENERY_H
float ground_speed_diff_pgain = 0.5;
float ground_speed_diff_igain = 0.1;
float ground_speed_diff_dgain = 0.015;
#endif


/*********************************
  Variables for follow_me module
*********************************/

float average_follow_me_distance;
float actual_enu_speed;
struct FloatVect3 dist_wp_follow; // distance to follow me wp
float ground_speed_diff = 0; // difference that is added to the desired ground speed
float lateral_offset = 0; // Amount in meters which the waypoint should be moved to the right with respect to the course itself

// Ground UTM variables used in order to calculate heading (they are only updated once heading calc counter is reached)
int counter; // counter which counts function executions
int heading_calc_counter = 15; // in case counter reaches heading_calc_counter the heading is calculated
struct UtmCoor_f ground_utm_old;
struct UtmCoor_f ground_utm_new;

// Old location for D gains
struct FloatVect3 dist_wp_follow_old; // old distance to follow me wp

// Variables initialised in functions themselves
static bool ground_set; // boolean to decide whether GPS message was received
static struct LlaCoor_i ground_lla; // lla coordinates received by the GPS message
float ground_speed; // ground speed received by the GPS message
static float ground_timestamp; // only executed set wp function if we received a newer timestamp
static float old_ground_timestamp;  // to compare it to the new timestamp
int fix_mode;  // GPS mode use for logging
struct FloatVect3 wp_follow_enu; // global because file logger needs access
struct UtmCoor_f ground_utm;  // global because required for file logger and called by soar_here





/*********************************
  Average heading calculator
*********************************/

// Calculate the average gps heading in order to predict where the boat is going
#define MAX_HEADING_SIZE 15
float average_heading[MAX_HEADING_SIZE]={0};
int8_t front_heading=-1,rear_heading=-1, count_heading=0;
float AverageHeading(float);
//function definition
float AverageHeading(float item)
{
	// This condition is required because otherwise the counter will reach 127 and continue counting from 0 again
	// Count = int8_t
	if (count_heading<MAX_HEADING_SIZE){
		count_heading += 1;
	}
    static float Sum=0;
    if(front_heading ==(rear_heading+1)%MAX_HEADING_SIZE)
    {
        if(front_heading==rear_heading)
            front_heading=rear_heading=-1;
        else
            front_heading = (front_heading+1)%MAX_HEADING_SIZE;
        Sum=Sum-average_heading[front_heading];
    }
    if(front_heading==-1)
        front_heading=rear_heading=0;
    else
        rear_heading=(rear_heading+1)%MAX_HEADING_SIZE;
    average_heading[rear_heading]=item;
    Sum=Sum+average_heading[rear_heading];
    // The following normalization is required because we allow inputs between -360 and 360 in the heading calculations
    float average = (float)Sum/fmin(MAX_HEADING_SIZE, count_heading);
    printf("Average is given by %f\n", average);
    if (average > 180){
    	average -= 360;
    } else if (average < -180){
    	average += 360;
    }
    return average;
}



/*********************************
  Average Distance calculator  // used by RL module
*********************************/

#ifdef RL_SOARING_H
#define MAX_DISTANCE_SIZE 30
float average_distance[MAX_DISTANCE_SIZE]={0};
int8_t front_distance=-1,rear_distance=-1, count_distance=0;
float AverageDistance(int8_t);
//function definition
float AverageDistance(int8_t item)
{
	// This condition is required because otherwise the counter will reach 127 and continue counting from 0 again
	// Count = int8_t
	if (count_distance<MAX_DISTANCE_SIZE){
		count_distance += 1;
	}
    static float Sum=0;
    if(front_distance ==(rear_distance+1)%MAX_DISTANCE_SIZE)
    {
        if(front_distance==rear_distance)
            front_distance=rear_distance=-1;
        else
            front_distance = (front_distance+1)%MAX_DISTANCE_SIZE;
        Sum=Sum-average_distance[front_distance];
    }
    if(front_distance==-1)
        front_distance=rear_distance=0;
    else
        rear_distance=(rear_distance+1)%MAX_DISTANCE_SIZE;
    average_distance[rear_distance]=item;
    Sum=Sum+average_distance[rear_distance];
    return ((float)Sum/fmin(MAX_DISTANCE_SIZE, count_distance));
}
#endif

/***********************************************************************************************************************
  Frame translation functions
***********************************************************************************************************************/

/*Translate frame to a new point
 * The frame is moved from its origin to the new point (transx, transy, transz)*/
struct FloatVect3 translate_frame(struct FloatVect3 *point, int trans_x, int trans_y, int trans_z);
struct FloatVect3 translate_frame(struct FloatVect3 *point, int trans_x, int trans_y, int trans_z){
	// Create return vectore for function
	struct FloatVect3 transformation;

	// Move frame
	transformation.x = point->x - trans_x;
	transformation.y = point->y - trans_y;
	transformation.z = point->z - trans_z;

	// Return
	return transformation;
}

/*Rotate a point in a frame by an angle theta (clockwise positive)*/
struct FloatVect3 rotate_frame(struct FloatVect3 *point, float theta);
struct FloatVect3 rotate_frame(struct FloatVect3 *point, float theta){
	// Create return Vector for function
	struct FloatVect3 transformation;

    // Rotate point
	transformation.x = cosf(theta)*point->x + sinf(theta)*point->y;
	transformation.y = -sinf(theta)*point->x + cosf(theta)*point->y;
	transformation.z = point->z;

	// Return
	return transformation;
}

/*Transformation from UTM coordinate system to the Body system
 * Pos UTM is used as input so that during a whole function execution it stays constant for all the transforms
 * Same for heading */
struct FloatVect3 UTM_to_ENU(struct FloatVect3 *point);
struct FloatVect3 UTM_to_ENU(struct FloatVect3 *point){
	// Create return vector for the function
	struct FloatVect3 transformation;

	// Obtain current UTM position
	struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();

    // Translate frame
	transformation = translate_frame(point, pos_Utm->east, pos_Utm->north, pos_Utm->alt);

	// Then rotate frame
	float heading = stateGetNedToBodyEulers_f()->psi;

	transformation = rotate_frame(&transformation, heading);
	// Return
	return transformation;
}


/*Transformation from the Body system to the UTM coordinate system */
struct FloatVect3 ENU_to_UTM(struct FloatVect3 *point);
struct FloatVect3 ENU_to_UTM(struct FloatVect3 *point){
	// Create return vector for the function
	struct FloatVect3 transformation;

	// Obtain current Utm position for translationg
	struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();

	float heading = stateGetNedToBodyEulers_f()->psi;

    // Rotate frame back
	transformation = rotate_frame(point, -heading);

	// Translate frame back
	transformation = translate_frame(&transformation, -pos_Utm->east, -pos_Utm->north, -pos_Utm->alt);
	//Return
	return transformation;
}


/***********************************************************************************************************************
  Reinforcement Learning functions
***********************************************************************************************************************/

#ifdef RL_SOARING_H
// Function to check whether we can hand control over to reinforcement learning again
int8_t check_handover_rl(void);
int8_t check_handover_rl(void){
	for (int i=0; i<HAND_RL_SIZE; i++){
		// In case we have only one incorrect value return false
		if (hand_rl[i] == 0){
			return 0;
		}
	}
	// In case we loop through the whole array return true
	return 1;
}
#endif

/***********************************************************************************************************************
  Follow me functions
***********************************************************************************************************************/


void follow_me_soar_here(void);
void follow_me_soar_here(void){
	// This condition is required because sometimes the ground_utm variable has not been updated yet in case the GROUND_GPS messages was not received yet
	if ((ground_utm.east != 0) && (ground_utm.north != 0)){
		// Based on the current Utm position and follow_me_heading we have to change the reference frame
		// such that it`s y axis is orthogonal to the heading and the origin is at the boat location
		// Create return vector for the function
		struct FloatVect3 transformation;

		// Obtain current UTM position
		struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();
		struct FloatVect3 point;
		point.x = pos_Utm->east;
		point.y = pos_Utm->north;
		point.z = pos_Utm->alt;

		// Translate frame
		transformation = translate_frame(&point, ground_utm.east, ground_utm.north, ground_utm.alt);

		// Then rotate frame
		transformation = rotate_frame(&transformation, follow_me_heading*M_PI/180);

		follow_me_distance = transformation.y;
		lateral_offset = transformation.x;

	}
}


// Variables that are send through IVY
static void send_follow_me(struct transport_tx *trans, struct link_device *dev){
	pprz_msg_send_FOLLOW_ME(trans, dev, AC_ID, &dist_wp_follow.y, &dist_wp_follow.x, &v_ctl_auto_groundspeed_setpoint);
}

// Called at compiling of module
void follow_me_init(void){
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FOLLOW_ME, send_follow_me);
}

// Called each time the follow me block is started
void follow_me_startup(void){
#ifdef RL_SOARING_H
	if (!rl_started){

    }
#endif
	// follow_me_soar_here();
    follow_me_set_wp();
    // Set the default altitude of waypoints to the current height so that the drone keeps the height
    struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();
    follow_me_height = pos_Utm->alt;
    ground_set = false;
    v_ctl_speed_mode = V_CTL_SPEED_GROUNDSPEED;
    if ((dist_wp_follow.y > roll_enable) || (dist_wp_follow.y < -roll_enable)){
    	nav_mode = NAV_MODE_FOLLOW;
    	lateral_mode = LATERAL_MODE_FOLLOW;
    } else {
    	nav_mode = NAV_MODE_COURSE;
    	lateral_mode = LATERAL_MODE_COURSE;
    }
}

// Sets the heading based on the average over several GPS positions
void follow_me_set_heading(void);
void follow_me_set_heading(void){
	// Obtain follow me heading based on position
    counter++;
    if (counter == heading_calc_counter){
    	counter = 0;
		float diff_y = ground_utm_new.north - ground_utm_old.north;
		float diff_x = ground_utm_new.east - ground_utm_old.east;
		// First check conditions in which we divide by 0
		// Note atan2 gives results between -180 and 180
		if (diff_y == 0){
			if (diff_x > 0){
				follow_me_heading = AverageHeading(90);
			}
			else if (diff_x < 0){
				follow_me_heading = AverageHeading(-90);
			}
			else if (diff_x == 0){
			}
		} else {
			// Atan2 gives a value between -180 and 180, this induces problems with the average calculation in case the result
			// Goes from 150 to -150 for example. This can be counteracted by allowing inputs from -360 to 360 and then after the
			// average calculation normalizing it to the required values
			printf("\n\nSetting heading to new value %f\n", heading )
			float heading = atan2(diff_x, diff_y)*180/M_PI;
			if (follow_me_heading - heading > 180){
				printf("Adding 360 because follow me is %f and new is %f\n", follow_me_heading, heading);
				heading += 360;
			} else if (follow_me_heading - heading < -180){
				printf("Removing 360 because follow me is %f and new is %f\n", follow_me_heading, heading);
				heading -= 360;
			}
			follow_me_heading = AverageHeading(heading);
		}
		ground_utm_old = ground_utm_new;
    }
}

// Function that is executed each time the GROUND_GPS message is received
void follow_me_parse_ground_gps(uint8_t *buf){

	if(DL_GROUND_GPS_ac_id(buf) != AC_ID)
		return;

	// Save the received values
	ground_lla.lat = DL_GROUND_GPS_lat(buf);
	ground_lla.lon = DL_GROUND_GPS_lon(buf);
	ground_lla.alt = DL_GROUND_GPS_alt(buf);
	ground_speed = DL_GROUND_GPS_speed(buf);
	// ground_climb = DL_GROUND_GPS_climb(buf);
	// ground_course = DL_GROUND_GPS_course(buf);
	old_ground_timestamp = ground_timestamp;
	ground_timestamp = DL_GROUND_GPS_timestamp(buf);
	fix_mode = DL_GROUND_GPS_mode(buf);
	ground_set = true;

	// Only set the new location if the new timestamp is later (otherwise probably due to package loss in between)
	if (ground_timestamp > old_ground_timestamp){
		follow_me_set_wp();
	}

	// Set heading here so that it can be calculated already during stdby or Manual execution
	follow_me_set_heading();
}


// Manage the throttle so that the groundspeed of both the boat and the uav are equivalent
void follow_me_set_groundspeed(void);
void follow_me_set_groundspeed(void){
	v_ctl_auto_groundspeed_setpoint = ground_speed + ground_speed_diff;
	if (v_ctl_auto_groundspeed_setpoint < 0){
		v_ctl_auto_groundspeed_setpoint = 0;
	}
}

// Roll angle controller
void follow_me_roll_pid(void);
void follow_me_roll_pid(void){
	// Roll rate controller
	// We either have the normal course mode or the nav follow mode.
	// If we have been in course and exceed the enable limits then nav follow is activated
	// If we have been in follow and exceed the disable limits then nav course is activated
	if (((dist_wp_follow.x > roll_enable && dist_wp_follow_old.x <= roll_enable)  || (dist_wp_follow.x < -roll_enable && dist_wp_follow_old.x >= -roll_enable))){
		nav_mode = NAV_MODE_FOLLOW;
		lateral_mode = LATERAL_MODE_FOLLOW;
	} else if (((dist_wp_follow.x <= roll_disable && dist_wp_follow_old.x > roll_disable) || (dist_wp_follow.x >= -roll_disable && dist_wp_follow_old.x < - roll_disable))) {
		nav_mode = NAV_MODE_COURSE;
		lateral_mode = LATERAL_MODE_COURSE;
	}
	// This condition is required in case the relative wind is slower than the stall speed of the UAV
	if (dist_wp_follow.y > 2*follow_me_distance){
		nav_mode = NAV_MODE_COURSE;
	    lateral_mode = LATERAL_MODE_COURSE;
	}
	roll_diff_sum_err += dist_wp_follow.x;
	BoundAbs(roll_diff_sum_err, 20);


	h_ctl_roll_setpoint_follow_me = +roll_diff_pgain*dist_wp_follow.x + roll_diff_igain*roll_diff_sum_err + (dist_wp_follow.x-dist_wp_follow_old.x)*roll_diff_dgain;
	// Bound groundspeed diff by limits
	if (h_ctl_roll_setpoint_follow_me > roll_diff_limit){
		h_ctl_roll_setpoint_follow_me = roll_diff_limit;
	}
	else if (h_ctl_roll_setpoint_follow_me < -roll_diff_limit){
		h_ctl_roll_setpoint_follow_me = -roll_diff_limit;
	}
}

// Throttle controller
//void follow_me_throttle_pid(void);
void follow_me_throttle_pid(void){
	// Ground speed controller
	ground_speed_diff_sum_err += dist_wp_follow.y;
	BoundAbs(ground_speed_diff_sum_err, 20);
	ground_speed_diff = +ground_speed_diff_pgain*dist_wp_follow.y + ground_speed_diff_igain*ground_speed_diff_sum_err + (dist_wp_follow.y-dist_wp_follow_old.y)*ground_speed_diff_dgain;
	// Bound groundspeed diff by limits
	if (ground_speed_diff > ground_speed_diff_limit){
		ground_speed_diff = ground_speed_diff_limit;
	}
	else if (ground_speed_diff < -ground_speed_diff_limit){
		ground_speed_diff = -ground_speed_diff_limit;
	}
}

// Sets all the waypoints based on GPS coordinates received by ground segment
void follow_me_set_wp(void){
	if(ground_set) {

		// Obtain lat lon coordinates for conversion
		struct LlaCoor_f lla;
		lla.lat = RadOfDeg((float)(ground_lla.lat / 1e7));
		lla.lon = RadOfDeg((float)(ground_lla.lon / 1e7));
		lla.alt = ((float)(ground_lla.alt))/1000.;

		// Convert LLA to UTM in oder to set watpoint in UTM system
		ground_utm.zone = nav_utm_zone0;
		utm_of_lla_f(&ground_utm, &lla);
        ground_utm_new = ground_utm;

		// Follow waypoint
		int32_t x_follow = ground_utm.east + follow_me_distance*sinf(follow_me_heading/180.*M_PI) + lateral_offset*cosf(-follow_me_heading/180.*M_PI);
		int32_t y_follow = ground_utm.north + follow_me_distance*cosf(follow_me_heading/180.*M_PI) + lateral_offset*sinf(-follow_me_heading/180.*M_PI);

		struct FloatVect3 wp_follow_utm;
		wp_follow_utm.x = x_follow;
		wp_follow_utm.y = y_follow;
		wp_follow_utm.z = follow_me_height;

		wp_follow_enu = UTM_to_ENU(&wp_follow_utm);

		// Dist wp follows using ENU system
		dist_wp_follow_old = dist_wp_follow;
		dist_wp_follow.x = wp_follow_enu.x;
		dist_wp_follow.y = wp_follow_enu.y;
		dist_wp_follow.z = wp_follow_enu.z;

		// Follow 2 waypoint
		int32_t x_follow2 = ground_utm.east  + dist_follow2*sinf(follow_me_heading/180.*M_PI);
		int32_t y_follow2 = ground_utm.north + dist_follow2*cosf(follow_me_heading/180.*M_PI);

		// Move stdby waypoint in front of the boat at the given distance
		int32_t x_stdby = ground_utm.east + stdby_distance*sinf(follow_me_heading/180.*M_PI);
		int32_t y_stdby = ground_utm.north + stdby_distance*cosf(follow_me_heading/180.*M_PI);

        // Update STBDY HOME AND FOLLOW ME WPS
		nav_move_waypoint(WP_FOLLOW, x_follow,  y_follow, follow_me_height);
		nav_move_waypoint(WP_FOLLOW2, x_follow2, y_follow2, follow_me_height);
		nav_move_waypoint(WP_STDBY, x_stdby, y_stdby, follow_me_height + 20); // Set STBDY and HOME waypoint so that they are above the boat
		nav_move_waypoint(WP_HOME, ground_utm.east, ground_utm.north, follow_me_height + 20);

		// Update allowable Flying Region
		nav_move_waypoint(WP_FR_TL, ground_utm.east - follow_me_region, ground_utm.north + follow_me_region, follow_me_height);
		nav_move_waypoint(WP_FR_TR, ground_utm.east + follow_me_region, ground_utm.north + follow_me_region, follow_me_height);
		nav_move_waypoint(WP_FR_BL, ground_utm.east - follow_me_region, ground_utm.north - follow_me_region, follow_me_height);
		nav_move_waypoint(WP_FR_BR, ground_utm.east + follow_me_region, ground_utm.north - follow_me_region, follow_me_height);

		// Reset the ground boolean
	    ground_set = false;

#ifdef RL_SOARING_H
	    if (rl_started){
			average_follow_me_distance = AverageDistance(dist_wp_follow.y);
			if ((average_follow_me_distance < hand_rl_threshold) && (average_follow_me_distance > -hand_rl_threshold) && rl_started){
				hand_rl[hand_rl_idx] = 1;
			} else {
				hand_rl[hand_rl_idx] = 0;
			}
			hand_rl_idx++;
			if (hand_rl_idx>HAND_RL_SIZE-1){
				hand_rl_idx = 0;
			}
	    }
#endif
	}
	return;
}


// Sets the general heading in the direction of FOLLOW2 (waypoint located far in front of the boat)
void follow_me_go(void);
void follow_me_go(void){
	NavGotoWaypoint(WP_FOLLOW2);
    NavVerticalAltitudeMode(follow_me_height, 0.);
}


// This is the main function executed by the follow_me_block
int follow_me_call(void){
    // Loop through controller
	follow_me_roll_pid();
	follow_me_throttle_pid();

    // Set heading and groundspeed and move to the correct location
	follow_me_set_groundspeed();
	follow_me_go();


#ifdef RL_SOARING_H
	if (check_handover_rl()){
		GotoBlock(7);
	}
#endif

	return 1;
}

// This function should be executed at the start of each other block so that it is executed whenever the flying region is left or a new block is called
void follow_me_stop(void){
	ground_speed_diff = 0;
	v_ctl_auto_groundspeed_setpoint = 100; // set to 100 in order to ensure the the groundspeed loop is not executed anymore in energy control
	h_ctl_roll_setpoint_follow_me = 0;
	nav_mode = NAV_MODE_COURSE;
	lateral_mode = LATERAL_MODE_COURSE;
	v_ctl_speed_mode = V_CTL_SPEED_THROTTLE;
}
