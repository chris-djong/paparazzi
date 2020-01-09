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
int16_t follow_me_distance = 20; // distance from which the follow me points are created
int16_t follow_me_distance_2 = 200;
int16_t stdby_distance = 80; // based on stbdy radius + 10
int16_t follow_me_height = 30; // desired height above ground station
float follow_me_altitude;
uint16_t follow_me_region = 200;
float follow_me_heading = 0;
float average_airspeed_sp;  // average airspeed setpoint
int32_t x_follow;
int32_t y_follow;
int32_t x_follow2;
int32_t y_follow2;

// Roll PID
float roll_enable = 1; // when this x distance is exceeded the roll PID is enabled
float roll_disable = 0.2; // when the x distance is lower the roll PID is disabled again
float roll_diff_limit = 0.6; // maximum and minimum allowable change in desired_roll_angle compared to the desired value by the controller -> 0.2 is around 10 degree
float roll_diff_pgain = 0.006;
float roll_diff_igain = 0.0;
float roll_diff_dgain = 0.11;
float roll_diff_sum_err = 0.0;
uint8_t follow_me_roll = 0; // boolean variable used to overwrite h_ctl_roll_setpoint in stab_adaptive and stab_attitude

// Throttle PID
float airspeed_sum_err = 0.0;

// Energy control defines FW_V_CTL_ENERGY_H
#ifdef FW_V_CTL_ENERGY_H
float airspeed_pgain = 0.6;
float airspeed_igain = 0.02;
float airspeed_dgain = 0.01;
#else
// New control does not define FW_V_CTL_ENERY_H
float airspeed_pgain = 0.04;
float airspeed_igain = 0.003;
float airspeed_dgain = 1.4;
#endif


/*********************************
  Variables for follow_me module
*********************************/

float average_follow_me_distance;
float actual_enu_speed;
struct FloatVect3 dist_wp_follow; // distance to follow me wp
struct FloatVect3 dist_wp_follow2; // distance to follow 2 waypoint
int8_t lateral_offset = 0; // Amount in meters which the waypoint should be moved to the right with respect to the course itself

// Ground UTM variables used in order to calculate heading (they are only updated once heading calc counter is reached)
int counter; // counter which counts function executions
int heading_calc_counter = 5; // in case counter reaches heading_calc_counter the heading is calculated
struct UtmCoor_f ground_utm_old;
struct UtmCoor_f ground_utm_new;

// Old location for D gains
struct FloatVect3 dist_wp_follow_old; // old distance to follow me wp

// Variables initialised in functions themselves
static bool ground_set; // boolean to decide whether GPS message was received
static struct LlaCoor_i ground_lla; // lla coordinates received by the GPS message
static uint32_t ground_timestamp; // only executed set wp function if we received a newer timestamp
static uint32_t old_ground_timestamp;  // to compare it to the new timestamp
struct UtmCoor_f ground_utm;  // global because required for file logger and called by soar_here


/*********************************
  Average speed calculator
*********************************/

// Calculate the average speed in order to obtain a general prediction of the speed of the ground station
#define MAX_SPEED_SIZE 5
float all_speed[MAX_SPEED_SIZE]={V_CTL_AUTO_AIRSPEED_SETPOINT};
int8_t front_speed=-1,rear_speed=-1, count_speed=0;
float AverageAirspeed(float);
// Function definition
float AverageAirspeed(float speed)
{
	// This condition is required because otherwise the counter will reach 127 and continue counting from 0 again
	// Count = int8_t
	if (count_speed<MAX_SPEED_SIZE){
		count_speed += 1;
	}
    float Sum = 0;

    if(front_speed ==(rear_speed+1)%MAX_SPEED_SIZE)
    {
        if(front_speed==rear_speed)
            front_speed=rear_speed=-1;
        else
            front_speed = (front_speed+1)%MAX_SPEED_SIZE;
    }
    if(front_speed==-1)
        front_speed=rear_speed=0;
    else
        rear_speed=(rear_speed+1)%MAX_SPEED_SIZE;

    all_speed[rear_speed] = speed;


    for (int i=0; i<MAX_SPEED_SIZE; i++){
    	Sum = Sum + all_speed[i];
    }

    return ((float)Sum/fmin(MAX_SPEED_SIZE, count_speed));

}



/*********************************
  Average heading calculator
*********************************/

// Calculate the average gps heading in order to predict where the boat is going
// This has to be done by summing up the difference in x and difference in y in order to obtain a vector addition
// The use of vectors makes it possible to also calculate the average over for example 359, 0 and 1 degree
#define MAX_HEADING_SIZE 5
float all_diff_x[MAX_HEADING_SIZE]={0};
float all_diff_y[MAX_HEADING_SIZE]={0};
int8_t front_heading=-1,rear_heading=-1, count_heading=0;
float AverageHeading(float, float);
//function definition
float AverageHeading(float diffx, float diffy)
{
	// This condition is required because otherwise the counter will reach 127 and continue counting from 0 again
	// Count = int8_t
	if (count_heading<MAX_HEADING_SIZE){
		count_heading += 1;
	}
    float Sum_x = 0;
    float Sum_y = 0;



    if(front_heading ==(rear_heading+1)%MAX_HEADING_SIZE)
    {
        if(front_heading==rear_heading)
            front_heading=rear_heading=-1;
        else
            front_heading = (front_heading+1)%MAX_HEADING_SIZE;
    }
    if(front_heading==-1)
        front_heading=rear_heading=0;
    else
        rear_heading=(rear_heading+1)%MAX_HEADING_SIZE;

    all_diff_x[rear_heading] = diffx;
    all_diff_y[rear_heading] = diffy;


    for (int i=0; i<MAX_HEADING_SIZE; i++){
    	Sum_x = Sum_x + all_diff_x[i];
    	Sum_y = Sum_y + all_diff_y[i];
    }

    // Check for condition in which we are not moving
    // In case we are not moving return the heading that is not present
    if ((fabs(Sum_x) < 2) && (fabs(Sum_y) < 2)){
    	return follow_me_heading;
    } else {
		float heading = 0.0;
		// First check cases which divide by 0
		if (Sum_y == 0.0){
			if (Sum_x > 0.0){
				heading = 90.0;
			} else if (Sum_x < 0.0){
				heading = -90.0;
			}
		} else {
			heading = atan2(Sum_x, Sum_y)*180.0/M_PI;
		}
		return heading;
    }
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

//void follow_me_soar_here(void);
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
		transformation = rotate_frame(&transformation, -follow_me_heading*M_PI/180);

		// Bound transformation values
        BoundAbs(transformation.x, 32766); // 16 bit signed integer
        BoundAbs(transformation.y, 32766); // 16 bit signed integer

		follow_me_distance = transformation.y;
		lateral_offset = transformation.x;
	}
}


// Variables that are send through IVY
static void send_follow_me(struct transport_tx *trans, struct link_device *dev){
	pprz_msg_send_FOLLOW_ME(trans, dev, AC_ID, &dist_wp_follow.y, &dist_wp_follow.x, &v_ctl_auto_airspeed_setpoint);
}

// Called at compiling of module
void follow_me_init(void){
	ground_set = false;
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FOLLOW_ME, send_follow_me);
}

// Called each time the follow me block is started
void follow_me_startup(void){
#ifdef RL_SOARING_H
	if (!rl_started){
    }
#endif
	// follow_me_soar_here();
    // Set the default altitude of waypoints to the current height so that the drone keeps the height
    struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();
    // In case we have a ground reference set the follow me height, otherwise the follow_me_altitude
    if (ground_set){
    	follow_me_height = pos_Utm->alt - ((float)(ground_lla.alt))/1000.;
    }
    else {
    	follow_me_altitude = pos_Utm->alt;
    }
    // v_ctl_speed_mode = V_CTL_SPEED_AIRSPEED;
    if ((dist_wp_follow.y > roll_enable) || (dist_wp_follow.y < -roll_enable)){
    	follow_me_roll = 0;
    	printf("Follow me roll is disabled\n");
    } else {
    	follow_me_roll = 0;
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
		follow_me_heading = AverageHeading(diff_x, diff_y);
		ground_utm_old = ground_utm_new;
    }
}

void follow_me_compute_wp(void);


// Function that is executed each time the GROUND_GPS message is received
void follow_me_parse_ground_gps(uint8_t *buf){
	if(DL_GROUND_GPS_ac_id(buf) != AC_ID)
		return;

	// Save the received values
	ground_lla.lat = DL_GROUND_GPS_lat(buf);
	ground_lla.lon = DL_GROUND_GPS_lon(buf);
	ground_lla.alt = DL_GROUND_GPS_alt(buf);
	// ground_speed = DL_GROUND_GPS_speed(buf);
	// ground_climb = DL_GROUND_GPS_climb(buf);
	// ground_course = DL_GROUND_GPS_course(buf);
	old_ground_timestamp = ground_timestamp;
	ground_timestamp = DL_GROUND_GPS_timestamp(buf);
	// fix_mode = DL_GROUND_GPS_mode(buf);

	// Only set the new location if the new timestamp is later (otherwise probably due to package loss in between)
	if (ground_timestamp > old_ground_timestamp){
		follow_me_compute_wp();
		ground_set = true;
	}

	// Set heading here so that it can be calculated already during stdby or Manual execution
	follow_me_set_heading();
}


// Roll angle controller
void follow_me_roll_pid(void);
void follow_me_roll_pid(void){
	// Roll rate controller
	// We either have the normal course mode or the nav follow mode.
	// If we have been in course and exceed the enable limits then nav follow is activated
	// If we have been in follow and exceed the disable limits then nav course is activated
	if (( fabs(dist_wp_follow.x) > roll_enable && fabs(dist_wp_follow_old.x) <= roll_enable)  ){
		follow_me_roll = 0;
		printf("Follow me roll is disabled\n");
	} else if ((fabs(dist_wp_follow.x) <= roll_disable && dist_wp_follow_old.x > roll_disable)) {
		follow_me_roll = 0;
	}
	// This condition is required in case the relative wind is slower than the stall speed of the UAV
	if (fabs(dist_wp_follow.y) > 2*follow_me_distance){
		follow_me_roll = 0;
	}
	roll_diff_sum_err += dist_wp_follow.x;
	BoundAbs(roll_diff_sum_err, 20);

	h_ctl_roll_setpoint_follow_me = +roll_diff_pgain*dist_wp_follow.x + roll_diff_igain*roll_diff_sum_err + (dist_wp_follow.x-dist_wp_follow_old.x)*roll_diff_dgain;

	// Bound roll diff by limits
	if (h_ctl_roll_setpoint_follow_me > roll_diff_limit){
		h_ctl_roll_setpoint_follow_me = roll_diff_limit;
	}
	else if (h_ctl_roll_setpoint_follow_me < -roll_diff_limit){
		h_ctl_roll_setpoint_follow_me = -roll_diff_limit;
	}
}

// Throttle controller
void follow_me_throttle_pid(void);
void follow_me_throttle_pid(void){
	// Airspeed controller
	airspeed_sum_err += dist_wp_follow.y;
	BoundAbs(airspeed_sum_err, 20);

	float airspeed_inc = +airspeed_pgain*dist_wp_follow.y + airspeed_igain*airspeed_sum_err + (dist_wp_follow.y-dist_wp_follow_old.y)*airspeed_dgain;

	// Add airspeed inc to average airspeed
	v_ctl_auto_airspeed_setpoint = AverageAirspeed(v_ctl_auto_airspeed_setpoint + airspeed_inc);

	if (v_ctl_auto_airspeed_setpoint < 0){
		v_ctl_auto_airspeed_setpoint = 0;
	}
}

// Function that computes distance of UAV toward a certain UTM position
struct FloatVect3 compute_dist_to_utm(float, float, float);
struct FloatVect3 compute_dist_to_utm(float utm_x, float utm_y, float utm_z){
	struct FloatVect3 wp_follow_utm;
	wp_follow_utm.x = utm_x;
	wp_follow_utm.y = utm_y;
	wp_follow_utm.z = utm_z;

	struct FloatVect3 distance = UTM_to_ENU(&wp_follow_utm);

	return distance;
}



// Sets all the waypoints based on GPS coordinates received by ground segment
void follow_me_compute_wp(void){
	// Obtain lat lon coordinates for conversion
	struct LlaCoor_f lla;
	lla.lat = RadOfDeg((float)(ground_lla.lat / 1e7));
	lla.lon = RadOfDeg((float)(ground_lla.lon / 1e7));
	lla.alt = ((float)(ground_lla.alt))/1000.;
	follow_me_altitude = lla.alt + follow_me_height;
	// Convert LLA to UTM in oder to set watpoint in UTM system
	ground_utm.zone = nav_utm_zone0;
	utm_of_lla_f(&ground_utm, &lla);
	ground_utm_new = ground_utm;

	// Follow waypoint
	x_follow = ground_utm.east + follow_me_distance*sinf(follow_me_heading/180.*M_PI) + lateral_offset*cosf(-follow_me_heading/180.*M_PI);
	y_follow = ground_utm.north + follow_me_distance*cosf(follow_me_heading/180.*M_PI) + lateral_offset*sinf(-follow_me_heading/180.*M_PI);

	// Follow 2 waypoint
	x_follow2 = ground_utm.east  + follow_me_distance_2*sinf(follow_me_heading/180.*M_PI) + lateral_offset*cosf(-follow_me_heading/180.*M_PI);
	y_follow2 = ground_utm.north + follow_me_distance_2*cosf(follow_me_heading/180.*M_PI) + lateral_offset*sinf(-follow_me_heading/180.*M_PI);

	// Move stdby waypoint in front of the boat at the given distance
	int32_t x_stdby = ground_utm.east + stdby_distance*sinf(follow_me_heading/180.*M_PI);
	int32_t y_stdby = ground_utm.north + stdby_distance*cosf(follow_me_heading/180.*M_PI);

	// Update STBDY HOME AND FOLLOW ME WPS
	nav_move_waypoint(WP_FOLLOW, x_follow,  y_follow, follow_me_altitude );
	nav_move_waypoint(WP_FOLLOW2, x_follow2, y_follow2, follow_me_altitude);
	nav_move_waypoint(WP_STDBY, x_stdby, y_stdby, follow_me_altitude + 20); // Set STBDY and HOME waypoint so that they are above the boat
	nav_move_waypoint(WP_HOME, ground_utm.east, ground_utm.north, follow_me_altitude + 20);

	// Update allowable Flying Region
	nav_move_waypoint(WP_FR_TL, ground_utm.east - follow_me_region, ground_utm.north + follow_me_region, follow_me_altitude + 20);
	nav_move_waypoint(WP_FR_TR, ground_utm.east + follow_me_region, ground_utm.north + follow_me_region, follow_me_altitude + 20);
	nav_move_waypoint(WP_FR_BL, ground_utm.east - follow_me_region, ground_utm.north - follow_me_region, follow_me_altitude + 20);
	nav_move_waypoint(WP_FR_BR, ground_utm.east + follow_me_region, ground_utm.north - follow_me_region, follow_me_altitude + 20);

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
	return;
}


// Sets the general heading in the direction of FOLLOW2 (waypoint located far in front of the boat)
void follow_me_go(void);
void follow_me_go(void){
	NavGotoWaypoint(WP_FOLLOW2);
    NavVerticalAltitudeMode(follow_me_altitude, 0.);
}


// This is the main function executed by the follow_me_block
int follow_me_call(void){
	// Compute errors towards waypoint
	// Calculate distance in main function as follow_me_compute_wp is not executed if GROUND_GPS message is not received
	dist_wp_follow_old = dist_wp_follow;
	dist_wp_follow = compute_dist_to_utm(x_follow, y_follow, follow_me_height);
    dist_wp_follow2 = compute_dist_to_utm(x_follow2, y_follow2, follow_me_height);


    // Loop through controller
    // In case we have reached follow 2, simply use the nav fly_to_xy function to hover above FOLLOW2 with minimum airspeed and no roll
	if (fabs(dist_wp_follow2.x) > 30 && fabs(dist_wp_follow2.y) > 30){
		follow_me_throttle_pid();
		follow_me_roll_pid();
	} else {
		v_ctl_auto_airspeed_setpoint = 10;
		follow_me_roll = 0;
	}
    // Move to the correct location
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
	v_ctl_auto_airspeed_setpoint = V_CTL_AUTO_AIRSPEED_SETPOINT;
	follow_me_roll = 0;
	h_ctl_roll_setpoint_follow_me = 0;
	// v_ctl_speed_mode = V_CTL_SPEED_THROTTLE;
}

