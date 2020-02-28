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
// Follow me distance +30 because we want to fly just in front of the UAV. In case we fly closer by we follow flower like patterns
int16_t follow_me_distance_2 = 20 + 30; //  this is where the uav will fly to
int16_t stdby_distance = 110; // based on stbdy radius (80) + 30
int16_t follow_me_height = 30; // desired height above ground station
float follow_me_altitude;
uint16_t follow_me_region = 200;
float follow_me_heading = 0;
int32_t x_follow;
int32_t y_follow;
int32_t x_follow2;
int32_t y_follow2;

// Roll PID
float roll_enable = 3; // when this x distance is exceeded the roll PID is enabled
float roll_disable = 1; // when the x distance is lower the roll PID is disabled again
float roll_diff_limit = 0.6; // maximum and minimum allowable change in desired_roll_angle compared to the desired value by the controller -> 0.2 is around 10 degree
float roll_diff_pgain = 0.0015;
float roll_diff_igain = 0.0;
float roll_diff_dgain = 0.0;
float roll_diff_sum_err = 0.0;
uint8_t follow_me_roll = 0; // boolean variable used to overwrite h_ctl_roll_setpoint in stab_adaptive and stab_attitude

// Throttle PID
float airspeed_sum_err = 0.0;

// Should be defined positive
float airspeed_pgain = 0.4;
float airspeed_igain = 0.00;
float airspeed_dgain = 0.00;


/*********************************
  Variables for follow_me module
*********************************/

float average_follow_me_distance;
float actual_enu_speed;
struct FloatVect3 dist_wp_follow; // distance to follow me wp
struct FloatVect3 dist_wp_follow2; // distance to follow 2 waypoint
int8_t lateral_offset = 0; // Amount in meters which the waypoint should be moved to the right with respect to the course itself

// Ground UTM variables used in order to calculate heading (they are only updated once heading calc counter is reached)
int counter_heading = 0; // counter which counts heading function executions
int heading_calc_counter = 10; // in case counter reaches heading_calc_counter the heading is calculated
struct UtmCoor_f ground_utm_old;
struct UtmCoor_f ground_utm_new;

// Counter for the case gps is lost
uint8_t counter_gps = 0;
uint8_t gps_lost_count = 100;

// Counter for the calculation of the old dist_wp_follow
uint8_t counter_old_distance = 0;
uint8_t old_distance_count = 20;

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


uint8_t average_speed_size = 10;
float AverageAirspeed(float);
// Calculates the average speed based on the previous average speed
// It should be noted that this function is incorrect in case we have not enough measurements (at least average_speed_size measurements are required)
float AverageAirspeed(float speed){
    return (v_ctl_auto_airspeed_setpoint * (average_speed_size-1) + speed) / (average_speed_size);
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
    if ((fabs(Sum_x) < 4) && (fabs(Sum_y) < 4)){
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

/*Rotate a point in a frame by an angle theta (clockwise positive) in radians*/
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
	float heading = follow_me_heading/180*M_PI;

	transformation = rotate_frame(&transformation, -heading);

	// Return
	return transformation;
}



/*Transformation from the Body system to the UTM coordinate system */
/*struct FloatVect3 ENU_to_UTM(struct FloatVect3 *point);
struct FloatVect3 ENU_to_UTM(struct FloatVect3 *point){
	// Create return vector for the function
	struct FloatVect3 transformation;

	// Obtain current Utm position for translationg
	struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();

	float heading = stateGetNedToBodyEulers_f()->psi;

    heading = follow_me_heading/180*M_PI;

    // Rotate frame back
	transformation = rotate_frame(point, -heading);

	// Translate frame back
	transformation = translate_frame(&transformation, -pos_Utm->east, -pos_Utm->north, -pos_Utm->alt);
	//Return
	return transformation;
}
*/


/***********************************************************************************************************************
  Reinforcement Learning functions
***********************************************************************************************************************/



/***********************************************************************************************************************
  Follow me functions
***********************************************************************************************************************/


// Compute both lateral offset and follow_me_distance
// Also used by RL algorithm
struct FloatVect3 compute_state(void);
struct FloatVect3 compute_state(void){
	// Obtain current position in order to calculate state as function of boat difference
	struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();


	// Based on the current Utm position and follow_me_heading we have to change the reference frame
	// such that it`s y axis is orthogonal to the heading of the boat and the origin is at the boat location
	// Create return vector for the function
	struct FloatVect3 transformation;

	// Create a point for the conversion
	struct FloatVect3 point;
	point.x = pos_Utm->east;
	point.y = pos_Utm->north;
	point.z = pos_Utm->alt;

	// Translate frame
	transformation = translate_frame(&point, ground_utm.east, ground_utm.north, ground_utm.alt);

	// Set heading first so that the transformations are correct
	float heading = follow_me_heading/180*M_PI;
	// Then rotate frame
	transformation = rotate_frame(&transformation, -heading);
	// Bound transformation values
	BoundAbs(transformation.x, 32766); // 16 bit signed integer
	BoundAbs(transformation.y, 32766); // 16 bit signed integer

	return transformation;
}


//void follow_me_soar_here(void);
// Sets all follow me parameters like the current
void follow_me_soar_here(void){
	// This condition is required because sometimes the ground_utm variable has not been updated yet in case the GROUND_GPS messages was not received yet
	if ((ground_utm.east != 0) && (ground_utm.north != 0)){
		// Obtain the current position to calculate waypoint positions
		struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();

		// Set the follow_me_heading to the current heading of the UAV
		follow_me_heading = stateGetNedToBodyEulers_f()->psi*180/M_PI;

		// In case we have a ground reference set the follow me height, otherwise the follow_me_altitude
		if (ground_set){
			follow_me_height = pos_Utm->alt - ((float)(ground_lla.alt))/1000.;
		}
		else {
			follow_me_altitude = pos_Utm->alt;
		}

        // Set Airspeed setpoint and the whole average array to current airspeed
        float current_airspeed = stateGetAirspeed_f();
        v_ctl_auto_airspeed_setpoint = current_airspeed;

		struct FloatVect3 state_in_boat_frame = compute_state();
		follow_me_distance = state_in_boat_frame.y;
		follow_me_distance_2 = follow_me_distance + 30;
		lateral_offset = state_in_boat_frame.x;

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
    follow_me_call();

    if ((dist_wp_follow.x > roll_enable) || (dist_wp_follow.x < -roll_enable)){
    	follow_me_roll = 1;
    } else {
     	follow_me_roll = 0;
    }
}

// Sets the heading based on the average over several GPS positions
void follow_me_set_heading(void);
void follow_me_set_heading(void){
	// Obtain follow me heading based on position
    counter_heading++;
    if (counter_heading == heading_calc_counter){
    	counter_heading = 0;
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
		counter_gps = 0;

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
	if (fabs(dist_wp_follow.x) > roll_enable){
		follow_me_roll = 1;
	} else if (fabs(dist_wp_follow.x) < roll_disable){
		follow_me_roll = 0;
	}
	// if (( fabs(dist_wp_follow.x) > roll_enable && fabs(dist_wp_follow_old.x) <= roll_enable)  ){
	// 	follow_me_roll = 1;
	// } else if ((fabs(dist_wp_follow.x) <= roll_disable && fabs(dist_wp_follow_old.x) > roll_disable)) {
	// 	follow_me_roll = 0;
	// }

	// This condition is required in case the relative wind is slower than the stall speed of the UAV
	if (fabs(dist_wp_follow.y) > 3*fabs(follow_me_distance)){
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

	printf("Roll setpoint follow me is given by %f based on\n P term: %f*%f=%f\n I term: %f*%f=%f\n D term: %f*%f=%f\n\n", h_ctl_roll_setpoint_follow_me, roll_diff_pgain, dist_wp_follow.x, roll_diff_pgain*dist_wp_follow.x, roll_diff_igain, roll_diff_sum_err, roll_diff_igain*roll_diff_sum_err, (dist_wp_follow.x - dist_wp_follow_old.x), roll_diff_dgain, (dist_wp_follow.x-dist_wp_follow_old.x)*roll_diff_dgain);
}


// Throttle controller
void follow_me_throttle_pid(void);
void follow_me_throttle_pid(void){
	// Airspeed controller
	airspeed_sum_err += dist_wp_follow.y;
	BoundAbs(airspeed_sum_err, 20);

	float airspeed_inc = +airspeed_pgain*dist_wp_follow.y + airspeed_igain*airspeed_sum_err - (dist_wp_follow.y-dist_wp_follow_old.y)*airspeed_dgain;

	// Add airspeed inc to average airspeed
	v_ctl_auto_airspeed_setpoint = AverageAirspeed(stateGetAirspeed_f() + airspeed_inc);

	if (v_ctl_auto_airspeed_setpoint < 0){
		v_ctl_auto_airspeed_setpoint = 0;
	}

	if (v_ctl_auto_airspeed_setpoint > 18){
		v_ctl_auto_airspeed_setpoint = 18;
	}

}

// Function that computes distance of UAV toward a certain UTM position (using follow me heading)
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
	// ground_utm.zone = nav_utm_zone0;
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
	nav_move_waypoint(WP__FOLLOW2, x_follow2, y_follow2, follow_me_altitude);
	nav_move_waypoint(WP_STDBY, x_stdby, y_stdby, follow_me_altitude + 20); // Set STBDY and HOME waypoint so that they are above the boat
	nav_move_waypoint(WP_HOME, ground_utm.east , ground_utm.north , follow_me_altitude + 20);

	// Update allowable Flying Region
	nav_move_waypoint(WP_FR_TL,  ground_utm.east - follow_me_region,  ground_utm.north + follow_me_region, follow_me_altitude + 20);
	nav_move_waypoint(WP_FR_TR,  ground_utm.east + follow_me_region,  ground_utm.north + follow_me_region, follow_me_altitude + 20);
	nav_move_waypoint(WP_FR_BL,  ground_utm.east - follow_me_region,  ground_utm.north - follow_me_region, follow_me_altitude + 20);
	nav_move_waypoint(WP_FR_BR,  ground_utm.east + follow_me_region,  ground_utm.north - follow_me_region, follow_me_altitude + 20);

	return;
}

// Sets the general heading in the direction of FOLLOW2 (waypoint located far in front of the boat)
void follow_me_go(void);
void follow_me_go(void){
	NavGotoWaypoint(WP__FOLLOW2);
    NavVerticalAltitudeMode(follow_me_altitude, 0.);
}

void compute_follow_distances(void);
void compute_follow_distances(void){
	// Increase the gps counter to verify whether gps has been lost
	counter_gps++;
	// Go to STDBY in case the ground GPS has been lost
	if (counter_gps > gps_lost_count){
		// GotoBlock(5);
		printf("Removed moving to stdby block because gps lost\n");
	}

	counter_old_distance++;
	// Compute errors towards waypoint
	// Calculate distance in main function as follow_me_compute_wp is not executed if GROUND_GPS message is not received
	if (counter_old_distance == old_distance_count){
	    dist_wp_follow_old = dist_wp_follow;
	    counter_old_distance = 0;
	}

	dist_wp_follow = compute_dist_to_utm(x_follow, y_follow, follow_me_height);
	dist_wp_follow2 = compute_dist_to_utm(x_follow2, y_follow2, follow_me_height);

	// Loop through controller
	// In case we have reached follow 2, simply increase the distance towards it so that it is never reached
	// This condition will increase the distance towards the carrot until we leave the flying zone
	if (dist_wp_follow2.y <  10){
		follow_me_distance_2 += 20;
	}
}

// This is the main function executed by the follow_me_block
int follow_me_call(void){
	compute_follow_distances();

    // Loop through controllers
	follow_me_roll_pid();
	follow_me_throttle_pid();

	// Move to the correct location
	follow_me_go();

	return 1;
}

// This function should be executed at the start of each other block so that it is executed whenever the flying region is left or a new block is called
void follow_me_stop(void){
	v_ctl_auto_airspeed_setpoint = V_CTL_AUTO_AIRSPEED_SETPOINT;
	follow_me_roll = 0;
	h_ctl_roll_setpoint_follow_me = 0;
}

