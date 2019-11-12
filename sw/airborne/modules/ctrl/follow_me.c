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
#include "modules/rl_soaring/rl_soaring.h"

// Parameters for follow_me module
uint8_t follow_me_distance = 20; // distance from which the follow me points are created
uint8_t follow_me_height = 10;
float follow_me_heading = 0;
float average_follow_me_distance;
#define HAND_RL_SIZE 20 // the amount of average_follow_me_distance that need to be below the threshold in order to hand control over to RL
int8_t hand_rl[HAND_RL_SIZE] = {0};
float hand_rl_threshold = 1;
int8_t hand_rl_idx = 0; // the index value that needs to be modified

// Variables that are send to the ground station for real time plotting or logging
float desired_ground_speed_max; // for the real time plotting
float desired_ground_speed_min; // for the real time plotting
float actual_ground_speed;
struct FloatVect3 dist_wp_follow; // distance to follow me wp
float dist_wp_follow_y_min; // for the real time plotting
float dist_wp_follow_y_max; // for the real time plotting
float dist_wp_follow_x_min;
float dist_wp_follow_x_max;
float safety_boat_distance = 1; // distance that the UAV should not move from the boat
float ground_speed_diff = 0; // counter which increases by 1 each time we are faster than the follow_me waypoint (in order to learn the ground speed of the boat )
float ground_speed_diff_limit = 1.5; // maximum and minimum allowable change in gruond speed compared to desired value from gps
float roll_diff = 0;
float roll_diff_limit = 100; // maximum and minimum allowable change in roll rate compared to the desired value by the controller


struct FloatVect3 wp_follow_utm;
struct FloatVect3 wp_follow_enu;
float ground_speed_diff_pgain = 0.3;
float ground_speed_diff_dgain = 0.15;
float ground_speed_diff_igain = 0.03;
float ground_speed_diff_sum_err = 0.0;

float roll_diff_pgain = 0.0006;
float roll_diff_dgain = 0.0006;
float roll_diff_igain = 0.00006;
float roll_diff_sum_err = 0.0;



// Old location to reset sum error
struct FloatVect3 dist_wp_follow_old; // old distance to follow me wp

// Variables initialised in functions themselves
static bool ground_set;
static struct LlaCoor_i ground_lla;
float ground_speed;
static float ground_climb;
static float ground_course;
static float ground_timestamp;
static float old_ground_timestamp;


//Calculate the average gps heading in order to predict where the boat is going
#define MAX_HEADING_SIZE 10
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
    return ((float)Sum/fmin(MAX_HEADING_SIZE, count_heading));
}


//Calculate the average gps heading in order to predict where the boat is going
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



static void send_follow_me(struct transport_tx *trans, struct link_device *dev){
	float roll_angle_min = 0; //-roll_diff_limit;
	float roll_angle_max = 0; // roll_diff_limit;
	float roll_angle = stateGetNedToBodyEulers_f()->phi;
	pprz_msg_send_FOLLOW_ME(trans, dev, AC_ID, &average_follow_me_distance, &v_ctl_auto_groundspeed_setpoint, &desired_ground_speed_min, &desired_ground_speed_max, &actual_ground_speed, &dist_wp_follow.y, &dist_wp_follow_y_min, &dist_wp_follow_y_max, &h_ctl_roll_setpoint, &roll_angle_min, &roll_angle_max, &roll_angle, &dist_wp_follow.x, &dist_wp_follow_x_min, &dist_wp_follow_x_max);
}



// Called at compiling of module
void follow_me_init(void){
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FOLLOW_ME, send_follow_me);
    ground_set = false;
}


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
	transformation.x = cosf(theta)*point->x - sinf(theta)*point->y;
	transformation.y = sinf(theta)*point->x + cosf(theta)*point->y;
	transformation.z = point->z;
	// Return
	return transformation;
}

// Function to check whether we can hand control over to reinforcement learning again
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




// Called each time the follow me block is started
void follow_me_startup(void){
	if (!rl_started){
        // Set the default altitude of waypoints to the current height so that the drone keeps the height
        struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();
        follow_me_height = pos_Utm->alt;
    }
}

void follow_me_parse_ground_gps(uint8_t *buf){

	if(DL_GROUND_GPS_ac_id(buf) != AC_ID)
		return;

	// Save the received values
	ground_lla.lat = DL_GROUND_GPS_lat(buf);
	ground_lla.lon = DL_GROUND_GPS_lon(buf);
	ground_lla.alt = DL_GROUND_GPS_alt(buf);
	ground_speed = DL_GROUND_GPS_speed(buf);
	desired_ground_speed_min = ground_speed - ground_speed_diff_limit;
	desired_ground_speed_max = ground_speed + ground_speed_diff_limit;
	ground_climb = DL_GROUND_GPS_climb(buf);
	ground_course = DL_GROUND_GPS_course(buf);
	old_ground_timestamp = ground_timestamp;
	ground_timestamp = DL_GROUND_GPS_timestamp(buf);
	follow_me_heading = AverageHeading(ground_course);
	ground_set = true;
}


// Manage the throttle so that the groundspeed of both the boat and the uav are equivalent
void follow_me_set_groundspeed(void);
void follow_me_set_groundspeed(void){
	v_ctl_auto_groundspeed_setpoint = ground_speed + ground_speed_diff;
	if (v_ctl_auto_groundspeed_setpoint < 0){
		v_ctl_auto_groundspeed_setpoint = 0;
	}
}

// Sets the WP_FOLLOW based on GPS coordinates received by ground segment
// Returns 0 if the waypoint is in front of the UAV and 1 otherwise
void follow_me_set_wp(void){
	if(ground_set) {
		actual_ground_speed = stateGetHorizontalSpeedNorm_f();  // store actual groundspeed in variable to send through pprzlink
		// Obtain lat lon coordinates for conversion
		struct LlaCoor_f lla;
		lla.lat = RadOfDeg((float)(ground_lla.lat / 1e7));
		lla.lon = RadOfDeg((float)(ground_lla.lon / 1e7));
		lla.alt = ((float)(ground_lla.alt))/1000.;

		// Convert LLA to UTM in oder to set watpoint in UTM system
		struct UtmCoor_f utm;
		utm.zone = nav_utm_zone0;
		utm_of_lla_f(&utm, &lla);

		// Follow waypoint
		int32_t x_follow = utm.east + follow_me_distance*sinf(follow_me_heading/180.*M_PI);
		int32_t y_follow = utm.north + follow_me_distance*cosf(follow_me_heading/180.*M_PI);

		// Follow 2 waypoint at twice the distance
		int32_t x_follow2 = utm.east + 3*follow_me_distance*sinf(follow_me_heading/180.*M_PI);
		int32_t y_follow2 = utm.north + 3*follow_me_distance*cosf(follow_me_heading/180.*M_PI);

		wp_follow_utm.x = x_follow;
		wp_follow_utm.y = y_follow;
		wp_follow_utm.z = follow_me_height;

		wp_follow_enu = UTM_to_ENU(&wp_follow_utm);

		// Dist wp follows using ENU system
		dist_wp_follow_old = dist_wp_follow;
		dist_wp_follow.x = wp_follow_enu.x;
		dist_wp_follow.y = wp_follow_enu.y;
		dist_wp_follow.z = wp_follow_enu.z;

		// these values are only for plotting for now
        dist_wp_follow_y_max = follow_me_distance - safety_boat_distance;
        dist_wp_follow_y_min = -2*follow_me_distance + 1; // distance of second waypoint which make the uav fly around (2* because wp is at 1*)
        dist_wp_follow_x_min = 0;
        dist_wp_follow_x_max = 0;


        // Update STBDY HOME AND FOLLOW ME WPS
		nav_move_waypoint(WP_FOLLOW, x_follow,  y_follow, follow_me_height);
		nav_move_waypoint(WP_FOLLOW2, x_follow2, y_follow2, follow_me_height);
		nav_move_waypoint(WP_STDBY, utm.east, utm.north, follow_me_height + 20); // Set STBDY and HOME waypoint so that they are above the boat
		nav_move_waypoint(WP_HOME, utm.east, utm.north, follow_me_height + 20);

		// Update allowable Flying Region
		nav_move_waypoint(WP_FR_TL, x_follow - 200, y_follow + 200, follow_me_height);
		nav_move_waypoint(WP_FR_TR, x_follow + 200, y_follow + 200, follow_me_height);
		nav_move_waypoint(WP_FR_BL, x_follow - 200, y_follow - 200, follow_me_height);
		nav_move_waypoint(WP_FR_BR, x_follow + 200, y_follow - 200, follow_me_height);

		// Reset the ground boolean
	    ground_set = false;

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
	}
	return;
}


// Which navigational procedure to use
// 3 possible location: -1 is behind the boat
//                       0 is between the boat and the Follow me waypoint
//                       1 is in front of the follow me waypoint
void follow_me_go(void);
void follow_me_go(void){
	NavGotoWaypoint(WP_FOLLOW2);
    NavVerticalAltitudeMode(follow_me_height, 0.);

}

// This function is executed each time before the follow_me_block is called
// It calculates the difference in groundspeed between the UAV and the system
// This ground speed diff is used in order to propagate errors in case the GPS speed error is not reliable
int follow_me_call(void){
	// Only set the new location if the new timestamp is later (otherwise probably due to package loss in between)
	if (ground_timestamp > old_ground_timestamp){
		follow_me_set_wp();
	}

	// Ground speed controller
    ground_speed_diff_sum_err += dist_wp_follow.y;
    BoundAbs(ground_speed_diff_sum_err, 20);
    ground_speed_diff = +ground_speed_diff_pgain*dist_wp_follow.y + ground_speed_diff_igain*ground_speed_diff_sum_err + (dist_wp_follow.y-dist_wp_follow_old.y)*ground_speed_diff_igain;
	// Bound groundspeed diff by limits
	if (ground_speed_diff > ground_speed_diff_limit){
		ground_speed_diff = ground_speed_diff_limit;
	}
	else if (ground_speed_diff < -ground_speed_diff_limit){
		ground_speed_diff = -ground_speed_diff_limit;
	}

	// Roll rate controller
    roll_diff_sum_err += dist_wp_follow.x;
    BoundAbs(roll_diff_sum_err, 5);
    roll_diff = +roll_diff_pgain*dist_wp_follow.x + roll_diff_igain*roll_diff_sum_err + (dist_wp_follow.x-dist_wp_follow_old.x)*roll_diff_igain;
	// Bound groundspeed diff by limits
	if (roll_diff > roll_diff_limit){
		roll_diff = roll_diff_limit;
	}
	else if (roll_diff < -roll_diff_limit){
		roll_diff = -roll_diff_limit;
	}

	h_ctl_roll_setpoint_follow_me = roll_diff;

	follow_me_go();
	follow_me_set_groundspeed();

	if (check_handover_rl()){
		GotoBlock(7);
	}

	return 1;
}

void follow_me_stop(void){
	ground_speed_diff = 0;
	v_ctl_auto_groundspeed_setpoint = V_CTL_AUTO_GROUNDSPEED_SETPOINT;
}

