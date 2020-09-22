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
 * @author fixedwing Chris de Jong <chris@dejong.lu>
 * Control a FIXEDWING to follow at a defined distance from the target
 *
*/ 

#include <stdio.h>

#include "follow_me.h"
#include "state.h"
#include "autopilot.h"


#include "math/pprz_geodetic_int.h"
#include "math/pprz_geodetic_float.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/navigation/common_nav.h"
#include "generated/modules.h"
#include "firmwares/fixedwing/nav.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"  // for h_ctl_roll_setpoint_follow_me
#include "generated/flight_plan.h" // for waypoint reference pointers
// #include <Ivy/ivy.h> // for go to block
#include "subsystems/datalink/telemetry.h"
#include "firmwares/fixedwing/guidance/energy_ctrl.h"


/*********************************
  Parameters for follow_me module
*********************************/

// Waypoint parameters
int16_t follow_me_distance = 20; // the desired distance that we want to soar in front of
// Follow me distance +30 because we want to fly just in front of the UAV. In case we fly closer by we follow flower like patterns
int16_t follow_me_distance_2 = 20 + 30; //  this is where the uav will fly to using its navigational loop
int8_t lateral_offset = 0; // Amount in meters which the waypoint should be moved to the right with respect to the course itself
int16_t stdby_distance = 110; // based on stbdy radius (80) + 30
int16_t follow_me_height = 30; // desired height above ground station
float follow_me_altitude;  // desired altitude in case the ground statin can not be reached for example
uint16_t follow_me_region = 200;  // rectangular size of the follow me region. in case we leave this region we go to stbdy
float follow_me_heading = 0;  // the heading that the boat is driving too / used to create the waypoints or calculate the position with respect to the boat
int32_t x_follow;
int32_t y_follow;
int32_t x_follow2;
int32_t y_follow2;
uint8_t follow_me_autopilot_mode = 0;  // this boolean is used in order to detect AUTO1 AUTO2 changes. A change in auto2 can result for example in a follow_me_soar_here execution

// Roll loop
float roll_enable = 0; // when this x distance is exceeded the roll PID is enabled
float roll_disable = 0; // when the x distance is lower the roll PID is disabled again
uint8_t roll_button_disable = 1;  // In order to disable roll controler using buttons
float roll_limit = 0.2; // maximum and minimum allowable roll angle
float roll_pgain = 0.015;
uint8_t follow_me_roll = 0; // boolean variable used to overwrite h_ctl_roll_setpoint in stab_adaptive and stab_attitude

// Pitch loop
float pitch_enable = 0; // when this y distance is exceeded the pitch PID is enabled
float pitch_disable = 0; // when the y distance is lower the pitch PID is disabled again
uint8_t follow_me_pitch = 0; // boolean variable used to overwrite v_ctl_pitch_setpoint in guidance_v.c
uint8_t pitch_button_disable = 1;  // in order to disable pitch controller using button
float pitch_pgain = -0.03;
float pitch_dgain = 0;
float pitch_igain = 0;
float pitch_sum_err = 0;
float pitch_limit = 1.047; // maximum and minimum allowable change in desired_pitch_angle compared to the desired value by the controller -> 0.2 is around 10 degree
float v_ctl_pitch_setpoint_follow_me = 0;

// Throttle loop
float airspeed_sum_err = 0.0;
// Should be defined positive
float airspeed_pgain = 0.4;
float airspeed_igain = 0.00;
float airspeed_dgain = 0.00;

// Keep track of current position with respect of waypoint
struct FloatVect3 dist_wp_follow; // distance to follow me wp
struct FloatVect3 dist_wp_follow2; // distance to follow 2 waypoint
// Counter for the calculation of the old dist_wp_follow
uint8_t counter_old_distance = 0;
uint8_t old_distance_count = 20;
struct FloatVect3 dist_wp_follow_old; // old distance to follow me wp

// Ground UTM variables used in order to calculate heading (they are only updated once heading calc counter is reached)
uint8_t counter_heading = 0; // counter which counts heading function executions and thus the amount of GROUND_GPS messages received
uint8_t heading_calc_counter = 5; // in case we have received heading_calc_counter GROUND_GPS messages the heading is calculated
struct UtmCoor_f ground_utm_old;  // old ground station location
struct UtmCoor_f ground_utm_new;  // new ground station location

// Assumed no stationary gruond so that heading is not updated (this is a safety measure)
// the stationary ground detection helps in detecting as well whether we are flying using a gps reference or not. If the gruond is stationary then we need to use the heading of the UAV as follow_me_heading
uint8_t stationary_ground = 0; // boolean to keep track on whether ground station is moving or not / used in order to find out whether to update the heading or not at initiation

// Variables initialised in functions themselves
static bool ground_set; // boolean to decide whether GPS message was received or not / in case this boolean is false then we upadte the altitude directly and not the height (as we dont have a relative position for the height)
static struct LlaCoor_i ground_lla; // lla coordinates received by the GPS message
static uint32_t ground_timestamp; // only execut set wp function if we received a newer timestamp
static uint32_t old_ground_timestamp;  // to compare it to the new timestamp
struct UtmCoor_f ground_utm;  // global because required for file logger and called by soar_here

/*********************************
  Average speed calculator
*********************************/

uint8_t average_speed_size = 10;  // the amount of datapoints we would like to calculate our average from
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
#define MAX_HEADING_SIZE 10
float all_diff_x[MAX_HEADING_SIZE]={0};
float all_diff_y[MAX_HEADING_SIZE]={0};
// Parameter which keeps track of the value that needs to be replaced
uint8_t current_heading_value = 0;
float AverageHeading(float, float);
//function definition
float AverageHeading(float diffx, float diffy)
{
    float Sum_x = 0;
    float Sum_y = 0;

    all_diff_x[current_heading_value] = diffx;
    all_diff_y[current_heading_value] = diffy;

    current_heading_value++;
    if (current_heading_value == MAX_HEADING_SIZE){
    	current_heading_value = 0;
    }

    for (int i=0; i<MAX_HEADING_SIZE; i++){
    	Sum_x = Sum_x + all_diff_x[i];
    	Sum_y = Sum_y + all_diff_y[i];
    }

    // Check for condition in which we are not moving
    // In case we are not moving keep the current heading
    if ((fabs(Sum_x) < 4) && (fabs(Sum_y) < 4)){
    	stationary_ground = 1;
    	return follow_me_heading;
    } else {
    	stationary_ground = 0;
		float heading = 0.0;
		// First check cases which divide by 0
		if (Sum_y == 0.0){
			if (Sum_x > 0.0){
				heading = 90.0;
			} else if (Sum_x < 0.0){
				heading = -90.0;
			}
		} else {
			heading = atan2(Sum_x, Sum_y)*180.0/M_PI;  // returns value between -180 and 180 (at least no other values have been found yet)
		}
		return heading;
    }
}



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
  Follow me functions
***********************************************************************************************************************/

// Compute both lateral offset and follow_me_distance
// Also used by RL algorithm
// struct FloatVect3 compute_state(void);
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

	return transformation;
}

// Wind speed predictor
struct FloatVect3 compute_wind_field(void);
struct FloatVect3 compute_wind_field(void){
    // Initial Parameters for wind field computation
	float R_ridge = 10.0; // Height of the hill in m
	float U_inf = 15.0; // Wind velocity at infinity in m/s
	float a = 10; // Defines loci x-position
	float x_stag = 14; // m, defines a-axis of standard oval
	float b = sqrt(x_stag*x_stag - a*a);

	struct FloatVect3 dist_from_boat = compute_state();
	struct FloatVect3 wind_vector;
	wind_vector.x = 0;

	// Start of computation
    float m = M_PI*U_inf/a*(x_stag*x_stag - a*a);
    wind_vector.y = U_inf + m / (2 * M_PI) * ((dist_from_boat.y + a) / ((dist_from_boat.y+0.001 + a) * (dist_from_boat.y+0.001 + a) + dist_from_boat.z * dist_from_boat.z) - (dist_from_boat.y+0.001 - a) / ((dist_from_boat.y+0.001 - a)*(dist_from_boat.y+0.001 - a) + dist_from_boat.z*dist_from_boat.z));
    wind_vector.z = -(m * dist_from_boat.z) / (2 * M_PI) * (1 / ((dist_from_boat.y+0.001 + a)*(dist_from_boat.y+0.001 + a) + dist_from_boat.z*dist_from_boat.z) - 1 / ((dist_from_boat.y+0.001 - a)*(dist_from_boat.y+0.001 - a) + dist_from_boat.z*dist_from_boat.z));

	float ellipse_eq = (dist_from_boat.y * dist_from_boat.y) / (x_stag * x_stag) + (dist_from_boat.z * dist_from_boat.z) / (b * b);

	if (ellipse_eq < 1){
	    wind_vector.x = 0;
	    wind_vector.y = 0;
	    wind_vector.z = 0;
	    return wind_vector;
	}

    if (dist_from_boat.y < x_stag){
	    float z_ellipse = -b / x_stag * sqrt(x_stag * x_stag- dist_from_boat.y * dist_from_boat.y);

	    // v_x *= ((z_i - z_ellipse)/-20)**(1/7)
	    // v_z *= ((z_i - z_ellipse) / -20) ** (1 / 7)

	    float mult_factor = (log(-(dist_from_boat.z - z_ellipse))/0.05)/(log(-(-20 - z_ellipse))/0.05);
		if (mult_factor){
            wind_vector.y *= mult_factor;
	        wind_vector.z *= mult_factor;
		} else {
	        if (dist_from_boat.z < 0){
	            z_ellipse = -0.01;
	            mult_factor = (log(-(dist_from_boat.z - z_ellipse)) / 0.05) / (log(-(-20 - z_ellipse)) / 0.05);
	            wind_vector.y *= mult_factor;
	            wind_vector.z *= mult_factor;
	        } else {
	            wind_vector.x = 0;
	            wind_vector.y = 0;
	            wind_vector.z = 0;
	        }
		}
    }
    return wind_vector;
}


//void follow_me_soar_here(void);
// Sets all follow me parameters like the current
void follow_me_soar_here(void){
	// This condition is required because sometimes the ground_utm variable has not been updated yet in case the GROUND_GPS messages was not received yet
	if ((ground_utm.east != 0) && (ground_utm.north != 0)){
		// Obtain the current position to calculate waypoint positions
		struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();

		// Set the follow_me_heading to the current heading of the UAV
		if (stationary_ground){
		    follow_me_heading = stateGetNedToBodyEulers_f()->psi*180/M_PI;
		}

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
	pprz_msg_send_FOLLOW_ME(trans, dev, AC_ID, &dist_wp_follow.y, &dist_wp_follow.x, &v_ctl_auto_airspeed_setpoint, &follow_me_roll, &follow_me_pitch);
}

// Called at compiling of module
void follow_me_init(void){
	ground_set = false;
	register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_FOLLOW_ME, send_follow_me);
}

// Called each time the follow me block is started
void follow_me_startup(void){
    follow_me_call();

    if (!roll_button_disable){
		if ((dist_wp_follow.x > roll_enable) || (dist_wp_follow.x < -roll_enable)){
			follow_me_roll = 1;
		} else {
			follow_me_roll = 0;
		}
    }
}

// Sets the heading based on the average over several GPS positions
void follow_me_set_heading(void);
void follow_me_set_heading(void){
	// Obtain follow me heading based on position
    counter_heading++;
    if (counter_heading == heading_calc_counter){
    	counter_heading = 0;
    	float diff_y;
    	float diff_x;
    	if (ground_utm_old.north != 0 && ground_utm_old.east != 0){
            diff_y = ground_utm_new.north - ground_utm_old.north;
			diff_x = ground_utm_new.east - ground_utm_old.east;
    	} else {
    		diff_x = 0;
    		diff_y = 0;
    	}

		// Obtain average heading over this new distance
		// Note atan2 gives results between -180 and 180
		follow_me_heading = AverageHeading(diff_x, diff_y);
		ground_utm_old = ground_utm_new;
    }
}

// void follow_me_compute_wp(void);
// Function that is executed each time the GROUND_GPS message is received
void follow_me_parse_ground_gps(uint8_t *buf){
	if(DL_GROUND_GPS_ac_id(buf) != AC_ID)
		return;

	// Automaitcally move waypoint in case AUTO2 is engaged
	if ((autopilot.mode == 2) && (follow_me_autopilot_mode != 2)){
		follow_me_soar_here();
	}
	follow_me_autopilot_mode =  autopilot.mode;

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
    // Verify whether this can move to the ground_timestamp if condition above
	follow_me_set_heading();
}


// Function to enable roll controller using buttons
void follow_me_enable_roll(void){
	roll_button_disable = 0;
}

// Function to disable roll controller using button
void follow_me_disable_roll(void){
	roll_button_disable = 1;
	follow_me_roll = 0;
}

// Function to enable pitch controller using buttons
void follow_me_enable_pitch(void){
	pitch_button_disable = 0;
}

// Function to disable pitch controller using button
void follow_me_disable_pitch(void){
	pitch_button_disable = 1;
	follow_me_pitch = 0;
}

// Pitch angle controller
// void follow_me_pitch_loop(void);
// NOTE: The pitch has been completely disabled until roll and airspeed loop have been tested
//void follow_me_pitch_loop(void);
void follow_me_pitch_loop(void){
	// Pitch rate controller
	// If we have the pitch loop disabled via the button set the pitch to 0
	if (!pitch_button_disable){
		if (fabs(dist_wp_follow.y) > pitch_enable){
			follow_me_pitch = 1;
		} else if (fabs(dist_wp_follow.y) < pitch_disable){
			follow_me_pitch = 0;
		}
	} else {
		follow_me_pitch = 0;
	}

	// This condition is required in case the relative wind is slower than the stall speed of the UAV
	if ((fabs(dist_wp_follow.y) > 3*fabs(follow_me_distance)) || fabs(dist_wp_follow.x > 5*roll_enable)){
		follow_me_pitch = 0;
	}

	pitch_sum_err += dist_wp_follow.y;
	BoundAbs(pitch_sum_err, 20);

	v_ctl_pitch_setpoint_follow_me = +pitch_pgain*dist_wp_follow.y + pitch_igain*pitch_sum_err + (dist_wp_follow.y-dist_wp_follow_old.y)*pitch_dgain;

	// Bound pitch by limits
	if (v_ctl_pitch_setpoint_follow_me > pitch_limit){
		v_ctl_pitch_setpoint_follow_me = pitch_limit;
	}
	else if (v_ctl_pitch_setpoint_follow_me < -pitch_limit){
		v_ctl_pitch_setpoint_follow_me = -pitch_limit;
	}
}

// Roll angle controller
// void follow_me_roll_loop(void);
// void follow_me_roll_loop(void);
void follow_me_roll_loop(void){
	// Roll rate controller
	// If we have the roll loop disabled via the button set the roll to 0
	// If we have been in course and exceed the enable limits then nav follow is activated
	// If we have been in follow and exceed the disable limits then nav course is activated
	if (!roll_button_disable){
		if (fabs(dist_wp_follow.x) >= roll_enable){
			follow_me_roll = 1;
		} else if (fabs(dist_wp_follow.x) <= roll_disable){
			follow_me_roll = 0;
		}
	} else {
		follow_me_roll = 0;
	}

	// This condition is required in case the relative wind is slower than the stall speed of the UAV
	if ((fabs(dist_wp_follow.y) > 3*fabs(follow_me_distance)) || fabs(dist_wp_follow.x > 5*roll_enable)){
		follow_me_roll = 0;
	}

	// roll_sum_err += dist_wp_follow.x;
	// BoundAbs(roll_sum_err, 20);
	// h_ctl_roll_setpoint_follow_me = +roll_pgain*dist_wp_follow.x + roll_igain*roll_sum_err + (dist_wp_follow.x-dist_wp_follow_old.x)*roll_dgain;

	// The -roll disable is used in order to excert a slightly inverted roll command once we reach the target
	// The stops the UAV at the exact right point
	// ToDo this probably gives an error as we move from one conditions to the next and change the loop
	if (dist_wp_follow.x > 0){
	    h_ctl_roll_setpoint_follow_me = roll_pgain*log(dist_wp_follow.x - roll_disable);
	}
	else {
	    h_ctl_roll_setpoint_follow_me = -roll_pgain*log(-(dist_wp_follow.x + roll_disable));
	}
	// Bound roll diff by limits
	if (h_ctl_roll_setpoint_follow_me > roll_limit){
		h_ctl_roll_setpoint_follow_me = roll_limit;
	}
	else if (h_ctl_roll_setpoint_follow_me < -roll_limit){
		h_ctl_roll_setpoint_follow_me = -roll_limit;
	}
}


// Throttle controller
// void follow_me_throttle_loop(void);
void follow_me_throttle_loop(void){
	// Airspeed controller
	airspeed_sum_err += dist_wp_follow.y;
	BoundAbs(airspeed_sum_err, 20);

	float airspeed_inc = +airspeed_pgain*dist_wp_follow.y + airspeed_igain*airspeed_sum_err - (dist_wp_follow.y-dist_wp_follow_old.y)*airspeed_dgain;

	// Add airspeed inc to average airspeed
	v_ctl_auto_airspeed_setpoint = AverageAirspeed(stateGetAirspeed_f() + airspeed_inc);

	if (v_ctl_auto_airspeed_setpoint < 0){
		v_ctl_auto_airspeed_setpoint = 0;
	}

	if (v_ctl_auto_airspeed_setpoint > 21){
		v_ctl_auto_airspeed_setpoint = 21;
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

// void compute_follow_distances(void);
void compute_follow_distances(void){
	counter_old_distance++;
	// Compute errors towards waypoint
	// Calculate distance in main function as follow_me_compute_wp is not executed if GROUND_GPS message is not received
	// We only update the old counter distance after x iterations
	// Currently (23 opf June 2020) the dist_wp_follow_old is only used for the airspeed d gain
	if (counter_old_distance == old_distance_count){
	    dist_wp_follow_old = dist_wp_follow;
	    counter_old_distance = 0;
	}

	// The distances toward each waypoint are computed
	// Dist wp follow 2 is currently (23 of June) only used in order to increase the waypoint distance in case we come to close to the wp
	dist_wp_follow = compute_dist_to_utm(x_follow, y_follow, follow_me_altitude);
	dist_wp_follow2 = compute_dist_to_utm(x_follow2, y_follow2, follow_me_altitude);

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
	follow_me_pitch_loop();
	follow_me_roll_loop();
	follow_me_throttle_loop();
	struct FloatVect3 wind = compute_wind_field();
   // init ivy and register callback for WORLD_ENV_REQ and NPS_SPEED_POS
    IvySendMsg("%s WORLD_ENV %f %f %f 400 1 1", "0", wind.x, -wind.y, wind.z);

	// struct FloatVect3* windspeed_f = stateGetWindspeed_f();
    // printf("Windspeed vector is given by: %f %f %f\n", windspeed_f->x, windspeed_f->y, windspeed_f->z);

	// Move to the correct location
	follow_me_go();

	return 1;
}

// This function should be executed at the start of each other block so that it is executed whenever the flying region is left or a new block is called
void follow_me_stop(void){
	v_ctl_auto_airspeed_setpoint = V_CTL_AUTO_AIRSPEED_SETPOINT;
	follow_me_roll = 0;
	h_ctl_roll_setpoint_follow_me = 0;
	roll_button_disable = 1;
	pitch_button_disable = 1;
}

