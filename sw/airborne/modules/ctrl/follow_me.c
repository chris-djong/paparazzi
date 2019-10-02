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
#include "generated/flight_plan.h" // for waypoint reference pointers

#include "subsystems/datalink/telemetry.h"



//declarations
#define MAXSIZE 10
float average_array[MAXSIZE]={0};
int front=-1,rear=-1, count=0;
float AverageForNewElement(float);

//function definition
float AverageForNewElement(float item)
{
	count += 1;
    static float Sum=0;
    if(front ==(rear+1)%MAXSIZE)
    {
        if(front==rear)
            front=rear=-1;
        else
            front = (front+1)%MAXSIZE;
        Sum=Sum-average_array[front];
    }
    if(front==-1)
        front=rear=0;
    else
        rear=(rear+1)%MAXSIZE;
    average_array[rear]=item;
    Sum=Sum+average_array[rear];
    return ((float)Sum/fmin(MAXSIZE, count));
}


// Parameters for follow_me module
int32_t follow_me_distance = 5; // distance from which the follow me points are created
int32_t follow_me_height = 10;
float follow_me_heading = 0;




// Parameters to be set for landing procedure
int32_t flare_height = 10; // height at which flaring starts
float land_vspeed = 2.7; // descend speed in m/s
int32_t land_hspeed = 13.5; //assumed airspeed during landing
int32_t flare_hspeed = 7;
float flare_vspeed = 0.8;
float flare_heading = 0.;     // heading used to set flare point based on gps
float ground_speed_diff = 0; // counter which increases by 1 each time we are faster than the follow_me waypoint (in order to learn the ground speed of the boat )


// Gains for the throttle management
// gains should be negative as a lower groundspeed should result in a higher throttle
float throttle_constant = 0.3; // necesarry so that a negative error does not result in 0 thrust immediately
float throttle_pgain = 0.17;
float throttle_dgain = 0.09;
float throttle_igain = 0.03;
float throttle_sum_err = 0;
float last_err = 0;

// Variables that are send to the ground station for real time plotting
float follow_me_location; //
float desired_ground_speed;
float actual_ground_speed;
float v_ctl_auto_throttle_cruise_throttle;
float p_thrust;
float i_thrust;
float d_thrust;
float difference_distance;
float dist_wp_follow; // distance to follow me wp

// Old location to reset sum error
float old_location;
float dist_wp_follow_old; // old distance to follow me wp


// Variables initialised in functions themselves
static bool ground_set;
static struct LlaCoor_i ground_lla;
static float ground_speed;
static float ground_climb;
static float ground_course;

// Called at compiling of module
void follow_me_init(void){
    ground_set = false;
}


/*Translate frame to a new point
 * The frame is moved from its origin to the new point (transx, transy, transz)*/
struct Int32Vect3 translate_frame(struct Int32Vect3 *point, int trans_x, int trans_y, int trans_z);
struct Int32Vect3 translate_frame(struct Int32Vect3 *point, int trans_x, int trans_y, int trans_z){
	// Create return vectore for function
	struct Int32Vect3 transformation;

	// Move frame
	transformation.x = point->x - trans_x;
	transformation.y = point->y - trans_y;
	transformation.z = point->z - trans_z;
	// Return
	return transformation;
}

/*Rotate a point in a frame by an angle theta (clockwise positive)*/
struct Int32Vect3 rotate_frame(struct Int32Vect3 *point, float theta);
struct Int32Vect3 rotate_frame(struct Int32Vect3 *point, float theta){
	// Create return Vector for function
	struct Int32Vect3 transformation;

    // Rotate point
	transformation.x = cosf(theta)*point->x - sinf(theta)*point->y;
	transformation.y = sinf(theta)*point->x + cosf(theta)*point->y;
	transformation.z = point->z;
	// Return
	return transformation;
}

/*Transformation from UTM coordinate system to the Body system
 * Pos UTM is used as input so that during a whole function execution it stays constant for all the transforms
 * Same for heading */
struct Int32Vect3 UTM_to_ENU(struct Int32Vect3 *point);
struct Int32Vect3 UTM_to_ENU(struct Int32Vect3 *point){
	// Create return vector for the function
	struct Int32Vect3 transformation;

	// Obtain current UTM position
	struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();

	// Obtain current ENU position and Euler Angles in order to calculate the heading
	// struct EnuCoor_i *pos             = stateGetPositionEnu_i();
	// struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
	// float heading = ANGLE_FLOAT_OF_BFP(eulerAngles->psi);

    // Translate frame
	transformation = translate_frame(point, pos_Utm->east, pos_Utm->north, pos_Utm->alt);
	// Then rotate frame
	transformation = rotate_frame(&transformation, follow_me_heading*M_PI/180);

	// Return
	return transformation;
}

/*Transformation from the Body system to the UTM coordinate system */
struct Int32Vect3 ENU_to_UTM(struct Int32Vect3 *point);
struct Int32Vect3 ENU_to_UTM(struct Int32Vect3 *point){
	// Create return vector for the function
	struct Int32Vect3 transformation;

	// Obtain current ENU position and Euler Angles in order to calculate the heading
	// struct EnuCoor_i *pos             = stateGetPositionEnu_i();
	// struct Int32Eulers *eulerAngles   = stateGetNedToBodyEulers_i();
	// float heading = ANGLE_FLOAT_OF_BFP(eulerAngles->psi);

	// Obtain current Utm position for translationg
	struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();

    // Rotate frame back
	transformation = rotate_frame(point, -follow_me_heading);

	// Translate frame back
	transformation = translate_frame(&transformation, -pos_Utm->east, -pos_Utm->north, -pos_Utm->alt);
	//Return
	return transformation;
}

// Called each time the follow me block is started
void follow_me_startup(void){
    // Set the default altitude of waypoints to the current height so that the drone keeps the height
    struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();
    follow_me_height = pos_Utm->alt;
}

void follow_me_parse_ground_gps(uint8_t *buf){
	if(DL_GROUND_GPS_ac_id(buf) != AC_ID)
		return;

	// Save the received values
	ground_lla.lat = DL_GROUND_GPS_lat(buf);
	ground_lla.lon = DL_GROUND_GPS_lon(buf);
	ground_lla.alt = DL_GROUND_GPS_alt(buf);
	ground_speed = DL_GROUND_GPS_speed(buf);
	ground_climb = DL_GROUND_GPS_climb(buf);
	ground_course = DL_GROUND_GPS_course(buf);
	follow_me_heading = ground_course;
	flare_heading = ground_course + 180.f;
	if(flare_heading > 360.f) flare_heading -= 360.f;
	ground_set = true;
}


// Manage the throttle so that the groundspeed of both the boat and the uav are equivalent
void follow_me_set_throttle(void);
void follow_me_set_throttle(void){
	struct EnuCoor_f *speedEnu;

	speedEnu = stateGetSpeedEnu_f();
	printf("Y speed is given by: %f\n", speedEnu->y);
	actual_ground_speed = sqrt(speedEnu->x * speedEnu->x + speedEnu->y * speedEnu->y);
	desired_ground_speed = ground_speed + ground_speed_diff;
	if (desired_ground_speed < 0){
		desired_ground_speed = 0;
	}
	float err = fmax(1, fmin(dist_wp_follow, 2))*(ground_speed + ground_speed_diff - actual_ground_speed);

	// The error is calculated based on whether we get closer to the target or not
    float d_err = err - last_err;
    last_err = err;
    throttle_sum_err += err;

	p_thrust = throttle_pgain * err;
	d_thrust = throttle_dgain * d_err;
	i_thrust = throttle_igain * throttle_sum_err;
	v_ctl_auto_throttle_cruise_throttle = throttle_constant + p_thrust + i_thrust + d_thrust;
	if (v_ctl_auto_throttle_cruise_throttle<0){
		v_ctl_auto_throttle_cruise_throttle = 0;
	}
	else if (v_ctl_auto_throttle_cruise_throttle>1.5){
		v_ctl_auto_throttle_cruise_throttle = 1.5;
	}
}

// Sets the WP_FOLLOW based on GPS coordinates received by ground segment
// Returns 0 if the waypoint is in front of the UAV and 1 otherwise
int follow_me_set_wp(void){
	if(ground_set) {
		// Obtain lat lon coordinates for conversion
		struct LlaCoor_f lla;
		lla.lat = RadOfDeg((float)(ground_lla.lat / 1e7));
		lla.lon = RadOfDeg((float)(ground_lla.lon / 1e7));
		lla.alt = ((float)(ground_lla.alt))/1000.;

		// Convert LLA to UTM
		struct UtmCoor_f utm;
		utm.zone = nav_utm_zone0;
		utm_of_lla_f(&utm, &lla);

		// Follow waypoint
		int32_t x_follow = utm.east + follow_me_distance*sinf(follow_me_heading/180.*M_PI);
		int32_t y_follow = utm.north + follow_me_distance*cosf(follow_me_heading/180.*M_PI);

		// Follow 2 waypoint at twice the distance
		int32_t x_follow2 = utm.east + 3*follow_me_distance*sinf(follow_me_heading/180.*M_PI);
		int32_t y_follow2 = utm.north + 3*follow_me_distance*cosf(follow_me_heading/180.*M_PI);

		struct Int32Vect3 wp_follow_utm;
		wp_follow_utm.x = x_follow;
		wp_follow_utm.y = y_follow;
		wp_follow_utm.z = follow_me_height;


		struct Int32Vect3 wp_ground_utm;
		wp_ground_utm.x = utm.east;
		wp_ground_utm.y = utm.north;
		wp_ground_utm.z = utm.alt;

		struct Int32Vect3 wp_follow_body = UTM_to_ENU(&wp_follow_utm);
		struct Int32Vect3 wp_ground_body = UTM_to_ENU(&wp_ground_utm);

		// Obtain current Utm position and calculate distance towards wp
		struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();

		dist_wp_follow_old = dist_wp_follow;
		dist_wp_follow = sqrt((x_follow - pos_Utm->east)*(x_follow - pos_Utm->east) + (y_follow - pos_Utm->north)*(y_follow - pos_Utm->north));

		// Update STBDY HOME AND FOLLOW ME WP
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

	    // If the follow me waypoint is behind the UAV then use backwards approach
		if (wp_follow_body.y < 1 && wp_follow_body.y > -30){
			// Obtain current ENU position and Euler Angles in order to calculate the heading
			return 1;
		} else if (wp_ground_body.y < -0.5){ // if the UAV is between the ship and the waypoint
			return 0;
		}
		else{ // if the UAV is behind the boat
			return -1;
		}
	}
	return 0;
}

// Which navigational procedure to use
// 3 possible location: -1 is behind the boat
//                       0 is between the boat and the Follow me waypoint
//                       1 is in front of the follow me waypoint
void follow_me_go(float location);
void follow_me_go(float location){
	// In case we switch from location reset the sum error so that they are not carried through to the next phase
	if (old_location == -1){
		throttle_sum_err = 0;
	}
	NavGotoWaypoint(WP_FOLLOW2);
    NavVerticalAltitudeMode(follow_me_height, 0.);

	old_location = location;
}

// This function is executed each time before the follow_me_block is called
// It calculates the difference in groundspeed between the UAV and the system
// This ground speed diff is used in order to propagate errors in case the GPS speed error is not reliable
int follow_me_call(void){
	follow_me_location = follow_me_set_wp();
	difference_distance = dist_wp_follow - dist_wp_follow_old;
	// In case we are between the boat and the waypoint t
	if (follow_me_location == 0){
		// If we were launched from the boat for example reset the difference
		// as otherwise the UAV will keep flying at the ground speed diff
		if (old_location == -1){
		    ground_speed_diff = 0;
		}
	    if (difference_distance > 0){
	    	ground_speed_diff += difference_distance/10;
	    } else if (difference_distance < 0){
	    	ground_speed_diff -= difference_distance/10;
	    }
    // In case we are in front of the waypoint
	} else if (follow_me_location == 1){
		if (difference_distance > 0){
			ground_speed_diff -= difference_distance/3;
		} else if (difference_distance < 0){
			ground_speed_diff += difference_distance/3;
		}
	// In case we are behind the boat
	}  else if (follow_me_location == -1){
		ground_speed_diff = 2;
	}
	// Bound groundspeed diff by 2 or -2
	if (ground_speed_diff > 2){
		ground_speed_diff = 2;
	}
	else if (ground_speed_diff < -2){
		ground_speed_diff = -2;
	}

      if (follow_me_location != -1){
        ground_speed_diff = AverageForNewElement(ground_speed_diff);
      }

	follow_me_go(follow_me_location);
	follow_me_set_throttle();
	return 1;
}

void follow_me_set_landing_points(void){
		// Obtain lat lon coordinates for conversion
		struct LlaCoor_f lla;


        lla.lat = RadOfDeg((float)(ground_lla.lat / 1e7));
		lla.lon = RadOfDeg((float)(ground_lla.lon / 1e7));
		lla.alt = ((float)(ground_lla.alt))/1000.;

		// Convert LLA to UTM
		struct UtmCoor_f utm;
		utm.zone = nav_utm_zone0;
		utm_of_lla_f(&utm, &lla);

		// Flare waypoint
		float flare_distance = flare_hspeed*flare_height/flare_vspeed;
		int32_t x = flare_distance*sinf(flare_heading/180.*M_PI);
		int32_t y = flare_distance*cosf(flare_heading/180.*M_PI);
		int32_t z = flare_height; // height above which we want to be followed
		nav_move_waypoint(WP_FLARE, utm.east + x, utm.north + y, utm.alt + z);

		// Touchdown waypoint
		nav_move_waypoint(WP_TD, utm.east, utm.north, utm.alt);

		// AF waypoint
		z = follow_me_height - flare_height + utm.alt;
		int32_t d = land_hspeed*z/land_vspeed + flare_distance;
		x = d*sinf(flare_heading/180.*M_PI);
		y = d*cosf(flare_heading/180.*M_PI);
		nav_move_waypoint(WP_AF, utm.east + x, utm.north + y, follow_me_height + utm.alt);
}


