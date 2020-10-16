/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *               2019 Tom van Dijk <tomvand@users.noreply.github.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include <sys/stat.h>
#include <time.h>
#include "autopilot.h"
#include <unistd.h>
#include "std.h"
#include "modules/ctrl/follow_me.h"
#include "subsystems/gps.h"
#include "firmwares/fixedwing/guidance/energy_ctrl.h"
#include "inter_mcu.h"


#include "subsystems/imu.h"
#ifdef COMMAND_THRUST
#include "firmwares/rotorcraft/stabilization.h"
#else
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#endif

#include "mcu_periph/sys_time.h"
#include "state.h"
#include "generated/airframe.h"
#ifdef COMMAND_THRUST
#include "firmwares/rotorcraft/stabilization.h"
#else
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#endif


/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;


/** Logging functions */
/** Start the file logger and open a new file */
void file_logger_start(void)
{
  // Create output folder if necessary
  if (access(STRINGIFY(FILE_LOGGER_PATH), F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", STRINGIFY(FILE_LOGGER_PATH));
    if (system(save_dir_cmd) != 0) {
      printf("[file_logger] Could not create log file directory %s.\n", STRINGIFY(FILE_LOGGER_PATH));
      return;
    }
  }

  // Get current date/time for filename
  char date_time[80];
  time_t now = time(0);
  struct tm  tstruct;
  tstruct = *localtime(&now);
  strftime(date_time, sizeof(date_time), "%Y%m%d-%H%M%S", &tstruct);

  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(FILE_LOGGER_PATH), date_time);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(FILE_LOGGER_PATH), date_time, counter);
    counter++;
  }

  file_logger = fopen(filename, "w");

  if (file_logger != NULL) {
    fprintf(
      file_logger,

	  //rotorcraft uses COMMAND_THRUST, fixedwing COMMAND_THROTTLE at this time
#ifdef COMMAND_THRUST
      "counter,gyro_unscaled_p,gyro_unscaled_q,gyro_unscaled_r,accel_unscaled_x,accel_unscaled_y,accel_unscaled_z,mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,COMMAND_THRUST,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,qi,qx,qy,qz\n"
#else
      "counter,gyro_p,gyro_q,gyro_r,accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,h_ctl_aileron_setpoint,h_ctl_elevator_setpoint,ground_utm.east,ground_utm.north,ground_utm.alt,dist_wp_follow.x,dist_wp_follow.y,dist_wp_follow.z,pos_Utm->east,pos_Utm->north,pos_Utm->alt,wind->x,wind->y,wind->z,airspeed,GPS state aircraft,v_ctl_auto_airspeed_setpoint,ap_mode,follow_me_height,follow_me_altitude,follow_me_heading,dist_wp_follow2.x,dist_wp_follow2.y,dist_wp_follow2.z,follow_me_roll,h_ctl_roll_setpoint_follow_me,roll,yaw,theta,radio_pitch,radio_roll,radio_yaw,stationary_ground,throttle\n"
#endif
    );
  }

  printf("[file_logger] Start logging to %s...\n", filename);

  // Removed header writing
  // file_logger_write_header(file_logger);
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
    }
}

/** Log the values to a csv file    */
/** Change the Variable that you are interested in here */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;
  struct UtmCoor_f *pos_Utm = stateGetPositionUtm_f();
  struct FloatVect3 *wind = stateGetWindspeed_f();
  float airspeed = stateGetAirspeed_f();
  struct FloatEulers *attitude  = stateGetNedToBodyEulers_f();
  int16_t radio_yaw = imcu_get_radio(RADIO_YAW);                          \
  int16_t radio_pitch = imcu_get_radio(RADIO_PITCH);                      \
  int16_t radio_roll =  imcu_get_radio(RADIO_ROLL);
  int16_t radio_throttle = imcu_get_radio(RADIO_THROTTLE);


#ifdef COMMAND_THRUST //For example rotorcraft
  fprintf(file_logger, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
          counter,
          imu.gyro_unscaled.p,
          imu.gyro_unscaled.q,
          imu.gyro_unscaled.r,
          imu.accel_unscaled.x,
          imu.accel_unscaled.y,
          imu.accel_unscaled.z,
          imu.mag_unscaled.x,
          imu.mag_unscaled.y,
          imu.mag_unscaled.z,
          stabilization_cmd[COMMAND_THRUST],
          stabilization_cmd[COMMAND_ROLL],
          stabilization_cmd[COMMAND_PITCH],
          stabilization_cmd[COMMAND_YAW],
          quat->qi,
          quat->qx,
          quat->qy,
          quat->qz
         );
#else  // For fixedwing    IMU stuff:
                           // add after first d:  ,%d,%d,%d,%d,%d,%d,%d,%d,%d
  fprintf(file_logger, "%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f,%d,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d\n",
          counter, // int 1
		  // imu.gyro.p, // int 2
		  // imu.gyro.q, // int3
		  // imu.gyro.r, // int 4
		  // imu.accel.x, // int 5
		  // imu.accel.y, // int 6
		  // imu.accel.z, // int 7
		  // imu.mag.x, // int 8
		  // imu.mag.y, // int 9
	      // imu.mag.z, // int 10

		  h_ctl_aileron_setpoint, // int 11
		  h_ctl_elevator_setpoint, // int 12
		  ground_utm.east, // float 13
		  ground_utm.north, // float 14
		  ground_utm.alt, // float 15
		  dist_wp_follow.x, // float 16
		  dist_wp_follow.y, // float 17
		  dist_wp_follow.z, // float 18
		  pos_Utm->east, // float 19
		  pos_Utm->north, // float 20
		  pos_Utm->alt, // float 21
		  wind->x, // float 22
		  wind->y, // float 23
		  wind->z, // float 24
		  airspeed, //float 25
          gps.fix, // int GPS state aircraft 26
		  v_ctl_auto_airspeed_setpoint, // float 27
		  autopilot.mode, //int 28
		  follow_me_height, // int 29
		  follow_me_altitude, // float 30
		  follow_me_heading, // float 31
		  dist_wp_follow2.x, // float 32
		  dist_wp_follow2.y, // float 33
		  dist_wp_follow2.z, // float 34
		  follow_me_roll, // int 35
		  h_ctl_roll_setpoint_follow_me, // float 36
		  attitude->phi, // float 37
		  attitude->psi, // float 38
 		  attitude->theta, // float 39
		  radio_pitch, // int16 40
		  radio_roll, // int16 41
		  radio_yaw, // int16 42
		  radio_throttle,
		  stationary_ground, //uint8_t 43
		  autopilot.throttle // 44
         );
#endif
  counter++;
}
