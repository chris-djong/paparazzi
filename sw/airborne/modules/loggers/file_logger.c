/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
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
#include "std.h"
#include "modules/ctrl/follow_me.h"
#include "subsystems/gps.h"

#include "subsystems/imu.h"
#ifdef COMMAND_THRUST
#include "firmwares/rotorcraft/stabilization.h"
#else
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#endif

#include "state.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;

/** Start the file logger and open a new file */
void file_logger_start(void)
{
  // check if log path exists
  struct stat s;
  int err = stat(STRINGIFY(FILE_LOGGER_PATH), &s);

  if(err < 0) {
    // try to make the directory
    mkdir(STRINGIFY(FILE_LOGGER_PATH), 0666);
  }

  // Get current date/time, format is YYYY-MM-DD.HH:mm:ss
  char date_time[80];
  time_t now = time(0);
  struct tm  tstruct;
  tstruct = *localtime(&now);
  strftime(date_time, sizeof(date_time), "%Y-%m-%d_%X", &tstruct);

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
      "counter,gyro_unscaled_p,gyro_unscaled_q,gyro_unscaled_r,accel_unscaled_x,accel_unscaled_y,accel_unscaled_z,mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,gyro_p,gyro_q,gyro_r,accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,h_ctl_aileron_setpoint,h_ctl_elevator_setpoint,h_ctl_roll_setpoint_follow_me, dist_wp_follow.x,dist_wp_follow.y,ground_utm.east,ground_utm.north,ground_utm.alt,wp_follow_enu.x,wp_follow_enu.y,wp_follow_enu.z,pos_Utm->east,pos_Utm->north,pos_Utm->alt,wind->x,wind->y,wind->z,airspeed,aoa,sideslip,GPS state aircraft\n"
#endif
    );
  }
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
  float aoa =  stateGetAngleOfAttack_f();
  float sideslip = stateGetSideslip_f();

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
#else  // For fixedwing
  fprintf(file_logger, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,\n",
          counter, // int
          imu.gyro_unscaled.p, // int
          imu.gyro_unscaled.q, // int
          imu.gyro_unscaled.r, // int
          imu.accel_unscaled.x, // int
          imu.accel_unscaled.y, // int
          imu.accel_unscaled.z, // int
          imu.mag_unscaled.x, // int
          imu.mag_unscaled.y, // int
          imu.mag_unscaled.z, // int
		  imu.gyro_unscaled.p, // int
		  imu.gyro.q, // int
		  imu.gyro.r, // int
		  imu.accel.x, // int
		  imu.accel.y, // int
		  imu.accel.z, // int
		  imu.mag.x, // int
		  imu.mag.y, // int
	      imu.mag.z, // int
		  h_ctl_aileron_setpoint, // int
		  h_ctl_elevator_setpoint, // int
		  h_ctl_roll_setpoint_follow_me, // float
		  dist_wp_follow.x, // float
		  dist_wp_follow.y, // float
		  ground_utm.east, // float
		  ground_utm.north, // float
		  ground_utm.alt, // float
		  wp_follow_enu.x, // float
		  wp_follow_enu.y, // float
		  wp_follow_enu.z, // float
		  pos_Utm->east, // float
		  pos_Utm->north, // float
		  pos_Utm->alt, // float
		  wind->x, // float
		  wind->y, // float
		  wind->z, // float
		  airspeed, //float
		  aoa, //float
		  sideslip, // float
          gps.fix // float GPS state aircraft
         );
#endif
  counter++;
}
