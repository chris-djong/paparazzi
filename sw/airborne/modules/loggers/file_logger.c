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
#include "firmwares/fixedwing/autopilot_firmware.h"
#include <unistd.h>
#include "std.h"
#include "modules/ctrl/follow_me.h"
#include "subsystems/gps.h"
#include "firmwares/fixedwing/guidance/energy_ctrl.h"

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

/** Write CSV header
 * Write column names at the top of the CSV file. Make sure that the columns
 * match those in file_logger_write_row! Don't forget the \n at the end of the
 * line.
 * @param file Log file pointer
 */
static void file_logger_write_header(FILE *file) {
  fprintf(file, "time,");
  fprintf(file, "pos_x,pos_y,pos_z,");
  fprintf(file, "vel_x,vel_y,vel_z,");
  fprintf(file, "att_phi,att_theta,att_psi,");
  fprintf(file, "rate_p,rate_q,rate_r,");
#ifdef COMMAND_THRUST
  fprintf(file, "cmd_thrust,cmd_roll,cmd_pitch,cmd_yaw\n");
#else
  fprintf(file, "h_ctl_aileron_setpoint,h_ctl_elevator_setpoint\n");
#endif
}

/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of file_logger_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
static void file_logger_write_row(FILE *file) {
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  fprintf(file, "%f,", get_sys_time_float());
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", att->phi, att->theta, att->psi);
  fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);
#ifdef COMMAND_THRUST
  fprintf(file, "%d,%d,%d,%d\n",
      stabilization_cmd[COMMAND_THRUST], stabilization_cmd[COMMAND_ROLL],
      stabilization_cmd[COMMAND_PITCH], stabilization_cmd[COMMAND_YAW]);
#else
  fprintf(file, "%d,%d\n", h_ctl_aileron_setpoint, h_ctl_elevator_setpoint);
#endif
}


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
      "counter,gyro_unscaled_p,gyro_unscaled_q,gyro_unscaled_r,accel_unscaled_x,accel_unscaled_y,accel_unscaled_z,mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,gyro_p,gyro_q,gyro_r,accel_x,accel_y,accel_z,mag_x,mag_y,mag_z,h_ctl_aileron_setpoint,h_ctl_elevator_setpoint,ground_utm.east,ground_utm.north,ground_utm.alt,dist_wp_follow.x,dist_wp_follow.y,dist_wp_follow.z,pos_Utm->east,pos_Utm->north,pos_Utm->alt,wind->x,wind->y,wind->z,airspeed,aoa,sideslip,GPS state aircraft,v_ctl_auto_airspeed_setpoint,ap_mode,follow_me_height,follow_me_altitude,follow_me_heading,dist_wp_follow2.x,dist_wp_follow2.y,dist_wp_follow2.z,follow_me_roll,h_ctl_roll_setpoint_follow_me\n"
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
  fprintf(file_logger, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%f,%d,%d,%f,%f,%f,%f,%f,%d,%f\n",
          counter, // int 1
          imu.gyro_unscaled.p, // int 2
          imu.gyro_unscaled.q, // int 3
          imu.gyro_unscaled.r, // int 4
          imu.accel_unscaled.x, // int 5
          imu.accel_unscaled.y, // int 6
          imu.accel_unscaled.z, // int 7
          imu.mag_unscaled.x, // int 8
          imu.mag_unscaled.y, // int 9
          imu.mag_unscaled.z, // int 10
		  imu.gyro_unscaled.p, // int 11
		  imu.gyro.q, // int 12
		  imu.gyro.r, // int 13
		  imu.accel.x, // int 14
		  imu.accel.y, // int 15
		  imu.accel.z, // int 16
		  imu.mag.x, // int 17
		  imu.mag.y, // int 18
	      imu.mag.z, // int 19
		  h_ctl_aileron_setpoint, // int 20
		  h_ctl_elevator_setpoint, // int 21
		  ground_utm.east, // float 22
		  ground_utm.north, // float 23
		  ground_utm.alt, // float 24
		  dist_wp_follow.x, // float 25
		  dist_wp_follow.y, // float 26
		  dist_wp_follow.z, // float 27
		  pos_Utm->east, // float 28
		  pos_Utm->north, // float 29
		  pos_Utm->alt, // float 30
		  wind->x, // float 31
		  wind->y, // float 32
		  wind->z, // float 33
		  airspeed, //float 34
		  aoa, //float 35
		  sideslip, // float 36
          gps.fix, // int GPS state aircraft 37
		  v_ctl_auto_airspeed_setpoint, // float 38
		  autopilot.mode, //int 39
		  follow_me_height, // int 40
		  follow_me_altitude, // float 41
		  follow_me_heading, // float 42
		  dist_wp_follow2.x, // float 43
		  dist_wp_follow2.y, // float 44
		  dist_wp_follow2.z, // float 45
		  follow_me_roll, // int 46
		  h_ctl_roll_setpoint_follow_me // float 47
         );
#endif
  counter++;
}
