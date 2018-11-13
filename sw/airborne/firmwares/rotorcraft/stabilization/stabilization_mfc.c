/*
 * Copyright (C) Jacson Barth <jacsonm2@gmail.com>
 * ENAC UAV Lab
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
 */

/** @file stabilization_mfc.c
 * @brief ENAC UAV Lab
 * This control algorithm is Model-Free Control (MFC)
 *
 * This is an implementation of the publication in the
 * International American Control Conference : Full Model-Free 
 * Control Architecture for Hybrid UAVs
 */

#include "firmwares/rotorcraft/stabilization/stabilization_mfc.h"
#include "state.h"
#include "std.h"
#include "generated/airframe.h"
#include "paparazzi.h"
#include "autopilot.h"
#include "mcu_periph/sys_time.h"
#include "subsystems/radio_control.h"
#include "subsystems/actuators.h"
#include <stdio.h>

// Debugging ...
#if PERIODIC_TELEMETRY
#include "subsystems/datalink/telemetry.h"
#endif

#ifndef STABILIZATION_MFC_CONTROL_FREQUENCY
#define STABILIZATION_MFC_CONTROL_FREQUENCY 500
#endif

/* Parameters for roll MFC */

#ifndef STABILIZATION_MFC_ROLL_TIME_TRAJECTORY
#define STABILIZATION_MFC_ROLL_TIME_TRAJECTORY 25.f
#endif

#ifndef STABILIZATION_MFC_ROLL_USE_TRAJECTORY_SP
#define STABILIZATION_MFC_ROLL_USE_TRAJECTORY_SP 1.f
#endif

#ifndef STABILIZATION_MFC_ROLL_INTEGRATION_WINDOW
#define STABILIZATION_MFC_ROLL_INTEGRATION_WINDOW 30.f
#endif

#ifndef STABILIZATION_MFC_ROLL_ALPHA
#define STABILIZATION_MFC_ROLL_ALPHA 0.5f
#endif

#ifndef STABILIZATION_MFC_ROLL_PROPORTIONAL_GAIN
#define STABILIZATION_MFC_ROLL_PROPORTIONAL_GAIN 1.f
#endif

#ifndef STABILIZATION_MFC_ROLL_DERIVATIVE_GAIN
#define STABILIZATION_MFC_ROLL_DERIVATIVE_GAIN 1.f
#endif

#ifndef STABILIZATION_MFC_ROLL_COMMAND_FILTER
#define STABILIZATION_MFC_ROLL_COMMAND_FILTER 1.f
#endif

/* Parameters for pitch MFC */

#ifndef STABILIZATION_MFC_PITCH_TIME_TRAJECTORY
#define STABILIZATION_MFC_PITCH_TIME_TRAJECTORY 25.0f
#endif

#ifndef STABILIZATION_MFC_PITCH_USE_TRAJECTORY_SP
#define STABILIZATION_MFC_PITCH_USE_TRAJECTORY_SP 1.
#endif

#ifndef STABILIZATION_MFC_PITCH_INTEGRATION_WINDOW
#define STABILIZATION_MFC_PITCH_INTEGRATION_WINDOW 30.f
#endif

#ifndef STABILIZATION_MFC_PITCH_ALPHA
#define STABILIZATION_MFC_PITCH_ALPHA 0.25f
#endif

#ifndef STABILIZATION_MFC_PITCH_PROPORTIONAL_GAIN
#define STABILIZATION_MFC_PITCH_PROPORTIONAL_GAIN 1.0f
#endif

#ifndef STABILIZATION_MFC_PITCH_DERIVATIVE_GAIN
#define STABILIZATION_MFC_PITCH_DERIVATIVE_GAIN 1.0f
#endif

#ifndef STABILIZATION_MFC_PITCH_COMMAND_FILTER
#define STABILIZATION_MFC_PITCH_COMMAND_FILTER 1.f
#endif

/* Parameters for yaw MFC */

#ifndef STABILIZATION_MFC_YAW_TIME_TRAJECTORY
#define STABILIZATION_MFC_YAW_TIME_TRAJECTORY 25.f
#endif

#ifndef STABILIZATION_MFC_YAW_USE_TRAJECTORY_SP
#define STABILIZATION_MFC_YAW_USE_TRAJECTORY_SP 1.f
#endif

#ifndef STABILIZATION_MFC_YAW_INTEGRATION_WINDOW
#define STABILIZATION_MFC_YAW_INTEGRATION_WINDOW 30.f
#endif

#ifndef STABILIZATION_MFC_YAW_ALPHA
#define STABILIZATION_MFC_YAW_ALPHA 0.5f
#endif

#ifndef STABILIZATION_MFC_YAW_PROPORTIONAL_GAIN
#define STABILIZATION_MFC_YAW_PROPORTIONAL_GAIN 1.f
#endif

#ifndef STABILIZATION_MFC_YAW_DERIVATIVE_GAIN
#define STABILIZATION_MFC_YAW_DERIVATIVE_GAIN 1.f
#endif

#ifndef STABILIZATION_MFC_YAW_COMMAND_FILTER
#define STABILIZATION_MFC_YAW_COMMAND_FILTER 1.f
#endif

#if LOG_MFC
#include "modules/loggers/sdlog_chibios.h"
bool log_started = false;
#endif

struct Mfc mfc;
struct MfcParameters mfc_roll;
struct MfcParameters mfc_pitch;
struct MfcParameters mfc_yaw;

//#if PERIODIC_TELEMETRY
static void send_mfc_values(struct transport_tx *trans, struct link_device *dev)
{
  //struct Int32Quat *quat = stateGetNedToBodyQuat_i();

  struct FloatEulers *eulers = stateGetNedToBodyEulers_f();

  pprz_msg_send_STAB_ATTITUDE_MFC(trans, dev, AC_ID,
      &mfc.time,
      &mfc.start_time,
      &eulers->phi,
      &eulers->theta,
      &eulers->psi,/*
      &mfc.mfc_roll.setpoint,
      &mfc.mfc_roll.setpoint_trajec[0],
      &mfc.mfc_roll.measure,
      &mfc_aileron_setpoint,
      &mfc_aileron_setpoint_pprz,
      &mfc.mfc_roll.time_trajec,
      &mfc.mfc_roll.int_window,
      &mfc.mfc_roll.alpha,
      &mfc.mfc_roll.kp,
      &mfc.mfc_roll.kd,
      &mfc.mfc_roll.setpoint_trajec[2],
      &mfc.mfc_roll.setpoint_trajec[1],
      &mfc.mfc_roll.setpoint_trajec[0],
      &mfc.mfc_roll.use_trajec_sp,*/
      &mfc_pitch.setpoint,
      &mfc_pitch.setpoint_trajec[0],
      &mfc_pitch.error[0],
      &mfc_pitch.measure,
      &mfc_pitch.command[0],
      &mfc_elevator_setpoint_pprz);
}
//#endif

/**
 * Function that initializes important values upon engaging MFC
 */
void stabilization_mfc_init(void)
{
  init_filters();
  //mfc_parameters.setpoint = mfc_pitch_setpoint;
  //

  mfc.sample_time 		= 1.f / STABILIZATION_MFC_CONTROL_FREQUENCY;

  // ROLL CONTROL PARAMETERS
  mfc_roll.time_trajec 	        = STABILIZATION_MFC_ROLL_TIME_TRAJECTORY;
  mfc_roll.use_trajec_sp    	= STABILIZATION_MFC_ROLL_USE_TRAJECTORY_SP;
  mfc_roll.int_window 	        = STABILIZATION_MFC_ROLL_INTEGRATION_WINDOW;
  mfc_roll.alpha 		= STABILIZATION_MFC_ROLL_ALPHA;
  mfc_roll.command_filter 	= STABILIZATION_MFC_ROLL_COMMAND_FILTER;
  mfc_roll.kp			= STABILIZATION_MFC_ROLL_PROPORTIONAL_GAIN;
  mfc_roll.kd			= STABILIZATION_MFC_ROLL_DERIVATIVE_GAIN;

  // PITCH CONTROL PARAMETERS
  mfc_pitch.time_trajec 	= STABILIZATION_MFC_PITCH_TIME_TRAJECTORY;
  mfc_pitch.use_trajec_sp   	= STABILIZATION_MFC_PITCH_USE_TRAJECTORY_SP; 
  mfc_pitch.int_window 		= STABILIZATION_MFC_PITCH_INTEGRATION_WINDOW;
  mfc_pitch.alpha 		= STABILIZATION_MFC_PITCH_ALPHA;
  mfc_pitch.command_filter 	= STABILIZATION_MFC_PITCH_COMMAND_FILTER;
  mfc_pitch.kp			= STABILIZATION_MFC_PITCH_PROPORTIONAL_GAIN;
  mfc_pitch.kd			= STABILIZATION_MFC_PITCH_DERIVATIVE_GAIN;

  // YAW CONTROL PARAMETERS
  mfc_yaw.time_trajec 	        = STABILIZATION_MFC_YAW_TIME_TRAJECTORY;
  mfc_yaw.use_trajec_sp    	= STABILIZATION_MFC_YAW_USE_TRAJECTORY_SP;
  mfc_yaw.int_window 	        = STABILIZATION_MFC_YAW_INTEGRATION_WINDOW;
  mfc_yaw.alpha 		= STABILIZATION_MFC_YAW_ALPHA;
  mfc_yaw.command_filter 	= STABILIZATION_MFC_YAW_COMMAND_FILTER;
  mfc_yaw.kp			= STABILIZATION_MFC_YAW_PROPORTIONAL_GAIN;
  mfc_yaw.kd			= STABILIZATION_MFC_YAW_DERIVATIVE_GAIN;

  //mfc_pitch.command[0] = 0.0;
  //mfc_pitch.command[1] = 0.0;
  //mfc_pitch.measure  = 0.0;
  //float_vect_zero(mfc_pitch.estimator_num, 3);
  //float_vect_zero(mfc_pitch.estimator_den, 3);
  //float_vect_zero(mfc_pitch.error, 3);

  // Attitude initialization
  //struct FloatEulers *att_eulers = stateGetNedToBodyEulers_f();
  //mfc_roll.setpoint_trajec[2] = att_eulers->phi;
  //mfc_roll.setpoint_trajec[1] = att_eulers->phi;
  //mfc_roll.setpoint_trajec[0] = att_eulers->phi; 

  //mfc_pitch.setpoint_trajec[2] = att_eulers->theta;
  //mfc_pitch.setpoint_trajec[1] = att_eulers->theta;
  //mfc_pitch.setpoint_trajec[0] = att_eulers->theta;

#if PERIODIC_TELEMETRY
  register_periodic_telemetry(DefaultPeriodic, PPRZ_MSG_ID_STAB_ATTITUDE_MFC, send_mfc_values);
#endif
}

/**
 * Function that initializes time and states
 */
void stabilization_mfc_init_time_state(void)
{
  // Time and state initialization every time that MFC is requested
  mfc.start_time = sec_of_sys_time_ticks(sys_time.nb_tick);
	
  // State initializations
  struct FloatEulers *att_eulers = stateGetNedToBodyEulers_f();

  // roll
  mfc_roll.setpoint_trajec[2] = att_eulers->phi;
  mfc_roll.setpoint_trajec[1] = att_eulers->phi;
  mfc_roll.setpoint_trajec[0] = att_eulers->phi;  
  mfc_roll.measure  = att_eulers->phi;
  float_vect_zero(mfc_roll.error, 3);
  float_vect_zero(mfc_roll.estimator_num, 3);
  float_vect_zero(mfc_roll.estimator_den, 3);  
  
  //pitch
  mfc_pitch.setpoint_trajec[2] = att_eulers->theta;
  mfc_pitch.setpoint_trajec[1] = att_eulers->theta;
  mfc_pitch.setpoint_trajec[0] = att_eulers->theta;
  mfc_pitch.measure  = att_eulers->theta;
  float_vect_zero(mfc_pitch.error, 3);
  float_vect_zero(mfc_pitch.estimator_num, 3);
  float_vect_zero(mfc_pitch.estimator_den, 3);
  //mfc_pitch.command[0] = 0.f;

  // yaw
  mfc_yaw.setpoint_trajec[2] = att_eulers->psi;
  mfc_yaw.setpoint_trajec[1] = att_eulers->psi;
  mfc_yaw.setpoint_trajec[0] = att_eulers->psi;  
  mfc_yaw.measure  = att_eulers->psi;
  float_vect_zero(mfc_yaw.error, 3);
  float_vect_zero(mfc_yaw.estimator_num, 3);
  float_vect_zero(mfc_yaw.estimator_den, 3);  
  
}

/**
 * Function that resets the filters to zeros
 */
void init_filters(void)
{
  // Nothing for the moment
}

/**
 * Function calculate the failsafe setpoint
 */
void stabilization_mfc_set_failsafe_setpoint(void)
{
  // Nothing for the moment
}

/**
 * Function that calculate the command with Model-free control for pitch control
 */
int16_t stabilization_mfc_calc_cmd(struct MfcParameters *mfc_stt, float measure)
{
  mfc.time = sec_of_sys_time_ticks(sys_time.nb_tick) - mfc.start_time;
  mfc_stt->measure  = measure;

  // 1) Reference filter
  mfc_stt->setpoint_trajec[0] = (mfc_stt->setpoint + (2.f * mfc_stt->time_trajec*mfc_stt->time_trajec + 2.f * mfc_stt->time_trajec)*
  mfc_stt->setpoint_trajec[1] + (-mfc_stt->time_trajec * mfc_stt->time_trajec)*mfc_stt->setpoint_trajec[2]) / 
  (mfc_stt->time_trajec*mfc_stt->time_trajec + 2.f * mfc_stt->time_trajec + 1.f);

  mfc_stt->error[0] = mfc_stt->measure - mfc_stt->setpoint_trajec[0];
  
  // Set the vectors with previous values
  mfc_stt->setpoint_trajec[2] = mfc_stt->setpoint_trajec[1];
  mfc_stt->setpoint_trajec[1] = mfc_stt->setpoint_trajec[0];
  mfc_stt->estimator_num[2] = mfc_stt->estimator_num[1];
  mfc_stt->estimator_num[1] = mfc_stt->estimator_num[0];
  mfc_stt->estimator_den[2] = mfc_stt->estimator_den[1];
  mfc_stt->estimator_den[1] = mfc_stt->estimator_den[0];
  mfc_stt->error[2] = mfc_stt->error[1];
  mfc_stt->error[1] = mfc_stt->error[0];
  mfc_stt->command[1] = mfc_stt->command[0];


  //float dot_setpoint_trajec = (mfc.setpoint_trajec[0] - mfc.setpoint_trajec[1]) / mfc.sample_time;
  float dot_dot_setpoint_trajec = (mfc_stt->setpoint_trajec[0] - 2.f * mfc_stt->setpoint_trajec[1] + mfc_stt->setpoint_trajec[2]) /
  (mfc.sample_time*mfc.sample_time);

  // 2) Control poles (s-p)^2=s^2+as+b
  float a = -2.f * mfc_stt->kp;
  float b = -mfc_stt->kp * mfc_stt->kp;

  // ddoty = F + alpha * u + a.dote + b.e

  float sde = -(mfc.time*mfc_stt->error[0] - (mfc.time - mfc.sample_time)*mfc_stt->error[1]) / mfc.sample_time;
  float s2d2e = (mfc.time*mfc.time*mfc_stt->error[0] - 2.f * (mfc.time - mfc.sample_time)*(mfc.time - mfc.sample_time)*mfc_stt->error[1] +
  (mfc.time - 2.f * mfc.sample_time)*(mfc.time - 2.f * mfc.sample_time) * mfc_stt->error[2]) / (mfc.sample_time*mfc.sample_time);
  float sd2e = (mfc.time*mfc.time * mfc_stt->error[0] - (mfc.time - mfc.sample_time)*(mfc.time - mfc.sample_time) * mfc_stt->error[1]) /  
  mfc.sample_time;
  float de = -mfc.time * mfc_stt->error[0];
  float d2e = mfc.time * mfc.time * mfc_stt->error[0];
  float d2u = mfc.time * mfc.time * mfc_stt->command[1];

  // d ^ 2 / ds ^ 2 ->
  float num = 2.f * mfc_stt->error[0] + 4.f * sde + s2d2e - a * (2.f * de + sd2e) - b * (d2e)-mfc_stt->alpha * d2u;
  float den = mfc.time * mfc.time;

  // 3) Filter and estimator F
  mfc_stt->estimator_num[0] = (num + (2.f * mfc_stt->int_window*mfc_stt->int_window + 2.f * mfc_stt->int_window)*mfc_stt->estimator_num[1]+
  (-mfc_stt->int_window*mfc_stt->int_window)*mfc_stt->estimator_num[2]) / (mfc_stt->int_window*mfc_stt->int_window + 
  2.f * mfc_stt->int_window + 1.f);
  mfc_stt->estimator_den[0] = (den + (2.f * mfc_stt->int_window*mfc_stt->int_window + 2.f * mfc_stt->int_window)*mfc_stt->estimator_den[1]+
  (-mfc_stt->int_window*mfc_stt->int_window)*mfc_stt->estimator_den[2]) / (mfc_stt->int_window*mfc_stt->int_window + 
  2.f * mfc_stt->int_window + 1.f);

  float F_k = 0.0f;
  if ((mfc_stt->estimator_den[0] != 0.0f) && (mfc.time > 0.1f))
  {
    F_k = mfc_stt->estimator_num[0] / mfc_stt->estimator_den[0];
  }

  // Command generation
  mfc_stt->command[0] = -F_k / mfc_stt->alpha + dot_dot_setpoint_trajec / mfc_stt->alpha;

  // Saturation
  // U_k = max(Umin, U_k);
  // U_k = min(Umax, U_k);

  // Filter of U eventually necessary
  mfc_stt->command[0] = (mfc_stt->command[0] + (mfc_stt->command_filter - 1.f)*mfc_stt->command[1]) / mfc_stt->command_filter;

  if (mfc_stt->command[0] >= 9600.f)
  {
    mfc_stt->command[0] = 9600.f;	
  }
  if (mfc_stt->command[0] <= -9600.f)
  {
    mfc_stt->command[0] = -9600.f;	
  }

  //mfc_stt->command[0] = TRIM_PPRZ(mfc_stt->command[0]);

  int16_t mfc_commnand_pprz = TRIM_PPRZ(mfc_stt->command[0]);
  //int16_t mfc_commnand_pprz = (int16_t) mfc_stt->command[0];
  return mfc_commnand_pprz;
}

void stabilization_mfc_run(void)
{
  // Update angles measurements
  struct FloatEulers *att = stateGetNedToBodyEulers_f();

  // Compute MFC
  int16_t mfc_roll_cmd	=  stabilization_mfc_calc_cmd(&mfc_roll, att->phi);
  int16_t mfc_pitch_cmd	=  stabilization_mfc_calc_cmd(&mfc_pitch, att->theta);
  int16_t mfc_yaw_cmd	=  stabilization_mfc_calc_cmd(&mfc_yaw, att->psi);
  int16_t throttle_cmd  =  radio_control.values[RADIO_THROTTLE] / MAX_PPRZ * STABILIZATION_MFC_MAX_THROTTLE;
  
  //mfc_throttle_setpoint_pprz = stabilization_mfc_calc_cmd(mfc.mfc_throttle);

   
  //                   Mixing MFC commands for attitude stabilization (rotorcraft setup) -- (coded )
  // Elevon left
  actuators_pprz[0] = (int16_t) -mfc_pitch_cmd + mfc_yaw_cmd;
  // Elevon right 
  actuators_pprz[1] = (int16_t) -mfc_pitch_cmd - mfc_yaw_cmd;
  // Motor right
  actuators_pprz[2] = (int16_t) -mfc_roll_cmd;
  // Motor left
  actuators_pprz[3] = (int16_t) -mfc_roll_cmd;
	
/*
#if LOG_MFC
  if (pprzLogFile != -1) {
    if (!log_started) {
      // log file header
      sdLogWriteLog(pprzLogFile,
                    "time start_t phi theta psi theta_sp theta_sp_traj theta_m command_0 elevator_sp_pprz\n");
      log_started = true;
    } else {
      sdLogWriteLog(pprzLogFile,
                    "%.5f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
                    mfc.time,
                    mfc.start_time,
		    att->phi,
		    att->theta,
		    att->psi,
		    mfc_pitch.setpoint,
		    mfc_pitch.setpoint_trajec[0],
		    mfc_pitch.measure,
		    mfc_pitch.command[0]);
    }
  }
#endif

  #define LOG_LENGTH_INT 12
  #define LOG_LENGTH_FLOAT 26

  int32_t sd_buffer_i[LOG_LENGTH_INT] = {0};
  float sd_buffer_f[LOG_LENGTH_FLOAT] = {0};

  #ifndef MFC_BUFFER
    static uint32_t log_counter = 0;
  #define MFC_BUFFER = 1.
  #endif
  struct FloatQuat *quat = stateGetNedToBodyQuat_f();
  struct FloatRates *body_rates_f = stateGetBodyRates_f();
  struct NedCoor_f *accelned = stateGetAccelNed_f();

  sd_buffer_i[0] = log_counter;

  sd_buffer_f[3] = quat->qi;
  sd_buffer_f[4] = quat->qx;
  sd_buffer_f[5] = quat->qy;
  sd_buffer_f[6] = quat->qz;

  sdLogWriteRaw(pprzLogFile, (uint8_t*) sd_buffer_i, LOG_LENGTH_INT*4);
  sdLogWriteRaw(pprzLogFile, (uint8_t*) sd_buffer_f, LOG_LENGTH_FLOAT*4);
  log_counter += 1;

*/
}
