/*
 * This control algorithm is Model Free Control (MFC)
 */

#ifndef STABILIZATION_MFC_H
#define STABILIZATION_MFC_H

#include <inttypes.h>
#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"


struct MfcParameters{
  // Vector that keep the previous values
  float setpoint;
  float setpoint_trajec[3];
  uint8_t use_trajec_sp;
  float measure;
  float error[3];
  float command[2];
  float estimator_num[3];
  float estimator_den[3];
  // Static values
  float time_trajec;
  float int_window;
  float command_filter;
  float alpha;
  float kp;
  float kd;
};

struct Mfc{
  float sample_time;
  float time;
  float start_time;
};

// MFC structs
extern struct Mfc mfc;
extern struct MfcParameters mfc_roll;
extern struct MfcParameters mfc_pitch;
extern struct MfcParameters mfc_yaw;

// PPRZ commands
extern pprz_t mfc_roll_cmd_pprz;  // roll  -> diff thrust
extern pprz_t mfc_pitch_cmd_pprz; // pitch -> elevator
extern pprz_t mfc_yaw_cmd_pprz;   // yaw  -> aileron

// MFC functions
extern void stabilization_mfc_init(void);
extern void stabilization_mfc_init_time_state(void);
extern void init_filters(void);
extern void stabilization_mfc_set_failsafe_setpoint(void);
extern int16_t stabilization_mfc_calc_cmd(struct MfcParameters *mfc_stt, float measure);
extern void stabilization_mfc_run(void);

#endif
