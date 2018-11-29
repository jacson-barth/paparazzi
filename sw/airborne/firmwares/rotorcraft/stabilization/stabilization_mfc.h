/*
 * This control algorithm is Model Free Control (MFC)
 */

#ifndef STABILIZATION_MFC_H
#define STABILIZATION_MFC_H

#include <inttypes.h>
#include "std.h"
#include "paparazzi.h"
#include "generated/airframe.h"

#include "firmwares/rotorcraft/stabilization/stabilization_attitude_common_int.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_ref_quat_int.h"

extern struct Int32Quat   stab_att_sp_quat;  ///< with #INT32_QUAT_FRAC
extern struct Int32Eulers stab_att_sp_euler; ///< with #INT32_ANGLE_FRAC

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
extern pprz_t mfc_elevator_setpoint_pprz;

// MFC functions
extern void stabilization_mfc_init(void);
extern void stabilization_mfc_enter(void);
extern void stabilization_mfc_set_failsafe_setpoint(void);
extern void stabilization_mfc_set_rpy_setpoint_i(struct Int32Eulers *rpy);
extern void stabilization_mfc_set_earth_cmd_i(struct Int32Vect2 *cmd, int32_t heading);
extern void stabilization_mfc_run(bool in_flight);
extern void stabilization_mfc_read_rc(bool in_flight, bool in_carefree, bool coordinated_turn);

//extern void init_filters(void);
//extern int16_t stabilization_mfc_calc_cmd(struct MfcParameters *mfc_stt, float measure);
//extern void stabilization_mfc_run(void);

#endif
