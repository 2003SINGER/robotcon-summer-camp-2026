#ifndef TUNING_H
#define TUNING_H

#include <stdint.h>
#include "motor.h"

/* These IDs are stable public names for the mechanism-board motors. */
typedef enum {
  TUNING_MOTOR_LOADER_A = 0,
  TUNING_MOTOR_LOADER_B,
  TUNING_MOTOR_LIFT,
  TUNING_MOTOR_ROTATOR,
  TUNING_MOTOR_COUNT
} TuningMotorId;

typedef struct {
  const char *name;
  MotorConfig motor;
  float default_feedforward_current;
  float default_directional_feedforward_current;
  float target_tolerance_counts;
  float target_speed_tolerance_rpm;
} MotorProfile;

/* Safety bounds are independent from task waypoints in robot_fsm.c.  Values
 * are conservative provisional limits and must be verified against hardware. */
typedef struct {
  bool has_soft_limits;
  float min_counts;
  float max_counts;
  uint32_t motion_timeout_ms;
  int16_t stall_speed_rpm;
  int16_t stall_current;
  uint32_t stall_duration_ms;
} MotionSafetyProfile;

/* Read-only factory defaults. Runtime writes are mediated by mechanism.c. */
const MotorProfile *Tuning_GetMotorProfile(TuningMotorId id);
float Tuning_GetRotatorCountsPerDegree(void);
/* M2006 rotor encoder counts per centimetre of Z-axis belt travel. */
float Tuning_GetLiftCountsPerCm(void);
const MotionSafetyProfile *Tuning_GetMotionSafetyProfile(TuningMotorId id);

#endif
