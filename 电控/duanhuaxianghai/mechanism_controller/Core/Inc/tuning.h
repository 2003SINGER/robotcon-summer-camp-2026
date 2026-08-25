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
  float target_tolerance_counts;
  float target_speed_tolerance_rpm;
} MotorProfile;

/* Read-only factory defaults. Runtime writes are mediated by mechanism.c. */
const MotorProfile *Tuning_GetMotorProfile(TuningMotorId id);
float Tuning_GetRotatorCountsPerDegree(void);

#endif
