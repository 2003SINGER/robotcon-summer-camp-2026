#include "tuning.h"

#define M2006_CURRENT_LIMIT 4000.0f
#define GM6020_CURRENT_LIMIT 20000.0f

#define POSITION_PID(kp_value, output_limit_value) \
  {kp_value, 0.0f, 0.0f, 0.0f, 0.0f, 2000.0f, output_limit_value}
#define VELOCITY_PID \
  {0.60f, 80.0f, 0.0f, 0.0f, 0.0f, 50.0f, M2006_CURRENT_LIMIT}

/*
 * The only source-edit tuning table. CAN IDs and signs are boot-time
 * properties; change them here, rebuild and reflash. Runtime tuning APIs only
 * permit PID, current limit, feedforward and completion tolerances.
 */
static const MotorProfile motor_profiles[TUNING_MOTOR_COUNT] = {
  [TUNING_MOTOR_LOADER_A] = {
    "loader_a",
    {MOTOR_KIND_M2006, 0x201U, 1, 1, M2006_CURRENT_LIMIT,
     POSITION_PID(0.05f, 1600.0f), VELOCITY_PID},
    0.0f, 1000.0f, 30.0f
  },
  [TUNING_MOTOR_LOADER_B] = {
    "loader_b",
    {MOTOR_KIND_M2006, 0x202U, 1, 1, M2006_CURRENT_LIMIT,
     POSITION_PID(0.05f, 1600.0f), VELOCITY_PID},
    0.0f, 1000.0f, 30.0f
  },
  [TUNING_MOTOR_LIFT] = {
    "lift",
    /* 0x203 is provisional: inspect the assembled CAN bus before motion. */
    {MOTOR_KIND_M2006, 0x203U, 1, 1, M2006_CURRENT_LIMIT,
     POSITION_PID(0.05f, 1600.0f), VELOCITY_PID},
    0.0f, 1000.0f, 30.0f
  },
  [TUNING_MOTOR_ROTATOR] = {
    "rotator",
    {MOTOR_KIND_GM6020, 0x206U, 1, 1, GM6020_CURRENT_LIMIT,
     POSITION_PID(2.63f, GM6020_CURRENT_LIMIT),
     {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}},
    0.0f, 1000.0f, 30.0f
  }
};

static const MotionProfile motion_profile = {
  8192.0f * 20.0f * 38.0f,
  8192.0f * 20.0f * 35.0f,
  8192.0f / 360.0f
};

const MotorProfile *Tuning_GetMotorProfile(TuningMotorId id)
{
  return id < TUNING_MOTOR_COUNT ? &motor_profiles[id] : 0;
}

const MotionProfile *Tuning_GetMotionProfile(void)
{
  return &motion_profile;
}
