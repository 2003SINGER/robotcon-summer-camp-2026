#include "tuning.h"

#define M2006_CURRENT_LIMIT 4000.0f
#define GM6020_CURRENT_LIMIT 20000.0f

#define PID_CONFIG(kp_value, ki_value, kd_value, tau_value, integral_limit_value, output_limit_value) \
  {kp_value, ki_value, kd_value, tau_value, 0.0f, 0.0f, 0.0f, integral_limit_value, output_limit_value, false, {0}}
#define POSITION_PID(kp_value, output_limit_value) \
  PID_CONFIG(kp_value, 0.0f, 0.0f, 0.010f, 2000.0f, output_limit_value)
#define VELOCITY_PID \
  PID_CONFIG(0.60f, 80.0f, 0.0f, 0.005f, 50.0f, M2006_CURRENT_LIMIT)

#define M2006_TRAJECTORY_VELOCITY 163840.0f
#define M2006_TRAJECTORY_ACCELERATION 546000.0f
#define GM6020_TRAJECTORY_VELOCITY 27300.0f
#define GM6020_TRAJECTORY_ACCELERATION 49200.0f
#define ENCODER_COUNTS_PER_REVOLUTION 8192.0f

/*
 * The only source-edit tuning table. CAN IDs and signs are boot-time
 * properties; change them here, rebuild and reflash. Runtime tuning APIs only
 * permit PID, current limit, feedforward and completion tolerances.
 */
static const MotorProfile motor_profiles[TUNING_MOTOR_COUNT] = {
  [TUNING_MOTOR_LOADER_A] = {
    "loader_a",
    {MOTOR_KIND_M2006, MOTOR_CAN1, 0x201U, 1, 1, M2006_CURRENT_LIMIT,
     POSITION_PID(0.05f, 1600.0f), VELOCITY_PID,
     M2006_TRAJECTORY_VELOCITY, M2006_TRAJECTORY_ACCELERATION},
    0.0f, 1000.0f, 30.0f
  },
  [TUNING_MOTOR_LOADER_B] = {
    "loader_b",
    {MOTOR_KIND_M2006, MOTOR_CAN1, 0x202U, 1, 1, M2006_CURRENT_LIMIT,
     POSITION_PID(0.05f, 1600.0f), VELOCITY_PID,
     M2006_TRAJECTORY_VELOCITY, M2006_TRAJECTORY_ACCELERATION},
    0.0f, 1000.0f, 30.0f
  },
  [TUNING_MOTOR_LIFT] = {
    "lift",
    /* The current Z-axis project receives 0x201 feedback on its dedicated
     * bus.  It must remain isolated from loader_a, which is also 0x201. */
    {MOTOR_KIND_M2006, MOTOR_CAN2, 0x201U, 1, 1, M2006_CURRENT_LIMIT,
     POSITION_PID(0.05f, 1600.0f), VELOCITY_PID,
     M2006_TRAJECTORY_VELOCITY, M2006_TRAJECTORY_ACCELERATION},
    755.0f, 1000.0f, 30.0f
  },
  [TUNING_MOTOR_ROTATOR] = {
    "rotator",
    {MOTOR_KIND_GM6020, MOTOR_CAN2, 0x206U, 1, 1, GM6020_CURRENT_LIMIT,
     POSITION_PID(2.63f, GM6020_CURRENT_LIMIT),
     PID_CONFIG(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
     GM6020_TRAJECTORY_VELOCITY, GM6020_TRAJECTORY_ACCELERATION},
    0.0f, 1000.0f, 30.0f
  }
};

static const MotionProfile motion_profile = {
  ENCODER_COUNTS_PER_REVOLUTION * 20.0f * 38.0f,
  ENCODER_COUNTS_PER_REVOLUTION * 20.0f * 35.0f,
  ENCODER_COUNTS_PER_REVOLUTION * 123.2f,
  ENCODER_COUNTS_PER_REVOLUTION * (123.2f + 81.8f),
  ENCODER_COUNTS_PER_REVOLUTION * 123.2f,
  ENCODER_COUNTS_PER_REVOLUTION * (123.2f - 72.0f),
  ENCODER_COUNTS_PER_REVOLUTION * (123.2f - 72.0f + 108.0f),
  ENCODER_COUNTS_PER_REVOLUTION / 360.0f
};

const MotorProfile *Tuning_GetMotorProfile(TuningMotorId id)
{
  return id < TUNING_MOTOR_COUNT ? &motor_profiles[id] : 0;
}

const MotionProfile *Tuning_GetMotionProfile(void)
{
  return &motion_profile;
}
