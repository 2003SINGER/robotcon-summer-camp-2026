#include "tuning.h"
#include "board_config.h"

#define M2006_CURRENT_LIMIT 4000.0f
#if Z_AXIS_BENCH_MODE
#define LIFT_CURRENT_LIMIT 800.0f
#else
#define LIFT_CURRENT_LIMIT M2006_CURRENT_LIMIT
#endif
#define GM6020_CURRENT_LIMIT 20000.0f

#define PID_CONFIG(kp_value, ki_value, kd_value, tau_value, integral_limit_value, output_limit_value) \
  {kp_value, ki_value, kd_value, tau_value, 0.0f, 0.0f, 0.0f, integral_limit_value, output_limit_value, false, {0}}
#define POSITION_PID(kp_value, output_limit_value) \
  PID_CONFIG(kp_value, 0.0f, 0.0f, 0.010f, 2000.0f, output_limit_value)
#define VELOCITY_PID \
  PID_CONFIG(0.60f, 80.0f, 0.0f, 0.005f, 50.0f, M2006_CURRENT_LIMIT)

#define M2006_TRAJECTORY_VELOCITY 163840.0f
#define M2006_TRAJECTORY_ACCELERATION 546000.0f
/* Z axis starts deliberately below the horizontal loader profile.  Tune it
 * upward only after direction, gravity feed-forward and soft travel are
 * verified on the real mechanism. */
#define M2006_LIFT_TRAJECTORY_VELOCITY 98304.0f
#define M2006_LIFT_TRAJECTORY_ACCELERATION 327600.0f
#define GM6020_TRAJECTORY_VELOCITY 27300.0f
#define GM6020_TRAJECTORY_ACCELERATION 49200.0f
#define ROTATOR_COUNTS_PER_DEGREE (8192.0f / 360.0f)
#define ENCODER_COUNTS_PER_REVOLUTION 8192.0f

/*
 * The only source-edit tuning table. CAN IDs and signs are boot-time
 * properties; change them here, rebuild and reflash. Runtime tuning APIs only
 * permit PID, current limit, feedforward and completion tolerances.
 */
static const MotorProfile motor_profiles[TUNING_MOTOR_COUNT] = {
  [TUNING_MOTOR_LOADER_A] = {
    "loader_a",
    {MOTOR_KIND_M2006, 0x201U, 1, 1, M2006_CURRENT_LIMIT,
     POSITION_PID(0.05f, 1600.0f), VELOCITY_PID,
     M2006_TRAJECTORY_VELOCITY, M2006_TRAJECTORY_ACCELERATION},
    0.0f, 1000.0f, 30.0f
  },
  [TUNING_MOTOR_LOADER_B] = {
    "loader_b",
    {MOTOR_KIND_M2006, 0x202U, 1, 1, M2006_CURRENT_LIMIT,
     POSITION_PID(0.05f, 1600.0f), VELOCITY_PID,
     M2006_TRAJECTORY_VELOCITY, M2006_TRAJECTORY_ACCELERATION},
    0.0f, 1000.0f, 30.0f
  },
  [TUNING_MOTOR_LIFT] = {
    "lift",
    /* Shared-CAN allocation: C610 ID 3 (feedback 0x203). Set the physical
     * Z-axis C610 to ID 3 with its Set button before arming this firmware. */
    {MOTOR_KIND_M2006, 0x203U, 1, 1, LIFT_CURRENT_LIMIT,
     POSITION_PID(0.05f, 1600.0f), VELOCITY_PID,
     M2006_LIFT_TRAJECTORY_VELOCITY, M2006_LIFT_TRAJECTORY_ACCELERATION},
    0.0f, 1000.0f, 30.0f
  },
  [TUNING_MOTOR_ROTATOR] = {
    "rotator",
    {MOTOR_KIND_GM6020, 0x206U, 1, 1, GM6020_CURRENT_LIMIT,
     POSITION_PID(2.63f, GM6020_CURRENT_LIMIT),
     PID_CONFIG(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
     GM6020_TRAJECTORY_VELOCITY, GM6020_TRAJECTORY_ACCELERATION},
    0.0f, 1000.0f, 30.0f
  }
};

static const MotionSafetyProfile motion_safety_profiles[TUNING_MOTOR_COUNT] = {
  [TUNING_MOTOR_LOADER_A] = {true, 0.0f, ENCODER_COUNTS_PER_REVOLUTION * 20.0f * 40.0f, 6000U, 10, 3000, 400U},
  [TUNING_MOTOR_LOADER_B] = {true, 0.0f, ENCODER_COUNTS_PER_REVOLUTION * 20.0f * 37.0f, 6000U, 10, 3000, 400U},
  [TUNING_MOTOR_LIFT] = {true, 0.0f, ENCODER_COUNTS_PER_REVOLUTION * 220.0f, 8000U, 10, 3000, 400U},
  [TUNING_MOTOR_ROTATOR] = {false, 0.0f, 0.0f, 4000U, 5, 8000, 400U}
};

const MotorProfile *Tuning_GetMotorProfile(TuningMotorId id)
{
  return id < TUNING_MOTOR_COUNT ? &motor_profiles[id] : 0;
}

float Tuning_GetRotatorCountsPerDegree(void) { return ROTATOR_COUNTS_PER_DEGREE; }
const MotionSafetyProfile *Tuning_GetMotionSafetyProfile(TuningMotorId id)
{
  return id < TUNING_MOTOR_COUNT ? &motion_safety_profiles[id] : 0;
}
