#include "tuning.h"
#include "board_config.h"

#define M2006_CURRENT_LIMIT 4000.0f
#define LIFT_CURRENT_LIMIT 8000.0f
#define GM6020_CURRENT_LIMIT 20000.0f
#define ENCODER_COUNTS_PER_REVOLUTION 8192.0f
#define M2006_GEAR_RATIO 36.0f
/* Confirmed hardware: one M2006 output-shaft / belt-pulley revolution moves
 * the Z axis 112 mm. Encoder feedback is measured at the motor rotor. */
#define LIFT_BELT_TRAVEL_MM_PER_OUTPUT_REVOLUTION 112.0f
#define LIFT_COUNTS_PER_CM \
  (ENCODER_COUNTS_PER_REVOLUTION * M2006_GEAR_RATIO / \
   (LIFT_BELT_TRAVEL_MM_PER_OUTPUT_REVOLUTION / 10.0f))

#define PID_CONFIG(kp_value, ki_value, kd_value, tau_value, integral_limit_value, output_limit_value) \
  {kp_value, ki_value, kd_value, tau_value, 0.0f, 0.0f, 0.0f, integral_limit_value, output_limit_value, false, {0}}
#define POSITION_PID(kp_value, output_limit_value) \
  PID_CONFIG(kp_value, 0.0f, 0.0f, 0.010f, 2000.0f, output_limit_value)
#define VELOCITY_PID \
  PID_CONFIG(0.60f, 80.0f, 0.0f, 0.005f, 50.0f, M2006_CURRENT_LIMIT)

/* Keep the proven control architecture: trajectory -> position P -> velocity
 * PI.  The old standalone test used a different position unit and integration
 * model; importing its I/D terms produced an opposing integral after a drop. */
/* Position PID output is the velocity-loop setpoint (rpm), so its ceiling
 * must match the 8000-rpm Z-axis commissioning cruise cap. */
#define LIFT_POSITION_PID POSITION_PID(0.05f, 8000.0f)
#define LIFT_VELOCITY_PID \
  PID_CONFIG(0.60f, 80.0f, 0.0f, 0.005f, 50.0f, LIFT_CURRENT_LIMIT)

#define M2006_TRAJECTORY_VELOCITY 163840.0f
#define M2006_TRAJECTORY_ACCELERATION 546000.0f
/* Preserve the trajectory -> position -> velocity cascade.  8000 rpm is the
 * Z-axis commissioning cruise-speed cap, expressed in encoder counts/s. */
#define M2006_LIFT_TRAJECTORY_VELOCITY \
  (8000.0f * ENCODER_COUNTS_PER_REVOLUTION / 60.0f)
/* Keep the 10000-rpm cruise target, but let the real Z axis follow the
 * virtual trajectory instead of entering its braking phase far ahead. */
#define M2006_LIFT_TRAJECTORY_ACCELERATION 2000000.0f
#define GM6020_TRAJECTORY_VELOCITY 27300.0f
#define GM6020_TRAJECTORY_ACCELERATION 49200.0f
#define ROTATOR_COUNTS_PER_DEGREE (8192.0f / 360.0f)

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
    0.0f, 0.0f, 1000.0f, 30.0f
  },
  [TUNING_MOTOR_LOADER_B] = {
    "loader_b",
    {MOTOR_KIND_M2006, 0x202U, 1, 1, M2006_CURRENT_LIMIT,
     POSITION_PID(0.05f, 1600.0f), VELOCITY_PID,
     M2006_TRAJECTORY_VELOCITY, M2006_TRAJECTORY_ACCELERATION},
    0.0f, 0.0f, 1000.0f, 30.0f
  },
  [TUNING_MOTOR_LIFT] = {
    "lift",
    /* Shared-CAN allocation: C610 ID 3 (feedback 0x203). Set the physical
     * Z-axis C610 to ID 3 with its Set button before arming this firmware. */
    {MOTOR_KIND_M2006, 0x203U, 1, 1, LIFT_CURRENT_LIMIT,
     LIFT_POSITION_PID, LIFT_VELOCITY_PID,
     M2006_LIFT_TRAJECTORY_VELOCITY, M2006_LIFT_TRAJECTORY_ACCELERATION},
    /* Bench-calibrated common gravity term plus opposite dry-friction terms.
     * The latter is applied only while the position loop requests motion. */
    628.0f, 1100.0f, 1000.0f, 30.0f
  },
  [TUNING_MOTOR_ROTATOR] = {
    "rotator",
    {MOTOR_KIND_GM6020, 0x206U, 1, 1, GM6020_CURRENT_LIMIT,
     POSITION_PID(2.63f, GM6020_CURRENT_LIMIT),
     PID_CONFIG(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f),
     GM6020_TRAJECTORY_VELOCITY, GM6020_TRAJECTORY_ACCELERATION},
    0.0f, 0.0f, 1000.0f, 30.0f
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
float Tuning_GetLiftCountsPerCm(void) { return LIFT_COUNTS_PER_CM; }
const MotionSafetyProfile *Tuning_GetMotionSafetyProfile(TuningMotorId id)
{
  return id < TUNING_MOTOR_COUNT ? &motion_safety_profiles[id] : 0;
}
