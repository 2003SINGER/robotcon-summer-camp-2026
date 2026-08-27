#include "tuning.h"
#include "board_config.h"

#define M2006_CURRENT_LIMIT 4000.0f
#define LIFT_CURRENT_LIMIT 8000.0f
/* GM6020 accepts a signed voltage command, unit mV, range +/-30000. */
#define GM6020_VOLTAGE_LIMIT_MV 16000.0f
#define ENCODER_COUNTS_PER_REVOLUTION 8192.0f
#define M2006_GEAR_RATIO 36.0f
/* Confirmed hardware: one M2006 output-shaft / belt-pulley revolution moves
 * the Z axis 112 mm. Encoder feedback is measured at the motor rotor. */
#define LIFT_BELT_TRAVEL_MM_PER_OUTPUT_REVOLUTION 112.0f
#define LIFT_COUNTS_PER_CM \
  (ENCODER_COUNTS_PER_REVOLUTION * M2006_GEAR_RATIO / \
   (LIFT_BELT_TRAVEL_MM_PER_OUTPUT_REVOLUTION / 10.0f))
/* Bench calibration from the retracted reference: -1,000,000 encoder counts
 * extended the upper screw by 6.3 cm and the lower screw by 6.1 cm. */
#define LOADER_UPPER_COUNTS_PER_CM (1000000.0f / 6.3f)
#define LOADER_LOWER_COUNTS_PER_CM (1000000.0f / 6.1f)

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

/* Front-loader commissioning envelope.  The prior 1200-rpm trajectory was
 * intentionally conservative.  Ten times would be 12000 rpm, above the
 * M2006's practical 10000-rpm ceiling, so use that ceiling instead.  The
 * acceleration is raised by ten times in the same change; the velocity PI is
 * left unchanged because its output is current, not speed. */
#define M2006_LOADER_TRAJECTORY_VELOCITY \
  (10000.0f * ENCODER_COUNTS_PER_REVOLUTION / 60.0f)
#define M2006_LOADER_TRAJECTORY_ACCELERATION 5460000.0f
#define M2006_LOADER_POSITION_PID POSITION_PID(0.05f, 10000.0f)
/* Preserve the trajectory -> position -> velocity cascade.  8000 rpm is the
 * Z-axis commissioning cruise-speed cap, expressed in encoder counts/s. */
#define M2006_LIFT_TRAJECTORY_VELOCITY \
  (8000.0f * ENCODER_COUNTS_PER_REVOLUTION / 60.0f)
/* Keep the 10000-rpm cruise target, but let the real Z axis follow the
 * virtual trajectory instead of entering its braking phase far ahead. */
#define M2006_LIFT_TRAJECTORY_ACCELERATION 2000000.0f
/* GM6020 initial bench envelope: 90 rpm = 12288 encoder counts/s.  This is
 * deliberately gentle for a non-centred KFS; raise only after empty-axis
 * direction, braking and holding tests are clean. */
#define GM6020_TRAJECTORY_VELOCITY (90.0f * ENCODER_COUNTS_PER_REVOLUTION / 60.0f)
#define GM6020_TRAJECTORY_ACCELERATION 45000.0f
#define ROTATOR_COUNTS_PER_DEGREE (8192.0f / 360.0f)
/* Bench tuning, iteration 2: a firmer position outer loop plus a mostly-P
 * velocity loop.  The former resists hand disturbance; the latter brakes
 * immediately after crossing target instead of waiting for stored I output. */
/* Known-good empty-axis bench baseline: responsive return with no unstable
 * filter phase lag added to the GM6020 speed feedback path. */
#define GM6020_POSITION_PID POSITION_PID(1.00f, 120.0f)
#define GM6020_VELOCITY_PID \
  PID_CONFIG(100.0f, 5.0f, 0.0f, 0.005f, 50.0f, GM6020_VOLTAGE_LIMIT_MV)

/*
 * The only source-edit tuning table. CAN IDs and signs are boot-time
 * properties; change them here, rebuild and reflash. Runtime tuning APIs only
 * permit PID, current limit, feedforward and completion tolerances.
 */
static const MotorProfile motor_profiles[TUNING_MOTOR_COUNT] = {
  [TUNING_MOTOR_LOADER_A] = {
    "loader_a",
    {MOTOR_KIND_M2006, 0x201U, 1, 1, M2006_CURRENT_LIMIT,
     M2006_LOADER_POSITION_PID, VELOCITY_PID,
     M2006_LOADER_TRAJECTORY_VELOCITY, M2006_LOADER_TRAJECTORY_ACCELERATION},
    /* 3000 counts is about 0.19 mm at this screw's measured calibration. */
    0.0f, 0.0f, 3000.0f, 50.0f, 120U
  },
  [TUNING_MOTOR_LOADER_B] = {
    "loader_b",
    {MOTOR_KIND_M2006, 0x202U, 1, 1, M2006_CURRENT_LIMIT,
     M2006_LOADER_POSITION_PID, VELOCITY_PID,
     M2006_LOADER_TRAJECTORY_VELOCITY, M2006_LOADER_TRAJECTORY_ACCELERATION},
    /* 3000 counts is about 0.18 mm at this screw's measured calibration. */
    0.0f, 0.0f, 3000.0f, 50.0f, 120U
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
    /* 2000 counts is about 0.76 mm of Z-axis belt travel. */
    628.0f, 1100.0f, 2000.0f, 50.0f, 150U
  },
  [TUNING_MOTOR_ROTATOR] = {
    "rotator",
    {MOTOR_KIND_GM6020, 0x206U, 1, 1, GM6020_VOLTAGE_LIMIT_MV,
     GM6020_POSITION_PID, GM6020_VELOCITY_PID,
     GM6020_TRAJECTORY_VELOCITY, GM6020_TRAJECTORY_ACCELERATION},
    /* GM6020 is direct-drive: 68 counts is 3 degrees, unlike an M2006. */
    0.0f, 0.0f, 68.0f, 10.0f, 180U
  }
};

static const MotionSafetyProfile motion_safety_profiles[TUNING_MOTOR_COUNT] = {
  /* Software zero is the fully retracted position. Positive feedback is a
   * retraction direction, so valid extension travel is negative. */
  [TUNING_MOTOR_LOADER_A] = {true, -41.29f * LOADER_UPPER_COUNTS_PER_CM, 0.0f, 6000U, 10, 3000, 400U},
  [TUNING_MOTOR_LOADER_B] = {true, -36.98f * LOADER_LOWER_COUNTS_PER_CM, 0.0f, 6000U, 10, 3000, 400U},
  [TUNING_MOTOR_LIFT] = {true, 0.0f, ENCODER_COUNTS_PER_REVOLUTION * 220.0f, 8000U, 10, 3000, 400U},
  [TUNING_MOTOR_ROTATOR] = {false, 0.0f, 0.0f, 4000U, 5, 8000, 400U}
};

const MotorProfile *Tuning_GetMotorProfile(TuningMotorId id)
{
  return id < TUNING_MOTOR_COUNT ? &motor_profiles[id] : 0;
}

float Tuning_GetRotatorCountsPerDegree(void) { return ROTATOR_COUNTS_PER_DEGREE; }
float Tuning_GetLiftCountsPerCm(void) { return LIFT_COUNTS_PER_CM; }
float Tuning_GetLoaderUpperCountsPerCm(void) { return LOADER_UPPER_COUNTS_PER_CM; }
float Tuning_GetLoaderLowerCountsPerCm(void) { return LOADER_LOWER_COUNTS_PER_CM; }
const MotionSafetyProfile *Tuning_GetMotionSafetyProfile(TuningMotorId id)
{
  return id < TUNING_MOTOR_COUNT ? &motion_safety_profiles[id] : 0;
}
