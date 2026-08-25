#include "mechanism.h"
#include "board_config.h"
#include "can_bus.h"
#include "motor.h"
#include "tuning.h"

static Motor loader_a;
static Motor loader_b;
static Motor lift;
static Motor rotator;
static volatile MechanismMode mode = MECHANISM_MODE_DISARMED;
static volatile MechanismFault fault = MECHANISM_FAULT_NONE;
volatile MechanismMotorTelemetry g_mechanism_telemetry[TUNING_MOTOR_COUNT];

typedef struct {
  float feedforward_current;
  float target_tolerance_counts;
  float target_speed_tolerance_rpm;
} RuntimeTuning;

static RuntimeTuning runtime_tuning[TUNING_MOTOR_COUNT];

static void PublishTelemetry(TuningMotorId id)
{
  MechanismMotorTelemetry snapshot;
  if (Mechanism_GetMotorTelemetry(id, &snapshot)) g_mechanism_telemetry[id] = snapshot;
}

static Motor *MotorFor(TuningMotorId id)
{
  switch (id) {
    case TUNING_MOTOR_LOADER_A: return &loader_a;
    case TUNING_MOTOR_LOADER_B: return &loader_b;
    case TUNING_MOTOR_LIFT: return &lift;
    case TUNING_MOTOR_ROTATOR: return &rotator;
    default: return 0;
  }
}

static void LoadDefaultProfile(TuningMotorId id)
{
  Motor *motor = MotorFor(id);
  const MotorProfile *profile = Tuning_GetMotorProfile(id);
  if (motor == 0 || profile == 0) return;
  Motor_Init(motor, &profile->motor);
  runtime_tuning[id].feedforward_current = profile->default_feedforward_current;
  runtime_tuning[id].target_tolerance_counts = profile->target_tolerance_counts;
  runtime_tuning[id].target_speed_tolerance_rpm = profile->target_speed_tolerance_rpm;
}

static bool IsFresh(uint32_t now_ms)
{
  return Motor_IsFeedbackFresh(&loader_a, now_ms, MOTOR_FEEDBACK_TIMEOUT_MS) &&
         Motor_IsFeedbackFresh(&loader_b, now_ms, MOTOR_FEEDBACK_TIMEOUT_MS) &&
         Motor_IsFeedbackFresh(&lift, now_ms, MOTOR_FEEDBACK_TIMEOUT_MS) &&
         Motor_IsFeedbackFresh(&rotator, now_ms, MOTOR_FEEDBACK_TIMEOUT_MS);
}

static float Absolute(float value)
{
  return value < 0.0f ? -value : value;
}

void Mechanism_Init(void)
{
  LoadDefaultProfile(TUNING_MOTOR_LOADER_A);
  LoadDefaultProfile(TUNING_MOTOR_LOADER_B);
  LoadDefaultProfile(TUNING_MOTOR_LIFT);
  LoadDefaultProfile(TUNING_MOTOR_ROTATOR);
  mode = MECHANISM_MODE_DISARMED;
  fault = MECHANISM_FAULT_NONE;
  for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) {
    PublishTelemetry(id);
  }
}

void Mechanism_OnCanFeedback(uint16_t identifier, const uint8_t data[8], uint32_t now_ms)
{
  for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) {
    Motor *motor = MotorFor(id);
    if (motor != 0 && identifier == motor->config.feedback_id) {
      Motor_OnFeedback(motor, data, now_ms);
      return;
    }
  }
}

bool Mechanism_Arm(uint32_t now_ms)
{
  if (mode == MECHANISM_MODE_FAULT || !IsFresh(now_ms)) return false;
  Motor_HoldCurrentPosition(&loader_a);
  Motor_HoldCurrentPosition(&loader_b);
  Motor_HoldCurrentPosition(&lift);
  Motor_HoldCurrentPosition(&rotator);
  lift.feedforward_current = runtime_tuning[TUNING_MOTOR_LIFT].feedforward_current;
  rotator.feedforward_current = runtime_tuning[TUNING_MOTOR_ROTATOR].feedforward_current;
  mode = MECHANISM_MODE_READY;
  return true;
}

void Mechanism_EStop(MechanismFault reason)
{
  mode = MECHANISM_MODE_FAULT;
  fault = reason;
}

void Mechanism_ControlTick(uint32_t now_ms)
{
  int16_t loader_a_current = 0;
  int16_t loader_b_current = 0;
  int16_t lift_current = 0;
  int16_t rotator_current = 0;

  if (mode == MECHANISM_MODE_READY) {
    loader_a_current = Motor_ControlStep(&loader_a, now_ms, CONTROL_PERIOD_S);
    loader_b_current = Motor_ControlStep(&loader_b, now_ms, CONTROL_PERIOD_S);
    lift_current = Motor_ControlStep(&lift, now_ms, CONTROL_PERIOD_S);
    rotator_current = Motor_ControlStep(&rotator, now_ms, CONTROL_PERIOD_S);
  }
  /* Do not place even zero-current frames on an unverified bench bus. Once
   * armed, a fault still transmits zero current so the motors stop promptly. */
  if (mode != MECHANISM_MODE_DISARMED) {
    CanBus_SendM2006Currents(loader_a_current, loader_b_current, lift_current);
    CanBus_SendGM6020Current(rotator_current);
  }
}

void Mechanism_Service(uint32_t now_ms)
{
  if (mode == MECHANISM_MODE_READY && !IsFresh(now_ms)) {
    Mechanism_EStop(MECHANISM_FAULT_FEEDBACK_TIMEOUT);
  }
  if (mode == MECHANISM_MODE_READY &&
      CanBus_GetConsecutiveTxDropCount() >= CAN_TX_DROP_FAULT_THRESHOLD) {
    Mechanism_EStop(MECHANISM_FAULT_HAL);
  }
  for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) {
    PublishTelemetry(id);
  }
}

MechanismMode Mechanism_GetMode(void) { return mode; }
MechanismFault Mechanism_GetFault(void) { return fault; }

bool Mechanism_MoveLiftTo(float counts)
{
  if (mode != MECHANISM_MODE_READY) return false;
  Motor_SetTargetCounts(&lift, counts);
  return true;
}

bool Mechanism_TurnRotatorTo(float motor_degrees)
{
  if (mode != MECHANISM_MODE_READY) return false;
  Motor_SetTargetCounts(&rotator, motor_degrees * Tuning_GetRotatorCountsPerDegree());
  return true;
}

bool Mechanism_MoveLoaderTo(float motor_a_counts, float motor_b_counts)
{
  if (mode != MECHANISM_MODE_READY) return false;
  Motor_SetTargetCounts(&loader_a, motor_a_counts);
  Motor_SetTargetCounts(&loader_b, motor_b_counts);
  return true;
}

static bool IsAtTarget(TuningMotorId id)
{
  const Motor *motor = MotorFor(id);
  return motor != 0 &&
         Motor_IsTrajectoryComplete(motor) &&
         Absolute(motor->goal_counts - (float)motor->total_counts) < runtime_tuning[id].target_tolerance_counts &&
         Absolute((float)motor->speed_rpm) < runtime_tuning[id].target_speed_tolerance_rpm;
}

bool Mechanism_IsLiftAtTarget(void) { return IsAtTarget(TUNING_MOTOR_LIFT); }
bool Mechanism_IsRotatorAtTarget(void) { return IsAtTarget(TUNING_MOTOR_ROTATOR); }
bool Mechanism_IsLoaderAtTarget(void) { return IsAtTarget(TUNING_MOTOR_LOADER_A) && IsAtTarget(TUNING_MOTOR_LOADER_B); }

bool Mechanism_SetPid(TuningMotorId id, MechanismPidLoop loop,
                      float kp, float ki, float kd,
                      float integral_limit, float output_limit)
{
  Motor *motor = MotorFor(id);
  PidController *pid;
  if (mode != MECHANISM_MODE_DISARMED || motor == 0 || integral_limit < 0.0f || output_limit <= 0.0f) return false;
  pid = loop == MECHANISM_PID_POSITION ? &motor->config.position_pid : &motor->config.velocity_pid;
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral_limit = integral_limit;
  pid->output_limit = output_limit;
  if ((motor->config.kind == MOTOR_KIND_M2006 && loop == MECHANISM_PID_VELOCITY) ||
      (motor->config.kind == MOTOR_KIND_GM6020 && loop == MECHANISM_PID_POSITION)) {
    motor->config.current_limit = output_limit;
  }
  Pid_Reset(pid);
  return true;
}

bool Mechanism_SetDerivativeFilter(TuningMotorId id, MechanismPidLoop loop, float tau_s)
{
  Motor *motor = MotorFor(id);
  PidController *pid;
  if (mode != MECHANISM_MODE_DISARMED || motor == 0 || tau_s < 0.0f) return false;
  pid = loop == MECHANISM_PID_POSITION ? &motor->config.position_pid : &motor->config.velocity_pid;
  pid->derivative_tau_s = tau_s;
  Pid_Reset(pid);
  return true;
}

bool Mechanism_SetCurrentLimit(TuningMotorId id, float current_limit)
{
  Motor *motor = MotorFor(id);
  if (mode != MECHANISM_MODE_DISARMED || motor == 0 || current_limit <= 0.0f) return false;
  motor->config.current_limit = current_limit;
  if (motor->config.kind == MOTOR_KIND_M2006) {
    motor->config.velocity_pid.output_limit = current_limit;
  } else {
    motor->config.position_pid.output_limit = current_limit;
  }
  return true;
}

bool Mechanism_SetFeedforward(TuningMotorId id, float current)
{
  if (mode != MECHANISM_MODE_DISARMED || id >= TUNING_MOTOR_COUNT) return false;
  runtime_tuning[id].feedforward_current = current;
  return true;
}

bool Mechanism_SetTargetTolerance(TuningMotorId id, float counts, float speed_rpm)
{
  if (mode != MECHANISM_MODE_DISARMED || id >= TUNING_MOTOR_COUNT || counts <= 0.0f || speed_rpm < 0.0f) return false;
  runtime_tuning[id].target_tolerance_counts = counts;
  runtime_tuning[id].target_speed_tolerance_rpm = speed_rpm;
  return true;
}

bool Mechanism_SetMotionLimits(TuningMotorId id, float max_velocity_counts_s,
                               float max_acceleration_counts_s2)
{
  Motor *motor = MotorFor(id);
  if (mode != MECHANISM_MODE_DISARMED || motor == 0 ||
      max_velocity_counts_s <= 0.0f || max_acceleration_counts_s2 <= 0.0f) return false;
  motor->config.trajectory_max_velocity_counts_s = max_velocity_counts_s;
  motor->config.trajectory_max_acceleration_counts_s2 = max_acceleration_counts_s2;
  return true;
}

bool Mechanism_RestoreDefaultTuning(void)
{
  if (mode != MECHANISM_MODE_DISARMED) return false;
  for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) LoadDefaultProfile(id);
  return true;
}

bool Mechanism_GetMotorTelemetry(TuningMotorId id, MechanismMotorTelemetry *telemetry)
{
  const Motor *motor = MotorFor(id);
  if (motor == 0 || telemetry == 0) return false;
  telemetry->feedback_id = motor->config.feedback_id;
  telemetry->total_counts = motor->total_counts;
  telemetry->speed_rpm = motor->speed_rpm;
  telemetry->measured_current = motor->measured_current;
  telemetry->goal_counts = motor->goal_counts;
  telemetry->target_counts = motor->target_counts;
  telemetry->trajectory_velocity_counts_s = motor->trajectory_velocity_counts_s;
  telemetry->trajectory_max_velocity_counts_s = motor->config.trajectory_max_velocity_counts_s;
  telemetry->trajectory_max_acceleration_counts_s2 = motor->config.trajectory_max_acceleration_counts_s2;
  telemetry->feedforward_current = runtime_tuning[id].feedforward_current;
  telemetry->current_limit = motor->config.current_limit;
  Pid_GetDiagnostics(&motor->config.position_pid, &telemetry->position_pid);
  Pid_GetDiagnostics(&motor->config.velocity_pid, &telemetry->velocity_pid);
  telemetry->last_feedback_ms = motor->last_feedback_ms;
  telemetry->has_feedback = motor->has_feedback;
  return true;
}
