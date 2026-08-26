#include "mechanism.h"
#include "main.h"
#include "board_config.h"
#include "robot_fsm.h"
#include "can_bus.h"
#include "motor.h"
#include "tuning.h"

static Motor loader_a;
static Motor loader_b;
static Motor lift;
static Motor rotator;
static volatile MechanismMode mode = MECHANISM_MODE_DISARMED;
static volatile MechanismFault fault = MECHANISM_FAULT_NONE;
/* A flip axis has no safe arbitrary hold angle at ARM.  It is enabled only
 * after a deliberate angle command, then remains position-controlled. */
static bool rotator_target_active;
volatile MechanismMotorTelemetry g_mechanism_telemetry[TUNING_MOTOR_COUNT];
volatile uint32_t g_mechanism_mode = MECHANISM_MODE_DISARMED;
volatile uint32_t g_mechanism_fault = MECHANISM_FAULT_NONE;

typedef struct {
  float feedforward_current;
  float directional_feedforward_current;
  float target_tolerance_counts;
  float target_speed_tolerance_rpm;
} RuntimeTuning;

static RuntimeTuning runtime_tuning[TUNING_MOTOR_COUNT];

typedef struct {
  bool active;
  uint32_t started_ms;
  uint32_t stalled_since_ms;
} MotionWatch;

static MotionWatch motion_watch[TUNING_MOTOR_COUNT];

static bool IsAtTarget(TuningMotorId id);

static bool IsMotorActive(TuningMotorId id)
{
#if Z_AXIS_BENCH_MODE
  return id == TUNING_MOTOR_LIFT;
#elif ROTATOR_BENCH_MODE
  return id == TUNING_MOTOR_ROTATOR;
#elif LOADER_BENCH_MODE
  return id == TUNING_MOTOR_LOADER_A || id == TUNING_MOTOR_LOADER_B;
#else
  (void)id;
  return true;
#endif
}

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
  runtime_tuning[id].directional_feedforward_current = profile->default_directional_feedforward_current;
  runtime_tuning[id].target_tolerance_counts = profile->target_tolerance_counts;
  runtime_tuning[id].target_speed_tolerance_rpm = profile->target_speed_tolerance_rpm;
}

static bool IsFresh(uint32_t now_ms)
{
  for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) {
    Motor *motor = MotorFor(id);
    if (IsMotorActive(id) && (motor == 0 || !Motor_IsFeedbackFresh(motor, now_ms, MOTOR_FEEDBACK_TIMEOUT_MS))) return false;
  }
  return true;
}

static float Absolute(float value)
{
  return value < 0.0f ? -value : value;
}

static bool IsWithinSoftLimits(TuningMotorId id, float counts)
{
  const MotionSafetyProfile *safety = Tuning_GetMotionSafetyProfile(id);
  return safety != 0 && (!safety->has_soft_limits ||
                         (counts >= safety->min_counts && counts <= safety->max_counts));
}

static bool SetTarget(TuningMotorId id, Motor *motor, float counts)
{
  if (!IsWithinSoftLimits(id, counts)) {
    Mechanism_EStop(MECHANISM_FAULT_SOFT_LIMIT);
    return false;
  }
  Motor_SetTargetCounts(motor, counts);
  if (id == TUNING_MOTOR_ROTATOR) rotator_target_active = true;
  motion_watch[id] = (MotionWatch){true, HAL_GetTick(), 0U};
  return true;
}

void Mechanism_Init(void)
{
  LoadDefaultProfile(TUNING_MOTOR_LOADER_A);
  LoadDefaultProfile(TUNING_MOTOR_LOADER_B);
  LoadDefaultProfile(TUNING_MOTOR_LIFT);
  LoadDefaultProfile(TUNING_MOTOR_ROTATOR);
  mode = MECHANISM_MODE_DISARMED;
  fault = MECHANISM_FAULT_NONE;
  rotator_target_active = false;
  for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) motion_watch[id] = (MotionWatch){0};
  for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) {
    PublishTelemetry(id);
  }
  g_mechanism_mode = (uint32_t)mode;
  g_mechanism_fault = (uint32_t)fault;
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
  for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) {
    Motor *motor = MotorFor(id);
    if (IsMotorActive(id) && motor != 0) Motor_HoldCurrentPosition(motor);
  }
  lift.feedforward_current = runtime_tuning[TUNING_MOTOR_LIFT].feedforward_current;
  lift.directional_feedforward_current = runtime_tuning[TUNING_MOTOR_LIFT].directional_feedforward_current;
  rotator.feedforward_current = runtime_tuning[TUNING_MOTOR_ROTATOR].feedforward_current;
  rotator_target_active = false;
  for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) motion_watch[id] = (MotionWatch){0};
  mode = MECHANISM_MODE_READY;
  g_mechanism_mode = (uint32_t)mode;
  g_mechanism_fault = (uint32_t)fault;
  return true;
}

void Mechanism_EStop(MechanismFault reason)
{
  rotator_target_active = false;
  mode = MECHANISM_MODE_FAULT;
  fault = reason;
  g_mechanism_mode = (uint32_t)mode;
  g_mechanism_fault = (uint32_t)fault;
}

bool Mechanism_ClearFault(void)
{
  if (mode != MECHANISM_MODE_FAULT) return false;
  mode = MECHANISM_MODE_DISARMED;
  fault = MECHANISM_FAULT_NONE;
  g_mechanism_mode = (uint32_t)mode;
  g_mechanism_fault = (uint32_t)fault;
  return true;
}

void Mechanism_ControlTick(uint32_t now_ms)
{
  int16_t loader_a_current = 0;
  int16_t loader_b_current = 0;
  int16_t lift_current = 0;
  int16_t rotator_current = 0;

  if (mode == MECHANISM_MODE_READY) {
    if (IsMotorActive(TUNING_MOTOR_LOADER_A)) loader_a_current = Motor_ControlStep(&loader_a, now_ms, CONTROL_PERIOD_S);
    if (IsMotorActive(TUNING_MOTOR_LOADER_B)) loader_b_current = Motor_ControlStep(&loader_b, now_ms, CONTROL_PERIOD_S);
    if (IsMotorActive(TUNING_MOTOR_LIFT)) lift_current = Motor_ControlStep(&lift, now_ms, CONTROL_PERIOD_S);
    if (IsMotorActive(TUNING_MOTOR_ROTATOR) && rotator_target_active) {
      rotator_current = Motor_ControlStep(&rotator, now_ms, CONTROL_PERIOD_S);
    }
  }
  /* A C610 may only refresh its feedback after it sees its command group.
   * During single-axis commissioning, send an explicit zero-current keepalive
   * while DISARMED so ARM can verify live feedback.  It commands no torque.
   * Production mode remains silent while DISARMED. */
#if Z_AXIS_BENCH_MODE
  {
    CanBus_SendM2006Currents(loader_a_current, loader_b_current, lift_current);
    if (mode != MECHANISM_MODE_DISARMED) {
      CanBus_SendGM6020Voltage(rotator_current);
    }
  }
#elif LOADER_BENCH_MODE
  /* C610 feedback needs a command-group keepalive even while DISARMED. */
  (void)lift_current;
  (void)rotator_current;
  CanBus_SendM2006Currents(loader_a_current, loader_b_current, 0);
#elif ROTATOR_BENCH_MODE
  /* Some GM6020 firmware revisions do not report feedback after an all-zero
   * command.  A deliberately limited DISARMED probe can solicit it for bench
   * commissioning; READY always uses only the closed-loop output. */
  int16_t probe_voltage_mV = 0;
  (void)loader_a_current;
  (void)loader_b_current;
  (void)lift_current;
  if (mode == MECHANISM_MODE_DISARMED) {
    probe_voltage_mV = g_rotator_bench_probe_voltage_mV;
    if (probe_voltage_mV > 3000) probe_voltage_mV = 3000;
    if (probe_voltage_mV < -3000) probe_voltage_mV = -3000;
  }
  if (mode != MECHANISM_MODE_READY) rotator_current = probe_voltage_mV;
  CanBus_SendGM6020Voltage(rotator_current);
#else
  if (mode != MECHANISM_MODE_DISARMED) {
    CanBus_SendM2006Currents(loader_a_current, loader_b_current, lift_current);
    CanBus_SendGM6020Voltage(rotator_current);
  }
#endif
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
  if (mode == MECHANISM_MODE_READY) {
#if !LOADER_BENCH_MODE
    for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) {
      Motor *motor = MotorFor(id);
      const MotionSafetyProfile *safety = Tuning_GetMotionSafetyProfile(id);
      if (!IsMotorActive(id) || motor == 0 || safety == 0 || !motion_watch[id].active) continue;
      if (IsAtTarget(id)) {
        motion_watch[id] = (MotionWatch){0};
        continue;
      }
      if ((uint32_t)(now_ms - motion_watch[id].started_ms) > safety->motion_timeout_ms) {
        Mechanism_EStop(MECHANISM_FAULT_MOTION_TIMEOUT);
        break;
      }
      if (Absolute(motor->goal_counts - (float)motor->total_counts) > runtime_tuning[id].target_tolerance_counts &&
          Absolute((float)motor->speed_rpm) <= (float)safety->stall_speed_rpm &&
          Absolute((float)motor->measured_current) >= (float)safety->stall_current) {
        if (motion_watch[id].stalled_since_ms == 0U) motion_watch[id].stalled_since_ms = now_ms;
        if ((uint32_t)(now_ms - motion_watch[id].stalled_since_ms) > safety->stall_duration_ms) {
          Mechanism_EStop(MECHANISM_FAULT_STALL);
          break;
        }
      } else {
        motion_watch[id].stalled_since_ms = 0U;
      }
    }
#endif
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
  return SetTarget(TUNING_MOTOR_LIFT, &lift, counts);
}

bool Mechanism_ZeroLiftPosition(void)
{
  if (!lift.has_feedback) return false;
  Motor_ZeroPosition(&lift);
  motion_watch[TUNING_MOTOR_LIFT] = (MotionWatch){0};
  PublishTelemetry(TUNING_MOTOR_LIFT);
  return true;
}

bool Mechanism_ZeroLoaderPositions(void)
{
  if (!loader_a.has_feedback || !loader_b.has_feedback) return false;
  Motor_ZeroPosition(&loader_a);
  Motor_ZeroPosition(&loader_b);
  motion_watch[TUNING_MOTOR_LOADER_A] = (MotionWatch){0};
  motion_watch[TUNING_MOTOR_LOADER_B] = (MotionWatch){0};
  PublishTelemetry(TUNING_MOTOR_LOADER_A);
  PublishTelemetry(TUNING_MOTOR_LOADER_B);
  return true;
}

bool Mechanism_ZeroRotatorPosition(void)
{
  if (!rotator.has_feedback) return false;
  Motor_ZeroPosition(&rotator);
  rotator_target_active = false;
  motion_watch[TUNING_MOTOR_ROTATOR] = (MotionWatch){0};
  PublishTelemetry(TUNING_MOTOR_ROTATOR);
  return true;
}

float Mechanism_LiftCmToCounts(float cm)
{
  return cm * Tuning_GetLiftCountsPerCm();
}

float Mechanism_LiftCountsToCm(float counts)
{
  return counts / Tuning_GetLiftCountsPerCm();
}

bool Mechanism_TurnRotatorTo(float motor_degrees)
{
  if (mode != MECHANISM_MODE_READY) return false;
  return SetTarget(TUNING_MOTOR_ROTATOR, &rotator, motor_degrees * Tuning_GetRotatorCountsPerDegree());
}

float Mechanism_LoaderUpperCountsToCm(float counts)
{
  return -counts / Tuning_GetLoaderUpperCountsPerCm();
}

float Mechanism_LoaderLowerCountsToCm(float counts)
{
  return -counts / Tuning_GetLoaderLowerCountsPerCm();
}

bool Mechanism_MoveLoaderToCm(float upper_cm, float lower_cm)
{
  if (mode != MECHANISM_MODE_READY) return false;
  return SetTarget(TUNING_MOTOR_LOADER_A, &loader_a,
                   -upper_cm * Tuning_GetLoaderUpperCountsPerCm()) &&
         SetTarget(TUNING_MOTOR_LOADER_B, &loader_b,
                   -lower_cm * Tuning_GetLoaderLowerCountsPerCm());
}

bool Mechanism_MoveLoaderBenchToCm(float upper_cm, float lower_cm)
{
  /* A reset can occur with the carriage already extended. In that case raw
   * count zero is not the retracted endpoint, and a legitimate recovery
   * target is negative cm. Bench control therefore bypasses only the loader
   * absolute soft bounds; production Mechanism_MoveLoaderToCm remains bound. */
  float upper_target;
  float lower_target;
  if (mode != MECHANISM_MODE_READY) return false;
  upper_target = -upper_cm * Tuning_GetLoaderUpperCountsPerCm();
  lower_target = -lower_cm * Tuning_GetLoaderLowerCountsPerCm();
  Motor_SetTargetCounts(&loader_a, upper_target);
  Motor_SetTargetCounts(&loader_b, lower_target);
  motion_watch[TUNING_MOTOR_LOADER_A] = (MotionWatch){true, HAL_GetTick(), 0U};
  motion_watch[TUNING_MOTOR_LOADER_B] = (MotionWatch){true, HAL_GetTick(), 0U};
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
  if (loop == MECHANISM_PID_VELOCITY) {
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
  motor->config.velocity_pid.output_limit = current_limit;
  return true;
}

bool Mechanism_SetFeedforward(TuningMotorId id, float current)
{
  if (mode != MECHANISM_MODE_DISARMED || id >= TUNING_MOTOR_COUNT) return false;
  runtime_tuning[id].feedforward_current = current;
  return true;
}

bool Mechanism_SetLiftFeedforwardLive(float current)
{
  if (mode == MECHANISM_MODE_FAULT) return false;
  runtime_tuning[TUNING_MOTOR_LIFT].feedforward_current = current;
  lift.feedforward_current = current;
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
  telemetry->directional_feedforward_current = runtime_tuning[id].directional_feedforward_current;
  telemetry->applied_feedforward_current = motor->applied_feedforward_current;
  telemetry->current_limit = motor->config.current_limit;
  Pid_GetDiagnostics(&motor->config.position_pid, &telemetry->position_pid);
  Pid_GetDiagnostics(&motor->config.velocity_pid, &telemetry->velocity_pid);
  telemetry->last_feedback_ms = motor->last_feedback_ms;
  telemetry->has_feedback = motor->has_feedback;
  return true;
}
