#include "mechanism.h"
#include "board_config.h"
#include "can_bus.h"
#include "motor.h"

static Motor loader_a;
static Motor loader_b;
static Motor lift;
static Motor rotator;
static volatile MechanismMode mode = MECHANISM_MODE_DISARMED;
static volatile MechanismFault fault = MECHANISM_FAULT_NONE;

static const MotorConfig loader_a_config = {
  MOTOR_KIND_M2006, CAN_ID_LOADER_MOTOR_A, LOADER_A_FEEDBACK_SIGN, LOADER_A_CURRENT_SIGN, M2006_CURRENT_LIMIT,
  {0.05f, 0.0f, 0.0f, 0.0f, 0.0f, 2000.0f, 1600.0f},
  {0.60f, 80.0f, 0.0f, 0.0f, 0.0f, 50.0f, M2006_CURRENT_LIMIT}
};
static const MotorConfig loader_b_config = {
  MOTOR_KIND_M2006, CAN_ID_LOADER_MOTOR_B, LOADER_B_FEEDBACK_SIGN, LOADER_B_CURRENT_SIGN, M2006_CURRENT_LIMIT,
  {0.05f, 0.0f, 0.0f, 0.0f, 0.0f, 2000.0f, 1600.0f},
  {0.60f, 80.0f, 0.0f, 0.0f, 0.0f, 50.0f, M2006_CURRENT_LIMIT}
};
static const MotorConfig lift_config = {
  MOTOR_KIND_M2006, CAN_ID_LIFT_MOTOR, LIFT_FEEDBACK_SIGN, LIFT_CURRENT_SIGN, M2006_CURRENT_LIMIT,
  {0.05f, 0.0f, 0.0f, 0.0f, 0.0f, 2000.0f, 1600.0f},
  {0.60f, 80.0f, 0.0f, 0.0f, 0.0f, 50.0f, M2006_CURRENT_LIMIT}
};
static const MotorConfig rotator_config = {
  MOTOR_KIND_GM6020, CAN_ID_ROTATOR_MOTOR, ROTATOR_FEEDBACK_SIGN, ROTATOR_CURRENT_SIGN, GM6020_CURRENT_LIMIT,
  /* Converted from the source's degree-based proportional controller; tune on hardware. */
  {2.63f, 0.0f, 0.0f, 0.0f, 0.0f, 2000.0f, GM6020_CURRENT_LIMIT},
  {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

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
  Motor_Init(&loader_a, &loader_a_config);
  Motor_Init(&loader_b, &loader_b_config);
  Motor_Init(&lift, &lift_config);
  Motor_Init(&rotator, &rotator_config);
  mode = MECHANISM_MODE_DISARMED;
  fault = MECHANISM_FAULT_NONE;
}

void Mechanism_OnCanFeedback(uint16_t identifier, const uint8_t data[8], uint32_t now_ms)
{
  if (identifier == CAN_ID_LOADER_MOTOR_A) Motor_OnFeedback(&loader_a, data, now_ms);
  else if (identifier == CAN_ID_LOADER_MOTOR_B) Motor_OnFeedback(&loader_b, data, now_ms);
  else if (identifier == CAN_ID_LIFT_MOTOR) Motor_OnFeedback(&lift, data, now_ms);
  else if (identifier == CAN_ID_ROTATOR_MOTOR) Motor_OnFeedback(&rotator, data, now_ms);
}

bool Mechanism_Arm(uint32_t now_ms)
{
  if (mode == MECHANISM_MODE_FAULT || !IsFresh(now_ms)) return false;
  Motor_HoldCurrentPosition(&loader_a);
  Motor_HoldCurrentPosition(&loader_b);
  Motor_HoldCurrentPosition(&lift);
  Motor_HoldCurrentPosition(&rotator);
  lift.feedforward_current = LIFT_GRAVITY_CURRENT;
  rotator.feedforward_current = ROTATOR_GRAVITY_CURRENT;
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
  CanBus_SendM2006Currents(loader_a_current, loader_b_current, lift_current);
  CanBus_SendGM6020Current(rotator_current);
}

void Mechanism_Service(uint32_t now_ms)
{
  if (mode == MECHANISM_MODE_READY && !IsFresh(now_ms)) {
    Mechanism_EStop(MECHANISM_FAULT_FEEDBACK_TIMEOUT);
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

bool Mechanism_TurnRotatorBy(float motor_degrees)
{
  if (mode != MECHANISM_MODE_READY) return false;
  Motor_SetTargetCounts(&rotator, rotator.target_counts + motor_degrees * (8192.0f / 360.0f));
  return true;
}

bool Mechanism_MoveLoaderTo(float motor_a_counts, float motor_b_counts)
{
  if (mode != MECHANISM_MODE_READY) return false;
  Motor_SetTargetCounts(&loader_a, motor_a_counts);
  Motor_SetTargetCounts(&loader_b, motor_b_counts);
  return true;
}

bool Mechanism_MoveLoaderOut(void)
{
  return Mechanism_MoveLoaderTo((float)LOADER_A_OUT_COUNTS, (float)LOADER_B_OUT_COUNTS);
}

bool Mechanism_RetractLoader(void) { return Mechanism_MoveLoaderTo(0.0f, 0.0f); }

static bool IsAtTarget(const Motor *motor)
{
  return Absolute(motor->target_counts - (float)motor->total_counts) < 1000.0f &&
         Absolute((float)motor->speed_rpm) < 30.0f;
}

bool Mechanism_IsLiftAtTarget(void) { return IsAtTarget(&lift); }
bool Mechanism_IsRotatorAtTarget(void) { return IsAtTarget(&rotator); }
bool Mechanism_IsLoaderAtTarget(void) { return IsAtTarget(&loader_a) && IsAtTarget(&loader_b); }
