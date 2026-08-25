#include "motor.h"

static int16_t ReadBe16(const uint8_t data[2])
{
  return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static float Clamp(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

void Motor_Init(Motor *motor, const MotorConfig *config)
{
  motor->config = *config;
  motor->raw_angle = 0U;
  motor->speed_rpm = 0;
  motor->measured_current = 0;
  motor->temperature = 0U;
  motor->total_counts = 0;
  motor->last_feedback_ms = 0U;
  motor->has_feedback = false;
  motor->last_raw_angle = 0;
  motor->target_counts = 0.0f;
  motor->feedforward_current = 0.0f;
  Pid_Reset(&motor->config.position_pid);
  Pid_Reset(&motor->config.velocity_pid);
}

void Motor_OnFeedback(Motor *motor, const uint8_t data[8], uint32_t now_ms)
{
  const int16_t raw_angle = ReadBe16(&data[0]);
  int16_t delta;

  motor->raw_angle = (uint16_t)raw_angle;
  motor->speed_rpm = (int16_t)(ReadBe16(&data[2]) * motor->config.feedback_sign);
  motor->measured_current = ReadBe16(&data[4]);
  motor->temperature = data[6];

  if (!motor->has_feedback) {
    motor->last_raw_angle = raw_angle;
    motor->has_feedback = true;
    motor->last_feedback_ms = now_ms;
    return;
  }

  delta = (int16_t)(raw_angle - motor->last_raw_angle);
  if (delta > 4096) delta = (int16_t)(delta - 8192);
  if (delta < -4096) delta = (int16_t)(delta + 8192);
  motor->total_counts += (int32_t)(delta * motor->config.feedback_sign);
  motor->last_raw_angle = raw_angle;
  motor->last_feedback_ms = now_ms;
}

bool Motor_IsFeedbackFresh(const Motor *motor, uint32_t now_ms, uint32_t timeout_ms)
{
  return motor->has_feedback && ((uint32_t)(now_ms - motor->last_feedback_ms) <= timeout_ms);
}

void Motor_HoldCurrentPosition(Motor *motor)
{
  motor->target_counts = (float)motor->total_counts;
  motor->feedforward_current = 0.0f;
  Pid_Reset(&motor->config.position_pid);
  Pid_Reset(&motor->config.velocity_pid);
}

void Motor_SetTargetCounts(Motor *motor, float target_counts)
{
  motor->target_counts = target_counts;
}

int16_t Motor_ControlStep(Motor *motor, uint32_t now_ms, float dt_s)
{
  float output;

  if (!Motor_IsFeedbackFresh(motor, now_ms, 20U)) {
    Pid_Reset(&motor->config.position_pid);
    Pid_Reset(&motor->config.velocity_pid);
    return 0;
  }

  if (motor->config.kind == MOTOR_KIND_M2006) {
    const float speed_target = Pid_Step(&motor->config.position_pid,
                                        motor->target_counts - (float)motor->total_counts,
                                        dt_s);
    output = Pid_Step(&motor->config.velocity_pid,
                      speed_target - (float)motor->speed_rpm,
                      dt_s);
  } else {
    output = Pid_Step(&motor->config.position_pid,
                      motor->target_counts - (float)motor->total_counts,
                      dt_s);
  }

  output = Clamp(output + motor->feedforward_current, motor->config.current_limit);
  return (int16_t)(output * motor->config.current_sign);
}
