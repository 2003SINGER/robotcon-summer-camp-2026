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

static float Absolute(float value)
{
  return value < 0.0f ? -value : value;
}

static void Motor_UpdateTrajectory(Motor *motor, float dt_s)
{
  const float max_velocity = motor->config.trajectory_max_velocity_counts_s;
  const float max_acceleration = motor->config.trajectory_max_acceleration_counts_s2;
  float error;
  float desired_velocity;
  float braking_distance;
  float velocity_delta;
  float step;

  if (max_velocity <= 0.0f || max_acceleration <= 0.0f) {
    motor->target_counts = motor->goal_counts;
    motor->trajectory_velocity_counts_s = 0.0f;
    return;
  }

  error = motor->goal_counts - motor->target_counts;
  if (Absolute(error) < 0.5f && Absolute(motor->trajectory_velocity_counts_s) < max_acceleration * dt_s) {
    motor->target_counts = motor->goal_counts;
    motor->trajectory_velocity_counts_s = 0.0f;
    return;
  }

  braking_distance = (motor->trajectory_velocity_counts_s * motor->trajectory_velocity_counts_s) /
                     (2.0f * max_acceleration);
  desired_velocity = Absolute(error) <= braking_distance ? 0.0f :
                     (error >= 0.0f ? max_velocity : -max_velocity);
  velocity_delta = Clamp(desired_velocity - motor->trajectory_velocity_counts_s,
                         max_acceleration * dt_s);
  motor->trajectory_velocity_counts_s += velocity_delta;
  step = motor->trajectory_velocity_counts_s * dt_s;

  if (Absolute(step) >= Absolute(error)) {
    motor->target_counts = motor->goal_counts;
    motor->trajectory_velocity_counts_s = 0.0f;
  } else {
    motor->target_counts += step;
  }
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
  motor->goal_counts = 0.0f;
  motor->target_counts = 0.0f;
  motor->trajectory_velocity_counts_s = 0.0f;
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
  motor->goal_counts = (float)motor->total_counts;
  motor->target_counts = (float)motor->total_counts;
  motor->trajectory_velocity_counts_s = 0.0f;
  motor->feedforward_current = 0.0f;
  Pid_Reset(&motor->config.position_pid);
  Pid_Reset(&motor->config.velocity_pid);
}

void Motor_SetTargetCounts(Motor *motor, float target_counts)
{
  motor->goal_counts = target_counts;
}

bool Motor_IsTrajectoryComplete(const Motor *motor)
{
  return motor->goal_counts == motor->target_counts && motor->trajectory_velocity_counts_s == 0.0f;
}

int16_t Motor_ControlStep(Motor *motor, uint32_t now_ms, float dt_s)
{
  float output;

  if (!Motor_IsFeedbackFresh(motor, now_ms, 20U)) {
    Pid_Reset(&motor->config.position_pid);
    Pid_Reset(&motor->config.velocity_pid);
    return 0;
  }

  Motor_UpdateTrajectory(motor, dt_s);

  if (motor->config.kind == MOTOR_KIND_M2006) {
    const float speed_target = Pid_Step(&motor->config.position_pid,
                                        motor->target_counts,
                                        (float)motor->total_counts,
                                        0.0f, dt_s);
    output = Pid_Step(&motor->config.velocity_pid, speed_target,
                      (float)motor->speed_rpm, motor->feedforward_current, dt_s);
  } else {
    output = Pid_Step(&motor->config.position_pid, motor->target_counts,
                      (float)motor->total_counts, motor->feedforward_current, dt_s);
  }

  return (int16_t)(output * motor->config.current_sign);
}
