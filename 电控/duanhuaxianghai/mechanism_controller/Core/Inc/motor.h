#ifndef MOTOR_H
#define MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "pid.h"

typedef enum {
  MOTOR_KIND_M2006,
  MOTOR_KIND_GM6020
} MotorKind;

typedef struct {
  MotorKind kind;
  uint16_t feedback_id;
  int8_t feedback_sign;
  int8_t current_sign;
  float current_limit;
  PidController position_pid;
  PidController velocity_pid;
  float trajectory_max_velocity_counts_s;
  float trajectory_max_acceleration_counts_s2;
} MotorConfig;

typedef struct {
  MotorConfig config;
  volatile uint16_t raw_angle;
  volatile int16_t speed_rpm;
  volatile int16_t measured_current;
  volatile uint8_t temperature;
  volatile int32_t total_counts;
  volatile uint32_t last_feedback_ms;
  volatile bool has_feedback;
  int16_t last_raw_angle;
  float goal_counts;
  float target_counts;
  float trajectory_velocity_counts_s;
  float feedforward_current;
} Motor;

void Motor_Init(Motor *motor, const MotorConfig *config);
void Motor_OnFeedback(Motor *motor, const uint8_t data[8], uint32_t now_ms);
bool Motor_IsFeedbackFresh(const Motor *motor, uint32_t now_ms, uint32_t timeout_ms);
void Motor_HoldCurrentPosition(Motor *motor);
void Motor_SetTargetCounts(Motor *motor, float target_counts);
bool Motor_IsTrajectoryComplete(const Motor *motor);
int16_t Motor_ControlStep(Motor *motor, uint32_t now_ms, float dt_s);

#endif
