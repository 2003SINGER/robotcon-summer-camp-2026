#ifndef MECHANISM_H
#define MECHANISM_H

#include <stdbool.h>
#include <stdint.h>
#include "tuning.h"

typedef enum {
  MECHANISM_MODE_DISARMED,
  MECHANISM_MODE_READY,
  MECHANISM_MODE_FAULT
} MechanismMode;

typedef enum {
  MECHANISM_FAULT_NONE = 0,
  MECHANISM_FAULT_FEEDBACK_TIMEOUT,
  MECHANISM_FAULT_HAL,
  MECHANISM_FAULT_SCHEDULER,
  MECHANISM_FAULT_EXTERNAL_ESTOP
} MechanismFault;

typedef enum {
  MECHANISM_PID_POSITION,
  MECHANISM_PID_VELOCITY
} MechanismPidLoop;

typedef struct {
  uint16_t feedback_id;
  int32_t total_counts;
  int16_t speed_rpm;
  int16_t measured_current;
  float target_counts;
  float feedforward_current;
  float current_limit;
  uint32_t last_feedback_ms;
  bool has_feedback;
} MechanismMotorTelemetry;

void Mechanism_Init(void);
void Mechanism_OnCanFeedback(uint16_t identifier, const uint8_t data[8], uint32_t now_ms);
void Mechanism_ControlTick(uint32_t now_ms);
void Mechanism_Service(uint32_t now_ms);
bool Mechanism_Arm(uint32_t now_ms);
void Mechanism_EStop(MechanismFault reason);
MechanismMode Mechanism_GetMode(void);
MechanismFault Mechanism_GetFault(void);

bool Mechanism_MoveLiftTo(float counts);
bool Mechanism_TurnRotatorBy(float motor_degrees);
bool Mechanism_MoveLoaderTo(float motor_a_counts, float motor_b_counts);
bool Mechanism_MoveLoaderOut(void);
bool Mechanism_RetractLoader(void);
bool Mechanism_IsLiftAtTarget(void);
bool Mechanism_IsRotatorAtTarget(void);
bool Mechanism_IsLoaderAtTarget(void);

/* Runtime tuning is deliberately permitted only while DISARMED. */
bool Mechanism_SetPid(TuningMotorId motor, MechanismPidLoop loop,
                      float kp, float ki, float kd,
                      float integral_limit, float output_limit);
bool Mechanism_SetCurrentLimit(TuningMotorId motor, float current_limit);
bool Mechanism_SetFeedforward(TuningMotorId motor, float current);
bool Mechanism_SetTargetTolerance(TuningMotorId motor, float counts, float speed_rpm);
bool Mechanism_RestoreDefaultTuning(void);
bool Mechanism_GetMotorTelemetry(TuningMotorId motor, MechanismMotorTelemetry *telemetry);

#endif
