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
  MECHANISM_FAULT_EXTERNAL_ESTOP,
  MECHANISM_FAULT_SOFT_LIMIT,
  MECHANISM_FAULT_MOTION_TIMEOUT,
  MECHANISM_FAULT_STALL
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
  float goal_counts;
  float target_counts;
  float trajectory_velocity_counts_s;
  float trajectory_max_velocity_counts_s;
  float trajectory_max_acceleration_counts_s2;
  float feedforward_current;
  float directional_feedforward_current;
  float applied_feedforward_current;
  float current_limit;
  PidDiagnostics position_pid;
  PidDiagnostics velocity_pid;
  uint32_t last_feedback_ms;
  bool has_feedback;
} MechanismMotorTelemetry;

/* Read-only snapshot for the SWD debugger. Updated by SafetyTask at 10 ms;
 * control ownership remains inside mechanism.c. */
extern volatile MechanismMotorTelemetry g_mechanism_telemetry[TUNING_MOTOR_COUNT];
/* SWD-visible state snapshots, updated by SafetyTask. */
extern volatile uint32_t g_mechanism_mode;
extern volatile uint32_t g_mechanism_fault;

void Mechanism_Init(void);
void Mechanism_OnCanFeedback(uint16_t identifier, const uint8_t data[8], uint32_t now_ms);
void Mechanism_ControlTick(uint32_t now_ms);
void Mechanism_Service(uint32_t now_ms);
bool Mechanism_Arm(uint32_t now_ms);
void Mechanism_EStop(MechanismFault reason);
bool Mechanism_ClearFault(void);
MechanismMode Mechanism_GetMode(void);
MechanismFault Mechanism_GetFault(void);

bool Mechanism_MoveLiftTo(float counts);
/* Set the current Z-axis location to 0 counts; raw CAN angle remains continuous. */
bool Mechanism_ZeroLiftPosition(void);
/* Set both front-extension motor locations to 0 counts at the current pose. */
bool Mechanism_ZeroLoaderPositions(void);
/* Set the current GM6020 angle to 0 degrees without disturbing CAN tracking. */
bool Mechanism_ZeroRotatorPosition(void);
/* Unit conversion only: motor control and CAN feedback remain in counts. */
float Mechanism_LiftCmToCounts(float cm);
float Mechanism_LiftCountsToCm(float counts);
bool Mechanism_TurnRotatorTo(float motor_degrees);
/* Both inputs are physical extension in cm from command-6's retracted zero.
 * `upper_cm` is C610 ID 1 / loader A; `lower_cm` is C610 ID 2 / loader B. */
bool Mechanism_MoveLoaderToCm(float upper_cm, float lower_cm);
/* Bench-only absolute targets. Unlike the competition API, values may be
 * negative after a firmware reset at an unknown physical position. */
bool Mechanism_MoveLoaderBenchToCm(float upper_cm, float lower_cm);
float Mechanism_LoaderUpperCountsToCm(float counts);
float Mechanism_LoaderLowerCountsToCm(float counts);
bool Mechanism_IsLiftAtTarget(void);
bool Mechanism_IsRotatorAtTarget(void);
bool Mechanism_IsLoaderAtTarget(void);

/* Runtime tuning is deliberately permitted only while DISARMED. */
bool Mechanism_SetPid(TuningMotorId motor, MechanismPidLoop loop,
                      float kp, float ki, float kd,
                      float integral_limit, float output_limit);
bool Mechanism_SetDerivativeFilter(TuningMotorId motor, MechanismPidLoop loop, float tau_s);
bool Mechanism_SetCurrentLimit(TuningMotorId motor, float current_limit);
bool Mechanism_SetFeedforward(TuningMotorId motor, float current);
/* Bench-only live adjustment.  Keeps the current motion target unchanged. */
bool Mechanism_SetLiftFeedforwardLive(float current);
bool Mechanism_SetTargetTolerance(TuningMotorId motor, float counts, float speed_rpm);
bool Mechanism_SetMotionLimits(TuningMotorId motor, float max_velocity_counts_s,
                               float max_acceleration_counts_s2);
bool Mechanism_RestoreDefaultTuning(void);
bool Mechanism_GetMotorTelemetry(TuningMotorId motor, MechanismMotorTelemetry *telemetry);

#endif
