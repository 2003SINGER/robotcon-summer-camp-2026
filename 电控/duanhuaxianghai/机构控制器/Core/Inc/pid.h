#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
  float proportional;
  float integral;
  float derivative;
  float unsaturated_output;
  float output;
  bool saturated;
} PidDiagnostics;

typedef struct {
  float kp;
  float ki;
  float kd;
  float derivative_tau_s;
  float integral;
  float previous_measurement;
  float filtered_derivative;
  float integral_limit;
  float output_limit;
  bool has_measurement;
  PidDiagnostics diagnostics;
} PidController;

void Pid_Reset(PidController *pid);
float Pid_Step(PidController *pid, float setpoint, float measurement,
               float feedforward, float dt_s);
void Pid_GetDiagnostics(const PidController *pid, PidDiagnostics *diagnostics);

#endif
