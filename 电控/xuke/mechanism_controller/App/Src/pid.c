#include "pid.h"

static float Clamp(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

void Pid_Reset(PidController *pid)
{
  pid->integral = 0.0f;
  pid->previous_measurement = 0.0f;
  pid->filtered_derivative = 0.0f;
  pid->has_measurement = false;
  pid->diagnostics.proportional = 0.0f;
  pid->diagnostics.integral = 0.0f;
  pid->diagnostics.derivative = 0.0f;
  pid->diagnostics.unsaturated_output = 0.0f;
  pid->diagnostics.output = 0.0f;
  pid->diagnostics.saturated = false;
}

float Pid_Step(PidController *pid, float setpoint, float measurement,
               float feedforward, float dt_s)
{
  const float error = setpoint - measurement;
  float raw_derivative = 0.0f;
  float derivative;
  float candidate_integral;
  float proportional;
  float integral;
  float unsaturated;
  float output;
  bool saturated;
  bool drives_further_into_saturation;

  if (dt_s <= 0.0f || pid->output_limit <= 0.0f) return 0.0f;

  /* Derivative on measurement avoids a current spike when the setpoint jumps. */
  if (pid->has_measurement) raw_derivative = -(measurement - pid->previous_measurement) / dt_s;
  if (pid->derivative_tau_s > 0.0f) {
    const float alpha = pid->derivative_tau_s / (pid->derivative_tau_s + dt_s);
    pid->filtered_derivative = alpha * pid->filtered_derivative + (1.0f - alpha) * raw_derivative;
  } else {
    pid->filtered_derivative = raw_derivative;
  }
  pid->previous_measurement = measurement;
  pid->has_measurement = true;

  candidate_integral = Clamp(pid->integral + error * dt_s, pid->integral_limit);
  proportional = pid->kp * error;
  derivative = pid->kd * pid->filtered_derivative;
  integral = pid->ki * candidate_integral;
  unsaturated = proportional + integral + derivative + feedforward;
  output = Clamp(unsaturated, pid->output_limit);
  saturated = output != unsaturated;
  drives_further_into_saturation = (unsaturated > pid->output_limit && error > 0.0f) ||
                                   (unsaturated < -pid->output_limit && error < 0.0f);

  /* Conditional integration uses the final output, including feedforward. */
  if (!saturated || !drives_further_into_saturation) {
    pid->integral = candidate_integral;
  }

  integral = pid->ki * pid->integral;
  unsaturated = proportional + integral + derivative + feedforward;
  output = Clamp(unsaturated, pid->output_limit);
  pid->diagnostics.proportional = proportional;
  pid->diagnostics.integral = integral;
  pid->diagnostics.derivative = derivative;
  pid->diagnostics.unsaturated_output = unsaturated;
  pid->diagnostics.output = output;
  pid->diagnostics.saturated = output != unsaturated;
  return output;
}

void Pid_GetDiagnostics(const PidController *pid, PidDiagnostics *diagnostics)
{
  if (diagnostics != 0) *diagnostics = pid->diagnostics;
}
