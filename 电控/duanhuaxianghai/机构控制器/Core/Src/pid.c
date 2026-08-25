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
  pid->previous_error = 0.0f;
}

float Pid_Step(PidController *pid, float error, float dt_s)
{
  float derivative;
  float candidate_integral;
  float output;

  if (dt_s <= 0.0f) return 0.0f;
  candidate_integral = Clamp(pid->integral + error * dt_s, pid->integral_limit);
  derivative = (error - pid->previous_error) / dt_s;
  output = pid->kp * error + pid->ki * candidate_integral + pid->kd * derivative;

  /* Do not integrate further into a saturated output. */
  if (output <= pid->output_limit && output >= -pid->output_limit) {
    pid->integral = candidate_integral;
  }
  pid->previous_error = error;
  return Clamp(output, pid->output_limit);
}
