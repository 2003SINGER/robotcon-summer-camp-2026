#ifndef PID_H
#define PID_H

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float previous_error;
  float integral_limit;
  float output_limit;
} PidController;

void Pid_Reset(PidController *pid);
float Pid_Step(PidController *pid, float error, float dt_s);

#endif
