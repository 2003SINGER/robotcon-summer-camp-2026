#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float output_limit;
    float integral_limit;
} PID_Handle_t;

void PID_Init(PID_Handle_t *pid, float Kp, float Ki, float Kd, float limit);
float PID_Calculate(PID_Handle_t *pid, float setpoint, float measured);

#endif