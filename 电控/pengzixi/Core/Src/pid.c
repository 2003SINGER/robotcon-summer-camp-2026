#include "pid.h"
#include "can.h"

float PID_Calc(PID_t *pid){
	pid->current_error = pid->target - pid->real;
	
	pid->integral += pid->current_error;

	pid->output = (pid->kp * pid->current_error) + (pid->ki * pid->integral) + pid->kd * (pid->current_error - pid->last_error);
	
	if(pid->output > pid->output_max)
        pid->output = pid->output_max;
  if(pid->output < pid->output_min)
        pid->output = pid->output_min;
		
	pid->last_error = pid->current_error;
		
	return pid->output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->current_error = 0.0f;
    pid->last_error = 0.0f;
    pid->target = 0.0f;
    pid->real = 0.0f;
    pid->output = 0.0f;
}



