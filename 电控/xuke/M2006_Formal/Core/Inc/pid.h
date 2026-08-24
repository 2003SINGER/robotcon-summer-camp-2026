#ifndef __PID_H
#define __PID_H

#include "main.h"
#include <stdint.h>
#include "can.h"

typedef struct{ //struct of pid
	float kp;
	float ki;
	float kd;
	
	float target;
	float real;
	
	float current_error;
	float last_error;
	
	float integral;
	
	float output;
	float output_max;
	float output_min;
}PID_t;


float PID_Calc(PID_t *pid);
void PID_Reset(PID_t *pid);

#endif
   
