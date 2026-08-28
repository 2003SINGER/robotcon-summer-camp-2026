#include "pid.h"

void PID_Init(PID_Handle_t *pid, float Kp, float Ki, float Kd, float limit) 
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->output_limit = limit;
    pid->integral_limit = limit * 0.8f;
}

float PID_Calculate(PID_Handle_t *pid, float setpoint, float measured) 
{
    float error = setpoint - measured;
    
    // 比例
    float P_out = pid->Kp * error;
    
    // 积分（带限幅）
	/*if (fabs(error) < 500.0f)   // 加一个积分分离逻辑，到达这个误差再开始积分
{
    pid->integral += error;
}
else
{
	pid->integral = 0;      // 误差特别大时候不启动积分，防止暴涨
}*/
    pid->integral += error;
    if (pid->integral > pid->integral_limit) 
        pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) 
        pid->integral = -pid->integral_limit;
    float I_out = pid->Ki * pid->integral;
    
    // 微分
    float D_out = pid->Kd * (error - pid->prev_error);
    pid->prev_error = error;
    
    // 输出
    float output = P_out + I_out + D_out;
    if (output > pid->output_limit) output = pid->output_limit;
    if (output < -pid->output_limit) output = -pid->output_limit;
    
    return output;
}