#include "m2006.h"
#include "can.h"
#include "pid.h"
#include <stdio.h>
#include <string.h>


Motor_t motor[MOTOR_NUM];
int32_t motor_target[MOTOR_NUM];
extern UART_HandleTypeDef huart3;

void Motor_Init(void){
	for(uint8_t i = 0; i < MOTOR_NUM; i++)
	{
			motor[i].pos_pid.kp = 0.05f;
			motor[i].pos_pid.ki = 0.0f;
			motor[i].pos_pid.kd = 0.0f;
			motor[i].pos_pid.output_max = 1600.0f;
			motor[i].pos_pid.output_min = -2000.0f;
	
			motor[i].vel_pid.kp = 0.6f;
			motor[i].vel_pid.ki = 0.08f;
			motor[i].vel_pid.kd = 0.0f;
			motor[i].vel_pid.output_max = 4000.0f;
			motor[i].vel_pid.output_min = -4000.0f;
		
			PID_Reset(&motor[i].pos_pid);
      PID_Reset(&motor[i].vel_pid);
		
			motor[i].angle = 0;
      motor[i].last_angle = 0;
      motor[i].total_angle = 0;
      motor[i].speed_rpm = 0;
      motor[i].temp = 0;			
	}

}

void Motor_UpdateTotalAngle(Motor_t *m, uint8_t *rx_data)
{
    m->angle = (rx_data[0] << 8) | rx_data[1];
    m->speed_rpm = (rx_data[2] << 8) | rx_data[3];
    m->temp = rx_data[7];

    int16_t delta = m->angle - m->last_angle;
    if(delta > 4096)  delta -= 8192;
    if(delta < -4096) delta += 8192;

    m->total_angle += delta;
    m->last_angle = m->angle; 
}

int16_t M2006_PosVel_Loop(Motor_t *m, int32_t target_total)
{
    m->pos_pid.target = (float)target_total;
    m->pos_pid.real = (float)m->total_angle;
    float vel_target = PID_Calc(&m->pos_pid);

    m->vel_pid.target = vel_target;
    m->vel_pid.real = (float)m->speed_rpm;
    float current_out = PID_Calc(&m->vel_pid);

    return (int16_t)current_out;
}

void Vofa_FireWater(float f0,float f1,float f2,float f3,
                    float f4,float f5,float f6,float f7)
{
    char buf[256];
    int len = sprintf(buf,"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                        f0,f1,f2,f3,f4,f5,f6,f7);
    HAL_UART_Transmit(&huart3,(uint8_t*)buf,len,20);
}
