#ifndef __M2006_H
#define __M2006_H

#include "can.h"
#include "pid.h"

#define MOTOR_NUM 2

typedef struct
{
    int16_t angle;         
    int16_t speed_rpm;     
    uint8_t temp;          
    int32_t total_angle;   
    int16_t last_angle;    
	
		PID_t pos_pid;
    PID_t vel_pid;		
} Motor_t;

extern Motor_t motor[MOTOR_NUM];
extern int32_t motor_target[MOTOR_NUM];

#ifdef __cplusplus
extern "C" {
#endif

void Motor_Init(void);
void Motor_UpdateTotalAngle(Motor_t *m, uint8_t *rx_data);
int16_t M2006_PosVel_Loop(Motor_t *m, int32_t target_total);
void Vofa_Send_JustFloat(float *p_data, uint8_t ch_num);

#ifdef __cplusplus
}
#endif

#endif

