#ifndef __CAN_H
#define __CAN_H

#include "stm32h7xx_hal.h"

/*typedef struct
{
    int16_t angle;         
    int16_t speed_rpm;     
    uint8_t temp;          
    int32_t total_angle;   
    int16_t last_angle;    
	
		PID_t pos_pid;
    PID_t vel_pid;		
} Motor_t;*/

//extern Motor_t motor[4];
//extern int32_t motor0_target;

void fdcan_filter_init(void);
void FDCAN_Send_Current(int16_t curr0, int16_t curr1);

#endif
