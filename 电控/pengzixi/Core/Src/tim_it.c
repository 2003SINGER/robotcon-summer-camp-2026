#include "tim_it.h"
#include "pid.h"
#include "m2006.h"
#include <stdio.h>

int16_t cmd0, cmd1;
float vofa_buf[8];
extern TIM_HandleTypeDef htim2;
extern int32_t motor_target[MOTOR_NUM];

extern uint8_t work_mode;
extern int32_t motor_target[2];

extern volatile uint32_t tim2_cnt;
extern volatile uint32_t tim2_flag;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        cmd0 = M2006_PosVel_Loop(&motor[0], motor_target[0]);
				cmd1 = M2006_PosVel_Loop(&motor[1], motor_target[1]);

        //__disable_irq();
			
			  tim2_flag = 1;
				tim2_cnt++;
        FDCAN_Send_Current(cmd0, cmd1);
        //__enable_irq();
				vofa_buf[0] = motor[0].total_angle;
				vofa_buf[1] = motor_target[0];
				vofa_buf[2] = motor[0].speed_rpm;
				vofa_buf[3] = cmd0;

				vofa_buf[4] = motor[1].total_angle;
				vofa_buf[5] = motor_target[1];
				vofa_buf[6] = motor[1].speed_rpm;
				vofa_buf[7] = cmd1;
    }
}

