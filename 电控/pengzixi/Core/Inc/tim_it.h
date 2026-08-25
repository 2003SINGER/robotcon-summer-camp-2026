#ifndef __TIM_IT_H
#define __TIM_IT_H

#include "main.h"

extern float vofa_buf[8];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif

