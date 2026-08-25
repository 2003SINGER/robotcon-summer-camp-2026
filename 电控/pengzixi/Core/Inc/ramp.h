#ifndef __RAMP_H
#define __RAMP_H
#include "stm32h7xx_hal.h"

typedef struct
{
    float out;
    float max_delta;
}Ramp_t;

float Ramp_Calc(Ramp_t *ramp, float target);
void Ramp_Init(Ramp_t *ramp, float init_val, float max_delta);
#endif
