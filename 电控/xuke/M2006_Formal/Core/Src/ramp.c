#include "ramp.h"

float Ramp_Calc(Ramp_t *ramp, float target)
{
    float delta = target - ramp->out;
    if(delta > ramp->max_delta) delta = ramp->max_delta;
    if(delta < -ramp->max_delta) delta = -ramp->max_delta;
    ramp->out += delta;
    return ramp->out;
}

void Ramp_Init(Ramp_t *ramp, float init_val, float max_delta)
{
    ramp->out = init_val;
    ramp->max_delta = max_delta;
}
