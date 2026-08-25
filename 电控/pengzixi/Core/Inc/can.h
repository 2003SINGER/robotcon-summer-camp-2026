#ifndef __CAN_H
#define __CAN_H

#include "stm32h7xx_hal.h"

void fdcan_filter_init(void);
void FDCAN_Send_Current(int16_t curr0, int16_t curr1);

#endif
