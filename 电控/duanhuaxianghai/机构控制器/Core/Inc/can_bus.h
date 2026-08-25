#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdint.h>

void CanBus_Init(void);
void CanBus_SendM2006Currents(int16_t loader_a, int16_t loader_b, int16_t lift);
void CanBus_SendGM6020Current(int16_t rotator);
uint32_t CanBus_GetTxDropCount(void);

#endif
