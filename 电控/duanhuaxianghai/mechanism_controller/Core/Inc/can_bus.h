#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdbool.h>
#include <stdint.h>
#include "motor.h"

typedef struct {
  uint32_t rx_frame_count;
  uint32_t rx_unknown_id_count;
  uint32_t tx_drop_count;
  uint8_t last_bus;
  uint16_t last_identifier;
  uint8_t last_data[8];
  uint32_t last_rx_ms;
} CanBusDiagnostics;

extern volatile CanBusDiagnostics g_can_bus_diagnostics;

void CanBus_Init(void);
void CanBus_SendM2006Currents(MotorCan bus, int16_t first, int16_t second, int16_t third);
void CanBus_SendGM6020Current(MotorCan bus, int16_t rotator);
uint32_t CanBus_GetTxDropCount(void);
uint32_t CanBus_GetConsecutiveTxDropCount(void);
bool CanBus_GetDiagnostics(CanBusDiagnostics *diagnostics);

#endif
