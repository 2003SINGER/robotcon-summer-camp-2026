#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t rx_frame_count;
  uint32_t rx_unknown_id_count;
  uint32_t tx_drop_count;
  uint16_t last_identifier;
  uint8_t last_data[8];
  uint32_t last_rx_ms;
} CanBusDiagnostics;

extern volatile CanBusDiagnostics g_can_bus_diagnostics;

void CanBus_Init(void);
void CanBus_SendM2006Currents(int16_t loader_a, int16_t loader_b, int16_t lift);
/* GM6020 protocol uses a signed voltage command in millivolts, not C610-style
 * current.  ID 2 occupies bytes 2-3 of CAN identifier 0x1FF. */
void CanBus_SendGM6020Voltage(int16_t voltage_mV);
uint32_t CanBus_GetTxDropCount(void);
uint32_t CanBus_GetConsecutiveTxDropCount(void);
bool CanBus_GetDiagnostics(CanBusDiagnostics *diagnostics);

/* Chassis box position API */
uint8_t CanBus_GetChassisBoxPosition(void);
void CanBus_ClearChassisBoxPosition(void);

#endif
