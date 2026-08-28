#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t rx_frame_count;
  uint32_t rx_unknown_id_count;
  uint32_t tx_drop_count;
  /* Index follows TuningMotorId: 0=loader upper (0x201), 1=loader lower
   * (0x202), 2=lift (0x203), 3=rotator (0x206).  These deliberately count
   * raw received frames rather than "fresh" status, so SWD can identify a
   * missing CAN branch or wrong ESC ID without the ARM gate hiding it. */
  uint32_t motor_feedback_frame_count[4];
  uint32_t motor_last_feedback_ms[4];
  uint16_t last_identifier;
  uint8_t last_data[8];
  uint32_t last_rx_ms;
  uint32_t chassis_rx_frame_count;
  uint16_t chassis_last_identifier;
  uint8_t chassis_last_data[8];
  uint32_t chassis_last_rx_ms;
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

#endif
