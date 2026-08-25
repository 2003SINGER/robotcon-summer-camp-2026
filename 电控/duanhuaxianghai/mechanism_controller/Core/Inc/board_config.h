#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* Board-wide timing and CAN packing. Motor profiles are in tuning.c. */
#include <stdint.h>

#define CONTROL_PERIOD_S                    0.001f
#define MOTOR_FEEDBACK_TIMEOUT_MS           20U

#define CAN_ID_M2006_COMMAND_GROUP          0x200U
#define CAN_ID_GM6020_COMMAND_GROUP         0x1FFU

/* Keep this enabled for the first bench session: the debugger can show every
 * standard ID seen on the local motor bus, including incorrectly configured
 * DJI IDs. Disable after CAN IDs are verified to reduce interrupt load. */
#define CAN_DIAGNOSTIC_ACCEPT_ALL_STANDARD_IDS 1U

/* Consecutive failures mean the transmit FIFO has remained unavailable long
 * enough to make a controlled stop safer than continuing blindly. */
#define CAN_TX_DROP_FAULT_THRESHOLD          20U

#endif
