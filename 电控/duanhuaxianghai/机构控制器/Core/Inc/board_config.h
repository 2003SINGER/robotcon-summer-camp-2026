#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

/* Board-wide timing and CAN packing. Motor profiles are in tuning.c. */
#include <stdint.h>

#define CONTROL_PERIOD_S                    0.001f
#define MOTOR_FEEDBACK_TIMEOUT_MS           20U

#define CAN_ID_M2006_COMMAND_GROUP          0x200U
#define CAN_ID_GM6020_COMMAND_GROUP         0x1FFU

#endif
