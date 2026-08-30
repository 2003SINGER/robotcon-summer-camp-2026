#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdbool.h>
#include <stdint.h>

/* CAN2 application protocol from the chassis controller.
 * Standard data frame 0x123, byte 0: ASCII 'A', 'B', 'C', or 'D'. */
#define CHASSIS_COMMAND_CAN_ID 0x123U

/* Called only by the CAN2 RX callback.  It validates and latches the latest
 * task request; it never calls a mechanism API from interrupt context. */
void Chassis_OnCanFeedback(uint16_t identifier, const uint8_t data[8]);

/* RobotFsmTask peeks while waiting for fresh motor feedback, then consumes at
 * a safe task boundary. */
bool Chassis_PeekTaskCommand(char *command);
void Chassis_ClearTaskCommand(void);

#endif
