#include "chassis.h"

#include <stddef.h>

/* A char-sized volatile latch is atomic on Cortex-M.  Last valid command wins
 * if the chassis publishes several commands before RobotFsmTask wakes. */
static volatile char pending_task_command = '\0';

void Chassis_OnCanFeedback(uint16_t identifier, const uint8_t data[8])
{
  if (identifier != CHASSIS_COMMAND_CAN_ID) return;

  switch ((char)data[0]) {
    case 'A':
    case 'B':
    case 'C':
    case 'D':
      pending_task_command = (char)data[0];
      break;
    default:
      break;
  }
}

bool Chassis_PeekTaskCommand(char *command)
{
  if (command == NULL || pending_task_command == '\0') return false;
  *command = pending_task_command;
  return true;
}

void Chassis_ClearTaskCommand(void)
{
  pending_task_command = '\0';
}
