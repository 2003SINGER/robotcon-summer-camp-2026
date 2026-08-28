#ifndef COMMAND_MAILBOX_H
#define COMMAND_MAILBOX_H

#include <stdbool.h>
#include <stdint.h>

/*
 * The communication layer may only publish intent here.  RobotFsmTask owns
 * state transitions; MotorControlTask remains the sole writer of motor
 * current commands.
 */
typedef enum {
  ROBOT_COMMAND_ARM,
  ROBOT_COMMAND_ESTOP,
  ROBOT_COMMAND_START_RETRIEVE,
  ROBOT_COMMAND_SET_GRIP_CONFIRMED,
  ROBOT_COMMAND_RESET_SEQUENCE
} RobotCommandType;

typedef struct {
  RobotCommandType type;
  bool value;
  uint32_t sequence;
} RobotCommand;

bool CommandMailbox_Init(void);
bool CommandMailbox_Submit(const RobotCommand *command);
bool CommandMailbox_SubmitFromIsr(const RobotCommand *command, bool *higher_priority_task_woken);
bool CommandMailbox_Take(RobotCommand *command);
uint32_t CommandMailbox_GetDropCount(void);

#endif
