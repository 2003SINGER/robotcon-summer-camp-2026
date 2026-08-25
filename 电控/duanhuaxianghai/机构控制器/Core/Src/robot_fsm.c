#include "robot_fsm.h"
#include "command_mailbox.h"
#include "mechanism.h"

static RobotFsmState state;
static bool grip_confirmed;

void RobotFsm_Init(void)
{
  state = ROBOT_FSM_IDLE;
  grip_confirmed = false;
}

bool RobotFsm_StartRetrieve(void)
{
  if (state != ROBOT_FSM_IDLE || Mechanism_GetMode() != MECHANISM_MODE_READY) return false;
  grip_confirmed = false;
  if (!Mechanism_MoveLoaderOut()) return false;
  state = ROBOT_FSM_LOADER_EXTENDING;
  return true;
}

void RobotFsm_SetExternalGripConfirmed(bool confirmed) { grip_confirmed = confirmed; }

void RobotFsm_Tick(uint32_t now_ms)
{
  RobotCommand command;
  /* Mailbox is deliberately consumed by this task, not by the CAN callback.
   * This prevents a communication frame from interrupting a motion sequence. */
  while (CommandMailbox_Take(&command)) {
    switch (command.type) {
      case ROBOT_COMMAND_ARM:
        (void)Mechanism_Arm(now_ms);
        break;
      case ROBOT_COMMAND_ESTOP:
        Mechanism_EStop(MECHANISM_FAULT_EXTERNAL_ESTOP);
        break;
      case ROBOT_COMMAND_START_RETRIEVE:
        (void)RobotFsm_StartRetrieve();
        break;
      case ROBOT_COMMAND_SET_GRIP_CONFIRMED:
        RobotFsm_SetExternalGripConfirmed(command.value);
        break;
      case ROBOT_COMMAND_RESET_SEQUENCE:
        if (state == ROBOT_FSM_COMPLETE) state = ROBOT_FSM_IDLE;
        break;
      default:
        break;
    }
  }
  if (Mechanism_GetMode() == MECHANISM_MODE_FAULT) {
    state = ROBOT_FSM_FAULT;
    return;
  }
  switch (state) {
    case ROBOT_FSM_LOADER_EXTENDING:
      if (Mechanism_IsLoaderAtTarget()) state = ROBOT_FSM_WAIT_GRIP_CONFIRM;
      break;
    case ROBOT_FSM_WAIT_GRIP_CONFIRM:
      /* Vacuum is another board: this event is supplied by its future status frame. */
      if (grip_confirmed && Mechanism_RetractLoader()) state = ROBOT_FSM_LOADER_RETRACTING;
      break;
    case ROBOT_FSM_LOADER_RETRACTING:
      if (Mechanism_IsLoaderAtTarget()) state = ROBOT_FSM_COMPLETE;
      break;
    default:
      break;
  }
}

RobotFsmState RobotFsm_GetState(void) { return state; }
