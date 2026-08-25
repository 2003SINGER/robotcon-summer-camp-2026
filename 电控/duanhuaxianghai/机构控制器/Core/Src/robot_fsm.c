#include "robot_fsm.h"
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
  (void)now_ms;
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
