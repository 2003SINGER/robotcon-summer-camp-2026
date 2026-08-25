#ifndef ROBOT_FSM_H
#define ROBOT_FSM_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  ROBOT_FSM_IDLE,
  ROBOT_FSM_LOADER_EXTENDING,
  ROBOT_FSM_WAIT_GRIP_CONFIRM,
  ROBOT_FSM_LOADER_RETRACTING,
  ROBOT_FSM_COMPLETE,
  ROBOT_FSM_FAULT
} RobotFsmState;

void RobotFsm_Init(void);
bool RobotFsm_StartRetrieve(void);
void RobotFsm_SetExternalGripConfirmed(bool confirmed);
void RobotFsm_Tick(uint32_t now_ms);
RobotFsmState RobotFsm_GetState(void);

#endif
