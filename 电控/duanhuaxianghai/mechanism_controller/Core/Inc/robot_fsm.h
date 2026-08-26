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

typedef enum {
  Z_AXIS_BENCH_NONE = 0,
  Z_AXIS_BENCH_ARM,
  Z_AXIS_BENCH_MOVE_TO_TARGET,
  Z_AXIS_BENCH_MOVE_HOME,
  Z_AXIS_BENCH_ESTOP,
  Z_AXIS_BENCH_CLEAR_FAULT
} ZAxisBenchCommand;

/* Edit from the debugger Watch window while Z_AXIS_BENCH_MODE is 1. */
extern volatile ZAxisBenchCommand g_z_axis_bench_command;
extern volatile float g_z_axis_bench_target_counts;

void RobotFsm_Init(void);
bool RobotFsm_StartRetrieve(void);
void RobotFsm_SetExternalGripConfirmed(bool confirmed);
void RobotFsm_Tick(uint32_t now_ms);
RobotFsmState RobotFsm_GetState(void);

#endif
