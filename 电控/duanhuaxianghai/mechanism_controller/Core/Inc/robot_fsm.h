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
  Z_AXIS_BENCH_CLEAR_FAULT,
  Z_AXIS_BENCH_CAPTURE_REFERENCE,
  Z_AXIS_BENCH_APPLY_FEEDFORWARD
} ZAxisBenchCommand;

typedef enum {
  ROTATOR_BENCH_NONE = 0,
  ROTATOR_BENCH_ARM,
  ROTATOR_BENCH_MOVE_TO_TARGET,
  ROTATOR_BENCH_MOVE_HOME,
  ROTATOR_BENCH_ESTOP,
  ROTATOR_BENCH_CLEAR_FAULT,
  ROTATOR_BENCH_CAPTURE_REFERENCE
} RotatorBenchCommand;

typedef enum {
  LOADER_BENCH_NONE = 0,
  LOADER_BENCH_ARM,
  LOADER_BENCH_MOVE_TO_TARGET,
  LOADER_BENCH_MOVE_HOME,
  LOADER_BENCH_ESTOP,
  LOADER_BENCH_CLEAR_FAULT,
  LOADER_BENCH_CAPTURE_REFERENCE
} LoaderBenchCommand;

/* Write from the GDB Debug Console while Z_AXIS_BENCH_MODE is 1.
 * Move targets are centimetres from the software zero set by command 6. */
extern volatile ZAxisBenchCommand g_z_axis_bench_command;
extern volatile float g_z_axis_bench_target_cm;
extern volatile float g_z_axis_bench_reference_counts;
extern volatile float g_z_axis_bench_feedforward_current;

/* Write from the GDB Debug Console while ROTATOR_BENCH_MODE is 1.
 * Target angle is in output-shaft degrees from command-6 software zero. */
extern volatile RotatorBenchCommand g_rotator_bench_command;
extern volatile float g_rotator_bench_target_degrees;
/* DISARMED-only CAN probe in millivolts.  It is hard-clamped to +/-3000 mV
 * and defaults to zero. Use it only with the axis clear, to solicit GM6020
 * feedback before ARM on firmware revisions that do not stream at zero. */
extern volatile int16_t g_rotator_bench_probe_voltage_mV;

/* Write from the GDB Debug Console while LOADER_BENCH_MODE is 1. Command 2
 * treats these as independent, absolute cm targets, so changing B later does
 * not move A again. Negative values are permitted for post-reset recovery
 * when the current physical pose was not the retracted endpoint. A/upper is
 * C610 ID 1; B/lower is C610 ID 2. */
extern volatile LoaderBenchCommand g_loader_bench_command;
extern volatile float g_loader_bench_target_upper_cm;
extern volatile float g_loader_bench_target_lower_cm;
/* SWD-friendly physical positions, refreshed by the 20 ms FSM task. */
extern volatile float g_loader_bench_upper_position_cm;
extern volatile float g_loader_bench_lower_position_cm;

void RobotFsm_Init(void);
bool RobotFsm_StartRetrieve(void);
void RobotFsm_SetExternalGripConfirmed(bool confirmed);
void RobotFsm_Tick(uint32_t now_ms);
RobotFsmState RobotFsm_GetState(void);

#endif
