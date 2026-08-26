#include "robot_fsm.h"
#include "board_config.h"
#include "command_mailbox.h"
#include "mechanism.h"

/* Competition-flow positions are local to this state machine. They are
 * relative to the encoder origin captured when the mechanism is armed. */
#define FSM_LOADER_A_RETRACTED_COUNTS  0.0f
#define FSM_LOADER_B_RETRACTED_COUNTS  0.0f
#define FSM_LOADER_A_PICK_COUNTS       (8192.0f * 20.0f * 38.0f)
#define FSM_LOADER_B_PICK_COUNTS       (8192.0f * 20.0f * 35.0f)

/* Current bench-sequence references: verify each value before it is used by
 * a competition state. */
#define FSM_LIFT_HOME_COUNTS           0.0f
#define FSM_LIFT_REFERENCE_1_COUNTS    (8192.0f * 123.2f)
#define FSM_LIFT_REFERENCE_2_COUNTS    (8192.0f * (123.2f + 81.8f))
#define FSM_LIFT_REFERENCE_3_COUNTS    (8192.0f * 123.2f)
#define FSM_LIFT_REFERENCE_4_COUNTS    (8192.0f * (123.2f - 72.0f))
#define FSM_LIFT_REFERENCE_5_COUNTS    (8192.0f * (123.2f - 72.0f + 108.0f))

#define FSM_ROTATOR_HOME_DEGREES       0.0f
#define FSM_ROTATOR_FLIPPED_DEGREES    180.0f

static RobotFsmState state;
static bool grip_confirmed;
volatile ZAxisBenchCommand g_z_axis_bench_command = Z_AXIS_BENCH_NONE;
volatile float g_z_axis_bench_target_counts = 0.0f;

static void ZAxisBench_Tick(uint32_t now_ms)
{
  switch (g_z_axis_bench_command) {
    case Z_AXIS_BENCH_ARM:
      (void)Mechanism_Arm(now_ms);
      break;
    case Z_AXIS_BENCH_MOVE_TO_TARGET:
      (void)Mechanism_MoveLiftTo(g_z_axis_bench_target_counts);
      break;
    case Z_AXIS_BENCH_MOVE_HOME:
      (void)Mechanism_MoveLiftTo(0.0f);
      break;
    case Z_AXIS_BENCH_ESTOP:
      Mechanism_EStop(MECHANISM_FAULT_EXTERNAL_ESTOP);
      break;
    case Z_AXIS_BENCH_CLEAR_FAULT:
      (void)Mechanism_ClearFault();
      break;
    default:
      return;
  }
  g_z_axis_bench_command = Z_AXIS_BENCH_NONE;
}

void RobotFsm_Init(void)
{
  state = ROBOT_FSM_IDLE;
  grip_confirmed = false;
}

bool RobotFsm_StartRetrieve(void)
{
  if (state != ROBOT_FSM_IDLE || Mechanism_GetMode() != MECHANISM_MODE_READY) return false;
  grip_confirmed = false;
  if (!Mechanism_MoveLoaderTo(FSM_LOADER_A_PICK_COUNTS, FSM_LOADER_B_PICK_COUNTS)) return false;
  state = ROBOT_FSM_LOADER_EXTENDING;
  return true;
}

void RobotFsm_SetExternalGripConfirmed(bool confirmed) { grip_confirmed = confirmed; }

void RobotFsm_Tick(uint32_t now_ms)
{
#if Z_AXIS_BENCH_MODE
  ZAxisBench_Tick(now_ms);
  return;
#else
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
      if (grip_confirmed &&
          Mechanism_MoveLoaderTo(FSM_LOADER_A_RETRACTED_COUNTS, FSM_LOADER_B_RETRACTED_COUNTS)) {
        state = ROBOT_FSM_LOADER_RETRACTING;
      }
      break;
    case ROBOT_FSM_LOADER_RETRACTING:
      if (Mechanism_IsLoaderAtTarget()) state = ROBOT_FSM_COMPLETE;
      break;
    default:
      break;
  }
#endif
}

RobotFsmState RobotFsm_GetState(void) { return state; }
