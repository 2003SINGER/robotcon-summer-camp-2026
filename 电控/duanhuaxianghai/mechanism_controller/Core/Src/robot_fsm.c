#include "robot_fsm.h"
#include "board_config.h"
#include "command_mailbox.h"
#include "mechanism.h"

/* Competition-flow positions are local to this state machine. They are
 * relative to the encoder origin captured when the mechanism is armed. */
#define FSM_LOADER_UPPER_RETRACTED_CM  0.0f
#define FSM_LOADER_LOWER_RETRACTED_CM  0.0f
/* These preserve the original xuke targets after the measured per-screw
 * calibration: old A=-8192*20*38 -> 39.22 cm; old B=-8192*20*35 -> 34.98 cm. */
#define FSM_LOADER_UPPER_PICK_CM       39.22f
#define FSM_LOADER_LOWER_PICK_CM       34.98f

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
volatile float g_z_axis_bench_target_cm = 0.0f;
volatile float g_z_axis_bench_reference_counts = 0.0f;
volatile float g_z_axis_bench_feedforward_current = 0.0f;
volatile RotatorBenchCommand g_rotator_bench_command = ROTATOR_BENCH_NONE;
volatile float g_rotator_bench_target_degrees = 0.0f;
volatile int16_t g_rotator_bench_probe_voltage_mV = 0;
volatile LoaderBenchCommand g_loader_bench_command = LOADER_BENCH_NONE;
volatile float g_loader_bench_target_upper_cm = 0.0f;
volatile float g_loader_bench_target_lower_cm = 0.0f;
volatile float g_loader_bench_upper_position_cm = 0.0f;
volatile float g_loader_bench_lower_position_cm = 0.0f;

#if Z_AXIS_BENCH_MODE
static void ZAxisBench_Tick(uint32_t now_ms)
{
  switch (g_z_axis_bench_command) {
    case Z_AXIS_BENCH_ARM:
      if (Mechanism_Arm(now_ms)) {
        g_z_axis_bench_command = Z_AXIS_BENCH_NONE;
      }
      /* Keep retrying until a fresh CAN feedback frame arrives.  A one-shot
       * ARM command is awkward under SWD because halting the core can make a
       * previously valid frame older than MOTOR_FEEDBACK_TIMEOUT_MS. */
      return;
    case Z_AXIS_BENCH_MOVE_TO_TARGET:
      (void)Mechanism_MoveLiftTo(Mechanism_LiftCmToCounts(g_z_axis_bench_target_cm));
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
    case Z_AXIS_BENCH_CAPTURE_REFERENCE:
      if (Mechanism_ZeroLiftPosition()) {
        g_z_axis_bench_reference_counts = 0.0f;
      }
      break;
    case Z_AXIS_BENCH_APPLY_FEEDFORWARD:
      (void)Mechanism_SetLiftFeedforwardLive(g_z_axis_bench_feedforward_current);
      break;
    default:
      return;
  }
  g_z_axis_bench_command = Z_AXIS_BENCH_NONE;
}
#endif

#if LOADER_BENCH_MODE
static void LoaderBench_Tick(uint32_t now_ms)
{
  g_loader_bench_upper_position_cm =
      Mechanism_LoaderUpperCountsToCm((float)g_mechanism_telemetry[TUNING_MOTOR_LOADER_A].total_counts);
  g_loader_bench_lower_position_cm =
      Mechanism_LoaderLowerCountsToCm((float)g_mechanism_telemetry[TUNING_MOTOR_LOADER_B].total_counts);
  switch (g_loader_bench_command) {
    case LOADER_BENCH_ARM:
      if (Mechanism_Arm(now_ms)) g_loader_bench_command = LOADER_BENCH_NONE;
      return;
    case LOADER_BENCH_MOVE_TO_TARGET:
      (void)Mechanism_MoveLoaderBenchToCm(g_loader_bench_target_upper_cm,
                                          g_loader_bench_target_lower_cm);
      break;
    case LOADER_BENCH_MOVE_HOME:
      (void)Mechanism_MoveLoaderToCm(0.0f, 0.0f);
      break;
    case LOADER_BENCH_ESTOP:
      Mechanism_EStop(MECHANISM_FAULT_EXTERNAL_ESTOP);
      break;
    case LOADER_BENCH_CLEAR_FAULT:
      (void)Mechanism_ClearFault();
      break;
    case LOADER_BENCH_CAPTURE_REFERENCE:
      (void)Mechanism_ZeroLoaderPositions();
      break;
    default:
      return;
  }
  g_loader_bench_command = LOADER_BENCH_NONE;
}
#endif

#if ROTATOR_BENCH_MODE
static void RotatorBench_Tick(uint32_t now_ms)
{
  switch (g_rotator_bench_command) {
    case ROTATOR_BENCH_ARM:
      if (Mechanism_Arm(now_ms)) g_rotator_bench_command = ROTATOR_BENCH_NONE;
      return;
    case ROTATOR_BENCH_MOVE_TO_TARGET:
      (void)Mechanism_TurnRotatorTo(g_rotator_bench_target_degrees);
      break;
    case ROTATOR_BENCH_MOVE_HOME:
      (void)Mechanism_TurnRotatorTo(0.0f);
      break;
    case ROTATOR_BENCH_ESTOP:
      Mechanism_EStop(MECHANISM_FAULT_EXTERNAL_ESTOP);
      break;
    case ROTATOR_BENCH_CLEAR_FAULT:
      (void)Mechanism_ClearFault();
      break;
    case ROTATOR_BENCH_CAPTURE_REFERENCE:
      (void)Mechanism_ZeroRotatorPosition();
      break;
    default:
      return;
  }
  g_rotator_bench_command = ROTATOR_BENCH_NONE;
}
#endif

void RobotFsm_Init(void)
{
  state = ROBOT_FSM_IDLE;
  grip_confirmed = false;
}

bool RobotFsm_StartRetrieve(void)
{
  if (state != ROBOT_FSM_IDLE || Mechanism_GetMode() != MECHANISM_MODE_READY) return false;
  grip_confirmed = false;
  if (!Mechanism_MoveLoaderToCm(FSM_LOADER_UPPER_PICK_CM, FSM_LOADER_LOWER_PICK_CM)) return false;
  state = ROBOT_FSM_LOADER_EXTENDING;
  return true;
}

void RobotFsm_SetExternalGripConfirmed(bool confirmed) { grip_confirmed = confirmed; }

void RobotFsm_Tick(uint32_t now_ms)
{
#if Z_AXIS_BENCH_MODE
  ZAxisBench_Tick(now_ms);
  return;
#elif LOADER_BENCH_MODE
  LoaderBench_Tick(now_ms);
  return;
#elif ROTATOR_BENCH_MODE
  RotatorBench_Tick(now_ms);
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
          Mechanism_MoveLoaderToCm(FSM_LOADER_UPPER_RETRACTED_CM, FSM_LOADER_LOWER_RETRACTED_CM)) {
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
