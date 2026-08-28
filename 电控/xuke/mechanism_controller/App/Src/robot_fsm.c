#include "robot_fsm.h"
#include "board_config.h"
#include "command_mailbox.h"
#include "mechanism.h"
#include "can_bus.h"
#include "FreeRTOS.h"
#include "task.h"

/* Competition-flow positions are local to this state machine. They are
 * relative to the encoder origin captured when the mechanism is armed. */
#define FSM_LOADER_UPPER_RETRACTED_CM  0.0f
#define FSM_LOADER_LOWER_RETRACTED_CM  0.0f
/* These preserve the original xuke targets after the measured per-screw
 * calibration: old A=-8192*20*38 -> 39.22 cm; old B=-8192*20*35 -> 34.98 cm. */
#define FSM_LOADER_UPPER_PICK_CM       38.0f
#define FSM_LOADER_LOWER_PICK_CM       30.0f

/* Current bench-sequence references: verify each value before it is used by
 * a competition state. */
#define FSM_LIFT_HOME_COUNTS           0.0f
#define FSM_LIFT_MAX_HEIGHT_CM         50.0f
#define FSM_LIFT_MAX_HEIGHT_COUNTS     (FSM_LIFT_MAX_HEIGHT_CM * Tuning_GetLiftCountsPerCm())

#define FSM_ROTATOR_HOME_DEGREES       0.0f
#define FSM_ROTATOR_FLIPPED_DEGREES    180.0f

/* Three-box retrieval task states */
typedef enum {
  TASK_IDLE = 0,           // Waiting for chassis arrival
  TASK_LOADER_EXTEND,      // Extend loader to pick position
  TASK_WAIT_LOADER_EXTEND, // Wait for loader at target
  TASK_WAIT_GRIP,          // Wait for gripper (800ms)
  TASK_LOADER_RETRACT,     // Retract loader
  TASK_WAIT_LOADER_RETRACT,// Wait for loader at target
  TASK_LIFT_UP,            // Lift to max height (box 1 only)
  TASK_WAIT_LIFT,          // Wait for lift at target
  TASK_ROTATOR_FLIP,       // Rotate 180 degrees (box 1 only)
  TASK_WAIT_ROTATOR,       // Wait for rotator at target
  TASK_COMPLETE,           // All boxes completed
  TASK_FAULT               // Fault state
} TaskState;

static TaskState task_state = TASK_IDLE;
static uint8_t current_box = 0;
static uint32_t grip_start_time = 0;

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
  task_state = TASK_IDLE;
  current_box = 0;
  grip_start_time = 0;
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
#if Z_AXIS_BENCH_MODE || LOADER_BENCH_MODE || ROTATOR_BENCH_MODE
  return;
#else
  // Check for faults first
  if (Mechanism_GetMode() == MECHANISM_MODE_FAULT) {
    task_state = TASK_FAULT;
    return;
  }
  
  // Must be READY to execute motions
  if (Mechanism_GetMode() != MECHANISM_MODE_READY) {
    return;
  }
  
  // Get chassis box position (1/2/3 = arrived, 0 = moving/none)
  uint8_t box = CanBus_GetChassisBoxPosition();
  
  switch (task_state) {
    case TASK_IDLE:
      // Wait for chassis to arrive at any box
      if (box == 1 || box == 2 || box == 3) {
        current_box = box;
        // Extend loader to pick position
        Mechanism_MoveLoaderToCm(FSM_LOADER_UPPER_PICK_CM, FSM_LOADER_LOWER_PICK_CM);
        task_state = TASK_WAIT_LOADER_EXTEND;
      }
      break;
      
    case TASK_LOADER_EXTEND:
      // Command already sent, just wait
      if (Mechanism_IsLoaderAtTarget()) {
        task_state = TASK_WAIT_GRIP;
      }
      break;
      
    case TASK_WAIT_LOADER_EXTEND:
      // Wait for loader to reach target
      if (Mechanism_IsLoaderAtTarget()) {
        grip_start_time = now_ms;
        task_state = TASK_WAIT_GRIP;
      }
      break;
      
    case TASK_WAIT_GRIP:
      // Wait for gripper to secure (800ms)
      if (now_ms - grip_start_time >= 800U) {
        Mechanism_MoveLoaderToCm(FSM_LOADER_UPPER_RETRACTED_CM, FSM_LOADER_LOWER_RETRACTED_CM);
        task_state = TASK_WAIT_LOADER_RETRACT;
      }
      break;
      
    case TASK_LOADER_RETRACT:
      // Command already sent, just wait
      if (Mechanism_IsLoaderAtTarget()) {
        task_state = TASK_LIFT_UP;
      }
      break;
      
    case TASK_WAIT_LOADER_RETRACT:
      // Wait for loader to retract
      if (Mechanism_IsLoaderAtTarget()) {
        // Box 1: lift and rotate; Box 2/3: skip to complete
        if (current_box == 1) {
          Mechanism_MoveLiftTo(Mechanism_LiftCmToCounts(FSM_LIFT_MAX_HEIGHT_CM));
          task_state = TASK_WAIT_LIFT;
        } else {
          // Box 2 or 3: clear flag and wait for next
          CanBus_ClearChassisBoxPosition();
          task_state = TASK_IDLE;
        }
      }
      break;
      
    case TASK_LIFT_UP:
      // Command already sent, just waitzhe
      if (Mechanism_IsLiftAtTarget()) {
        task_state = TASK_ROTATOR_FLIP;
      }
      break;
      
    case TASK_WAIT_LIFT:
      // Wait for lift to reach max height
      if (Mechanism_IsLiftAtTarget()) {
        Mechanism_TurnRotatorTo(FSM_ROTATOR_FLIPPED_DEGREES);
        task_state = TASK_WAIT_ROTATOR;
      }
      break;
      
    case TASK_ROTATOR_FLIP:
      // Command already sent, just wait
      if (Mechanism_IsRotatorAtTarget()) {
        task_state = TASK_COMPLETE;
      }
      break;
      
    case TASK_WAIT_ROTATOR:
      // Wait for rotator to reach 180 degrees
      if (Mechanism_IsRotatorAtTarget()) {
        task_state = TASK_COMPLETE;
      }
      break;
      
    case TASK_COMPLETE:
      // All three boxes completed
      // Optionally reset or stay here
      break;
      
    case TASK_FAULT:
      // Stay in fault until reset
      break;
  }
#endif
}

/* Reset task state for next round */
void RobotFsm_ResetTask(void)
{
  task_state = TASK_IDLE;
  current_box = 0;
  grip_start_time = 0;
  CanBus_ClearChassisBoxPosition();
}

RobotFsmState RobotFsm_GetState(void) { return state; }
