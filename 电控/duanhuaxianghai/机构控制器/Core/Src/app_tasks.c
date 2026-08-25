#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mechanism.h"
#include "robot_fsm.h"
#include "command_mailbox.h"

static volatile uint32_t app_time_ms = 0U;
static TaskHandle_t motor_task_handle;
static TaskHandle_t safety_task_handle;
static TaskHandle_t fsm_task_handle;
static StaticTask_t motor_task_tcb;
static StaticTask_t safety_task_tcb;
static StaticTask_t fsm_task_tcb;
static StackType_t motor_task_stack[512U];
static StackType_t safety_task_stack[384U];
static StackType_t fsm_task_stack[384U];
volatile AppRuntimeDiagnostics g_app_runtime_diagnostics;

static void RecordControlTiming(uint32_t now_ms, uint32_t *last_tick_ms)
{
  if (*last_tick_ms != 0U && now_ms > *last_tick_ms + 1U) {
    g_app_runtime_diagnostics.control_deadline_miss_count += now_ms - *last_tick_ms - 1U;
  }
  *last_tick_ms = now_ms;
  ++g_app_runtime_diagnostics.control_tick_count;
}

static void MotorControlTask(void *argument)
{
  TickType_t next_wake = xTaskGetTickCount();
  uint32_t last_tick_ms = 0U;
  (void)argument;
  for (;;) {
    vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(1));
    RecordControlTiming(app_time_ms, &last_tick_ms);
    Mechanism_ControlTick(app_time_ms);
  }
}

static void SafetyTask(void *argument)
{
  TickType_t next_wake = xTaskGetTickCount();
  (void)argument;
  for (;;) {
    vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(10));
    Mechanism_Service(app_time_ms);
    ++g_app_runtime_diagnostics.safety_tick_count;
    g_app_runtime_diagnostics.motor_stack_min_words = (uint16_t)uxTaskGetStackHighWaterMark(motor_task_handle);
    g_app_runtime_diagnostics.safety_stack_min_words = (uint16_t)uxTaskGetStackHighWaterMark(safety_task_handle);
    g_app_runtime_diagnostics.fsm_stack_min_words = (uint16_t)uxTaskGetStackHighWaterMark(fsm_task_handle);
  }
}

static void RobotFsmTask(void *argument)
{
  TickType_t next_wake = xTaskGetTickCount();
  (void)argument;
  for (;;) {
    vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(20));
    RobotFsm_Tick(app_time_ms);
  }
}

bool AppTasks_Create(void)
{
  if (!CommandMailbox_Init()) return false;
  RobotFsm_Init();
  motor_task_handle = xTaskCreateStatic(MotorControlTask, "motor", 512U, NULL, 5U,
                                         motor_task_stack, &motor_task_tcb);
  safety_task_handle = xTaskCreateStatic(SafetyTask, "safety", 384U, NULL, 4U,
                                          safety_task_stack, &safety_task_tcb);
  fsm_task_handle = xTaskCreateStatic(RobotFsmTask, "robot_fsm", 384U, NULL, 3U,
                                       fsm_task_stack, &fsm_task_tcb);
  return motor_task_handle != NULL && safety_task_handle != NULL && fsm_task_handle != NULL;
}

uint32_t AppTime_GetMs(void) { return app_time_ms; }
void AppTime_IncrementFromSysTick(void) { ++app_time_ms; }

bool AppTasks_GetDiagnostics(AppRuntimeDiagnostics *diagnostics)
{
  if (diagnostics == NULL) return false;
  *diagnostics = g_app_runtime_diagnostics;
  return true;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb_buffer,
                                   StackType_t **stack_buffer,
                                   uint32_t *stack_size)
{
  static StaticTask_t idle_task_tcb;
  static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];
  *tcb_buffer = &idle_task_tcb;
  *stack_buffer = idle_task_stack;
  *stack_size = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
  (void)task;
  (void)task_name;
  Mechanism_EStop(MECHANISM_FAULT_SCHEDULER);
  __disable_irq();
  for (;;) { }
}
