#include "app_tasks.h"
#include "FreeRTOS.h"
#include "task.h"
#include "mechanism.h"
#include "robot_fsm.h"

static volatile uint32_t app_time_ms = 0U;

static void MotorControlTask(void *argument)
{
  TickType_t next_wake = xTaskGetTickCount();
  (void)argument;
  for (;;) {
    vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(1));
    Mechanism_ControlTick(app_time_ms);
  }
}

static void MechanismTask(void *argument)
{
  TickType_t next_wake = xTaskGetTickCount();
  (void)argument;
  for (;;) {
    vTaskDelayUntil(&next_wake, pdMS_TO_TICKS(10));
    Mechanism_Service(app_time_ms);
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
  BaseType_t result;
  RobotFsm_Init();
  result = xTaskCreate(MotorControlTask, "motor", 384U, NULL, 5U, NULL);
  if (result != pdPASS) return false;
  result = xTaskCreate(MechanismTask, "mechanism", 320U, NULL, 3U, NULL);
  if (result != pdPASS) return false;
  result = xTaskCreate(RobotFsmTask, "robot_fsm", 320U, NULL, 2U, NULL);
  return result == pdPASS;
}

uint32_t AppTime_GetMs(void) { return app_time_ms; }
void AppTime_IncrementFromSysTick(void) { ++app_time_ms; }
