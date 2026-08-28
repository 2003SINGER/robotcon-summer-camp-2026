#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  uint32_t control_tick_count;
  uint32_t control_deadline_miss_count;
  uint32_t safety_tick_count;
  uint16_t motor_stack_min_words;
  uint16_t safety_stack_min_words;
  uint16_t fsm_stack_min_words;
} AppRuntimeDiagnostics;

extern volatile AppRuntimeDiagnostics g_app_runtime_diagnostics;

bool AppTasks_Create(void);
uint32_t AppTime_GetMs(void);
void AppTime_IncrementFromSysTick(void);
bool AppTasks_GetDiagnostics(AppRuntimeDiagnostics *diagnostics);

#endif
