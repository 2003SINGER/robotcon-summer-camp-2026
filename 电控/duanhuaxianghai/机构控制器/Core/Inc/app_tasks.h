#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdbool.h>
#include <stdint.h>

bool AppTasks_Create(void);
uint32_t AppTime_GetMs(void);
void AppTime_IncrementFromSysTick(void);

#endif
