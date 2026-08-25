#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <stdint.h>
#include "stm32h7xx.h"

#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  1
#define configUSE_TICKLESS_IDLE                  0
#define configCPU_CLOCK_HZ                       ((uint32_t)500000000)
#define configTICK_RATE_HZ                       ((TickType_t)1000)
#define configMAX_PRIORITIES                     7
#define configMINIMAL_STACK_SIZE                 ((uint16_t)192)
#define configTOTAL_HEAP_SIZE                    ((size_t)16384)
#define configMAX_TASK_NAME_LEN                  16
#define configUSE_16_BIT_TICKS                   0
#define configIDLE_SHOULD_YIELD                  1
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              0
#define configUSE_COUNTING_SEMAPHORES            0
#define configQUEUE_REGISTRY_SIZE                0
#define configUSE_APPLICATION_TASK_TAG           0
#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_TIMERS                         0
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configUSE_MALLOC_FAILED_HOOK             0
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_TRACE_FACILITY                 0
#define configGENERATE_RUN_TIME_STATS            0
#define configUSE_STATS_FORMATTING_FUNCTIONS     0
#define configUSE_CO_ROUTINES                    0
#define configUSE_NEWLIB_REENTRANT               0
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         0

#define configPRIO_BITS                          __NVIC_PRIO_BITS
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY          (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

#define vPortSVCHandler                          SVC_Handler
#define xPortPendSVHandler                       PendSV_Handler

#define INCLUDE_vTaskPrioritySet                 0
#define INCLUDE_uxTaskPriorityGet                0
#define INCLUDE_vTaskDelete                      0
#define INCLUDE_vTaskCleanUpResources            0
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_xTaskAbortDelay                  0
#define INCLUDE_xTaskGetHandle                   0
#define INCLUDE_xTaskGetCurrentTaskHandle        0
#define INCLUDE_uxTaskGetStackHighWaterMark      1

void AppTime_IncrementFromSysTick(void);
void xPortSysTickHandler(void);

#endif
