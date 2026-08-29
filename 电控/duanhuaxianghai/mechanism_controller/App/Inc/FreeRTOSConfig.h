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

/*
 * Keep this a plain assembler-safe literal.  The RVDS FreeRTOS port expands
 * configMAX_SYSCALL_INTERRUPT_PRIORITY inside an __asm block; STM32H7's
 * __NVIC_PRIO_BITS is 4U, and ARMCC5's assembler does not accept the U suffix.
 */
#define configPRIO_BITS                          4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY  15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
/* 15 << (8 - 4) = 240 and 5 << (8 - 4) = 80.  Keep these literal for
 * ARMCC5: the RVDS port embeds configMAX_SYSCALL_INTERRUPT_PRIORITY in
 * inline assembly, where its assembler rejects the shift expression. */
#define configKERNEL_INTERRUPT_PRIORITY          240
#define configMAX_SYSCALL_INTERRUPT_PRIORITY     80

/* The SVC and PendSV FreeRTOS handlers are naked assembly routines.  They
 * must be the vector entries themselves: calling either one through a normal
 * CubeMX C wrapper corrupts the exception stack on the first task switch. */
#define vPortSVCHandler                       SVC_Handler
#define xPortPendSVHandler                    PendSV_Handler

/* SysTick is an ordinary C handler and remains called from CubeMX's protected
 * SysTick block so application time and HAL time advance alongside the RTOS. */

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
