#include "command_mailbox.h"
#include "FreeRTOS.h"
#include "queue.h"

static StaticQueue_t queue_storage;
static uint8_t queue_buffer[sizeof(RobotCommand)];
static QueueHandle_t command_queue;
static volatile uint32_t drop_count;

bool CommandMailbox_Init(void)
{
  command_queue = xQueueCreateStatic(1U, sizeof(RobotCommand), queue_buffer, &queue_storage);
  drop_count = 0U;
  return command_queue != NULL;
}

bool CommandMailbox_Submit(const RobotCommand *command)
{
  if (command_queue == NULL || command == NULL) return false;
  if (xQueueOverwrite(command_queue, command) != pdPASS) {
    ++drop_count;
    return false;
  }
  return true;
}

bool CommandMailbox_SubmitFromIsr(const RobotCommand *command, bool *higher_priority_task_woken)
{
  BaseType_t woken = pdFALSE;
  if (command_queue == NULL || command == NULL) return false;
  if (xQueueOverwriteFromISR(command_queue, command, &woken) != pdPASS) {
    ++drop_count;
    return false;
  }
  if (higher_priority_task_woken != NULL) *higher_priority_task_woken = woken == pdTRUE;
  return true;
}

bool CommandMailbox_Take(RobotCommand *command)
{
  return command_queue != NULL && command != NULL && xQueueReceive(command_queue, command, 0U) == pdPASS;
}

uint32_t CommandMailbox_GetDropCount(void) { return drop_count; }
