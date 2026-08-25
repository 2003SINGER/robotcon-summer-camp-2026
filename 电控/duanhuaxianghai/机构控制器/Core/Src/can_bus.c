#include "can_bus.h"
#include "board_config.h"
#include "fdcan.h"
#include "mechanism.h"

static volatile uint32_t tx_drop_count = 0U;

static void AddExactFilter(uint32_t index, uint16_t identifier)
{
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = index;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = identifier;
  filter.FilterID2 = 0x7FFU;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) Error_Handler();
}

void CanBus_Init(void)
{
  AddExactFilter(0U, CAN_ID_LOADER_MOTOR_A);
  AddExactFilter(1U, CAN_ID_LOADER_MOTOR_B);
  AddExactFilter(2U, CAN_ID_LIFT_MOTOR);
  AddExactFilter(3U, CAN_ID_ROTATOR_MOTOR);
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK ||
      HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
    Error_Handler();
  }
}

static void StoreBe16(uint8_t *destination, int16_t value)
{
  destination[0] = (uint8_t)(((uint16_t)value >> 8) & 0xFFU);
  destination[1] = (uint8_t)((uint16_t)value & 0xFFU);
}

static void Send(uint16_t identifier, const uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = identifier;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U ||
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header, (uint8_t *)data) != HAL_OK) {
    ++tx_drop_count;
  }
}

void CanBus_SendM2006Currents(int16_t loader_a, int16_t loader_b, int16_t lift)
{
  uint8_t data[8] = {0};
  StoreBe16(&data[0], loader_a);
  StoreBe16(&data[2], loader_b);
  StoreBe16(&data[4], lift);
  Send(CAN_ID_M2006_COMMAND_GROUP, data);
}

void CanBus_SendGM6020Current(int16_t rotator)
{
  uint8_t data[8] = {0};
  StoreBe16(&data[2], rotator); /* ID 0x206 uses group 0x1FF slot 2. */
  Send(CAN_ID_GM6020_COMMAND_GROUP, data);
}

uint32_t CanBus_GetTxDropCount(void) { return tx_drop_count; }

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t notifications)
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8];
  if (hfdcan->Instance != FDCAN1 || (notifications & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) == HAL_OK) {
      Mechanism_OnCanFeedback((uint16_t)header.Identifier, data, HAL_GetTick());
    }
  }
}
