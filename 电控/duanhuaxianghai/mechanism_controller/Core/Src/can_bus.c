#include "can_bus.h"
#include "board_config.h"
#include "fdcan.h"
#include "mechanism.h"
#include "tuning.h"

static volatile uint32_t tx_drop_count = 0U;
static volatile uint32_t consecutive_tx_drop_count = 0U;
/* Deliberately global for a first bench session: CLion/ST-Link can watch this
 * symbol without adding a UART protocol before CAN IDs are known. */
volatile CanBusDiagnostics g_can_bus_diagnostics;

static FDCAN_HandleTypeDef *HandleFor(MotorCan bus)
{
  return bus == MOTOR_CAN2 ? &hfdcan2 : &hfdcan1;
}

static void AddExactFilter(MotorCan bus, uint32_t index, uint16_t identifier)
{
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = index;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = identifier;
  filter.FilterID2 = 0x7FFU;
  if (HAL_FDCAN_ConfigFilter(HandleFor(bus), &filter) != HAL_OK) Error_Handler();
}

static void AddAcceptAllStandardFilter(MotorCan bus, uint32_t index)
{
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = index;
  filter.FilterType = FDCAN_FILTER_RANGE;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0U;
  filter.FilterID2 = 0x7FFU;
  if (HAL_FDCAN_ConfigFilter(HandleFor(bus), &filter) != HAL_OK) Error_Handler();
}

void CanBus_Init(void)
{
  g_can_bus_diagnostics = (CanBusDiagnostics){0};
  for (MotorCan bus = MOTOR_CAN1; bus <= MOTOR_CAN2; ++bus) {
    uint32_t filter_index = 0U;
    for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) {
      const MotorProfile *profile = Tuning_GetMotorProfile(id);
      if (profile->motor.can == bus) AddExactFilter(bus, filter_index++, profile->motor.feedback_id);
    }
    if (CAN_DIAGNOSTIC_ACCEPT_ALL_STANDARD_IDS != 0U) AddAcceptAllStandardFilter(bus, filter_index++);
    if (HAL_FDCAN_ConfigGlobalFilter(HandleFor(bus), FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(HandleFor(bus)) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(HandleFor(bus), FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
      Error_Handler();
    }
  }
}

static void StoreBe16(uint8_t *destination, int16_t value)
{
  destination[0] = (uint8_t)(((uint16_t)value >> 8) & 0xFFU);
  destination[1] = (uint8_t)((uint16_t)value & 0xFFU);
}

static void Send(MotorCan bus, uint16_t identifier, const uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = identifier;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  FDCAN_HandleTypeDef *handle = HandleFor(bus);
  if (HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0U ||
      HAL_FDCAN_AddMessageToTxFifoQ(handle, &header, (uint8_t *)data) != HAL_OK) {
    ++tx_drop_count;
    ++consecutive_tx_drop_count;
  } else {
    consecutive_tx_drop_count = 0U;
  }
}

void CanBus_SendM2006Currents(MotorCan bus, int16_t first, int16_t second, int16_t third)
{
  uint8_t data[8] = {0};
  StoreBe16(&data[0], first);
  StoreBe16(&data[2], second);
  StoreBe16(&data[4], third);
  Send(bus, CAN_ID_M2006_COMMAND_GROUP, data);
}

void CanBus_SendGM6020Current(MotorCan bus, int16_t rotator)
{
  uint8_t data[8] = {0};
  StoreBe16(&data[2], rotator); /* ID 0x206 uses group 0x1FF slot 2. */
  Send(bus, CAN_ID_GM6020_COMMAND_GROUP, data);
}

uint32_t CanBus_GetTxDropCount(void) { return tx_drop_count; }
uint32_t CanBus_GetConsecutiveTxDropCount(void) { return consecutive_tx_drop_count; }

bool CanBus_GetDiagnostics(CanBusDiagnostics *destination)
{
  if (destination == NULL) return false;
  *destination = g_can_bus_diagnostics;
  destination->tx_drop_count = tx_drop_count;
  return true;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t notifications)
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8];
  MotorCan bus;
  if (hfdcan->Instance == FDCAN1) bus = MOTOR_CAN1;
  else if (hfdcan->Instance == FDCAN2) bus = MOTOR_CAN2;
  else return;
  if ((notifications & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) == HAL_OK) {
      bool known = false;
      ++g_can_bus_diagnostics.rx_frame_count;
      g_can_bus_diagnostics.last_bus = (uint8_t)bus;
      g_can_bus_diagnostics.last_identifier = (uint16_t)header.Identifier;
      for (uint32_t index = 0U; index < sizeof(data); ++index) g_can_bus_diagnostics.last_data[index] = data[index];
      g_can_bus_diagnostics.last_rx_ms = HAL_GetTick();
      for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) {
        const MotorProfile *profile = Tuning_GetMotorProfile(id);
        if (profile->motor.can == bus && (uint16_t)header.Identifier == profile->motor.feedback_id) {
          known = true;
          break;
        }
      }
      if (!known) ++g_can_bus_diagnostics.rx_unknown_id_count;
      Mechanism_OnCanFeedback(bus, (uint16_t)header.Identifier, data, HAL_GetTick());
    }
  }
}
