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

static void AddExactFilter(FDCAN_HandleTypeDef *hfdcan, uint32_t index, uint16_t identifier)
{
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = index;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = identifier;
  filter.FilterID2 = 0x7FFU;
  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) Error_Handler();
}

static void AddAcceptAllStandardFilter(FDCAN_HandleTypeDef *hfdcan, uint32_t index)
{
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = index;
  filter.FilterType = FDCAN_FILTER_RANGE;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0U;
  filter.FilterID2 = 0x7FFU;
  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) Error_Handler();
}

void CanBus_Init(void)
{
  g_can_bus_diagnostics = (CanBusDiagnostics){0};
  for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) {
    AddExactFilter(&hfdcan1, (uint32_t)id, Tuning_GetMotorProfile(id)->motor.feedback_id);
  }
  if (CAN_DIAGNOSTIC_ACCEPT_ALL_STANDARD_IDS != 0U)
    AddAcceptAllStandardFilter(&hfdcan1, TUNING_MOTOR_COUNT);
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK ||
      HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
    Error_Handler();
  }

  /* Chassis CAN is physically independent from the motor CAN.  Its protocol
   * has not been frozen yet, so receive standard frames for diagnostics only.
   * Do not interpret a payload or alter the FSM here. */
  AddAcceptAllStandardFilter(&hfdcan2, 0U);
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK ||
      HAL_FDCAN_Start(&hfdcan2) != HAL_OK ||
      HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U) != HAL_OK) {
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
    ++consecutive_tx_drop_count;
  } else {
    consecutive_tx_drop_count = 0U;
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

void CanBus_SendGM6020Voltage(int16_t voltage_mV)
{
  uint8_t data[8] = {0};
  StoreBe16(&data[2], voltage_mV); /* ID 0x206 uses group 0x1FF slot 2. */
  Send(CAN_ID_GM6020_COMMAND_GROUP, data);
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
  if ((notifications & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U) return;
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) == HAL_OK) {
      if (hfdcan->Instance == FDCAN2) {
        ++g_can_bus_diagnostics.chassis_rx_frame_count;
        g_can_bus_diagnostics.chassis_last_identifier = (uint16_t)header.Identifier;
        for (uint32_t index = 0U; index < sizeof(data); ++index)
          g_can_bus_diagnostics.chassis_last_data[index] = data[index];
        g_can_bus_diagnostics.chassis_last_rx_ms = HAL_GetTick();
        continue;
      }
      if (hfdcan->Instance != FDCAN1) continue;
      bool known = false;
      ++g_can_bus_diagnostics.rx_frame_count;
      g_can_bus_diagnostics.last_identifier = (uint16_t)header.Identifier;
      for (uint32_t index = 0U; index < sizeof(data); ++index) g_can_bus_diagnostics.last_data[index] = data[index];
      g_can_bus_diagnostics.last_rx_ms = HAL_GetTick();
      for (TuningMotorId id = TUNING_MOTOR_LOADER_A; id < TUNING_MOTOR_COUNT; ++id) {
        if ((uint16_t)header.Identifier == Tuning_GetMotorProfile(id)->motor.feedback_id) {
          known = true;
          ++g_can_bus_diagnostics.motor_feedback_frame_count[id];
          g_can_bus_diagnostics.motor_last_feedback_ms[id] = HAL_GetTick();
          break;
        }
      }
      if (!known) ++g_can_bus_diagnostics.rx_unknown_id_count;
      Mechanism_OnCanFeedback((uint16_t)header.Identifier, data, HAL_GetTick());
    }
  }
}
