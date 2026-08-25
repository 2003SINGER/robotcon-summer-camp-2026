/*
 * CAN_6020.c
 *
 *  Created on: Aug 23, 2026
 *      Author: Ivy
 */

#include "CAN_6020.h"
extern FDCAN_HandleTypeDef hfdcan1;

// ====== 私有变量 ======
static volatile uint16_t raw_angle = 0;
static volatile uint8_t  angle_updated = 0;

// ====== 公开函数：初始化（包含过滤器配置） ======
void CAN_6020_Init(void)
{
    // 1. 启动 CAN
    HAL_FDCAN_Start(&hfdcan1);

    // 2. 配置过滤器：只接收 GM6020 的反馈帧 (ID = 0x206)
    FDCAN_FilterTypeDef sFilterConfig = {0};
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x206;      // 允许通过的 ID
    sFilterConfig.FilterID2 = 0x7FF;      // 掩码：11 位全匹配
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    // 3. 开启接收中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

// ====== 发送电压 ======
void CAN_6020_Send_Voltage(int16_t voltage_mV)
{
    FDCAN_TxHeaderTypeDef TxHeader = {0};
    uint8_t TxData[8] = {0};

    if (voltage_mV > 30000) voltage_mV = 30000;
    if (voltage_mV < -30000) voltage_mV = -30000;

    // 数据放在字节2-3（ID=2 的 GM6020）
    TxData[2] = (uint8_t)((voltage_mV >> 8) & 0xFF);
    TxData[3] = (uint8_t)(voltage_mV & 0xFF);

    TxHeader.Identifier = 0x1FF;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

// ====== 获取原始角度 ======
uint16_t CAN_6020_Get_Raw_Angle(void)
{
    return raw_angle;
}

// ====== 检查角度是否更新 ======
uint8_t CAN_6020_Is_Angle_Updated(void)
{
   return angle_updated;
}

// ====== 清除更新标志 ======
void CAN_6020_Clear_Angle_Flag(void)
{
    angle_updated = 0;
}

// ====== CAN 接收中断回调 ======
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if (hfdcan->Instance == FDCAN1) {
        FDCAN_RxHeaderTypeDef RxHeader;
        uint8_t RxData[8];

        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
            if (RxHeader.Identifier == 0x206) {
                raw_angle = (uint16_t)((RxData[0] << 8) | RxData[1]);
                angle_updated = 1;
            }
        }
    }
}
