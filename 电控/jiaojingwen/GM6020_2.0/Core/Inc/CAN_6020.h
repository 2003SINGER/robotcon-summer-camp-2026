#ifndef INC_CAN_6020_H_
#define INC_CAN_6020_H_

#include "main.h"

// ====== 初始化 ======
// 包含：启动 CAN、开启中断、配置过滤器
void CAN_6020_Init(void);

// ====== 发送 ======
// 发送电压到 GM6020（ID=0x1FF，数据放字节2-3）
void CAN_6020_Send_Voltage(int16_t voltage_mV);

// ====== 接收 ======
// 获取最新接收到的原始角度（0~8191），由中断更新
uint16_t CAN_6020_Get_Raw_Angle(void);

// 检查是否有新的角度数据（用于主循环判断）
uint8_t CAN_6020_Is_Angle_Updated(void);

// 清除角度更新标志（读取后调用）
void CAN_6020_Clear_Angle_Flag(void);

#endif
