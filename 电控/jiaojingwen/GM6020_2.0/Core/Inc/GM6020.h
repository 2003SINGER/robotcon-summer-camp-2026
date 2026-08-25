/*
 * GM6020.h
 *
 *  Created on: Aug 23, 2026
 *      Author: Ivy
 */

#ifndef INC_GM6020_H_
#define INC_GM6020_H_

#include "CAN_6020.h"

// 初始化电机控制
void GM6020_Init(void);

// 让电机转指定角度（度），正数逆时针，负数顺时针
// 内部累加到目标角度
void GM6020_Turn_Degrees(float angle_deg);

// 重置目标角度到当前位置（归零）
void GM6020_Reset_Position(void);

// 获取当前角度（度）
float GM6020_Get_Current_Angle(void);

// 获取目标角度（度）
float GM6020_Get_Target_Angle(void);

// 控制循环（每1ms调用一次）
void GM6020_Update(void);

#endif /* INC_GM6020_H_ */
