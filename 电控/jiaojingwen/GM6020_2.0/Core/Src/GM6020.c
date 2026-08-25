/*
 * GM6020.c
 *
 *  Created on: Aug 23, 2026
 *      Author: Ivy
 */
#include "GM6020.h"
#include <math.h>

// ====== 常量 ======
#define ANGLE_MAX_RAW  8192.0f
#define VOLTAGE_LIMIT  20000

// ====== PID 参数 ======
#define KP             60.0f    // 比例项：保证响应力度，配合D项可略降
#define KI             8.0f    // 积分项：消除静差，不宜过大
#define KD             10.0f    // 微分项：核心阻尼，越大越稳，太大会反应迟钝
// ====== 私有变量 ======
static float current_angle = 0.0f;   // 当前角度（度）
static float target_angle = 0.0f;    // 目标角度（度）
// 多圈跟踪变量
static uint8_t  first_measure = 1;
static uint16_t prev_raw = 0;

//积分项变量
static float integral = 0.0f;
static float last_error = 0.0f;    // 上一次误差，用于计算D项
static float integral_limit = 8000.0f;  // 积分限幅（防止积分饱和）

// ====== 内部函数：更新当前角度 ======
static void update_current_angle(void)
{
    uint16_t raw = CAN_6020_Get_Raw_Angle();

    if (first_measure) {
        current_angle = (float)raw * 360.0f / 8192.0f;
        prev_raw = raw;
        first_measure = 0;
        return;
    }

    int32_t delta = raw - prev_raw;
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;

    current_angle += (float)delta * 360.0f / 8192.0f;
    prev_raw = raw;
}

// ====== 公开函数：初始化 ======
void GM6020_Init(void)
{
    CAN_6020_Init();
    first_measure = 1;
    current_angle = 0.0f;
    target_angle = 0.0f;
    integral = 0.0f;
    last_error = 0.0f;

    // 等待第一帧反馈到达（最多等 50ms）
        uint32_t timeout = 50;
        while (!CAN_6020_Is_Angle_Updated() && timeout > 0) {
            HAL_Delay(1);
            timeout--;
        }

        // 如果收到了反馈，更新当前角度
        if (CAN_6020_Is_Angle_Updated()) {
            CAN_6020_Clear_Angle_Flag();
            update_current_angle();
            // 关键：把目标角度设为当前角度，让电机保持在当前位置
            target_angle = current_angle;
        }
}

// ====== 公开函数：累加目标角度 ======
void GM6020_Turn_Degrees(float angle_deg)
{
    target_angle += angle_deg;
}

// ====== 公开函数：归零 ======
void GM6020_Reset_Position(void)
{
    target_angle = 0.0f;
}

// ====== 公开函数：获取当前角度 ======
float GM6020_Get_Current_Angle(void)
{
    return current_angle;
}

// ====== 公开函数：获取目标角度 ======
float GM6020_Get_Target_Angle(void)
{
    return target_angle;
}

void GM6020_Update(void)
{
    // 1. 如果有新反馈，更新当前角度
    if (CAN_6020_Is_Angle_Updated()) {
        CAN_6020_Clear_Angle_Flag();
        update_current_angle();
    }

    // 2. 计算误差
    float error = target_angle - current_angle;

    // 3.积分累加（带限幅）
    if(fabsf(error)<0.5f)
    {
       integral += error;
       if (integral > integral_limit) integral = integral_limit;
       if (integral < -integral_limit) integral = -integral_limit;
    }
    else
    {
    	integral=0.0f;
    }

    // 4.微分项D：误差变化率 × KD，提供阻尼
        float derivative = error - last_error;
        last_error = error;

    // 5.PID计算输出电压
        float voltage = KP * error + KI * integral + KD * derivative;

    // 6.输出限幅
        if (voltage > VOLTAGE_LIMIT) voltage = VOLTAGE_LIMIT;
        if (voltage < -VOLTAGE_LIMIT) voltage = -VOLTAGE_LIMIT;

    // 7.发送控制指令
            CAN_6020_Send_Voltage((int16_t)voltage);
        }
/* 开环测试：直接发送固定电压15000
    CAN_6020_Send_Voltage(15000);*/
