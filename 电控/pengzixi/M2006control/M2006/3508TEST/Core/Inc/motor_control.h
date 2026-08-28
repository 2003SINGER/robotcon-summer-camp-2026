#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "main.h"
#include "fdcan.h"

#define MOTOR_CAN_ID         0x200      // 控制帧ID[reference:17]
#define MOTOR_FEEDBACK_ID_BASE 0x201    // 电机1反馈ID基址

#define CURRENT_MAX          10000      // 最大电流值[reference:18]
#define CURRENT_MIN          -10000

typedef struct {
    uint16_t angle;          // 机械角度（0-8191，对应0-360°）[reference:19]
    int16_t speed_rpm;       // 转速（RPM）[reference:20]
    int16_t current;         // 实际电流[reference:21]
    uint8_t temperature;     // 温度[reference:22]
} Motor_Feedback_t;

typedef struct {
    int16_t target_current;  // 目标电流
    Motor_Feedback_t feedback;
} Motor_Handle_t;

void Motor_Init(void);
void Motor_SendCurrent(int16_t current);  // 发送电流指令[reference:23]
uint8_t Motor_ReceiveFeedback(Motor_Feedback_t *fb);  // 接收反馈[reference:24]
void Motor_Update(void);  // 更新电机状态（在定时器中断中调用）
void Motor_IRQ_Handler(void);//这个还没写具体的函数

extern volatile uint8_t motor_update_flag;//定义一个全局的变量，用来接受返回的信息
extern Motor_Feedback_t motor_feedback_data;//定义一个全局的变量，用来接受返回的信息
#endif