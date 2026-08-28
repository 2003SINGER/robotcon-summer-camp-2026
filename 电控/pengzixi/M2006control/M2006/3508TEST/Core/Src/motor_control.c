#include "motor_control.h"
#include "pid.h"  // 后续添加

static Motor_Handle_t motor;
volatile uint8_t new_feedback_available = 0;
// 发送电流指令[reference:25]
void Motor_SendCurrent(int16_t current) 
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];
    uint32_t tx_mailbox;

    // 限制电流范围
    if (current > CURRENT_MAX) current = CURRENT_MAX;
    if (current < CURRENT_MIN) current = CURRENT_MIN;

    // 配置发送帧头[reference:26]
    tx_header.Identifier = MOTOR_CAN_ID;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    // 数据域：高8位在前，低8位在后[reference:27]
    tx_data[0] = (current >> 8) & 0xFF;
    tx_data[1] = current & 0xFF;
    tx_data[2] = 0;
    tx_data[3] = 0;
    tx_data[4] = 0;
    tx_data[5] = 0;
    tx_data[6] = 0;
    tx_data[7] = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
}

// 接收电机反馈数据[reference:28]
uint8_t Motor_ReceiveFeedback(Motor_Feedback_t *fb) 
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) 
    {
        // 判断是否为电机反馈帧（ID 0x201-0x204）
        if (rx_header.Identifier >= 0x201 && rx_header.Identifier <= 0x204) 
        {
            fb->angle = (rx_data[0] << 8) | rx_data[1];      // 角度[reference:29]
            fb->speed_rpm = (int16_t)((rx_data[2] << 8) | rx_data[3]);  // 速度[reference:30]
            fb->current = (int16_t)((rx_data[4] << 8) | rx_data[5]);   // 电流[reference:31]
            fb->temperature = rx_data[6];                    // 温度[reference:32]
            return 1;  // 接收成功
        }
    }
    return 0;  // 无数据
}

void Motor_Init(void) 
{
    // 启动FDCAN[reference:33]   
    
    // 使能接收FIFO0通知（如果使用中断）
    // HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void Motor_Update(void) 
{
 /*   
	 Motor_Feedback_t fb;
    static uint32_t last_print = 0;
    Motor_SendCurrent(1000);
    // ★★★ 1. 只接收，绝对不要调用 Motor_SendCurrent ★★★
    // 因为发大电流电机会锁住，你就转不动轴来测反馈了
    if (Motor_ReceiveFeedback(&fb))
    {
        // ★★★ 2. 每隔 200ms 打印一次（防止刷爆串口）★★★
        if (HAL_GetTick() - last_print > 200)
        {
            printf("Speed:%d RPM, Current:%d, Angle:%d\n", 
                   fb.speed_rpm, fb.current, fb.angle);
            last_print = HAL_GetTick();
        }
    }
    else
    {
        // 如果收不到，每秒提示一次
        if (HAL_GetTick() - last_print > 1000)
        {
            printf("Waiting for CAN feedback...\n");
            last_print = HAL_GetTick();
        }
    }*/
	
    Motor_Feedback_t fb;   
   
	// 在定时器中断中调用：接收反馈 → PID计算 → 发送指令
    if (Motor_ReceiveFeedback(&motor.feedback)) 
    {       
       Motor_Feedback_t fb ;
			
    }//这个是原来的函数		
			//这里原来就没有函数 PID计算（见下文）
        // int16_t output = PID_Calculate(&pid, motor.feedback.speed_rpm, target_speed);
	// Motor_SendCurrent(output);   //这是要用的函数*/
    }


// motor_control.c

// 1. 定义一个全局标志位，告诉主循环有数据来了
volatile uint8_t motor_update_flag = 0; 

// 2. 定义一个全局变量，存放最新解析出的电机数据
Motor_Feedback_t motor_feedback_data;

// 3. 实现中断处理函数（由 stm32h7xx_it.c 调用）
void Motor_IRQ_Handler(void)
{
    Motor_Feedback_t temp_fb; // 临时变量，用于接收解析结果
    
    // 第一步：调用你的拆包函数，从硬件FIFO读取并解析
    // 注意：Motor_ReceiveFeedback 内部会调用 HAL_FDCAN_GetRxMessage
    if (Motor_ReceiveFeedback(&temp_fb) == 1) 
    {
        // 第二步：解析成功，将数据备份到全局变量中（供主循环使用）
        // 为了防止中断中数据被主循环读乱，建议关中断或直接赋值（H7是32位，赋值是原子的，结构体可直接复制）
        motor_feedback_data.angle = temp_fb.angle;
        motor_feedback_data.speed_rpm = temp_fb.speed_rpm;
        motor_feedback_data.current = temp_fb.current;
        motor_feedback_data.temperature = temp_fb.temperature;
        
        // 第三步：置位标志位，通知主循环（main.c）来做PID计算
        // 主循环中检测到 motor_update_flag == 1，就读取 motor_feedback_data 并计算
        motor_update_flag = 1;
			new_feedback_available =1;
    }
}

