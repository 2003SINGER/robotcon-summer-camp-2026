/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "pid.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ANGLE_DEG_TO_RAW(deg)  ((deg) / 360.0f * 8192.0f)//这是角度环相关的宏定义
#define ANGLE_RAW_TO_DEG(raw)  ((raw) * 360.0f /8192.0f)//这是角度环相关的宏定义
//状态机与目标位置
typedef enum {
    STATE_INIT,           // 初始位置（最底下）
    STATE_GO_TO_POS1,     // 前往第一个高度（任务一三层高）
    STATE_HOLD_POS1,      // 在第一个高度保持（三层）
    STATE_GO_TO_POS2,     // 前往第二个高度（最高）
    STATE_HOLD_POS2,      // 在第二个高度保持（最高）
	  STATE_GO_TO_POS3,     // 前往第二个高度（最高）
    STATE_HOLD_POS3,  
	STATE_GO_TO_POS4,     // 前往第二个高度（最高）
    STATE_HOLD_POS4, 
	STATE_GO_TO_POS5,     // 前往第二个高度（最高）
    STATE_HOLD_POS5, 
    STATE_GO_BACK,        // 回到初始高度（最下面？）
    STATE_HOLD_BACK       // 在初始高度保持（完成）
} MotorState_t;
//定义高度的圈数，先用圈数，后面再换高度吧
#define POS_INITIAL  0.0f     // 初始位置（0圈）
#define POS_LEVEL1   123.2f     // 第一个高度（1.5圈）
#define POS_LEVEL2   0.0f  // 第二个高度（3.0圈）这个数据是乱写的，别懒，去量
#define POS_LEVEL3   0.0f  
//测试用保持时间，在某处暂停，后续看反馈
#define HOLD_TIME_MS 5000//停留5s
//位置到达判定阈值（没看懂这半圈是干嘛的）
#define POS_TOLERANCE 0.5f
#define GRAVITY_COMPENSATION  755.0   // 抵消重力用的前馈 755.0   
#include "stm32h7xx.h"                  // Device header
#define MAX_FEEDFORWARD_LIMIT  (CURRENT_MAX * 0.6f)//前馈限幅
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint32_t loop_counter = 0;
float  setpoint= 2500;//这个会被位置环输出覆盖的
PID_Handle_t speed_pid;
PID_Handle_t pos_pid;                   //这是角度环相关的宏定义

MotorState_t motor_state = STATE_INIT;      // 当前状态
uint32_t hold_start_tick = 0;               // 进入保持时刻
float target_revolutions =0.0f;     // 当前目标圈数（位置环设定值）float target_revolutions = POS_INITIAL;
float current_revolutions = 0.0f;   // 当前圈数，是累计吗？
uint16_t last_angle = 0;
int32_t total_rounds = 0;
uint8_t first_angle_flag = 1;  
float gravity_feedforward = 0; 
float gf_unloaded = 755.0f;    // 空载前馈（你之前测的值）
float gf_loaded = 1150.0f;      // 带载前馈（刚测出来的）
float target_gf = 755.0f;       // 目标前馈值（用于斜坡过渡）
float gf_step = 10.0f;          // 每 10ms 变化步长（mA）

//串口接受用
uint8_t uart_rx_buffer[1];     // 单字节接收缓存
char uart_cmd_buffer[64];           // 命令字符串缓存
uint8_t cmd_index = 0;       // 当前缓存位置
volatile uint8_t uart_cmd_ready = 0; //标志位，收到1代表收到完整命令
//uint8_t control_mode = 1;              //多环控制用到的，测试调pid的时候记得注释掉

float speed_target = 0.0f;
float output = 0.0f;
extern volatile uint8_t new_feedback_available;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//int16_t last_current = 0;   //上次发送的电流
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
//float  setpoint= 1000;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
    Motor_Init();      // ����CAN������ motor_control.c ��ĺ�����
			HAL_TIM_Base_Start_IT(&htim1);
    PID_Init(&speed_pid, 5.0f, 0.3f, 0.05f, 8000.0f);//这个是具体的pid值，然后就是kp小了第一版值是0.5，现在要调大一点？ 1.8f, 0.6f, 0.03f,8000.0f
		
PID_Init(&pos_pid, 250.0f, 0.05f, 6.0f, 5500.0f); //这是角度环相关的pid参数200 0.02 0.8
HAL_UART_Receive_IT(&huart3, uart_rx_buffer, 1); //串口接受pid参数用于调参
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		/*  if (uart_cmd_ready == 1)
    {
        uart_cmd_ready = 0;
        parse_uart_command(uart_cmd_buffer);  // 改 PID 参数
    }*/
			// Motor_SendCurrent((int16_t)output);
			//下面的是串口打印用的
			loop_counter++;
    if (loop_counter >= 20) {
        loop_counter = 0;
        // 打印当前状态，如果反馈标志为0则提示，但不要刷屏
         static uint8_t no_fb_cnt = 0;

        if (new_feedback_available == 1) {
            // 有新反馈，打印数据
            new_feedback_available = 0;   // 清除标志
            no_fb_cnt = 0;                // 重置无反馈计数
            printf("%.2f,%.2f,%.1f,%.1f,%.1f\r\n",
                   target_revolutions,
                   current_revolutions,
                   setpoint,
                   (float)motor_feedback_data.speed_rpm,
                   output);
        } else {
            // 无新反馈，累计警告
            no_fb_cnt++;
            if (no_fb_cnt >= 10) {
							// 连续10次（2秒）无反馈才警告
                no_fb_cnt = 0;
                printf("Warning: No CAN feedback\n");
            }
        }
			}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void Update_Current_Revolutions(uint16_t new_angle)
{
    // 首次上电：记录初始角度，累计圈数从0开始
       if (first_angle_flag) {
        last_angle = new_angle;
        total_rounds = 0;
        current_revolutions = (float)new_angle / 8192.0f;
        first_angle_flag = 0;
        return;
    }

    // ★★★ 关键修复：使用 int32_t 计算差值，避免截断 ★★★
    int32_t delta = (int32_t)new_angle - (int32_t)last_angle;

    // 如果变化超过半圈，说明跨过了 0 点
    if (delta >  4096) {
        total_rounds--;   // 反向跨过0（角度减小）
    } else if (delta < -4096) {
        total_rounds++;   // 正向跨过0（角度增大）
    }

    // 更新上次角度
    last_angle = new_angle;

    // 当前总圈数 = 整数圈 + 当前单圈百分比
    current_revolutions = (float)total_rounds + (float)new_angle / 8192.0f;
}

//串口信息接受
void parse_uart_command(char *cmd)
{
    char key[10];
    float value = 0.0f;
    
    // 尝试解析 "key value" 格式（例如 "kp 1.5"）
    if (sscanf(cmd, "%s %f", key, &value) == 2)
    {
        // 位置环的pid
        if (strcmp(key, "kp") == 0)      speed_pid.Kp = value;
        else if (strcmp(key, "ki") == 0) speed_pid.Ki = value;
        else if (strcmp(key, "kd") == 0) speed_pid.Kd = value;
        
        // 位置环
        else if (strcmp(key, "pkp") == 0) pos_pid.Kp = value;
        else if (strcmp(key, "pki") == 0) pos_pid.Ki = value;
        else if (strcmp(key, "pkd") == 0) pos_pid.Kd = value;
        
        //重力前馈
        else if (strcmp(key, "gf") == 0)  gravity_feedforward = value;
        
        // 发送当前数据到电脑
        else if (strcmp(key, "show") == 0)
        {
            printf("Speed: Kp=%.2f Ki=%.2f Kd=%.4f\r\n", speed_pid.Kp, speed_pid.Ki, speed_pid.Kd);
            printf("Pos:   Kp=%.2f Ki=%.2f Kd=%.4f\r\n", pos_pid.Kp, pos_pid.Ki, pos_pid.Kd);
            printf("Gravity FF: %.2f mA\r\n", gravity_feedforward);
        }
        else
        {
            printf("Unknown cmd: %s\r\n", key);
        }
    }
    else
    {
        printf("Format error, use: kp 1.5\r\n");
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        char c = uart_rx_buffer[0];
        
        // 遇到换行符（回车或换行），结束一条命令
        if (c == '\r' || c == '\n')
        {
            if (cmd_index > 0)
            {
               uart_cmd_buffer[cmd_index] = '\0';        // 字符串结尾
                parse_uart_command(uart_cmd_buffer);      // 解析命令
                cmd_index = 0;                       // 复位缓存
            }
        }
        else
        {
            // 普通字符，存入缓存（防止溢出）
            if (cmd_index < 63) 
            {
                uart_cmd_buffer[cmd_index++] = c;
            }
        }
        
        // 记得重启下一次接受
        HAL_UART_Receive_IT(&huart3, uart_rx_buffer, 1);
    }
}



// 定义一个标志，用于通知主循环执行控制
volatile uint8_t timer1_tick = 0;

// TIM1 周期中断回调（由 HAL 库在 TIM1_UP_IRQHandler 中调用）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {

			if (motor_update_flag == 1)  		
    {    
        motor_update_flag = 0; 
			Update_Current_Revolutions(motor_feedback_data.angle);
			//电机状态切换，等会先测转一圈多少
			// ★ 前馈平滑过渡（每 10ms 执行一次）
if (gravity_feedforward < target_gf) {
    gravity_feedforward += gf_step;
    if (gravity_feedforward > target_gf) gravity_feedforward = target_gf;
} else if (gravity_feedforward > target_gf) {
    gravity_feedforward -= gf_step;
    if (gravity_feedforward < target_gf) gravity_feedforward = target_gf;
}

// 然后用 gravity_feedforward 作为前馈值
float feedforward = gravity_feedforward;
			switch (motor_state)
{
    case STATE_INIT:
			  target_gf = gf_unloaded;
        motor_state = STATE_GO_TO_POS1;//初始化完成后直接进入状态一
        break;

    case STATE_GO_TO_POS1:
    {      
        static uint8_t target_locked = 0;  // 使用静态变量，保证进入此状态只计算一次？没看懂
        if (target_locked == 0) {
            target_revolutions = current_revolutions + POS_LEVEL1; // 当前位置 +123.3圈
            target_locked = 1;
        }
        if (fabsf(current_revolutions - target_revolutions) < POS_TOLERANCE) {
            target_locked = 0;         //解锁进入下一个状态
            motor_state = STATE_HOLD_POS1;
            hold_start_tick = HAL_GetTick();
        }
    }
    break;

    case STATE_HOLD_POS1:
       if (HAL_GetTick() - hold_start_tick >= HOLD_TIME_MS) {
        /*  motor_state = STATE_GO_TO_POS2; //固定秒数后进入下一阶段，具体情况是收其他人的信号
				   target_gf = gf_loaded;//最好是吸盘完成后*/
        }
        break;

    case STATE_GO_TO_POS2:
    {
        static uint8_t target_locked_2 = 0;
        if (target_locked_2 == 0) {
            // 注意：此时 current_revolutions 已经在 1.5 圈位置
            // 再往上走 1.5 圈，正好到达 3.0 圈（即 POS_LEVEL2）
            target_revolutions = current_revolutions + 81.8; //1.0是随便写的，还要改
            target_locked_2 = 1;
        }
        if (fabsf(current_revolutions - target_revolutions) < POS_TOLERANCE) {
            target_locked_2 = 0;
            motor_state = STATE_HOLD_POS2;
            hold_start_tick = HAL_GetTick();
        }
    }
    break;

    case STATE_HOLD_POS2:
        if (HAL_GetTick() - hold_start_tick >= HOLD_TIME_MS) {
            motor_state =STATE_GO_TO_POS3; // 5秒后回到初始位置
        }
        break;

   case STATE_GO_TO_POS3:
{
    static uint8_t target_locked_3 = 0;
    if (target_locked_3 == 0) {
        target_revolutions = current_revolutions -81.8f; //第三个位置是从位置在往下几圈 
        target_locked_3 = 1;
    }
    if (fabsf(current_revolutions - target_revolutions) < POS_TOLERANCE) {
        target_locked_3 = 0;
        motor_state = STATE_HOLD_POS3;   // 切换到保持状态
        hold_start_tick = HAL_GetTick();
    }
}
break;

case STATE_HOLD_POS3:
    if (HAL_GetTick() - hold_start_tick >= HOLD_TIME_MS) {
        // 在 POS3 保持5秒后，再统一回到初始位置（调用 GO_BACK）
			  //target_gf = gf_unloaded;//放下箱子后
        motor_state = STATE_GO_TO_POS4;
    }
    break;
		
  case STATE_GO_TO_POS4:
{
    static uint8_t target_locked_3 = 0;
    if (target_locked_3 == 0) {
        target_revolutions = current_revolutions -72.0f; //第三个位置是从位置在往下几圈 
        target_locked_3 = 1;
    }
    if (fabsf(current_revolutions - target_revolutions) < POS_TOLERANCE) {
        target_locked_3 = 0;
        motor_state = STATE_HOLD_POS4;   // 切换到保持状态
        hold_start_tick = HAL_GetTick();
    }
}
break;
		case STATE_HOLD_POS4:
    if (HAL_GetTick() - hold_start_tick >= HOLD_TIME_MS) {
        // 在 POS3 保持5秒后，再统一回到初始位置（调用 GO_BACK）
			//  target_gf = gf_loaded;//吸盘吸到后
        motor_state = STATE_GO_TO_POS5;
    }
    break;
		
		  case STATE_GO_TO_POS5:
{
    static uint8_t target_locked_3 = 0;
    if (target_locked_3 == 0) {
        target_revolutions = current_revolutions +108.0f; //第三个位置是从位置在往下几圈 
        target_locked_3 = 1;
    }
    if (fabsf(current_revolutions - target_revolutions) < POS_TOLERANCE) {
        target_locked_3 = 0;
        motor_state = STATE_HOLD_POS5;   // 切换到保持状态
        hold_start_tick = HAL_GetTick();
    }
}
break;
		case STATE_HOLD_POS5:
  
    break;
    default:
        break;
}
				
		/*	static uint8_t target_locked = 0;
    if (target_locked == 0) {
        target_revolutions = current_revolutions+36.0;
        target_locked = 1;
    }
				*/

    float speed_target = PID_Calculate(&pos_pid, target_revolutions, current_revolutions);
    // 限幅（防止速度目标过大）
    if (speed_target > 5000.0f) speed_target = 5000.0f;
  if (speed_target < -4500.0f) speed_target = -4500.0f; 
   setpoint = speed_target;
		//float speed_target=-500.0;
		//setpoint=speed_target;
			
			//这个在速度环里也要用，不要注释掉了（下）
             output = PID_Calculate(&speed_pid, setpoint, motor_feedback_data.speed_rpm);
				// 添加pid前馈
         
		if (speed_target < -10.0) {
    feedforward=gravity_feedforward*0.6; 
}
//前馈的限幅保护
         if (feedforward > MAX_FEEDFORWARD_LIMIT) feedforward = MAX_FEEDFORWARD_LIMIT;
         if (feedforward < -MAX_FEEDFORWARD_LIMIT) feedforward = -MAX_FEEDFORWARD_LIMIT;
				//电流输出变成了pid前馈加上动态调节
         int16_t final_current = (int16_t)(output + feedforward);
				//总限幅
				if (final_current > CURRENT_MAX) final_current = CURRENT_MAX;
if (final_current < CURRENT_MIN) final_current = CURRENT_MIN;
               Motor_SendCurrent(final_current);
       // timer1_tick = 1;  			// 置位标志，主循环检测到后执行一次控制
			
    }
}
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
