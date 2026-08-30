# CubeMX 配置清单

这份清单用于以后在 `E:\STM32CubeMX` 里核对或重建 BSP；不要把任一旧工程的 `.ioc` 直接覆盖到新工程。

## 芯片与时钟

- MCU：`STM32H723ZGT6`，LQFP144；
- 外部晶振：25 MHz HSE；PLL 后 Cortex 为 500 MHz、HCLK 为 250 MHz、FDCAN kernel clock 为 125 MHz；
- FreeRTOS tick：1 kHz，抢占式、静态分配；任务由 `Core/Src/app_tasks.c` 创建，不要在 CubeMX 额外生成 DefaultTask。

## FDCAN1

| 项目 | 值 |
| --- | --- |
| RX | PA11，AF9 |
| TX | PA12，AF9 |
| 协议 | Classic CAN，Normal mode，1 Mbps |
| 标称时序 | Prescaler 1，Seg1 99，Seg2 25（125 MHz 基准） |
| Rx FIFO0 | 至少 8 元素 |
| Tx FIFO/Queue | 至少 3 元素 |

FDCAN IRQ 可以优先级 0，因为中断内不调用任何 FreeRTOS API；它只取帧并更新电机反馈。所有 `...FromISR` API 若后来加入，必须把该 IRQ 优先级调到 5 或更低优先级（数值更大）。

## FDCAN2（板间通信预留）

| 项目 | 值 |
| --- | --- |
| RX | PB12，AF9 |
| TX | PB13，AF9 |
| 协议 | Classic CAN，Normal mode，1 Mbps |
| 标称时序 | 与 FDCAN1 相同：Prescaler 1，Seg1 99，Seg2 25 |
| **Message RAM Offset** | **54**（FDCAN1 为 0；该值由 FDCAN1 的过滤器与 FIFO 分配决定，不能随意写为 1） |

`main()` 会执行 `MX_FDCAN2_Init()`，`can_bus.c` 已为 CAN2 配置标准帧 `0x123` 的精确过滤器、FIFO0 中断与收发绑定。`data[0]` 为 ASCII `'A'` / `'B'` / `'C'` / `'D'` 时，`Chassis_OnCanFeedback()` 仅锁存最新任务字母；不允许在 CAN 回调里直接调用 `Mechanism_Move...`。它仍是机构板与底盘/气路板的独立总线（与 FDCAN1 的四台电机总线分离）。

## USART3

| 项目 | 值 |
| --- | --- |
| TX | PB10，AF7 |
| RX | PB11，AF7 |
| 波特率 | 115200，8N1 |

USART3 已在启动时初始化，当前只作为预留调试/上位机接口，尚未定义通讯协议或启动接收中断。

## 必须避免的冲突

- 不要同时启用 PB8/PB9 与 PA11/PA12 的 FDCAN1；本板选择 PA11/PA12。
- FDCAN1 与 FDCAN2 的 Message RAM Offset 必须保持不同（当前 0 / 54）；都在 0 会导致两路同时收发时消息 RAM 冲突。
- 不要保留旧项目的 TIM2 PID 中断。控制周期来自 FreeRTOS 的 MotorControlTask（1 ms）。
- 不要把旧项目的阻塞 `HAL_Delay()`、启动后自动 180° 翻转、或目标 38/35 圈测试动作放回 `main()`。
- **SVC/PendSV 必须由 FreeRTOS 端口直接提供**（`FreeRTOSConfig.h` 中 `vPortSVCHandler`/`xPortPendSVHandler` 映射到向量名，CubeMX 生成的同名 C 包装须保持在 `#if 0` 内）。多套一层普通 C 函数会在首次任务切换时破坏异常栈，整个调度器静默冻结（2026-08-27 的事故根因）。
- SysTick/PendSV/SVC 属于 FreeRTOS。若 CubeMX 重生成中断文件，须保留 `FreeRTOSConfig.h` 的 FreeRTOS handler 映射和 `SysTick_Handler()` 中的 `HAL_IncTick()`、`AppTime_IncrementFromSysTick()`、`xPortSysTickHandler()` 调用。

`mechanism_controller.ioc` 只作为引脚基线，当前可构建入口是 `CMakeLists.txt`，无需重新生成即可使用。
