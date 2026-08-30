# mechanism_controller

这是夏令营机器人**机构板**的独立工程：一块 `STM32H723ZGT6` 控制 Z 轴 M2006、前部伸缩机构两台 M2006，以及翻转 GM6020。底盘和气路均是其他板，不在本工程直接驱动。

## 运行结构

```text
FDCAN1 ISR（只收电机反馈快照/总线诊断，不碰机构状态）
        ↓
FDCAN2 ISR（只锁存底盘任务字母，不碰机构状态）
        ↓
RobotFsmTask，20 ms：唯一的动作状态迁移与机构目标下发
        ↓
MotorControlTask，1 ms：四个闭环 → CAN1 电流帧
        ↑
SafetyTask，10 ms：反馈超时、CAN 发送异常、任务栈余量
```

四电机不各自开任务；伸缩双电机、Z 轴和翻转由同一个 1 ms 控制任务计算并在同一周期打包发出，因此不会因任务调度把两段伸缩拆散。FreeRTOS 任务与队列均改为**静态分配**，没有运行期堆分配；栈溢出直接进入机构故障。

## 流程状态与命令

| FSM 状态 | 含义 |
| --- | --- |
| `IDLE` | 空闲，等待取料请求。 |
| `LOADER_EXTENDING` | 前部伸缩机构向取料位运动。 |
| `WAIT_GRIP_CONFIRM` | 等待气路板/传感器确认吸附。 |
| `LOADER_RETRACTING` | 吸附确认后收回机构。 |
| `COMPLETE` | 本次取料流程完成。 |
| `FAULT` | 流程或机构故障，等待复位。 |

| 命令 | 含义 |
| --- | --- |
| `ARM` | 反馈正常后使能机构。 |
| `ESTOP` | 立即锁存故障并输出零电流。 |
| `START_RETRIEVE` | 开始一次前部取料。 |
| `SET_GRIP_CONFIRMED` | 写入吸附成功/失败状态。 |
| `RESET_SEQUENCE` | 复位流程；故障仍须先显式清除。 |

硬件与控制参数集中在 `App/Src/tuning.c`：四台电机的 CAN ID、正反方向、电流上限、双环 PID、重力前馈、到位容差以及最大速度/加速度。比赛动作的伸缩距离、各段 Z 轴高度和翻转角度集中在 `App/Src/robot_fsm.c` 顶部。`mechanism.c` 只处理机构状态与目标，`can_bus.c` 只处理 CAN 打包/反馈。

上电后处于 `DISARMED`；没有任何自动翻转、抬升或伸缩，也不会主动在 CAN1 发送电流帧。上层通信接入后，必须通过命令邮箱请求 `ARM`，并且四台电机均已收到新反馈，才会进入 `READY`。反馈超过 20 ms 未更新、CAN 发送连续异常、软行程越界、动作超时或持续堵转都会切到锁存故障并发送零电流；调用 `Mechanism_ClearFault()` 后才可重新使能。

## 已确认的映射

| 执行器 | 电机 | CAN 反馈 ID | 说明 |
| --- | --- | --- | --- |
| 前部伸缩 | M2006 A/B | `0x201` / `0x202` | 原工程的伸出目标分别为 38 / 35 圈，保留为独立原始目标。 |
| Z 轴 | M2006 | `0x203` | 单 CAN 地址规划为 C610 ID 3；须先在物理电调上设定。已预留重力前馈。 |
| 翻转 | GM6020 | `0x206` | 发送组 `0x1FF` 的第二槽位。 |

CAN1 已同步写入 `mechanism_controller.ioc`：PA11 (RX) / PA12 (TX)、1 Mbps 时序与关闭自动重传。本工程以 `xuke`、`jiaojingwen` 与 `pengzixi` 的实际源码共同使用的 PA11/PA12 为准；xuke 的 `.ioc` 中 PB8/PB9 是旧配置，不作为依据。系统时钟为 25 MHz HSE → 500 MHz Cortex、250 MHz HCLK、125 MHz FDCAN。

FDCAN2 使用 PB12 (RX) / PB13 (TX)、1 Mbps，与 FDCAN1 的电机总线物理隔离。两路共用 FDCAN Message RAM：FDCAN1 offset 为 `0`，FDCAN2 的生成配置为 `54`，不得改成 `0` 或 `1`。当前应用层只接收标准数据帧 `0x123`：`data[0]` 为 ASCII `'A'` / `'B'` / `'C'` / `'D'` 时，由 `Chassis_OnCanFeedback()` 锁存最新命令；CAN 回调不直接调用任何 `Mechanism_Move...` API。

当前比赛集成版本故意不由 CAN2 启动流程：上电后等待四台电机 CAN1 反馈新鲜，自动 Arm 并仅执行一次 `robot_fsm.c` 的 `case 'E'` 临时流程。CAN2 的 A/B/C/D 锁存已可用于后续恢复“底盘命令选择任务”模式，但现在不会改变自动 E 的流程。

首次去基地上电时，`CAN_DIAGNOSTIC_ACCEPT_ALL_STANDARD_IDS=1`，CAN1 会接收本地电机总线的全部标准帧，但仍保持 `DISARMED`。在 CLion 的 Live Watch 直接看：

- `g_can_bus_diagnostics.rx_frame_count`：是否真的收到总线帧；
- `g_can_bus_diagnostics.last_identifier` / `last_data`：最后一帧 ID 和 8 字节原始反馈；
- `g_can_bus_diagnostics.rx_unknown_id_count`：不在当前 `tuning.c` 表里的 ID 数；
- `g_app_runtime_diagnostics.control_deadline_miss_count` 和三个 `*_stack_min_words`：控制任务是否漏周期、余栈是否够。

确认四个 ID 后将该宏改为 `0` 后重新刷写；此时硬件过滤器只接收已登记的四台电机反馈。

## 构建与烧录

### 目录边界与 CubeMX 重生成

```text
Core/、Drivers/、cmake/stm32cubemx/  ← CubeMX 生成与维护
App/                                ← 机构控制代码，永不由 CubeMX 生成
MDK-ARM/、STM32CubeIDE/、CMakeLists.txt ← 三套构建入口，共用上面两层
```

`App/` 包含机构控制、PID、CAN、FSM 与手写 FreeRTOS；无论在 CubeMX 中选择 MDK、CMake 还是 STM32CubeIDE 生成，CubeMX 都不会删除或改写该目录。`Core/Src/main.c` 和 `Core/Src/stm32h7xx_it.c` 对 App 的调用均放在 `USER CODE BEGIN/END` 区内，重生成时会保留。

更新引脚、时钟或外设时，只打开根目录的 `mechanism_controller.ioc` 并 Generate Code；它固定使用 **CMake** 工具链。不要在这个主 `.ioc` 中轮换到 MDK 或 STM32CubeIDE 后再生成：CubeMX 会重写相应工程描述，MDK 可能把旧组和新组并存，造成重复编译。不要把 `App/` 中的文件复制回 `Core/`。

Keil 与 CubeIDE 都是已生成、已验证的同源工程：它们直接编译根目录的 `Core/`、`Drivers/` 与 `App/`。因此改完硬件配置后的正常顺序是 **CubeMX（CMake）生成 → VS Code/CMake 构建 → Keil 或 CubeIDE 按需重新构建**，而不是让三种工具链依次从同一个 `.ioc` 生成。若有人误切到 MDK 并重生成，运行 [tools/normalize-mdk-project.ps1](tools/normalize-mdk-project.ps1) 后再打开 Keil，它会移除重复的 CubeMX 源组并保留 `App/` 组。

推荐直接用 VS Code 打开本目录，按 `Ctrl+Shift+B` 并选择 **Build STM32 firmware**。它调用 [tools/build.ps1](tools/build.ps1)，再调用根目录 `CMakePresets.json` 的同一份 GNU 构建图；VS Code 不再维护第二份手写源码清单。

产物位于 `build/Debug/`：`mechanism_controller.elf/.hex/.bin`。打开 STM32CubeProgrammer，连接 ST-Link，选择 `mechanism_controller.bin`，下载地址填 `0x08000000` 后烧录。该目录不纳入 Git。

CLion 可直接使用 `Debug` / `Release` CMake preset；VS Code 的构建、烧录和 Cortex-Debug 也都指向同一套 `build/<preset>/` 产物。

### Keil MDK（队友可直接打开）

Keil 工程入口是 [MDK-ARM/mechanism_controller.uvprojx](MDK-ARM/mechanism_controller.uvprojx)。它与 CMake、CubeIDE 共用 `Core/`、`Drivers/` 和 `App/`；不要把代码复制进 MDK 文件夹。用 µVision 打开后选择 `mechanism_controller` target，按 `F7` 构建、`F8` 下载。该工程使用 ARM Compiler 5 的 FreeRTOS RVDS 端口；本地编译产物均已忽略，不应提交。

> CubeMX 重生成警示：`stm32h7xx_it.c` 的 `SysTick_Handler()` 必须保留 USER CODE 区中的 `AppTime_IncrementFromSysTick()`，以及后续的 `HAL_IncTick()`、`xPortSysTickHandler()` 调用；丢失后超时和轨迹时基会静默失准。

用于调试时，安装 VS Code 的 **Cortex-Debug** 扩展，连接 ST-Link 的 SWDIO、SWCLK、GND、3.3V 后按 `F5` 选择 **Debug STM32H723 via ST-Link**。不需要 USB：先运行几秒再暂停，在 Watch 中加入 `g_can_bus_diagnostics` 查看原始 CAN 帧和 ID，加入 `g_mechanism_telemetry` 查看四台电机已解码的总编码器、转速、电流、目标与 PID 状态。USB/串口只在后续需要 VOFA 不停机连续画波形时再接。

## 调参

### Z 轴台架模式

`board_config.h` 的三个 bench 模式（`Z_AXIS_BENCH_MODE` / `LOADER_BENCH_MODE` / `ROTATOR_BENCH_MODE`）当前均为 `0`：整机模式，四电机闭环。单轴台架调试时把对应宏改为 `1` 重新构建：仅该轴需要在线，其余电机槽位保持零电流。烧录、上电后仍是 `DISARMED`，不会自动运动。连接 ST-Link 并暂停/继续运行后，在 Watch 中修改：

| Watch 变量 | 填入值 |
| --- | --- |
| `g_z_axis_bench_command` | `1` 使能并保持当前点；`2` 移到 `g_z_axis_bench_target_counts`；`3` 回 0；`4` 急停；`5` 清故障。每个命令被消费后自动回到 0。 |
| `g_z_axis_bench_target_counts` | 相对 ARM 时刻的编码器目标。首次只填 `200`，确认正方向后再逐步增加。 |

台架模式将 Z 轴电流上限临时降为 `800`，前馈默认为 0。完成台架验证后必须把 `Z_AXIS_BENCH_MODE` 改回 `0`，再接入完整四电机机构。

日常默认参数只改 [App/Src/tuning.c](App/Src/tuning.c)，随后重新构建、刷写：

- `motor_profiles`：CAN ID、反馈/输出方向、限流、位置/速度 PID、重力前馈和到位容差；
- `robot_fsm.c` 顶部的 `FSM_*` 宏：每个比赛动作的伸缩目标、Z 轴高度、翻转角度；
- `motion_safety_profiles`：独立于比赛流程的保守软行程、动作超时和堵转阈值；上板后必须逐项确认，不应用其替代机械保护。
- `trajectory_max_velocity_counts_s` / `trajectory_max_acceleration_counts_s2`：位置目标的速度/加速度约束。目标不会一步跳到远端，而是以受限轨迹进入位置环。
- CAN ID 和正反向属于启动时总线过滤配置，必须改源码、重新刷写，不能运行时改。

PID 使用“测量值微分 + 一阶低通滤波”，避免设定值突变给 D 项造成电流尖峰；其积分冻结判断使用加入重力前馈后的最终电流限幅，避免 Z 轴等带前馈机构在饱和时继续积累积分。PID 的 P/I/D、未限幅输出、最终输出和饱和标志会随遥测一起给出。

为接入串口或板间 CAN 调试器，`mechanism.h` 已提供运行时 API：`Mechanism_SetPid`、`Mechanism_SetDerivativeFilter`、`Mechanism_SetCurrentLimit`、`Mechanism_SetFeedforward`、`Mechanism_SetTargetTolerance`、`Mechanism_SetMotionLimits`、`Mechanism_GetMotorTelemetry` 与 `Mechanism_RestoreDefaultTuning`。所有写参数操作只在 `DISARMED` 状态接受；它们是 RAM 临时值，断电或恢复默认后回到 `tuning.c` 的值。USART3 已初始化在 PB10/PB11，但目前不占用它，也没有凭空定义板间 CAN 协议。

将来接底盘板时，通信层只调用 `CommandMailbox_Submit()`，投递 `ARM`、急停、开始取料、吸附确认和流程复位等事件；不允许从 CAN 回调直接调用 `Mechanism_Move...`。这样即使底盘板、气路板或视觉状态帧临时异常，也不会在中断里打断正在执行的机构序列。

## 暂不实现的待办

- **FDCAN2 正式任务调度**：`0x123` 的 A/B/C/D 接收、过滤器、中断和命令锁存已完成；比赛集成版仍固定自动运行 E。后续应把锁存命令映射到可恢复的正式 FSM 状态，而非用阻塞式临时流程。
- **吸附确认联锁**：流程到位已改为“真到位才转场”（`WaitForMechanismTarget()` + 每轴独立的到位资格判定：轨迹完成 + 位置/速度死带持续 120–180 ms，参数在 `tuning.c` 的 `target_settle_time_ms`）。涉及吸盘的步骤还需同时等待“吸盘状态确认”事件；两者任一超时应进入可诊断故障。
- **GM6020 空载偶发抖动**：当前位置环仍偶尔在目标附近抖动。保持当前可用参数；下次单轴台架调试时，记录位置误差、转速和电流，再小步调整翻转轴位置环/速度环与到位死区，不能直接照搬 Z 轴参数。
- **调试脚本替换为正式流程 FSM**：当前 `robot_fsm.c` 的 `switch ('E')` 是一次性自动集成流程；`e_sequence_started` 置位后，遇到故障不会从中断步骤恢复。后续改为显式步骤状态、步骤编号、超时处理和“清故障后从安全步骤重试”的状态机。
- **CAN 命令入口**：正式 `CommandMailbox` 消费逻辑当前仍在 `#if 0` 中。FDCAN2 的最小 A/B/C/D 协议已存在，但当前自动 E 模式未消费它；后续须将其转换为可恢复的命令/状态事件。
- **统一控制时基**：任务周期由 `xTaskGetTickCount()` 调度，控制层另使用同源的 `app_time_ms`。目前二者同由 SysTick 驱动，不会自然漂移；后续应统一为一个时基，降低诊断和维护成本。
- **轨迹完成浮点比较**：`Motor_IsTrajectoryComplete()` 仍依赖 `goal_counts == target_counts`。后续改为容差比较，避免浮点赋值路径变化造成“永不到位”。
- **安全任务优先级复核**：当前 `SafetyTask` 优先级低于 `MotorControlTask`。当前控制周期短，尚非现场阻塞项；比赛版应评估提升到同级或更高优先级。
- **轨迹规划说明/升级**：当前制动采用工程近似而非严格梯形速度规划，1 ms 周期下可用。若后续出现末端冲击、速度不连续或更高精度需求，再替换为严格梯形/S 曲线规划。

上机顺序：确认 ID/方向 → 先用很低限流验证编码器方向 → 调速度环 P/I → 再调位置环 P → 最后测重力前馈、速度/加速度约束和到位容差。默认 `kd=0`；有明确的高频抖动或制动不足证据后，才逐步启用 D 和滤波时间常数。

## 上板前必须确认

1. 四台电机的实际 CAN ID 是否依次为 `0x201` / `0x202` / `0x203` / `0x206`，以及正方向；
2. PA11/PA12 是否确实接到该板的 CAN 收发器；
3. 机构在上电/归零时的机械位置。当前伸缩“收回”为两个相对编码器零点，不能替代限位开关归零；
   当前开发阶段要求每次 `ARM` 前人工将 Z 轴置于最低点，程序将该时刻的相对编码器零点视作底点，并以软件行程保护。它不能发现“实际上未在底点”的错误；接入电气下限位后可再升级为自动回零。
4. Z 轴重力补偿电流、GM6020 是否需要角度相关补偿，以及所有 PID 参数；
5. 与气路/底盘板的命令和状态帧。`RobotFsm` 已保留“吸附确认”事件，但尚未虚构板间协议。

详细 CubeMX 引脚与时钟清单见 [CubeMX配置.md](CubeMX配置.md)。
