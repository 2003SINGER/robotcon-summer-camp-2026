# mechanism_controller

这是夏令营机器人**机构板**的独立工程：一块 `STM32H723ZGT6` 控制 Z 轴 M2006、前部伸缩机构两台 M2006，以及翻转 GM6020。底盘和气路均是其他板，不在本工程直接驱动。

## 运行结构

```text
FDCAN1 ISR（只收反馈快照/总线诊断，不碰机构状态）
        ↓
命令邮箱（未来 CAN2/串口只投递意图，长度 1、最新命令覆盖旧命令）
        ↓
RobotFsmTask，20 ms：唯一的动作状态迁移与机构目标下发
        ↓
ActuatorControlTask，1 ms：四个闭环 → CAN1 电流帧
        ↑
SafetyTask，10 ms：反馈超时、CAN 发送异常、任务栈余量
```

四电机不各自开任务；伸缩双电机、Z 轴和翻转由同一个 1 ms 控制任务计算并在同一周期打包发出，因此不会因任务调度把两段伸缩拆散。FreeRTOS 任务与队列均改为**静态分配**，没有运行期堆分配；栈溢出直接进入机构故障。

硬件与控制参数集中在 `Core/Src/tuning.c`：四台电机的 CAN ID、正反方向、电流上限、双环 PID、重力前馈、到位容差以及最大速度/加速度。比赛动作的伸缩距离、各段 Z 轴高度和翻转角度集中在 `Core/Src/robot_fsm.c` 顶部。`mechanism.c` 只处理机构状态与目标，`can_bus.c` 只处理 CAN 打包/反馈。

上电后处于 `DISARMED`；没有任何自动翻转、抬升或伸缩，也不会主动在 CAN1 发送电流帧。上层通信接入后，必须通过命令邮箱请求 `ARM`，并且四台电机均已收到新反馈，才会进入 `READY`。反馈超过 20 ms 未更新、或 CAN 发送连续异常会切到故障并发送零电流。

## 已确认的映射

| 执行器 | 电机 | CAN 反馈 ID | 说明 |
| --- | --- | --- | --- |
| 前部伸缩 | M2006 A/B | `0x201` / `0x202` | 原工程的伸出目标分别为 38 / 35 圈，保留为独立原始目标。 |
| Z 轴 | M2006 | `0x203` | 单 CAN 地址规划为 C610 ID 3；须先在物理电调上设定。已预留重力前馈。 |
| 翻转 | GM6020 | `0x206` | 发送组 `0x1FF` 的第二槽位。 |

CAN1 已同步写入 `mechanism_controller.ioc`：PA11 (RX) / PA12 (TX)、1 Mbps 时序与关闭自动重传。本工程以 `xuke`、`jiaojingwen` 与 `pengzixi` 的实际源码共同使用的 PA11/PA12 为准；xuke 的 `.ioc` 中 PB8/PB9 是旧配置，不作为依据。

首次去基地上电时，`CAN_DIAGNOSTIC_ACCEPT_ALL_STANDARD_IDS=1`，CAN1 会接收本地电机总线的全部标准帧，但仍保持 `DISARMED`。在 CLion 的 Live Watch 直接看：

- `g_can_bus_diagnostics.rx_frame_count`：是否真的收到总线帧；
- `g_can_bus_diagnostics.last_identifier` / `last_data`：最后一帧 ID 和 8 字节原始反馈；
- `g_can_bus_diagnostics.rx_unknown_id_count`：不在当前 `tuning.c` 表里的 ID 数；
- `g_app_runtime_diagnostics.control_deadline_miss_count` 和三个 `*_stack_min_words`：控制任务是否漏周期、余栈是否够。

确认四个 ID 后将该宏改为 `0` 后重新刷写；此时硬件过滤器只接收已登记的四台电机反馈。

## 构建与烧录

推荐直接用 VS Code 打开本目录，按 `Ctrl+Shift+B` 并选择 **Build STM32 firmware**。它调用 [tools/build.ps1](tools/build.ps1)，直接使用已安装的 STM32CubeCLT GCC，不依赖 CMake 首次配置缓存。

产物位于 `build/`：`mechanism_controller.elf/.hex/.bin`。打开 STM32CubeProgrammer，连接 ST-Link，选择 `mechanism_controller.bin`，下载地址填 `0x08000000` 后烧录。该目录不纳入 Git。

`CMakePresets.json` 仍保留给 CLion，但本机 CubeCLT 的 CMake 首次裸机探测会异常退出；当前上板与日常开发以 VS Code 任务为准。

用于调试时，安装 VS Code 的 **Cortex-Debug** 扩展，连接 ST-Link 的 SWDIO、SWCLK、GND、3.3V 后按 `F5` 选择 **Debug STM32H723 via ST-Link**。不需要 USB：先运行几秒再暂停，在 Watch 中加入 `g_can_bus_diagnostics` 查看原始 CAN 帧和 ID，加入 `g_mechanism_telemetry` 查看四台电机已解码的总编码器、转速、电流、目标与 PID 状态。USB/串口只在后续需要 VOFA 不停机连续画波形时再接。

## 调参

日常默认参数只改 [Core/Src/tuning.c](Core/Src/tuning.c)，随后重新构建、刷写：

- `motor_profiles`：CAN ID、反馈/输出方向、限流、位置/速度 PID、重力前馈和到位容差；
- `robot_fsm.c` 顶部的 `FSM_*` 宏：每个比赛动作的伸缩目标、Z 轴高度、翻转角度；
- `trajectory_max_velocity_counts_s` / `trajectory_max_acceleration_counts_s2`：位置目标的速度/加速度约束。目标不会一步跳到远端，而是以受限轨迹进入位置环。
- CAN ID 和正反向属于启动时总线过滤配置，必须改源码、重新刷写，不能运行时改。

PID 使用“测量值微分 + 一阶低通滤波”，避免设定值突变给 D 项造成电流尖峰；其积分冻结判断使用加入重力前馈后的最终电流限幅，避免 Z 轴等带前馈机构在饱和时继续积累积分。PID 的 P/I/D、未限幅输出、最终输出和饱和标志会随遥测一起给出。

为接入串口或板间 CAN 调试器，`mechanism.h` 已提供运行时 API：`Mechanism_SetPid`、`Mechanism_SetDerivativeFilter`、`Mechanism_SetCurrentLimit`、`Mechanism_SetFeedforward`、`Mechanism_SetTargetTolerance`、`Mechanism_SetMotionLimits`、`Mechanism_GetMotorTelemetry` 与 `Mechanism_RestoreDefaultTuning`。所有写参数操作只在 `DISARMED` 状态接受；它们是 RAM 临时值，断电或恢复默认后回到 `tuning.c` 的值。当前没有占用串口，也没有凭空定义板间 CAN 协议。

将来接底盘板时，通信层只调用 `CommandMailbox_Submit()`，投递 `ARM`、急停、开始取料、吸附确认和流程复位等事件；不允许从 CAN 回调直接调用 `Mechanism_Move...`。这样即使底盘板、气路板或视觉状态帧临时异常，也不会在中断里打断正在执行的机构序列。

上机顺序：确认 ID/方向 → 先用很低限流验证编码器方向 → 调速度环 P/I → 再调位置环 P → 最后测重力前馈、速度/加速度约束和到位容差。默认 `kd=0`；有明确的高频抖动或制动不足证据后，才逐步启用 D 和滤波时间常数。

## 上板前必须确认

1. 四台电机的实际 CAN ID 是否依次为 `0x201` / `0x202` / `0x203` / `0x206`，以及正方向；
2. PA11/PA12 是否确实接到该板的 CAN 收发器；
3. 机构在上电/归零时的机械位置。当前伸缩“收回”为两个相对编码器零点，不能替代限位开关归零；
4. Z 轴重力补偿电流、GM6020 是否需要角度相关补偿，以及所有 PID 参数；
5. 与气路/底盘板的命令和状态帧。`RobotFsm` 已保留“吸附确认”事件，但尚未虚构板间协议。

详细 CubeMX 引脚与时钟清单见 [CubeMX配置.md](CubeMX配置.md)。
