# 机构控制器

这是夏令营机器人**机构板**的独立工程：一块 `STM32H723ZGT6` 控制 Z 轴 M2006、前部伸缩机构两台 M2006，以及翻转 GM6020。底盘和气路均是其他板，不在本工程直接驱动。

## 运行结构

```text
FDCAN1 ISR（只更新反馈快照）
        ↓
MotorControlTask，1 ms：四个闭环 → CAN 电流帧
MechanismTask，10 ms：反馈超时与故障处理
RobotFsmTask，20 ms：非阻塞动作序列
```

参数不再散落在控制逻辑中：四台电机的 CAN ID、正反方向、电流上限、双环 PID、重力前馈、到位容差和伸缩行程都集中在 `Core/Src/tuning.c`。`mechanism.c` 只处理机构状态与目标，`can_bus.c` 只处理 CAN 打包/反馈，后续改一套机构参数不会波及流程和总线代码。

上电后处于 `DISARMED`；没有任何自动翻转、抬升或伸缩。上层通信接入后，必须在四台电机均已收到新反馈的条件下调用 `Mechanism_Arm()`，之后才可发机构命令。反馈超过 20 ms 未更新会切到故障并持续发送零电流。

## 已确认的映射

| 执行器 | 电机 | CAN 反馈 ID | 说明 |
| --- | --- | --- | --- |
| 前部伸缩 | M2006 A/B | `0x201` / `0x202` | 原工程的伸出目标分别为 38 / 35 圈，保留为独立原始目标。 |
| Z 轴 | M2006 | `0x203` 暂定 | 必须上电监听后确认。已预留重力前馈。 |
| 翻转 | GM6020 | `0x206` | 发送组 `0x1FF` 的第二槽位。 |

CAN1 统一为 PA11 (RX) / PA12 (TX)、经典 CAN 1 Mbps。Xuke 的 `.ioc` 曾写 PB8/PB9，但其实际 `fdcan.c` 也是 PA11/PA12；本工程以实际代码与 GM 工程一致的 PA11/PA12 为准。

## 构建

在 CLion 打开本目录，选择 `CMakePresets.json` 中的 `stm32-clt`；或者运行：

```powershell
cmake --preset stm32-clt
cmake --build --preset stm32-clt
```

产物位于 `build/`：`mechanism_controller.elf/.hex/.bin`。该目录不纳入 Git。

## 调参

日常默认参数只改 [Core/Src/tuning.c](Core/Src/tuning.c)，随后重新构建、刷写：

- `motor_profiles`：CAN ID、反馈/输出方向、限流、位置/速度 PID、重力前馈和到位容差；
- `motion_profile`：两台伸缩电机各自的伸出目标、翻转的编码器换算；
- CAN ID 和正反向属于启动时总线过滤配置，必须改源码、重新刷写，不能运行时改。

为接入串口或板间 CAN 调试器，`mechanism.h` 已提供运行时 API：`Mechanism_SetPid`、`Mechanism_SetCurrentLimit`、`Mechanism_SetFeedforward`、`Mechanism_SetTargetTolerance`、`Mechanism_GetMotorTelemetry` 与 `Mechanism_RestoreDefaultTuning`。所有写参数操作只在 `DISARMED` 状态接受；它们是 RAM 临时值，断电或恢复默认后回到 `tuning.c` 的值。当前没有占用串口，也没有凭空定义板间 CAN 协议。

## 上板前必须确认

1. Z 轴 M2006 的实际 CAN ID，以及四台电机正方向；
2. PA11/PA12 是否确实接到该板的 CAN 收发器；
3. 机构在上电/归零时的机械位置。当前伸缩“收回”为两个相对编码器零点，不能替代限位开关归零；
4. Z 轴重力补偿电流、GM6020 是否需要角度相关补偿，以及所有 PID 参数；
5. 与气路/底盘板的命令和状态帧。`RobotFsm` 已保留“吸附确认”事件，但尚未虚构板间协议。

详细 CubeMX 引脚与时钟清单见 [CubeMX配置.md](CubeMX配置.md)。
