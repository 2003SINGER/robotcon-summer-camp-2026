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

## 上板前必须确认

1. Z 轴 M2006 的实际 CAN ID，以及四台电机正方向；
2. PA11/PA12 是否确实接到该板的 CAN 收发器；
3. 机构在上电/归零时的机械位置。当前伸缩“收回”为两个相对编码器零点，不能替代限位开关归零；
4. Z 轴重力补偿电流、GM6020 是否需要角度相关补偿，以及所有 PID 参数；
5. 与气路/底盘板的命令和状态帧。`RobotFsm` 已保留“吸附确认”事件，但尚未虚构板间协议。

详细 CubeMX 引脚与时钟清单见 [CubeMX配置.md](CubeMX配置.md)。
