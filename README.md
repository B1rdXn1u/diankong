# STM32F103 + MPU6050 姿态解算与 OLED 显示（FreeRTOS / CMSIS-RTOS V2）

基于 STM32F103C8T6 的多任务姿态测量小系统：MPU6050 六轴数据经**互补滤波**解算出 Roll / Pitch / Yaw，
通过 **FreeRTOS（CMSIS-RTOS V2）** 划分为「IMU 采样解算」「串口收发」「OLED 显示」三个任务并行运行，
姿态角实时显示在 SSD1306 OLED 上，同时以 1 Hz 通过串口上报，串口收到的数据会原样回显。

> 课程作业：2.4 通过 FreeRTOS 完成任务划分与调度。

---

## 目录

1. [项目简介](#1-项目简介)
2. [硬件清单与芯片型号](#2-硬件清单与芯片型号)
3. [引脚接线表](#3-引脚接线表)
4. [CubeMX 配置说明](#4-cubemx-配置说明)
5. [FreeRTOS 任务结构](#5-freertos-任务结构)
6. [代码目录说明](#6-代码目录说明)
7. [串口通信收发处理流程](#7-串口通信收发处理流程)
8. [IMU 型号、接口、校准与姿态解算方法](#8-imu-型号接口校准与姿态解算方法)
9. [OLED 显示说明](#9-oled-显示说明)
10. [编译烧录步骤](#10-编译烧录步骤)
11. [遇到的问题与解决方法](#11-遇到的问题与解决方法)
12. [上电验收现象](#12-上电验收现象)
13. [演示视频与学习笔记](#13-演示视频与学习笔记)

---

## 1. 项目简介

| 项目 | 内容 |
|---|---|
| 功能 | 六轴姿态测量：100 Hz 采样 MPU6050，互补滤波解算 Roll/Pitch/Yaw；10 Hz 刷 OLED；串口 1 Hz 上报 + 数据回显 |
| 架构 | FreeRTOS（CMSIS-RTOS V2），三个任务 + 信号量 / 事件标志 / 互斥锁 |
| 芯片 | STM32F103C8T6（Cortex-M3，72 MHz，64KB Flash / 20KB RAM） |
| 开发环境 | STM32CubeMX 6.1.2 + Keil MDK-ARM 5（ARM Compiler 5） |
| 资源占用 | Flash ≈ 24 KB / 64 KB，RAM ≈ 12.6 KB / 20 KB（其中 8 KB 为 FreeRTOS 堆） |

数据流：

```
                 ┌──────────────────────┐
   MPU6050 ──I2C1─►  IMU_Task (100Hz)   │  采样 + 互补滤波解算
                 └──────────┬───────────┘
                            │ 最新姿态（att_mutex 保护）
              ┌─────────────┴─────────────┐
              ▼                            ▼
      ┌───────────────┐            ┌───────────────┐
      │ UART_Task     │            │ OLED_Task     │
      │ 回显 +1Hz上报 │            │ 10Hz 刷屏     │
      └───────┬───────┘            └───────┬───────┘
              │ USART1(DMA)                │ I2C1
              ▼                            ▼
         上位机串口助手              SSD1306 128x64
```

## 2. 硬件清单与芯片型号

| 名称 | 型号 / 规格 | 说明 |
|---|---|---|
| 主控 MCU | **STM32F103C8T6**（LQFP48，Cortex-M3，72MHz，64KB Flash，20KB RAM） | 最小系统板（Blue Pill 类） |
| 姿态传感器 | **MPU6050**（GY-521 模块，6 轴：3 轴陀螺仪 + 3 轴加速度计） | 硬件 I2C1 接口，地址 0x68 |
| 显示屏 | **SSD1306** OLED，128×64 单色 | 硬件 I2C1 接口，地址 0x3C |
| 下载调试器 | ST-Link V2 | SWD 两线调试 + 下载 |
| 串口工具 | USB-TTL（CH340/CP2102 等） | 115200 8N1，接 PA9/PA10 |
| 晶振 | 8 MHz 无源晶振 | HSE，PLL ×9 → 72 MHz |
| 电源 | 3.3 V | MCU / MPU6050 / OLED 共地 |

## 3. 引脚接线表

| 信号 | STM32 引脚 | GPIO 模式 | 接到哪里 |
|---|---|---|---|
| USART1_TX | **PA9** | 复用推挽输出 | USB-TTL 的 **RX** |
| USART1_RX | **PA10** | 浮空输入 | USB-TTL 的 **TX** |
| I2C1_SCL | **PB6** | 复用开漏（模块自带 4.7k 上拉） | MPU6050 SCL、OLED SCL |
| I2C1_SDA | **PB7** | 复用开漏（模块自带 4.7k 上拉） | MPU6050 SDA、OLED SDA |
| SWDIO | **PA13** | Serial Wire | ST-Link SWDIO |
| SWCLK | **PA14** | Serial Wire | ST-Link SWCLK |
| OSC_IN / OSC_OUT | **PD0 / PD1** | 外部晶振 | 8MHz 晶振 + 22pF×2 |
| LED（预留） | **PC13** | 推挽输出 | 板载 LED，当前未使用 |
| 按键（预留） | **PA1** | 上拉输入 | 当前未使用 |
| 电源 | 3V3 / GND | — | 三块板子 **必须共地** |

> 注意：MPU6050 与 OLED **挂在同一条 I2C1 总线**上，二者从机地址不同（0x68 / 0x3C）不冲突，
> 但**总线同一时刻只能有一个主设备发起传输**——这正是代码里需要 `i2c_mutex` 的原因（见第 5 节）。

## 4. CubeMX 配置说明

配置文件：`001Test.ioc`（用 STM32CubeMX 6.1.2 打开）

### 4.1 时钟树

| 项目 | 值 |
|---|---|
| HSE | 8 MHz 外部晶振 |
| PLL | HSE × 9 = **72 MHz**（SYSCLK） |
| AHB / APB2 | 72 MHz / 72 MHz |
| APB1 | 36 MHz（HCLK/2，注意定时器时钟 ×2 = 72 MHz） |
| Flash Latency | 2 WS |

### 4.2 外设配置

| 外设 | 配置 |
|---|---|
| **SYS** | Debug = **Serial Wire**；Timebase Source = **TIM4**（见下方说明） |
| **RCC** | HSE = Crystal/Ceramic Resonator，PLL ×9 |
| **USART1** | Asynchronous，**115200 / 8 / N / 1**，无硬件流控；DMA：RX = **DMA1_Channel5**、TX = **DMA1_Channel4**（Normal 模式，字节对齐，内存自增） |
| **I2C1** | I2C **Fast Mode 400 kHz**，7 位地址，PB6/PB7 |
| **NVIC** | USART1、DMA1_CH4、DMA1_CH5 抢占优先级 **5**；TIM4 = **0** |
| **FREERTOS** | Interface = **CMSIS_V2**（未创建默认任务，三个任务在 `app_rtos.c` 里创建） |
| **Project Manager** | Toolchain = **MDK-ARM V5**，勾选 *Generate peripheral initialization as a pair of .c/.h files* |

### 4.3 两个关键点（配置错了会直接跑不起来）

1. **HAL 时基必须改成 TIM4，不能是 SysTick。**
   SysTick 被 FreeRTOS 内核独占用于任务节拍；HAL 库又需要 1 ms 时基来支撑
   `HAL_GetTick() / HAL_Delay()` 和 I2C 超时。两者抢同一个 SysTick 会导致
   HAL 时基错乱或链接冲突。所以 CubeMX 里把 SYS → Timebase Source 设为 **TIM4**。

2. **中断优先级不能高于 `configMAX_SYSCALL_INTERRUPT_PRIORITY`。**
   `FreeRTOSConfig.h` 里该值为 5，所以 USART1 / DMA 的中断优先级设成 **5**（数值 5 ≥ 5 才允许），
   TIM4 设成 0（它只在中断里做 `HAL_IncTick()`，不调用任何 RTOS API，可以更高）。

### 4.4 `FreeRTOSConfig.h` 中手工修改的三处

| 宏 | 值 | 原因 |
|---|---|---|
| `configTOTAL_HEAP_SIZE` | **8192**（CubeMX 默认 3072） | 三个任务栈 768+1024+640 + 空闲任务 + 定时器任务 + TCB/信号量/互斥锁 ≈ 4.5KB，默认堆不够，`osThreadNew()` 会返回 NULL |
| `INCLUDE_xTaskGetCurrentTaskHandle` | **1** | CMSIS-RTOS V2 的 `osThreadGetId()` 需要，否则 `cmsis_os2.c` 直接 `#error` |
| `CMSIS_device_header` | **`"stm32f1xx.h"`** | 本工程 RTE 组件未生成该宏，`freertos_os2.h` 里 `#include CMSIS_device_header` 会报错 |

另外 `configCHECK_FOR_STACK_OVERFLOW` 设为 **2**，栈溢出时调用 `vApplicationStackOverflowHook()` 停住，便于定位。

## 5. FreeRTOS 任务结构

### 5.1 任务划分

| 任务 | 入口 | 优先级 | 周期 / 触发方式 | 栈 | 职责 |
|---|---|---|---|---|---|
| **IMU_Task** | `IMU_Task()` | `osPriorityRealtime`（**48**，最高） | 100 Hz（`osDelayUntil` 10 ms） | 768 B | MPU6050 初始化与校准、100 Hz 采样、互补滤波解算、发布最新姿态 |
| **UART_Task** | `UART_Task()` | `osPriorityHigh`（**40**） | 事件驱动（等信号量，最长 100 ms 超时） | 1024 B | 串口数据回显；1 Hz 主动上报姿态角 |
| **OLED_Task** | `OLED_Task()` | `osPriorityAboveNormal`（**32**，最低） | 10 Hz（`osDelayUntil` 100 ms） | 640 B | 只刷新变化的角度数字（I2C 慢，不能更快） |

**优先级依据**：采样周期决定角度解算的时间精度（`dt` 直接进积分），必须最及时 → 最高；
串口是事件型、延迟几十毫秒无感 → 次高；OLED 刷屏是三者中最慢且最不实时的工作 → 最低，
这样它只会在采样任务空闲时占用 CPU。

### 5.2 任务间通信

| 对象 | 类型 | 生产者 → 消费者 | 作用 |
|---|---|---|---|
| `uart_rx_sem` | 二值信号量 | USART1 空闲中断 → `UART_Task` | "收到一帧串口数据了" |
| `init_evt` | 事件标志 | `IMU_Task` → `UART_Task` / `OLED_Task` | "校准完成，可以开始工作" |
| `att_mutex` | 互斥锁 | 保护共享的"最新一帧姿态" | 多读者共享最新状态 |
| `i2c_mutex` | 互斥锁（`osMutexPrioInherit`） | 保护 I2C1 总线 | MPU6050 与 OLED 共用同一条物理总线 |

**为什么"初始化完成"用事件标志而不是信号量**：二值信号量计数只有 1，而
`UART_Task` 和 `OLED_Task` **都要**等这件事，用信号量只有一个能拿到、另一个永远等下去；
事件标志一次 `osEventFlagsSet()` 会唤醒所有等待该位的任务（用 `osFlagsNoClear` 让标志常驻）。

**为什么"最新姿态"用互斥锁而不是消息队列**：`osMessageQueueGet()` 是消费型的，取走即删除，
两个读者去 Get 同一个队列会互抢消息；"最新姿态"本质是共享状态而非一次性事件，
所以用互斥锁保护一份数据，两个读者各自取快照。

**为什么 I2C 需要互斥锁**：HAL 的 `HAL_I2C_Mem_Read/Write` 是阻塞且**不可重入**的，
OLED 刷屏与 MPU6050 读数若同时发起传输，会返回 `HAL_BUSY` 甚至把总线状态机搞乱。
`osMutexPrioInherit` 打开优先级继承，避免"低优先级的 OLED 持锁、高优先级的采样任务无限等"的优先级反转。

### 5.3 启动顺序

```
main()
 ├ HAL_Init()                       // HAL 时基 = TIM4
 ├ SystemClock_Config()             // 72MHz
 ├ MX_GPIO_Init / MX_DMA_Init / MX_USART1_UART_Init / MX_I2C1_Init
 ├ OLED_Init()                      // 调度器启动前先点亮屏幕
 ├ osKernelInitialize()
 ├ MX_FREERTOS_Init()               // 内部调用 AppRTOS_Init() 创建信号量/事件标志/互斥锁/三个任务
 └ osKernelStart()                  // 调度器启动，不再返回
        │
        ├ IMU_Task   : 初始化 → 校准 → 置位 init_evt → 100Hz 循环
        ├ UART_Task  : 等 init_evt → 启动 DMA 接收 → 事件循环
        └ OLED_Task  : 等 init_evt → 画界面框架 → 10Hz 刷数字
```

## 6. 代码目录说明

```
001Test/
├── 001Test.ioc                     # ★ STM32CubeMX 工程配置（引脚/时钟/外设/FreeRTOS）
├── .mxproject                      #   CubeMX 自动生成的工程索引
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   ├── stm32f1xx_hal_conf.h    #   HAL 模块开关
│   │   ├── FreeRTOSConfig.h        # ★ FreeRTOS 配置（堆大小/优先级/中断优先级/调试检查）
│   │   └── gpio.h / dma.h / i2c.h / usart.h / tim.h / stm32f1xx_it.h
│   └── Src/
│       ├── main.c                  # ★ 初始化 + 启动调度器；TIM4 时基回调
│       ├── freertos.c              # ★ MX_FREERTOS_Init()，调用 AppRTOS_Init()
│       ├── gpio.c / dma.c / i2c.c / usart.c
│       ├── stm32f1xx_hal_timebase_tim.c   # 用 TIM4 做 HAL 1ms 时基
│       ├── stm32f1xx_it.c          # 中断服务函数（USART1/DMA/TIM4）
│       ├── stm32f1xx_hal_msp.c     # 外设底层 GPIONVIC 配置
│       └── system_stm32f1xx.c
├── Drivers/
│   ├── STM32F1xx_HAL_Driver/       # ST HAL 库
│   └── CMSIS/                      # Cortex-M3 内核与器件头文件
├── Middlewares/
│   └── Third_Party/FreeRTOS/
│       ├── Source/                 # FreeRTOS 内核（tasks/queue/list/timers/...）
│       │   ├── CMSIS_RTOS_V2/      # ★ CMSIS-RTOS V2 包装层（cmsis_os2.c）
│       │   └── portable/RVDS/ARM_CM3/  # Cortex-M3 移植层 port.c
│       └── ...
└── MDK-ARM/
    ├── 001Test.uvprojx             # ★ Keil5 工程文件
    ├── startup_stm32f103xb.s       # 启动文件（向量表 / Reset_Handler）
    ├── app_rtos.c / app_rtos.h     # ★★ 三个应用任务 + 信号量/事件标志/互斥锁（本工程核心）
    ├── attitude.c / attitude.h     # ★★ 姿态解算（互补滤波 + 零偏校准 + 静止检测）
    ├── MPU6050.c / MPU6050.h / MPU6050_Reg.h   # MPU6050 寄存器级驱动（I2C）
    ├── oled.c / oled.h / oled_font.h           # SSD1306 底层驱动（写命令/写数据/字符）
    └── oled_app.c / oled_app.h     # 显示排版层（角度格式化、字符级差异刷新）
```

> 说明：用户自己写的应用代码放在 `MDK-ARM/` 下（与 CubeMX 生成代码分开），
> CubeMX 重新生成代码时不会覆盖它们。

## 7. 串口通信收发处理流程

参数：**USART1，115200 8N1，无流控**（PA9 = TX，PA10 = RX）。

### 7.1 接收（DMA + 空闲中断 + 信号量）

```
USART1 RX 引脚
   │  硬件逐字节搬运，不占 CPU
   ▼
s_uartDmaBuf[64]  ←── DMA1_Channel5
   │
   │ 一帧结束（总线空闲 IDLE 中断）
   ▼
HAL_UARTEx_RxEventCallback()          ← 在 USART1 中断里（优先级 5）
   ├ 1. memcpy 到 s_uartReadyBuf（先把数据搬走，防止被下一帧覆盖）
   ├ 2. HAL_UARTEx_ReceiveToIdle_DMA() 重新挂 DMA（实现不定长连续接收）
   ├ 3. 关掉 DMA 半传输中断（只要"收完/空闲"这一次）
   └ 4. osSemaphoreRelease(uart_rx_sem)     ← 中断里只发通知，不做耗时操作

UART_Task（阻塞在 osSemaphoreAcquire(uart_rx_sem, 100ms)）
   ├ 醒来 → 把数据拷到本任务栈上的 rxCopy
   └ HAL_UART_Transmit() 原样回显
```

中断里只做「搬数据 + 挂 DMA + 发信号量」三件微秒级的事，绝不 `printf`、绝不刷屏；
等待期间任务处于阻塞态，CPU 可以跑别的任务或空闲任务。

### 7.2 发送（1 Hz 主动上报）

`UART_Task` 每次被信号量唤醒或 100 ms 超时后，检查"距上次上报是否满 1 秒"，
满了就取一份姿态快照（锁内只拷贝，出锁再格式化）并发送一行：

```
Roll=12.34 Pitch=-5.67 Yaw=179.99 OK\r\n          （正常）
Roll=12.34 Pitch=-5.67 Yaw=179.99 [I2C ERR]\r\n   （本次读取传感器失败）
```

角度先换算成「0.01° 的整数」再手工拼字符串（`AppRTOS_AngleToStr`），
**不使用 `%f`**：Keil 默认 C 库不带浮点打印支持，用 `%f` 会输出乱码或直接崩。
这样也省掉了整个 printf 库的 Flash 开销。

## 8. IMU 型号、接口、校准与姿态解算方法

### 8.1 型号与接口

| 项目 | 内容 |
|---|---|
| 型号 | **MPU6050**（3 轴陀螺仪 + 3 轴加速度计，内置 DMP） |
| 接口 | **硬件 I2C1**，Fast Mode **400 kHz**（PB6/PB7），与 OLED 总线共用 |
| 从机地址 | **0x68**（HAL 用的 8 位形式为 **0xD0**） |
| 数据读取 | 从 `ACCEL_XOUT_H(0x3B)` 起**一次连读 14 字节**：AX AY AZ（6）+ 温度（2）+ GX GY GZ（6） |
| 采样率 | `SMPLRT_DIV = 0x09` → 1 kHz / (1+9) = **100 Hz**；任务按 100 Hz 读取 |
| 量程 | 陀螺仪 ±2000 °/s（`16.4 LSB/(°/s)`）、加速度计 ±16 g（`2048 LSB/g`） |

初始化寄存器序列（`MPU6050_Init()`）：

| 寄存器 | 值 | 含义 |
|---|---|---|
| `PWR_MGMT_1` (0x6B) | 0x01 | 退出睡眠，时钟源 = X 轴陀螺仪 PLL |
| `PWR_MGMT_2` (0x6C) | 0x00 | 所有轴均不待机 |
| `SMPLRT_DIV` (0x19) | 0x09 | 采样率 100 Hz |
| `CONFIG` (0x1A) | 0x06 | DLPF 约 5 Hz，抑制高频振动噪声 |
| `GYRO_CONFIG` (0x1B) | 0x18 | 陀螺仪 ±2000 °/s |
| `ACCEL_CONFIG` (0x1C) | 0x18 | 加速度计 ±16 g |

### 8.2 校准（开机一次性，`IMU_Task` 里完成）

1. **上电等待 500 ms**：MPU6050 从睡眠唤醒后内部 PLL / DLPF 需要时间稳定，
   刚上电立刻采样容易采到错误零偏。
2. **陀螺仪零偏校准** `Attitude_CalibrateGyro(1000)`：静止采样 1000 次取平均，
   得到三轴零偏 `s_GyroOffset[]`（单位 °/s），之后每帧从角速度里减掉。
3. **有效性检查 + 自动重试（最多 3 次）**：
   - 器件不应答（I2C 无 ACK）→ 重试
   - 三轴零偏**同时精确为 0** → 说明一个样本都没采到（假成功）→ 重试
   - 任一轴 |零偏| > **5 °/s** → 说明校准时板子在转，平均转速被当成了零偏 → 重试
4. **水平零点** `Attitude_LevelZero(500)`：用 500 次加速度计平均得到
   `s_AccRefX / s_AccRefY`（加速度计零偏 + 安装倾斜参考），
   并把"当前姿态"定义为 **0°** 基准。

> **校准期间必须保持静止**，这是精度的前提条件。如果校准时板子在缓慢转动，
> 平均转速会被当成零偏扣掉，之后角度就会**朝反方向匀速漂**（本项目曾出现
> Roll 漂 1 °/s、Yaw 漂 10 °/s，正是校准时被运动污染所致）。
> 所以开机画面会先提示 `Keep still!`，OLED 上还会显示三轴零偏供自查。

### 8.3 互补滤波

对 Roll / Pitch：用加速度计的重力分量修正陀螺仪积分，长期不漂；
Yaw 没有磁力计参考，只能靠陀螺仪积分（必然存在缓慢漂移，属正常物理限制）。

```
角速度积分 + 加速度计修正：

  dRoll/dt  = gx' + gate · K · errRoll
  dPitch/dt = gy' + gate · K · errPitch
  dYaw/dt   = gz'
```

其中：

| 符号 | 含义 |
|---|---|
| `gx' / gy' / gz'` | 陀螺仪角速度（已扣零偏，|ω| < 0.3 °/s 时置 0 的死区，抑制静置抖动） |
| `K = 2.0` | 加速度计修正增益（1/s），时间常数 τ ≈ 1/K ≈ 0.5 s |
| `errRoll  = -ayf / |a| / cos(Roll)` | 加速度计指示的横滚误差（小角度近似，单位 °） |
| `errPitch = -axf / |a| · cos(Roll) / cos(Pitch)` | 加速度计指示的俯仰误差 |
| `|a|` | 重力模长归一化，抑制晃动时的线性加速度 |
| `±2°` 死区 | 误差小于 2° 不修正，抵消加速度计的直流偏差/安装倾斜 |
| `gate` | 静止锁：三轴角速度持续 400 ms 都很小 → 冻结加速度计修正，只靠陀螺仪保持 |
| `dt` | **实测节拍差**（`osKernelGetTickCount()` 差分），限幅在 2~50 ms |

输出归一化：Roll / Yaw 归一化到 ±180°，Pitch 限幅 ±90°。
`cos()` 用小角度泰勒展开近似（只用乘除法），避免每帧调用 `sinf/cosf/atan2f` 的软件浮点开销。

## 9. OLED 显示说明

| 项目 | 内容 |
|---|---|
| 驱动芯片 | **SSD1306**，128×64 单色 |
| 接口 / 地址 | I2C1（与 MPU6050 共用总线），7 位地址 **0x3C**（HAL 8 位形式 0x78） |
| 字库 | 8×16 ASCII（`oled_font.h`），**4 行 × 16 字符** |
| 刷新率 | 10 Hz（`OLED_Task`，100 ms） |

### 9.1 开机显示顺序

```
① 开机提示（约 1.5 s）            ② 校准结果（约 1.7 s）          ③ 主界面（常驻，10Hz 刷新）
┌────────────────┐              ┌────────────────┐             ┌────────────────┐
│IMU Attitude    │              │IMU Attitude    │             │IMU Attitude    │
│MPU6050:OK      │  ─────────►  │Bias X   00.42o │  ─────────► │Roll    12.34o  │
│Calibrating...  │              │Bias Y  -00.18o │             │Pitch  -05.67o  │
│Keep still!     │              │Bias Z   09.87o │             │Yaw     45.67o  │
└────────────────┘              └────────────────┘             └────────────────┘
```

- `MPU6050:OK / ERR`：上电时 `HAL_I2C_IsDeviceReady()` 的应答检测结果
- `Bias X/Y/Z`：本次校准测出的三轴陀螺零偏（0.01° 单位），**用来一眼判断这次校准可不可信**；
  数值大但每次复位都稳定 = 芯片零偏偏大但已被扣干净；**每次都不一样 = 校准时板子在动**
- 校准 3 次都不可信时，第 1 行显示 `CALIB UNSURE`
- 主界面第 1 行第 14~16 列是 I2C 错误指示：读传感器失败时显示 `ERR`，恢复后自动清除

### 9.2 两个实现细节

1. **字符级差异刷新**：`oled.c` 是"1 个字节 1 次 I2C 传输"的写法（1 个字符 = 16 次传输），
   刷满一行 16 字符要 256 次传输、几十毫秒。`oled_app.c` 因此维护了一份
   "屏上现在是什么"的影子缓存，**内容没变的字符不重发 I2C**，稳定时一帧通常只写几个字符。
   注意：整屏清过（`OLED_AppClear()`）后必须让缓存失效，否则会漏写。
2. **没有 `°` 字形**：字库只有 ASCII 0x20~0x7F，直接传 `'°'(0xB0)` 会越界读数组
   （乱码甚至 HardFault），所以用字母 **`o`** 代替度符号。

## 10. 编译烧录步骤

### 10.1 环境准备

1. 安装 **Keil MDK-ARM ≥ 5.27**（本工程用 ARM Compiler 5 构建）
2. 安装器件包 **Keil.STM32F1xx_DFP**（本工程使用 2.4.1）
3. 安装 **ST-Link 驱动**

### 10.2 编译

1. 用 Keil 打开 `MDK-ARM/001Test.uvprojx`
2. `Project → Build Target`（快捷键 **F7**）
3. 编译选项（工程已配好）：ARM Compiler 5、`-O3`、
   *One ELF Section per Function*（未引用的 HAL 函数会被裁掉，Flash 从 63KB 降到 24KB）、
   **Use MicroLIB**
4. 正常结果：`0 Error(s), 0 Warning(s)`，约 **Flash 24 KB / RAM 12.6 KB**

### 10.3 烧录与运行

1. ST-Link 接 PA13/PA14/GND/3V3
2. Keil 中 `Flash → Download`（**F8**）→ 复位运行
3. 串口工具打开对应 COM 口，**115200 8N1**，即可看到 1 Hz 的姿态上报并回显输入

## 11. 遇到的问题与解决方法

| # | 现象 | 原因 | 解决方法 |
|---|---|---|---|
| 1 | `cmsis_os2.c` 编译报大量未定义 | 工程自己的 `task.c / task.h` 与 FreeRTOS 内核头文件 `task.h` **重名**，而 `..\MDK-ARM` 在包含路径最前面，内核的 `#include "task.h"` 拿到了用户的头文件 | 用户文件改名为 **`app_rtos.c / app_rtos.h`** |
| 2 | 三个任务一个都不运行 | `main.c` 仍是裸机大循环，且从未调用 `AppRTOS_Init()` | `main()` 只保留硬件初始化+启动调度器，任务在 `MX_FREERTOS_Init()` 里创建 |
| 3 | `osThreadNew()` 全部返回 NULL | `configTOTAL_HEAP_SIZE` 只有 3072，装不下三个任务栈+TCB | 改为 **8192** |
| 4 | `cmsis_os2.c` 报 `#error: Definition INCLUDE_xTaskGetCurrentTaskHandle must equal 1` | CubeMX 生成的配置缺该宏 | 在 `FreeRTOSConfig.h` 补 `#define INCLUDE_xTaskGetCurrentTaskHandle 1` |
| 5 | 链接报 `L6200E: Symbol SysTick_Handler multiply defined` | V1 时代的 `#define xPortSysTickHandler SysTick_Handler` 留在了 V2 工程里，与 `cmsis_os2.c` 自带实现冲突 | 按 CubeMX 的 V2 模板把该宏注释掉（HAL 时基已改为 TIM4） |
| 6 | `freertos_os2.h` 报 `#error: expected a file name` | 工程 RTE 组件未定义 `CMSIS_device_header` | 在 `FreeRTOSConfig.h` 的 USER CODE 段 `#define CMSIS_device_header "stm32f1xx.h"` |
| 7 | OLED 任务永远卡在等初始化 | 二值信号量计数只有 1，而 `UART_Task`、`OLED_Task` **两个**任务都在等它 | 改用 **事件标志** `init_evt` + `osFlagsNoClear`，一次置位全部唤醒 |
| 8 | 刷屏时角度数据偶发错乱 / I2C 返回 `HAL_BUSY` | OLED 与 MPU6050 共用 I2C1，HAL 阻塞传输不可重入 | 加 **`i2c_mutex`**（`osMutexPrioInherit`）串行化总线访问，并把"锁内只拷贝、出锁再刷屏/发送"作为规则 |
| 9 | 复位后角度匀速漂：Roll ≈ 1 °/s、Yaw ≈ 10 °/s；再复位又正常 | 零偏校准是**开机一次性**的，校准时板子被手扶/在转动，平均转速被当成零偏扣掉 | ①上电先等 500 ms 再校准 ②采样 1000 次 ③**有效性检查+最多 3 次重试** ④OLED 显示三轴零偏、串口开机打印 `Bias X/Y/Z`，便于自查 |
| 10 | 刷屏时数字闪烁、I2C 占用高 | 每字符 16 次 I2C 传输，整行重写几十毫秒 | 显示层加字符级缓存，只重写变化的字符 |
| 11 | 用 `printf("%f")` 输出角度乱码/崩溃 | Keil 默认 C 库不带浮点 printf | 角度先换算成 0.01° 整数，手工拼字符串上报 |

### 调试经验小结

- **调度相关**：遇到"任务不跑"先查三步 —— ①`osKernelInitialize()` 之后才创建对象吗 ②堆够不够 ③`AppRTOS_Init()` 到底调没调用。
- **I2C 相关**：`HAL_BUSY` / 数据错乱 90% 是"多任务共用一条总线却没加锁"。
- **角度漂移**：先分清"匀速漂"（陀螺零偏没扣干净）和"往一个方向加速/指数趋近"（加速度计零偏或符号错），
  两者排查方向完全不同。
- **中断里只做最少的事**：搬数据 + 发通知，其余交给任务。

## 12. 上电验收现象

1. OLED 显示 `IMU Attitude / MPU6050:OK / Calibrating... / Keep still!`
2. 约 1.5 s 后显示三轴零偏 `Bias X/Y/Z`（此时可判断校准是否可信）
3. 约 1.2 s 后进入主界面，显示 `Roll / Pitch / Yaw`
4. 串口（115200 8N1）收到开机一行 `Bias X=.. Y=.. Z=..`，之后每秒一行
   `Roll=.. Pitch=.. Yaw=.. OK`
5. 向串口发送任意字符，会被原样回显
6. 静止放置：Roll / Pitch 基本不动；Yaw 因无磁力计参考会极缓慢漂移（正常现象）
7. 倾斜模块：Roll / Pitch 实时跟随，响应约 0.5 s 时间常数

## 13. 演示视频与学习笔记

| 内容 | 文件 | 说明 |
|---|---|---|
| 姿态解算演示 | [`docs/media/attitude-demo.mp4`](docs/media/attitude-demo.mp4) | 上电 → `Keep still!` 校准 → 三轴零偏 `Bias X/Y/Z` → 主界面；倾斜/旋转模块观察 Roll / Pitch / Yaw 跟随 |
| 串口收发演示 | [`docs/media/uart-dma-demo.mp4`](docs/media/uart-dma-demo.mp4) | DMA + 空闲中断接收、原样回显、1 Hz 姿态上报 |
| 学习笔记 | [`docs/notes/`](docs/notes/) | 开发过程中的原理整理与调试记录（图片） |

> 在 GitHub 网页上直接点开 `.mp4` 会在浏览器里在线播放，无需下载。
> 仓库整体约 60 MB，其中 `docs/` 占 40 MB；若只需要代码，可浅克隆：
> `git clone --depth 1 <仓库地址>`。

---

## 附：仓库内容自检

- [x] 完整源码（`Core/`、`MDK-ARM/` 下的应用代码、FreeRTOS 与 HAL）
- [x] Keil5 工程文件（`MDK-ARM/001Test.uvprojx`）
- [x] STM32CubeMX 配置（`001Test.ioc`）
- [x] `README.md`（本文件）
- [x] `.gitignore`（已排除编译产物、个人设置、与工程无关的 CMSIS 大体积库）
- [x] `docs/media/`、`docs/notes/`（演示视频与学习笔记）
