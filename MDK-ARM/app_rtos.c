#include "app_rtos.h"
#include "main.h"
#include "usart.h"
#include "oled.h"
#include "oled_app.h"
#include "MPU6050.h"
#include "attitude.h"
#include "FreeRTOS.h"
#include "task.h"			/* 这里拿到的是 FreeRTOS 内核的 task.h（本工程自己的任务头文件已改名 app_rtos.h） */
#include <string.h>		/* memcpy / memset / strlen / strncat */
#include <math.h>		/* fabsf：判断零偏是否离谱 */

/* ======================================================================
 * FreeRTOS 应用层：任务划分、优先级、周期、任务间通信
 *
 *   任务              周期        优先级                 干什么
 *   IMU_Task          10ms        osPriorityRealtime     MPU6050 采样 + 互补滤波解算
 *   UART_Task         事件驱动    osPriorityHigh         回显收到的串口数据 + 1Hz 上报角度
 *   OLED_Task         100ms       osPriorityAboveNormal  刷新屏幕上的三个姿态角
 *
 *   通信：uart_rx_sem(信号量) / init_evt(事件标志) / att_mutex(互斥锁) / i2c_mutex(互斥锁)
 *   详见 app_rtos.h 顶部注释。
 * ==================================================================== */

/* ---------------------------- 任务参数 ---------------------------- */
#define IMU_PERIOD_MS        10U     /* 100Hz 采样：决定角度解算的时间精度 */
#define OLED_PERIOD_MS       100U    /* 10Hz 刷屏：I2C 很慢，别更快 */
#define UART_REPORT_MS       1000U   /* 1Hz 主动上报 */
#define UART_WAIT_MS         100U    /* 等串口数据的最长时间，超时就顺便检查上报 */
#define UART_RX_BUF_SIZE     64U     /* DMA 接收缓冲区大小 */

#define ATT_SNAPSHOT_WAIT    50U     /* 取姿态快照最多等多久（ms） */
#define I2C_WAIT_MS          100U    /* 申请 I2C 总线最多等多久（ms） */

/* dt 保护：被刷屏拖慢时不能拿过大的 dt 去积分，否则角度会跳 */
#define DT_MAX_SEC           0.05f
#define DT_MIN_SEC           0.002f

/* ---------------------- 开机校准参数 ----------------------
 * 校准是"开机只做一次"的事，采到的零偏会被扣一整场：
 *   采对了 → 静置时角度不漂
 *   采错了 → 之后 Roll/Yaw 一直匀速漂（而且不会自己回来）
 * 所以这里给足时间、并做有效性检查与重试。 */
#define POWER_UP_DELAY_MS    500U    /* 上电到开始校准之间先等的时长 */
#define CALIB_SAMPLES        1000U   /* 零偏采样次数（约 1ms 一次 → 约 1 秒） */
#define LEVEL_SAMPLES        500U    /* 水平零点（加速度计参考）采样次数 */
#define CALIB_MAX_TRY        3U      /* 最多校准几次 */
#define CALIB_SANE_BIAS      5.0f    /* 单轴零偏超过 5°/s → 认为校准时板子在动，不可信 */
#define BIAS_SHOW_MS         1200U   /* 三轴零偏在屏上停留的时间 */

/* 优先级：用 CMSIS-RTOS V2 的枚举，不写魔数 */
#define PRIO_IMU_TASK        osPriorityRealtime      /* 48 */
#define PRIO_UART_TASK       osPriorityHigh          /* 40 */
#define PRIO_OLED_TASK       osPriorityAboveNormal   /* 32 */

/* 栈大小单位是"字节"（CMSIS-RTOS V2 的约定，不是 FreeRTOS 原生的"字"）。
   C8T6 只有 20KB RAM，堆（configTOTAL_HEAP_SIZE）要装下所有任务的栈和 TCB，
   所以每个任务按实际需要给，够用就行。 */
#define STACK_IMU_TASK       768U    /* 姿态解算里浮点运算 + HAL I2C 调用 */
#define STACK_UART_TASK      1024U   /* 含 64 字节回显副本 + 3 个角度字符串 + HAL 发送调用 */
#define STACK_OLED_TASK      640U    /* 排版缓冲区 17 字节 + 调用深度 */

/* ==================== 任务间通信对象 ==================== */
static osSemaphoreId_t  uart_rx_sem;   /* 串口收到数据（中断 → UART 任务） */
static osEventFlagsId_t init_evt;      /* 初始化完成（IMU 任务 → 其他任务） */
static osMutexId_t      att_mutex;     /* 保护"最新姿态"共享数据 */
static osMutexId_t      i2c_mutex;     /* 保护 I2C1 总线（OLED 与 MPU6050 共用） */

/* 初始化完成标志位 */
#define EVT_INIT_DONE   0x00000001U

/* 【为什么"初始化完成"用事件标志而不是信号量】
 *   二值信号量的计数只有 1，而 UART 任务和 OLED 任务**都要**等这件事。
 *   用信号量时只有一个任务能拿到，另一个会永远等下去（典型死等 bug）。
 *   事件标志是"置位/清零"的，一次 Set 会把所有在等这个位的事件任务全部唤醒，
 *   而且用 osFlagsNoClear 把标志一直留着，谁先到谁都能立刻通过。*/

/* ==================== 共享姿态数据 ====================
 * 只有一份：IMU 任务写，OLED/UART 两个任务读，读写都在 att_mutex 保护下进行。
 * 不加 volatile：结构体整体拷贝时 volatile 没有意义，互斥锁的调用本身
 * 就是编译器无法跨越的屏障，数据一定会重新从内存读。 */
typedef struct
{
	Attitude_t att;     /* 姿态角，单位：度 */
	uint8_t    error;   /* 1：本次 I2C 读取失败，数据未更新 */
	uint32_t   tick;    /* 生成该帧时的毫秒节拍，便于看数据新鲜度 */
} AttitudeMsg_t;

static AttitudeMsg_t g_attShared;
static uint8_t       g_attValid;        /* 0：还没有过有效数据 */

/* ==================== 串口接收（DMA + 空闲中断） ==================== */
static uint8_t  s_uartDmaBuf[UART_RX_BUF_SIZE];    /* DMA 正在往里收的缓冲区 */
static uint8_t  s_uartReadyBuf[UART_RX_BUF_SIZE];  /* 已收完、等任务处理的数据 */
static uint8_t  s_uartReadyLen;                    /* s_uartReadyBuf 里的有效字节数 */

/* 上报用的格式化缓冲区（不放任务栈上，省栈） */
static char s_ReportBuf[96];

/* ======================================================================
 * 串口接收完成回调（在 USART1 中断里被调用，优先级 5）
 *
 * 中断里只做三件事：把数据搬走、重新挂上 DMA、释放信号量。
 * 绝不在这里 printf、绝不在这里刷屏 —— 中断里做耗时操作会拖垮整个系统。
 *
 * 为什么要"把数据搬走"：HAL_UARTEx_ReceiveToIdle_DMA() 会重新把 DMA 挂到
 * s_uartDmaBuf 上，如果不先拷贝，任务回显时缓冲区可能已经被下一帧覆盖。
 * 所以中断里 memcpy 一份到 s_uartReadyBuf（64 字节，几十个周期，很快）。
 *
 * 注意：osSemaphoreRelease() 可以在中断里直接调用（V2 不需要 ...FromISR 版本），
 *       但该中断的优先级数值必须 >= configMAX_SYSCALL_INTERRUPT_PRIORITY(5)，
 *       否则会触发断言，USART1 和 DMA1 的中断都设成 5 就是这个原因。
 * ==================================================================== */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance != USART1)
	{
		return;
	}

	if (Size > UART_RX_BUF_SIZE)
	{
		Size = UART_RX_BUF_SIZE;
	}

	memcpy(s_uartReadyBuf, s_uartDmaBuf, Size);
	s_uartReadyLen = (uint8_t)Size;

	/*重新挂上接收，实现不定长连续收*/
	if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uartDmaBuf, UART_RX_BUF_SIZE) == HAL_OK)
	{
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);	/*关掉过半中断，只要"收完/空闲"那一次*/
	}

	/*通知 UART 任务有新数据*/
	osSemaphoreRelease(uart_rx_sem);
}

/* 下面两个函数在文件后半部分定义（UART 任务上报时也用），这里先声明 */
static void AppRTOS_AngleToStr(char *Buf, uint32_t BufSize, int32_t Centi);
static void AppRTOS_Append(char *Buf, uint32_t BufSize, const char *Text);

/* ======================================================================
 * 这次零偏校准的结果可不可信？
 *
 * 有三种"看着成功、其实白校准"的情况必须挡住：
 *   1) 器件不应答：Attitude_CalibrateGyro() 一次都读不到时直接 return，
 *      零偏保持 0，屏幕显示 00.00o 看着很干净，其实一点都没校准；
 *   2) 三轴零偏同时精确等于 0.000：同上，真传感器不可能三轴一点偏都没有；
 *   3) 零偏大得离谱：校准那一秒里板子在转，平均转速被当成零偏扣掉了，
 *      结果就是之后角度朝反方向一直匀速漂——这就是 "Roll 涨 1°/s、
 *      Yaw 涨 10°/s" 那次复位出现的原因（复位时手还扶着板子）。
 * ==================================================================== */
static uint8_t AppRTOS_CalibSane(float BiasX, float BiasY, float BiasZ)
{
	/* 1) 器件必须在线 */
	if (MPU6050_IsReady() == 0U)
	{
		return 0U;
	}

	/* 2) 三轴同时精确为 0 = 一个样本都没采到 */
	if ((BiasX == 0.0f) && (BiasY == 0.0f) && (BiasZ == 0.0f))
	{
		return 0U;
	}

	/* 3) 零偏大得离谱 = 校准时板子在动 */
	if ((fabsf(BiasX) > CALIB_SANE_BIAS) ||
	    (fabsf(BiasY) > CALIB_SANE_BIAS) ||
	    (fabsf(BiasZ) > CALIB_SANE_BIAS))
	{
		return 0U;
	}

	return 1U;
}

/* ======================================================================
 * 把三轴零偏打一行到串口（开机只发一次）
 * 例：Bias X=01.23 Y=-00.45 Z=09.87
 * ==================================================================== */
static void AppRTOS_SendBiasReport(float BiasX, float BiasY, float BiasZ)
{
	char     Str[9];
	uint32_t Len;

	s_ReportBuf[0] = '\0';

	AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), "Bias X=");
	AppRTOS_AngleToStr(Str, sizeof(Str), OLED_AppAngleToCenti(BiasX));
	AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), Str);

	AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), " Y=");
	AppRTOS_AngleToStr(Str, sizeof(Str), OLED_AppAngleToCenti(BiasY));
	AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), Str);

	AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), " Z=");
	AppRTOS_AngleToStr(Str, sizeof(Str), OLED_AppAngleToCenti(BiasZ));
	AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), Str);

	AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), "\r\n");

	Len = (uint32_t)strlen(s_ReportBuf);
	if (Len > 0U)
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)s_ReportBuf, (uint16_t)Len, 100U);
	}
}

/* ======================================================================
 * 任务 1：IMU 采样与姿态解算（最高优先级，10ms 周期）
 * ==================================================================== */
void IMU_Task(void *argument)
{
	AttitudeMsg_t msg;
	uint32_t      lastWake;
	uint32_t      lastTick;
	float         dt;
	float         biasX, biasY, biasZ;
	uint8_t       ok;
	uint8_t       attempt;
	uint8_t       calibOk;

	(void)argument;

	/*---- 传感器初始化 ----*/
	MPU6050_Init();

	/*---- 开机提示 ----
	 * 这几句在 init_evt 置位之前执行，此时 OLED 任务还在等事件标志，
	 * 不会有第二个任务来抢 I2C1 总线，所以这里不用加锁。
	 * 注意：下面几行的字符串故意都写成 13 个字符，靠显示缓存的"只改变化字符"
	 *       机制刷新，长度不一致会留下上一帧的残字。*/
	OLED_AppClear();
	OLED_AppShowText(1, 1, "IMU Attitude");
	OLED_AppShowText(2, 1, "MPU6050:");
	OLED_AppShowText(2, 9, MPU6050_IsReady() ? "OK " : "ERR");
	OLED_AppShowText(3, 1, "Calibrating...");
	OLED_AppShowText(4, 1, "calib in 0.5s");

	/*---- 先等传感器稳定 ----
	 * MPU6050 从睡眠唤醒后内部 PLL/DLPF 要几十到上百毫秒才稳，
	 * 刚上电立刻采样容易采到错的零偏。这段等待也顺便让用户看清提示。*/
	osDelay(POWER_UP_DELAY_MS);

	/*---- 零偏校准：最多试 CALIB_MAX_TRY 次，采到可信的就收手 ----*/
	Attitude_Init(&msg.att);

	calibOk = 0U;
	for (attempt = 1U; attempt <= CALIB_MAX_TRY; attempt++)
	{
		OLED_AppShowText(3, 1, "Calibrating...");
		if (attempt > 1U)
		{
			OLED_AppShowText(4, 1, "Still! Try   ");
			OLED_ShowNum(4, 12, attempt, 1);
		}
		else
		{
			OLED_AppShowText(4, 1, "Keep still!  ");
		}

		Attitude_CalibrateGyro(CALIB_SAMPLES);
		Attitude_GetGyroOffset(&biasX, &biasY, &biasZ);

		if (AppRTOS_CalibSane(biasX, biasY, biasZ) != 0U)
		{
			calibOk = 1U;
			break;
		}

		/*这次采到的不可信：明确提示一下，然后重来*/
		OLED_AppShowText(3, 1, "Calib bad!    ");
		osDelay(400);
	}

	/*---- 三轴零偏全部显示出来 ----
	 * 只显示 Z 不够：Roll 漂不漂取决于 X 轴零偏，三个数一起看最省事。
	 * 另外这行数也发到串口一份，方便脱离屏幕判断这次校准好不好。*/
	OLED_AppShowAngle(2, "Bias X ", 9, OLED_AppAngleToCenti(biasX));
	OLED_AppShowAngle(3, "Bias Y ", 9, OLED_AppAngleToCenti(biasY));
	OLED_AppShowAngle(4, "Bias Z ", 9, OLED_AppAngleToCenti(biasZ));

	if (calibOk == 0U)
	{
		/*三次都不可信：仍然继续跑（用户可能就是把板子晃了），但要提醒*/
		OLED_AppShowText(1, 1, "CALIB UNSURE");
	}

	AppRTOS_SendBiasReport(biasX, biasY, biasZ);

	/*---- 水平零点：把当前摆法定义成 0°，同时记下加速度计零偏 ----*/
	Attitude_LevelZero(&msg.att, LEVEL_SAMPLES);

	osDelay(BIAS_SHOW_MS);

	/*---- 通知其他任务：初始化完成 ----
	 * 用事件标志而不是信号量：两个读者都要等，置位后一次全部唤醒。*/
	osEventFlagsSet(init_evt, EVT_INIT_DONE);

	/*---- 进入 100Hz 采样循环 ----*/
	lastTick = osKernelGetTickCount();
	lastWake = lastTick;

	for (;;)
	{
		/*周期性调度：比 osDelay 准，不会累积漂移*/
		osDelayUntil(lastWake + IMU_PERIOD_MS);
		lastWake += IMU_PERIOD_MS;

		/*---- 用"真实经过的时间"作为积分步长 ----
		 * 理想情况下一拍就是 10ms；但刷屏时 OLED 任务占着 I2C 总线，
		 * 本任务可能会晚一点拿到总线，实际间隔就不止 10ms。
		 * 用节拍差算 dt 比写死 0.01 更准，角度才不会因为"少算"而滞后。*/
		{
			uint32_t nowTick = osKernelGetTickCount();
			dt = (float)(nowTick - lastTick) * 0.001f;
			lastTick = nowTick;
		}
		if (dt > DT_MAX_SEC)
		{
			dt = DT_MAX_SEC;		/*被拖慢太多时限幅，避免一次积分过头*/
		}
		if (dt < DT_MIN_SEC)
		{
			dt = DT_MIN_SEC;
		}

		/*---- 读传感器 + 解算，全程占住 I2C 总线 ----
		 * MPU6050 和 OLED 挂在同一条 I2C1 上，HAL 的阻塞式传输不可重入：
		 * 必须用 i2c_mutex 保证一次完整传输期间没有别的任务插进来。
		 * 这里用 osWaitForever：采样是最重要的，宁可等也要拿到总线。*/
		ok = 1;
		if (osMutexAcquire(i2c_mutex, osWaitForever) == osOK)
		{
			ok = Attitude_Update(&msg.att, dt);
			osMutexRelease(i2c_mutex);
		}

		msg.error = (ok != 0) ? 1 : 0;
		msg.tick  = HAL_GetTick();

		/*---- 发布最新姿态（写共享区，加锁保护）----
		 * 锁内只做结构体拷贝（约 16 字节），几十个周期就结束，
		 * 不会长时间占锁，也就不会拖慢读者。*/
		if (osMutexAcquire(att_mutex, osWaitForever) == osOK)
		{
			g_attShared = msg;
			g_attValid  = 1;
			osMutexRelease(att_mutex);
		}
	}
}

/* ======================================================================
 * 把角度（单位 0.01°）格式化成 "-12.34" 形式的字符串
 *
 * 不用 printf/snprintf 的 %f：Keil 的 C 库默认不带浮点打印支持，
 * 要么输出乱码，要么直接把程序拖死。所以这里把角度换成"0.01° 的整数"
 * 之后手动拼字符串，既没有浮点开销，也不依赖 C 库的 printf（省几 KB Flash）。
 * 缓冲区至少 8 字节：符号 1 + 整数 3 + '.' + 小数 2 + '\0'。
 * ==================================================================== */
static void AppRTOS_AngleToStr(char *Buf, uint32_t BufSize, int32_t Centi)
{
	int32_t  AbsValue = (Centi < 0) ? -Centi : Centi;
	uint32_t IntPart  = (uint32_t)(AbsValue / 100);
	uint32_t FracPart = (uint32_t)(AbsValue % 100);
	uint32_t Pos      = 0U;
	uint32_t Div      = 1U;

	if ((Buf == 0) || (BufSize < 8U))
	{
		if ((Buf != 0) && (BufSize > 0U))
		{
			Buf[0] = '\0';
		}
		return;
	}

	if (Centi < 0)
	{
		Buf[Pos++] = '-';
	}

	/*先找出整数部分的最高位，再逐位输出（不能输出前导 0）*/
	while ((IntPart / Div) >= 10U)
	{
		Div *= 10U;
	}
	while (Div > 0U)
	{
		Buf[Pos++] = (char)('0' + (IntPart / Div) % 10U);
		Div /= 10U;
	}

	Buf[Pos++] = '.';
	Buf[Pos++] = (char)('0' + (FracPart / 10U));	/*十分位*/
	Buf[Pos++] = (char)('0' + (FracPart % 10U));	/*百分位*/
	Buf[Pos]   = '\0';
}

/* 字符串追加（带长度保护），拼上报报文用，省掉 snprintf */
static void AppRTOS_Append(char *Buf, uint32_t BufSize, const char *Text)
{
	uint32_t Len = (uint32_t)strlen(Buf);

	if ((BufSize == 0U) || (Len >= BufSize - 1U))
	{
		return;
	}
	strncat(Buf, Text, BufSize - 1U - Len);
}

/* ======================================================================
 * 任务 2：串口收发（次高优先级，事件驱动）
 * ==================================================================== */
void UART_Task(void *argument)
{
	AttitudeMsg_t snapshot;
	char          rxCopy[UART_RX_BUF_SIZE];	/* 回显用的本地副本 */
	char          rollStr[9];
	char          pitchStr[9];
	char          yawStr[9];
	int32_t       n;
	uint32_t      lastReport;
	uint8_t       haveData;

	(void)argument;

	/*等 IMU 任务完成校准，避免读到无效数据*/
	if (osEventFlagsWait(init_evt, EVT_INIT_DONE, osFlagsWaitAny | osFlagsNoClear, osWaitForever) != EVT_INIT_DONE)
	{
		osThreadExit();
	}

	/*---- 启动接收 ----
	 * 放在这里而不是 main() 里，是为了保证"调度器已经在跑"。
	 * 若在调度器启动前就开始收，中断里调 osSemaphoreRelease() 属于未定义行为。*/
	if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uartDmaBuf, UART_RX_BUF_SIZE) == HAL_OK)
	{
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}

	lastReport = osKernelGetTickCount();

	for (;;)
	{
		/*---- 等待"串口收到数据"的信号量，最多等 100ms ----
		 * 用信号量而不是轮询标志位：等待期间不消耗 CPU，调度器可以把时间
		 * 让给别的任务。超时醒来是为了顺便做 1Hz 主动上报。*/
		if (osSemaphoreAcquire(uart_rx_sem, UART_WAIT_MS) == osOK)
		{
			uint8_t len = s_uartReadyLen;

			if (len > 0U)
			{
				/*先拷到本任务的栈上再发：s_uartReadyBuf 随时可能被下一个
				  串口中断覆盖，一边发一边被改写会把回显数据弄花。*/
				memcpy(rxCopy, s_uartReadyBuf, len);
				s_uartReadyLen = 0U;

				/*把收到的内容原样回显（阻塞发送，超时 100ms）*/
				HAL_UART_Transmit(&huart1, (uint8_t *)rxCopy, len, 100U);
			}
		}

		/*---- 1Hz 主动上报姿态角 ----*/
		if ((osKernelGetTickCount() - lastReport) >= UART_REPORT_MS)
		{
			lastReport = osKernelGetTickCount();

			/*---- 读共享区：先在锁里取一份快照，出锁之后再慢慢格式化、发送 ----
			 * 关键：锁内只拷贝，绝不在锁里调用串口发送（发送要几毫秒），
			 *       否则会长时间阻塞 IMU 任务。*/
			haveData = 0;
			if (osMutexAcquire(att_mutex, ATT_SNAPSHOT_WAIT) == osOK)
			{
				if (g_attValid)
				{
					snapshot = g_attShared;
					haveData = 1;
				}
				osMutexRelease(att_mutex);
			}

			if (haveData)
			{
				AppRTOS_AngleToStr(rollStr,  sizeof(rollStr),  OLED_AppAngleToCenti(snapshot.att.roll));
				AppRTOS_AngleToStr(pitchStr, sizeof(pitchStr), OLED_AppAngleToCenti(snapshot.att.pitch));
				AppRTOS_AngleToStr(yawStr,   sizeof(yawStr),   OLED_AppAngleToCenti(snapshot.att.yaw));

				/*拼一行报文，例如：Roll=-12.34 Pitch=5.67 Yaw=179.99 OK*/
				s_ReportBuf[0] = '\0';
				AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), "Roll=");
				AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), rollStr);
				AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), " Pitch=");
				AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), pitchStr);
				AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), " Yaw=");
				AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), yawStr);
				AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), snapshot.error ? " [I2C ERR]" : " OK");
				AppRTOS_Append(s_ReportBuf, sizeof(s_ReportBuf), "\r\n");

				n = (int32_t)strlen(s_ReportBuf);
				if (n > 0)
				{
					HAL_UART_Transmit(&huart1, (uint8_t *)s_ReportBuf, (uint16_t)n, 100U);
				}
			}
		}
	}
}

/* ======================================================================
 * 任务 3：OLED 显示（最低优先级，100ms 周期）
 * ==================================================================== */
void OLED_Task(void *argument)
{
	AttitudeMsg_t snapshot;
	uint32_t      lastWake;
	uint8_t       haveData;

	(void)argument;

	/*等 IMU 任务初始化完成（它也负责画开机提示）*/
	if (osEventFlagsWait(init_evt, EVT_INIT_DONE, osFlagsWaitAny | osFlagsNoClear, osWaitForever) != EVT_INIT_DONE)
	{
		osThreadExit();
	}

	/*画一次界面框架，之后每拍只刷新变化的数字*/
	OLED_AppDrawFrame();

	lastWake = osKernelGetTickCount();

	for (;;)
	{
		osDelayUntil(lastWake + OLED_PERIOD_MS);
		lastWake += OLED_PERIOD_MS;

		/*---- 取姿态快照（锁内只拷贝）----*/
		haveData = 0;
		if (osMutexAcquire(att_mutex, ATT_SNAPSHOT_WAIT) == osOK)
		{
			if (g_attValid)
			{
				snapshot = g_attShared;
				haveData = 1;
			}
			osMutexRelease(att_mutex);
		}

		/*---- 刷屏（走 I2C1，最慢的一步，放在锁外面）----
		 * 只在拿到总线时才刷；拿不到就跳过这一拍，下一拍再试，
		 * 绝不因为显示把采样任务堵死在这里。*/
		if (haveData)
		{
			if (osMutexAcquire(i2c_mutex, I2C_WAIT_MS) == osOK)
			{
				OLED_AppUpdateAngles(&snapshot.att, snapshot.error);
				osMutexRelease(i2c_mutex);
			}
		}
	}
}

/* ======================================================================
 * 创建失败时：把错误码显示在屏幕上并停住，方便用调试器/眼睛定位问题
 * ==================================================================== */
static void AppRTOS_InitFail(uint8_t Code)
{
	OLED_AppClear();
	OLED_AppShowText(1, 1, "RTOS INIT FAIL");
	OLED_AppShowText(2, 1, "code");
	OLED_ShowNum(2, 6, Code, 1);
	OLED_AppShowText(3, 1, "heap too small?");
	OLED_AppShowText(4, 1, "check config");

	for (;;)
	{
		/*停在这里等调试器看 Code；不进调度器，所以不能用 osDelay*/
	}
}

/* ======================================================================
 * 创建通信对象和三个任务（CMSIS-RTOS V2）
 * ==================================================================== */
uint8_t AppRTOS_Init(void)
{
	osThreadAttr_t threadAttr;
	osMutexAttr_t  mutexAttr;

	/*---- 二值信号量：初始计数 0，最大 1 ----
	 * uart_rx_sem 表示"串口有数据了"：中断放、一个任务取，标准的信号量场景。*/
	uart_rx_sem = osSemaphoreNew(1U, 0U, NULL);
	if (uart_rx_sem == NULL)
	{
		AppRTOS_InitFail(1U);
		return 1U;
	}

	init_evt = osEventFlagsNew(NULL);
	if (init_evt == NULL)
	{
		AppRTOS_InitFail(2U);
		return 2U;
	}

	/*---- 互斥锁：osMutexPrioInherit 打开优先级继承 ----
	 * 这是互斥锁和信号量最本质的区别：低优先级任务占着锁时，如果高优先级
	 * 任务来抢锁，持锁任务会被临时提升到高优先级，尽快把锁放掉，
	 * 避免"优先级反转"（中间优先级的任务把持锁任务抢占，导致高优先级任务
	 * 无限等待）。本工程里 OLED 任务(32) 持着 I2C 锁时，IMU 任务(48) 来抢，
	 * 内核就会把 OLED 任务临时提到 48 直到它释放总线。
	 *
	 * 之前用 osMutexNew(NULL) 是不带这个属性的（普通互斥锁），这里显式给出。*/
	memset(&mutexAttr, 0, sizeof(mutexAttr));
	mutexAttr.attr_bits = osMutexPrioInherit;

	att_mutex = osMutexNew(&mutexAttr);
	if (att_mutex == NULL)
	{
		AppRTOS_InitFail(3U);
		return 3U;
	}

	i2c_mutex = osMutexNew(&mutexAttr);
	if (i2c_mutex == NULL)
	{
		AppRTOS_InitFail(4U);
		return 4U;
	}

	/*---- 创建三个任务 ----
	 * V2 必须传 osThreadAttr_t 属性结构体，没有 V1 那种 osThreadDef 宏了。
	 * 每次创建前把结构体清零，避免残留字段影响结果。*/
	memset(&threadAttr, 0, sizeof(threadAttr));
	threadAttr.name       = "IMU_Task";
	threadAttr.stack_size = STACK_IMU_TASK;
	threadAttr.priority   = PRIO_IMU_TASK;
	if (osThreadNew(IMU_Task, NULL, &threadAttr) == NULL)
	{
		AppRTOS_InitFail(5U);
		return 5U;
	}

	memset(&threadAttr, 0, sizeof(threadAttr));
	threadAttr.name       = "UART_Task";
	threadAttr.stack_size = STACK_UART_TASK;
	threadAttr.priority   = PRIO_UART_TASK;
	if (osThreadNew(UART_Task, NULL, &threadAttr) == NULL)
	{
		AppRTOS_InitFail(6U);
		return 6U;
	}

	memset(&threadAttr, 0, sizeof(threadAttr));
	threadAttr.name       = "OLED_Task";
	threadAttr.stack_size = STACK_OLED_TASK;
	threadAttr.priority   = PRIO_OLED_TASK;
	if (osThreadNew(OLED_Task, NULL, &threadAttr) == NULL)
	{
		AppRTOS_InitFail(7U);
		return 7U;
	}

	return 0U;
}

/* ======================================================================
 * 栈溢出钩子（FreeRTOSConfig.h 里 configCHECK_FOR_STACK_OVERFLOW = 2 时启用）
 *
 * 任务栈给小了，FreeRTOS 在切换任务时会发现并调用到这里。与其让内存被悄悄
 * 踩坏、跑出莫名其妙的死机，不如直接停在这里：调试器里看 pcTaskName 就知道
 * 是哪个任务的栈不够。
 * ==================================================================== */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	(void)xTask;
	(void)pcTaskName;

	taskDISABLE_INTERRUPTS();
	for (;;)
	{
	}
}
