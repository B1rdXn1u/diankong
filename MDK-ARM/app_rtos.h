#ifndef __APP_RTOS_H
#define __APP_RTOS_H

/* ==================== FreeRTOS 应用任务（CMSIS-RTOS V2） ====================
 *
 * 【为什么文件叫 app_rtos.c 而不是 task.c】
 *   FreeRTOS 内核自己的头文件就叫 task.h。工程里再放一个 task.h，而 UV 的
 *   包含路径里 ..\MDK-ARM 排在 FreeRTOS/Source/include 前面，
 *   cmsis_os2.c、queue.h、timers.h 里的 #include "task.h" 就会先找到我们的
 *   这个头文件，内核结构体、函数原型全部缺失，编译直接崩掉。
 *   改名为 app_rtos.* 之后，工程头文件与内核头文件不再撞名。
 *
 * 【任务划分与优先级】（数值越大优先级越高）
 *   1. IMU_Task   osPriorityRealtime(48)      最高：100Hz 采样 + 姿态解算
 *       决定角度解算的时间精度，必须最及时；只有它一个任务用这么高的优先级。
 *   2. UART_Task  osPriorityHigh(40)          次高：收到数据回显 + 1Hz 上报
 *   3. OLED_Task  osPriorityAboveNormal(32)   最低：10Hz 刷屏
 *
 *   为什么 OLED 最低：I2C 刷屏是三者里最慢、最不实时的工作。它排在最低优先级，
 *   采样任务一到期就能抢占它，只有它已经占着 I2C 总线时才需要等。
 *
 * 【任务间通信（全部 CMSIS-RTOS V2 接口）】
 *   uart_rx_sem   二值信号量  串口空闲中断 → UART 任务："有新数据了"
 *   init_evt      事件标志    IMU 任务校准完成 → OLED/UART 任务："可以开始了"
 *                            （两个任务都要等这件事，用信号量只有一个能拿到，
 *                              所以这里用事件标志，一次置位全部唤醒）
 *   att_mutex     互斥锁      保护"最新一帧姿态"这份共享数据（多读者）
 *   i2c_mutex     互斥锁      I2C1 是 OLED 与 MPU6050 共用的物理总线，
 *                             一次传输期间不能让另一个任务插进来
 *
 * 【为什么"最新姿态"用互斥锁而不是消息队列】
 *   V2 的 osMessageQueueGet() 是消费型的：取走即删除。若 OLED 和 UART 两个
 *   任务都去 Get 同一个队列，它们会互相抢消息，谁先取得谁得到，另一个就不
 *   更新了。"最新姿态"本质是共享状态而不是一次性事件，所以用互斥锁保护一份
 *   共享数据，两个读者各取一份快照，互不影响。
 *       队列   → 适合"生产者 → 单一消费者"的一次性事件流
 *       互斥锁 → 适合"多读者共享最新状态"
 *
 * 【初始化顺序】（V2 严格规定，顺序错了对象创建会失败）
 *   1. osKernelInitialize()   ← main.c 里调用
 *   2. MX_FREERTOS_Init()     ← 内部调用本模块的 AppRTOS_Init() 创建任务
 *   3. osKernelStart()        ← 启动调度器，不再返回
 * ========================================================================== */

#include <stdint.h>
#include "cmsis_os2.h"

/* 三个任务的入口。CMSIS-RTOS V2 的线程签名固定是 void fn(void *argument)，
   和 V1 的 void fn(void const *argument) 不同，不能混用。 */
void IMU_Task(void *argument);
void UART_Task(void *argument);
void OLED_Task(void *argument);

/**
  * @brief  创建信号量、互斥锁和三个任务
  * @note   必须在 osKernelInitialize() 之后、osKernelStart() 之前调用。
  *         由 CubeMX 生成的 MX_FREERTOS_Init() 在 USER CODE 段里调用。
  * @retval 0：成功
  * @retval 1~7：创建失败。此时函数已经在 OLED 上显示错误码并停住，
  *         不会返回（最常见的原因是 configTOTAL_HEAP_SIZE 不够）。
  */
uint8_t AppRTOS_Init(void);

#endif
