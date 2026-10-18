/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.0.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * These parameters and more are described within the 'configuration' section of the
 * FreeRTOS API documentation available on the FreeRTOS.org web site.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* Section where include file can be added */
/* USER CODE END Includes */

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
#endif
#define configUSE_PREEMPTION                     1
#define configSUPPORT_STATIC_ALLOCATION          1
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )
#define configTICK_RATE_HZ                       ((TickType_t)1000)
#define configMAX_PRIORITIES                     ( 56 )
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)

/* ==========================================================================
 * 堆大小（FreeRTOS 的堆，任务栈和 TCB 都从这里申请）
 *
 * CubeMX 默认只给 3072 字节，装不下本工程的 3 个任务：
 *     IMU_Task   768 + UART_Task 1024 + OLED_Task 640   = 2432 字节栈
 *     加上空闲任务 512、软件定时器任务 512、4 个 TCB、2 个信号量、2 个互斥锁
 *     以及 heap_4 的对齐开销，一共约 4.5KB。
 * 3072 字节时 osThreadNew() 会返回 NULL（任务创建失败），
 * 所以这里给到 8192，留足余量。C8T6 有 20KB RAM，够用。
 *
 * 【注意】CubeMX 重新生成代码会把这行改回默认值，届时记得再改回来。
 * ======================================================================= */
#define configTOTAL_HEAP_SIZE                    ((size_t)8192)
#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configUSE_TRACE_FACILITY                 1
#define configUSE_16_BIT_TICKS                   0
#define configUSE_MUTEXES                        1
#define configQUEUE_REGISTRY_SIZE                8
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0

/* ==========================================================================
 * 栈溢出检查：2 = 除了检查栈顶的标记字，切换任务时还检查栈指针有没有越界。
 *
 * 任务栈给小了是很常见的坑，踩了以后表现为"跑着跑着就死机/数据乱掉"，
 * 很难查。打开这个检查后，FreeRTOS 一发现栈溢出就调用
 * vApplicationStackOverflowHook()（本工程在 app_rtos.c 里实现：停住，
 * 调试器里看 pcTaskName 就知道是哪个任务）。
 * 代价是每次任务切换多几十个周期，本工程不在乎这点开销。
 * ======================================================================= */
#define configCHECK_FOR_STACK_OVERFLOW           2

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* Software timer definitions. */
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                ( 2 )
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             128

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_xTimerPendFunctionCall      1
#define INCLUDE_xQueueGetMutexHolder        1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_eTaskGetState               1

/* CMSIS-RTOS V2 的 wrapper（cmsis_os2.c）里用 xTaskGetCurrentTaskHandle() 实现
   osThreadGetId() 等接口，没打开这个宏会在 freertos_os2.h 里直接 #error：
   "Definition INCLUDE_xTaskGetCurrentTaskHandle must equal 1 ..." */
#define INCLUDE_xTaskGetCurrentTaskHandle   1

/*
 * The CMSIS-RTOS V2 FreeRTOS wrapper is dependent on the heap implementation used
 * by the application thus the correct define need to be enabled below
 */
#define USE_FreeRTOS_HEAP_4

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
 /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* Interrupt priorities used by the kernel port layer itself.  These are generic
to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
/* USER CODE BEGIN 1 */
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
/* USER CODE END 1 */

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: This define is commented when used with STM32Cube firmware, when the timebase source is SysTick,
              to prevent overwriting SysTick_Handler defined within STM32Cube HAL */

/* 【注意】这一行在 CMSIS-RTOS V2 工程里必须注释掉：
 *   cmsis_os2.c 自己提供了一个 SysTick_Handler（内部调用 xPortSysTickHandler()），
 *   如果这里再把 port.c 里的 xPortSysTickHandler 改名成 SysTick_Handler，
 *   链接时会报 L6200E: Symbol SysTick_Handler multiply defined (port.o 与 cmsis_os2.o)。
 *   HAL 的 1ms 时基已经换成 TIM4，SysTick 交给 FreeRTOS 独占，所以注释掉这行是安全的。 */
/* #define xPortSysTickHandler SysTick_Handler */

/* USER CODE BEGIN Defines */
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */

/* ==========================================================================
 * 补上器件头文件宏，否则 cmsis_os2.c 编译不过
 *
 * CMSIS-RTOS V2 的 wrapper 文件 freertos_os2.h 第 31 行是：
 *     #include CMSIS_device_header
 * 这个宏本该由 Keil 的 RTE 组件（Device::Startup）自动生成到
 * MDK-ARM/RTE/RTE_Components.h 里。本工程的 RTE_Components.h 是空的
 * （CubeMX 生成 RTE 时没带上 Device 组件），于是编译 cmsis_os2.c 会报
 *     freertos_os2.h(31): error: #13: expected a file name
 * 在这里定义等价于在工程的 "C/C++ → Define" 里加
 *     CMSIS_device_header=stm32f1xx.h
 * 放这里的好处：CubeMX 重新生成代码时 USER CODE 段不会被覆盖。
 * ======================================================================= */
#define CMSIS_device_header "stm32f1xx.h"
/* USER CODE END Defines */

#endif /* FREERTOS_CONFIG_H */
