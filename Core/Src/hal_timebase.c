#include "main.h"
#include "tim.h"

/* ======================================================================
 * HAL 时基（用 TIM4 代替 SysTick）
 *
 * 【为什么需要这个文件】
 * 启用 FreeRTOS 后，SysTick 被 FreeRTOS 内核独占用于任务节拍。
 * 而 HAL 库需要另一个定时器提供 1ms 节拍来支撑 HAL_GetTick()/HAL_Delay()
 * 以及 I2C 等外设的超时机制。CubeMX 在 SYS 里把 Timebase Source 设成 TIM4
 * 就是这个目的。
 *
 * 但 CubeMX 对"虚拟时基"（VP_SYS_VS_tim4）的处理不一定生成 tim.c，
 * 结果是 stm32f1xx_it.c 里有 TIM4_IRQHandler 却没有 htim4 定义，
 * 且 HAL_InitTick() 仍是 SysTick 版本 —— 这会导致 HAL_GetTick() 永不递增，
 * HAL_Delay() 死循环、I2C 超时永不触发。
 *
 * 本文件用强符号覆盖 HAL 的 __weak HAL_InitTick()，自己配好 TIM4。
 * 如果以后 CubeMX 真的生成了 tim.c，把它排除出工程即可（或保留本文件，
 * 靠下面的 #if 保证不会重复定义）。
 * ====================================================================== */

#if !defined(TIM_BASE)

TIM_HandleTypeDef htim4;

/**
  * @brief  用 TIM4 产生 1ms 的 HAL 时基节拍
  * @param  TickPriority 时基中断优先级
  * @retval HAL status
  * @note   覆盖 stm32f1xx_hal.c 里的 __weak HAL_InitTick()。
  *         本函数由 HAL_Init() 在启动时自动调用。
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
	RCC_ClkInitTypeDef clkconfig;
	uint32_t           pFLatency;
	uint32_t           uwTimclock;
	uint32_t           uwPrescalerValue;

	/* ---------- 1. 计算 TIM4 的时钟频率 ----------
	 * TIM4 挂在 APB1 总线上。STM32F1 有个规矩：
	 *   APB1 分频 = 1 时，定时器时钟 = PCLK1
	 *   APB1 分频 ≠ 1 时，定时器时钟 = PCLK1 × 2
	 * 本例 APB1 是 HCLK/2 = 36MHz，所以 TIM4 时钟 = 72MHz。
	 * 这里动态读取 RCC 配置，换主频时不需要改代码。*/
	HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);
	uwTimclock = HAL_RCC_GetPCLK1Freq();
	if (clkconfig.APB1CLKDivider != RCC_HCLK_DIV1)
	{
		uwTimclock *= 2U;
	}

	/* ---------- 2. 算预分频，让计数频率为 1MHz ----------
	 * 1MHz 计数下，计 1000 次正好是 1ms，方便计算周期*/
	uwPrescalerValue = (uint32_t)((uwTimclock / 1000000U) - 1U);

	/* ---------- 3. 配置 TIM4 ---------- */
	__HAL_RCC_TIM4_CLK_ENABLE();

	htim4.Instance               = TIM4;
	htim4.Init.Prescaler         = uwPrescalerValue;                 /* 72MHz → 1MHz */
	htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim4.Init.Period            = (uint32_t)(1000U / uwTickFreq) - 1U;  /* 1000 → 1ms */
	htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
	{
		return HAL_ERROR;
	}

	/* ---------- 4. 配置并开启中断 ----------
	 * 优先级由 HAL_Init() 传入（默认 TICK_INT_PRIORITY = 0）。
	 * 这里用 0 是合适的：HAL_IncTick 只做自增，不调用任何 RTOS API，
	 * 所以可以比 configMAX_SYSCALL_INTERRUPT_PRIORITY 更高优先。*/
	HAL_NVIC_SetPriority(TIM4_IRQn, TickPriority, 0U);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	uwTickPrio = TickPriority;

	return HAL_TIM_Base_Start_IT(&htim4);
}

/**
  * @brief  TIM4 中断服务函数
  * @note   stm32f1xx_it.c 里的 TIM4_IRQHandler 会调用 HAL_TIM_IRQHandler(&htim4)，
  *         最终进到这里的 HAL_TIM_PeriodElapsedCallback 回调。
  *         把 HAL_IncTick() 放在回调里，保证 HAL_GetTick() 正常递增。
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM4)
	{
		HAL_IncTick();
	}
}

#endif /* !TIM_BASE */
