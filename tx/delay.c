#include "stm32f0xx.h"
#include "delay.h"

//每次systick中断触发SystemTime递增
volatile uint32_t SystemTime = 0;

/**
  * @brief  初始化延迟功能
  * @param  TimeBase: SystemTick发生中断的时间基准
  *     @arg INT_100MS
  *     @arg INT_10MS
  *     @arg INT_1MS
  *     @arg INT_100US
  *     @arg INT_10US
  *     @arg INT_1US
  */
void Delay_Init(uint32_t TimeBase)
{
	//systemtick 定时器优先级设置为最高
	NVIC_SetPriority(SysTick_IRQn, 0);
	SystemCoreClockUpdate();
	if (SysTick_Config(SystemCoreClock / TimeBase))
	{
		while (1);
	}
}

/**
  * @brief  等待n个微秒时间
  * @param  us: 微秒值
  */
void Delay_us(uint32_t us)
{
	uint32_t dwCurCounter;				  /* 当前时间计数值 */
	uint32_t dwPreTickVal = SysTick->VAL; /* 上一次SYSTICK计数值 */
	uint32_t dwCurTickVal;				  /* 当前的SYSTICK计数值 */

	us = us * (SystemCoreClock / 1000000); /* 需延时时间，共多少时间节拍 */
	do
	{
		dwCurTickVal = SysTick->VAL;
		if (dwCurTickVal < dwPreTickVal)
		{
			//systick是减计数器
			dwCurCounter = dwCurCounter + dwPreTickVal - dwCurTickVal;
		}
		else
		{
			//systick是加计数器
			dwCurCounter = dwCurCounter + dwPreTickVal + SysTick->LOAD - dwCurTickVal;
		}
		dwPreTickVal = dwCurTickVal;

	} while (dwCurCounter < us);
}

/**
  * @brief  等待n个毫秒时间
  * @param  ms: 毫秒值(最大65535)
  */
void Delay_ms(uint16_t ms)
{
	uint32_t setTime = ms;
	setTime += SystemTime;
	while (SystemTime < setTime);
}

/**
  * @brief  等待CPU空转n个周期
  * @param  cycle: 周期计数
  */
void Delay_cycles(uint32_t cycle)
{
	while (cycle--);
}

/**
  * @brief  等待一个NOP时间
  */
void Delay_nop(void)
{
#if __GNUC__
	asm("NOP");
#else
	__nop();
#endif
}