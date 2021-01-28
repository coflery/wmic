#include "stm32f0xx.h"
#include "delay.h"
#define TIM4_PERIOD    62

void delay_init()
{
  SystemCoreClockUpdate();
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    while (1)
      ;
  }
}

void delay_cycles(uint16_t cycle)
{
    while (cycle != 0)
    {
        cycle--;
    }
}

void delay_ms(uint16_t nTime)
{
  uint32_t setTime = nTime;
  setTime += SystemTime;
  while (SystemTime < setTime);
}

// void TIM4_Config(void)
// {
//   disableInterrupts();
  
//   /* TIM4 configuration:
//    - TIM4CLK is set to 8 MHz, the TIM4 Prescaler is equal to 128 so the TIM1 counter
//    clock used is 8 MHz / 128 = 62 500 Hz
//   - With 62 500 Hz we can generate time base:
//       max time base is 4.096 ms if TIM4_PERIOD = 255 --> (255 + 1) / 62500 = 4.096 ms
//       min time base is 0.032 ms if TIM4_PERIOD = 1   --> (  1 + 1) / 62500 = 0.032 ms
//   - In this example we need to generate a time base equal to 1.008 ms
//    so TIM4_PERIOD = (0.001008 * 62500 - 1) = 62 */

//   /* Time base configuration */
//   TIM4_TimeBaseInit(TIM4_PRESCALER_128, TIM4_PERIOD);
//   /* Clear TIM4 update flag */
//   TIM4_ClearFlag(TIM4_FLAG_UPDATE);
//   /* Enable update interrupt */
//   TIM4_ITConfig(TIM4_IT_UPDATE, ENABLE);
  
//   ITC_SetSoftwarePriority(ITC_IRQ_TIM4_OVF,ITC_PRIORITYLEVEL_1);
  
//   /* enable interrupts */
//   enableInterrupts();

//   /* Enable TIM4 */
//   TIM4_Cmd(ENABLE);
// }
