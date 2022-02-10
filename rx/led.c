#include "led.h"
#include "gpio.h"
#include "delay.h"

uint32_t led_run_time = 0;
uint32_t led_data_time = 0;

void led_task(void)
{
	if (SystemTime > led_data_time)
	{
		LED3(false);
	}

	if (SystemTime < led_run_time + 50)
	{
		LED3(true);
	}
	else if (SystemTime < led_run_time + 1000)
	{
		LED3(false);
	}
	else
	{
		led_run_time = SystemTime;
	}
}

void led_data_trig(void)
{
	LED3(true);
	led_data_time = SystemTime + 3;
}

void led_hard_fault(void)
{
	LED3(true);
	Delay_cycles(0xFFFFF);
	LED3(false);
	Delay_cycles(0xFFFFF);
}