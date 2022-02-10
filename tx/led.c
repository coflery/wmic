/*  Copyright (C) 2020-2022 Frand Ren
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#include "led.h"
#include <stm32f0xx.h>

void led_init()
{
    GPIO_InitTypeDef led_gpio;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, 1);

    led_gpio.GPIO_Mode = GPIO_Mode_OUT;
    led_gpio.GPIO_OType = GPIO_OType_PP;
    led_gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    led_gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;    
    led_gpio.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(GPIOA, &led_gpio);

    GPIO_ResetBits(GPIOA, GPIO_Pin_0 | GPIO_Pin_1);
}

static void led_set(GPIO_TypeDef *gpiox, uint16_t pin, bool on)
{
    if (on)
        GPIO_SetBits(gpiox, pin);
    else
        GPIO_ResetBits(gpiox, pin);
}

void led_wr_set(bool on)
{
    led_set(GPIOA, GPIO_Pin_0, on);
}

void led_rd_set(bool on)
{
    led_set(GPIOA, GPIO_Pin_1, on);
}

