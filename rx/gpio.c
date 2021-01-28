/*  Copyright (C) 2020 WMIC authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */
#include <stdbool.h>
#include <stm32f0xx.h>
#include "gpio.h"

void gpio_init()
{
    GPIO_InitTypeDef GPIO_InitStructure;

    //PF0 PF1 as GPIO
    RCC_HSEConfig(RCC_HSE_OFF);

    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);

    /****************** OUTPUT *******************/
    //LED Red
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    //LED Green
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //EN1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_ResetBits(GPIOF, GPIO_Pin_1);
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    //EN2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /****************** INPUT *******************/
    //Button Mute & Pair
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //IRQ1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOF, &GPIO_InitStructure);
    //IRQ2
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void exti_init(void)
{
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource0);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource3);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource5);
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource6);

    /* Configure EXTI0 EXTI3 line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0 | EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
    /* Configure EXTI5 EXTI6 line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line5 | EXTI_Line6;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set EXTI5 EXTI6 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable and set EXTI0 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /* Enable and set EXTI3 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI2_3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void radio_power(uint8_t group, bool Enable)
{
    if (group & RF1)
    {
        if (Enable)
            GPIO_SetBits(GPIOF, GPIO_Pin_1);
        else
            GPIO_ResetBits(GPIOF, GPIO_Pin_1);
    }
    if (group & RF2)
    {
        if (Enable)
            GPIO_SetBits(GPIOA, GPIO_Pin_4);
        else
            GPIO_ResetBits(GPIOA, GPIO_Pin_4);
    }
}

void led_control(uint8_t led, bool Enable)
{
    if (led & LED1)
    {
        if (Enable)
            GPIO_SetBits(GPIOA, GPIO_Pin_7);
        else
            GPIO_ResetBits(GPIOA, GPIO_Pin_7);
    }
    if (led & LED2)
    {
        if (Enable)
            GPIO_SetBits(GPIOB, GPIO_Pin_1);
        else
            GPIO_ResetBits(GPIOB, GPIO_Pin_1);
    }
}