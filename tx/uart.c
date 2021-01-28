/*  Copyright (C) 2020 WMIC authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#include "uart.h"
#include <stm32f0xx.h>

void uart_init()
{
    USART_InitTypeDef usart;
    GPIO_InitTypeDef usart_gpio;

    /* Enable GPIO clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    /* Enable UART clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 

    /* Configure USART Tx as alternate function push-pull */
    usart_gpio.GPIO_Mode = GPIO_Mode_AF;
    usart_gpio.GPIO_OType = GPIO_OType_PP;
    usart_gpio.GPIO_Pin = GPIO_Pin_9;
    usart_gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &usart_gpio);

    /* Configure USART Rx as input floating */
    usart_gpio.GPIO_Mode = GPIO_Mode_IN;
    usart_gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    usart_gpio.GPIO_Pin = GPIO_Pin_10;
    GPIO_Init(GPIOA, &usart_gpio);

    /* USART configuration */
    usart.USART_BaudRate = 115200;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &usart);

    /* Enable USART */
    USART_Cmd(USART1, ENABLE);
}

int __io_putchar(int ch)
{
    USART_SendData(USART1, (uint8_t)ch);

    /* Loop until the end of transmission */
    while (!USART_GetFlagStatus(USART1, USART_FLAG_TC));

    return ch;
}
