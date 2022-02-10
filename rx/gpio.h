/*  Copyright (C) 2020-2022 Frand Ren
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */
 
#ifndef _GPIO_H
#define _GPIO_H
#include <stdbool.h>

/**************************** GPIO-OUT ***************************/
//LED
#define LED1(VALUE)       GPIO_WriteBit(GPIOA, GPIO_Pin_10, (BitAction)(!VALUE))
#define LED2(VALUE)       GPIO_WriteBit(GPIOA, GPIO_Pin_9, (BitAction)(!VALUE))
#define LED3(VALUE)       GPIO_WriteBit(GPIOA, GPIO_Pin_8, (BitAction)(!VALUE))
//NRF-DO
#define NRF1_D0(VALUE)    GPIO_WriteBit(GPIOA, GPIO_Pin_0, (BitAction)(VALUE))
#define NRF1_D1(VALUE)    GPIO_WriteBit(GPIOA, GPIO_Pin_1, (BitAction)(VALUE))
#define NRF2_D0(VALUE)    GPIO_WriteBit(GPIOA, GPIO_Pin_11, (BitAction)(VALUE))
#define NRF2_D1(VALUE)    GPIO_WriteBit(GPIOA, GPIO_Pin_12, (BitAction)(VALUE))
//SELFBOOT
#define SELF_BOOT(VALUE)  GPIO_WriteBit(GPIOB, GPIO_Pin_11, (BitAction)(VALUE))
//RESETn
#define RESETn(VALUE)     GPIO_WriteBit(GPIOB, GPIO_Pin_14, (BitAction)(VALUE))

/***************************** GPIO-IN ***************************/
//IRQ
#define BK1_IRQ()         GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_14)
#define BK2_IRQ()         GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5)
#define BK3_IRQ()         GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)
#define BK4_IRQ()         GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)
#define NRF1_IRQ()        GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)
#define NRF2_IRQ()        GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)
#define USBI_WPn()        GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)
//KEY
#define KEY1()            GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7)
#define KEY2()            GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_8)
#define KEY3()            GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_9)

void gpio_init(void);
void exti_init(void);
#endif
