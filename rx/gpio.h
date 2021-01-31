/*  Copyright (C) 2020 WMIC authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */
 
#ifndef _GPIO_H_
#define _GPIO_H_

#define LED1 1
#define LED2 2

#define RF1 1
#define RF2 2

void gpio_init(void);
void exti_init(void);
void radio_power(uint8_t group, bool Enable);
void led_control(uint8_t led, bool Enable);

void button_reset(void);

#endif
