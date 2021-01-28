/*  Copyright (C) 2020 WMIC authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */
 
#ifndef _GPIO_H_
#define _GPIO_H_

void gpio_init(void);
void exti_init(void);
void tx_power(bool Enable);

#endif
