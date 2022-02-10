/*  Copyright (C) 2020-2022 Frand Ren
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#ifndef _LED_H
#define _LED_H

#include <stdbool.h>

void led_init();
void led_wr_set(bool on);
void led_rd_set(bool on);

#endif

