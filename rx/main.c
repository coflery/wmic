/*  Copyright (C) 2020 WMIC authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#include <stdbool.h>
#include "stm32f0xx.h"
#include "uart.h"
#include "jtag.h"
#include "delay.h"
#include "gpio.h"
#include "i2c.h"
#include "rx.h"
#include "spdif.h"
#include "fm.h"
#include "version.h"

/* Private function prototypes -----------------------------------------------*/
void nvic_config(void);
void hw_i2c_config(void);

uint8_t res;
int main(void)
{
    jtag_init();
    delay_init();
    gpio_init();
    exti_init();
    i2c_init();

    radio_power(RF1 | RF2, DISABLE);
    delay_ms(50);
    radio_power(RF1, ENABLE);
    delay_ms(50);
    radio_power(RF2, ENABLE);
    delay_ms(50);

    res = init_rx();
    if (res == 0)
    {
        led_control(LED2, ENABLE);
        delay_ms(500);
    }
    led_control(LED2, DISABLE);
    // hw_i2c_config();
    // nvic_config();
    while (1)
    {
        res = spdif_init();
        led_control(LED1, res == 0);
        delay_ms(250);
        led_control(LED1, DISABLE);

        res = fm_init();
        led_control(LED2, res == 0);
        delay_ms(250);
        led_control(LED2, DISABLE);
    }
    return 0;
}