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
    radio_power(RF1 | RF2, ENABLE);
    delay_ms(50);

    button_reset();

    init_rx();
    res = spdif_init();
    if (res == 0)
    {
        led_control(LED1, ENABLE);
        delay_ms(500);
    }
    led_control(LED1, DISABLE);
    delay_ms(500);
    // hw_i2c_config();
    // nvic_config();
    res = fm_init();
    while (1)
    {
        if (res != 0)
            res = fm_init();

        led_control(LED2, res == 0);
        delay_ms(1000);
        led_control(LED2, DISABLE);
        delay_ms(1000);
    }
    return 0;
}