/*  Copyright (C) 2020-2022 Frand Ren
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#include <stdbool.h>
#include "stm32f0xx.h"
#include "uart.h"
#include "delay.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "rx.h"
#include "nrf.h"


bool NRF1_IRQ = false;
bool NRF2_IRQ = false;
bool KEY1_IRQ = false;
bool KEY2_IRQ = false;
bool KEY3_IRQ = false;

int main(void)
{
    Delay_Init(INT_1MS);
    gpio_init();
    exti_init();
    SPI1_Init();
    SwitchSpiHiZ(true);
    i2c_init();
    //RX_Init(1, 800000, 0);
    RX_Init(2, 803000, 1);
    RX_Init(3, 806000, 0);
    //RX_Init(4, 809000, 1);
    NRF_init(1);
    NRF_init(2);

    SELF_BOOT(1);
    RESETn(0);
    Delay_ms(50);
    RESETn(1);

    while (1)
    {
        RX_Check_PhaseLock(1);
        RX_Check_PhaseLock(2);
        RX_Check_PhaseLock(3);
        RX_Check_PhaseLock(4);
        Delay_ms(250);
    }
    return 0;
}