/*  Copyright (C) 2020 WMIC authors
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 3.
 */

#include "led.h"
#include "uart.h"
#include "delay.h"
#include "gpio.h"
#include "i2c.h"
#include "tx.h"

int main()
{
    delay_init();
    gpio_init();
    exti_init();
    delay_ms(1000);
    if (GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1))
    {
        tx_power(DISABLE);
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        //开启芯片电源管理的时钟
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
        //进入停止（stop）模式，选择低压调节器（这样启动会慢一些，当然功耗会更低），启用外部中断唤醒。
        PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
        NVIC_SystemReset();
    }
    //led_init();
    i2c_init();
    tx_power(ENABLE);
    init_tx();

    GPIO_SetBits(GPIOA, GPIO_Pin_0);
    GPIO_ResetBits(GPIOA, GPIO_Pin_6);
    while (!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1));

    while (1)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_0);
        delay_ms(500);
        GPIO_ResetBits(GPIOA, GPIO_Pin_0);
        delay_ms(500);

        if (!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1))
        {
            GPIO_SetBits(GPIOA, GPIO_Pin_0);
            uint32_t time = SystemTime;
            while (!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1))
            {
                if (SystemTime - time > 1000)
                {
                    tx_power(DISABLE);
                    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
                    while (!GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_1));
                    //开启芯片电源管理的时钟
                    //RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
                    //进入停止（stop）模式，选择低压调节器（这样启动会慢一些，当然功耗会更低），启用外部中断唤醒。
                    //PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
                    NVIC_SystemReset();
                }
            }
        }
        TX_RF_UnLock_Check(); //定时(几十ms或几百ms均可)执行此函数，防止发射RF失锁
    }
    return 0;
}
