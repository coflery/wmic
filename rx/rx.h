#ifndef _RX_H
#define _RX_H
#include "i2c.h"

typedef struct
{
    I2C_BUS device;
    bool bus_busy;
} BK;

typedef enum
{
    V160_178,
    V178_270,
    U500_710,
    U710_980
} RF;

uint8_t RX_Init(uint8_t index, uint32_t freq, bool DataOutAtLRCK);
void RX_Check_PhaseLock(uint8_t index);
bool RX_Set_I2C_Bus(uint8_t index);
void RX_IRQ(uint8_t index);
#endif