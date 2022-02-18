#ifndef _I2C_H
#define _I2C_H
#include <stdbool.h>

#define I2C_WR(VALUE) (VALUE & (uint8_t)0xFE) /* 写 */
#define I2C_RD(VALUE) (VALUE | (uint8_t)0x01) /* 读 */

typedef enum
{
    I2C_ISACK = 0,
    I2C_NOACK = 1
} I2C_ACK;

typedef struct
{
    uint8_t tHD_STA; /* Hold time for a repeated START condition */
    uint8_t tSU_STA; /* Setup time for a repeated START condition */
    uint8_t tHD_DAT; /* Data hold time */
    uint8_t tSU_DAT; /* Data setup time */
    uint8_t tSU_STO; /* Setup time for STOP condition */
    uint8_t tHIGH;   /* HIGH period of the SCL clock */
    uint8_t tBUF;    /* Bus free time between a STOP and START condition */
} I2C_TIMING;

typedef struct
{
    void (*SetSDAIn)(void);        /* 设置SDA为输入 */
    void (*SetSDAOut)(void);       /* 设置SDA为输出 */
    void (*SetSCLIn)(void);        /* 设置SCL为输入 */
    void (*SetSCLOut)(void);       /* 设置SCL为输出 */
    bool (*ReadSDA)(void);         /* 读SDA */
    bool (*ReadSCL)(void);         /* 读SCL */
    void (*WriteSDA)(bool isHigh); /* 写SDA */
    void (*WriteSCL)(bool isHigh); /* 写SCL */
    const I2C_TIMING *Timing;      /* 时序参数 */
} I2C_BUS;

extern const I2C_BUS I2C_Bus1;

void i2c_init(void);
bool I2C_BUS_SendStart(const I2C_BUS *bus);
void I2C_BUS_SendRestart(const I2C_BUS *bus);
void I2C_BUS_SendStop(const I2C_BUS *bus);
I2C_ACK I2C_BUS_SendByte(const I2C_BUS *bus, uint8_t data);
uint8_t I2C_BUS_ReceiveByte(const I2C_BUS *bus, I2C_ACK ack);
#endif