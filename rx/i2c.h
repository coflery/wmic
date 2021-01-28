#ifndef _I2C_H
#define _I2C_H

enum i2c_port
{
    I2C_PORT1 = 1,
    I2C_PORT2 = 2
};

enum ack_bit
{
    ACK = 0,
    NACK = 1
};

void i2c_init(void);
void I2C_Start(enum i2c_port port);
void I2C_Stop(enum i2c_port port);
void I2C_Ack(enum i2c_port port);
void I2C_NAck(enum i2c_port port);
void I2C_WriteByte(enum i2c_port port, uint8_t byte);
uint8_t I2C_ReadByte(enum i2c_port port);
enum ack_bit I2C_CheckAck(enum i2c_port port);

#endif