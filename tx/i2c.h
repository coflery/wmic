#ifndef _I2C_H
#define _I2C_H

#define I2C_WR ((uint8_t)0) /* 写控制bit */
#define I2C_RD ((uint8_t)1) /* 读控制bit */

#define ACK ((uint8_t)0)
#define NOACK ((uint8_t)1)

void i2c_init(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Ack(void);
void I2C_NAck(void);
void I2C_WriteByte(uint8_t byte);
uint8_t I2C_ReadByte(void);
uint8_t I2C_CheckAck(void);

#endif /* _I2C_H */