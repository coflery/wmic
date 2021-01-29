#include <stdbool.h>
#include "stm32f0xx.h"
#include "delay.h"
#include "spdif.h"
#include "i2c.h"
#include "gpio.h"

#define I2C_PORT I2C_PORT1
#define I2C_ADDRESS 0x74
#define TIMEOUT ((uint8_t)20)

static const uint8_t wm8804_reg_defs[] = {
    0x05, /* R0  - RST/DEVID1 */
    0x88, /* R1  - DEVID2 */
    0x04, /* R2  - DEVREV */
    0x21, /* R3  - PLL1 */
    0xFD, /* R4  - PLL2 */
    0x36, /* R5  - PLL3 */
    0x07, /* R6  - PLL4 */
    0x16, /* R7  - PLL5 */
    0x18, /* R8  - PLL6 */
    0xFF, /* R9  - SPDMODE */
    0x00, /* R10 - INTMASK */
    0x00, /* R11 - INTSTAT */
    0x00, /* R12 - SPDSTAT */
    0x00, /* R13 - RXCHAN1 */
    0x00, /* R14 - RXCHAN2 */
    0x00, /* R15 - RXCHAN3 */
    0x00, /* R16 - RXCHAN4 */
    0x00, /* R17 - RXCHAN5 */
    0x00, /* R18 - SPDTX1 */
    0x00, /* R19 - SPDTX2 */
    0x00, /* R20 - SPDTX3 */
    0x71, /* R21 - SPDTX4 */
    0x0B, /* R22 - SPDTX5 */
    0x70, /* R23 - GPO0 */
    0x57, /* R24 - GPO1 */
    0x00, /* R25 */
    0x42, /* R26 - GPO2 */
    0x06, /* R27 - AIFTX */
    0x06, /* R28 - AIFRX */
    0x80, /* R29 - SPDRX1 */
    0x07, /* R30 - PWRDN */
};

uint8_t SPDIF_I2C_Read(uint8_t reg, uint8_t *pBuf);
uint8_t SPDIF_I2C_Write(uint8_t reg, const uint8_t *pBuf, uint8_t len);

uint8_t spdif_init(void)
{
  uint8_t res;
  res = SPDIF_I2C_Write(0, wm8804_reg_defs, sizeof(wm8804_reg_defs));
  if (res)
    return 1;

  uint8_t x[sizeof(wm8804_reg_defs)];
  for (uint8_t i = 0; i < sizeof(wm8804_reg_defs); i++)
  {
    res = SPDIF_I2C_Read(i, &x[i]);
    if (res)
      return 2;
  }

  return 0;
}

uint8_t SPDIF_I2C_Read(uint8_t reg, uint8_t *pBuf)
{
  uint8_t res = 0;

  /* Send START condition */
  I2C_Start(I2C_PORT);

  /* Send slave device address */
  I2C_WriteByte(I2C_PORT, I2C_ADDRESS & 0xFE);
  if (I2C_CheckAck(I2C_PORT) == NACK)
  {
    res = 1;
    goto END;
  }

  /* Send the slave device reg address to read */
  I2C_WriteByte(I2C_PORT, reg & 0x7F);
  if (I2C_CheckAck(I2C_PORT) == NACK)
  {
    res = 2;
    goto END;
  }

  /* Repeated start signal */
  I2C_Start(I2C_PORT);

  /* Send slave device address */
  I2C_WriteByte(I2C_PORT, I2C_ADDRESS | 0x01);
  if (I2C_CheckAck(I2C_PORT) == NACK)
  {
    res = 3;
    goto END;
  }

  /* Read reg data */
  *pBuf = I2C_ReadByte(I2C_PORT);
  /* Disable Acknowledgement */
  I2C_NAck(I2C_PORT);

END:
  /* Send STOP Condition */
  I2C_Stop(I2C_PORT);
  return res;
}

uint8_t SPDIF_I2C_Write(uint8_t reg, const uint8_t *pBuf, uint8_t len)
{
  uint8_t res = 0;

  /* Send START condition */
  I2C_Start(I2C_PORT);

  /* Send slave device address */
  I2C_WriteByte(I2C_PORT, I2C_ADDRESS & 0xFE);
  if (I2C_CheckAck(I2C_PORT) == NACK)
  {
    res = 1;
    goto END;
  }

  /* Send the slave device reg address to write */
  I2C_WriteByte(I2C_PORT, reg & 0x7F);
  if (I2C_CheckAck(I2C_PORT) == NACK)
  {
    res = 2;
    goto END;
  }

  /* Send data bytes to write */
  for (uint8_t i = 0; i < len; i++)
  {
    I2C_WriteByte(I2C_PORT, *pBuf);
    if (I2C_CheckAck(I2C_PORT) == NACK)
    {
      res = 3;
      goto END;
    }
    pBuf++;
  }

END:
  /* Send STOP Condition */
  I2C_Stop(I2C_PORT);
  return res;
}
