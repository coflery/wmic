#include <stdbool.h>
#include "stm32f0xx.h"
#include "delay.h"
#include "spdif.h"
#include "i2c.h"
#include "gpio.h"
#include "bitmap.h"

#define I2C_ADDRESS 0x74
#define TIMEOUT ((uint8_t)20)

// static const uint8_t wm8804_reg_defs[] = {
//     0x05, /* R0  - RST/DEVID1 */
//     0x88, /* R1  - DEVID2 */
//     0x04, /* R2  - DEVREV */
//     0x21, /* R3  - PLL1 */
//     0xFD, /* R4  - PLL2 */
//     0x36, /* R5  - PLL3 */
//     0x07, /* R6  - PLL4 */
//     0x16, /* R7  - PLL5 */
//     0x18, /* R8  - PLL6 */
//     0xFF, /* R9  - SPDMODE */
//     0x00, /* R10 - INTMASK */
//     0x00, /* R11 - INTSTAT */
//     0x00, /* R12 - SPDSTAT */
//     0x00, /* R13 - RXCHAN1 */
//     0x00, /* R14 - RXCHAN2 */
//     0x00, /* R15 - RXCHAN3 */
//     0x00, /* R16 - RXCHAN4 */
//     0x00, /* R17 - RXCHAN5 */
//     0x00, /* R18 - SPDTX1 */
//     0x00, /* R19 - SPDTX2 */
//     0x00, /* R20 - SPDTX3 */
//     0x71, /* R21 - SPDTX4 */
//     0x0B, /* R22 - SPDTX5 */
//     0x70, /* R23 - GPO0 */
//     0x57, /* R24 - GPO1 */
//     0x00, /* R25 */
//     0x42, /* R26 - GPO2 */
//     0x06, /* R27 - AIFTX */
//     0x06, /* R28 - AIFRX */
//     0x80, /* R29 - SPDRX1 */
//     0x07, /* R30 - PWRDN */
// };

uint8_t SPDIF_I2C_Read(uint8_t reg, uint8_t *pBuf);
uint8_t SPDIF_I2C_Write(uint8_t reg, uint8_t data);

uint8_t spdif_init(void)
{
  uint8_t i;
  uint8_t res = 0;
  uint8_t deviceID_L, deviceID_H;
  uint8_t devREV;

  for (i = 0; i < TIMEOUT; i++,res=0)
  {
    Delay_ms(20);
    res += SPDIF_I2C_Read(0, &deviceID_L);
    res += SPDIF_I2C_Read(1, &deviceID_H);
    res += SPDIF_I2C_Read(2, &devREV);
    if (res)
      continue;
    break;
  }
  if (i == TIMEOUT)
    return 1;

  if (0x8805 != (deviceID_H << 8 | deviceID_L))
    return 2;

  res += SPDIF_I2C_Write(0, B00000000); //Reset device

  // REGISTER 7
  // bit 7:6 - always 0
  // bit 5:4 - CLKOUT divider select => 00 = 512 fs, 01 = 256 fs, 10 = 128 fs, 11 = 64 fs
  // bit 3 - MCLKDIV select => 0
  // bit 2 - FRACEN => 1
  // bit 1:0 - FREQMODE => 10
  res += SPDIF_I2C_Write(7, B00000110);

  // REGISTER 8
  // set clock outputs and turn off last data hold
  // bit 7 - MCLK output source select is CLK2 => 0
  // bit 6 - always valid => 0
  // bit 5 - fill mode select => 1 (we need to see errors when they happen)
  // bit 4 - CLKOUT pin disable => 1
  // bit 3 - CLKOUT pin select is CLK1 => 0
  // bit 2:0 - always 0
  res += SPDIF_I2C_Write(8, B00110000);

  // set masking for interrupts
  res += SPDIF_I2C_Write(10, B01111110); // 1+2+3+4+5+6 => 0111 1110. We only care about unlock and rec_freq

  // set the AIF TX
  // bit 7:6 - always 0
  // bit   5 - LRCLK polarity => 0
  // bit   4 - BCLK invert => 0
  // bit 3:2 - data word length => 10 (24b) or 00 (16b)
  // bit 1:0 - format select: 11 (dsp), 10 (i2s), 01 (LJ), 00 (RJ)
  res += SPDIF_I2C_Write(27, B00001010);

  // set the AIF RX
  // bit   7 - SYNC => 1
  // bit   6 - master mode => 0
  // bit   5 - LRCLK polarity => 0
  // bit   4 - BCLK invert => 0
  // bit 3:2 - data word length => 10 (24b) or 00 (16b)
  // bit 1:0 - format select: 11 (dsp), 10 (i2s), 01 (LJ), 00 (RJ)
  res += SPDIF_I2C_Write(28, B10001010);

  // set PLL K and N factors
  // this should be sample rate dependent, but makes hardly any difference
  res += SPDIF_I2C_Write(6, 8);   // set PLL_N to 8
  res += SPDIF_I2C_Write(5, 0x0C); // set PLL_K to 0C49BA (0C)
  res += SPDIF_I2C_Write(4, 0x49); // set PLL_K to 0C49BA (49)
  res += SPDIF_I2C_Write(3, 0xBA); // set PLL_K to 0C49BA (BA)

  // set the power up
  // bit 7:6 - always 0
  // bit   5 - tri-state all outputs => 0
  // bit   4 - digital audio interface => 0
  // bit   3 - oscillator power down => 0
  // bit   2 - S/PDIF transmitter powerdown => 0
  // bit   1 - S/PDIF receiver powerdown => 1
  // bit   0 - PLL powerdown => 0
  res += SPDIF_I2C_Write(30, B00000010);

  if (res)
    return 3;

  return 0;
}

uint8_t SPDIF_I2C_Read(uint8_t reg, uint8_t *pBuf)
{
  uint8_t res = 0;

  /* Send START condition */
  I2C_BUS_SendStart(&I2C_Bus1);

  /* Send slave device address */
  if(I2C_BUS_SendByte(&I2C_Bus1,I2C_WR(I2C_ADDRESS)) == I2C_NOACK)
  {
    res = 1;
    goto END;
  }

  /* Send the slave device reg address to read */
  if (I2C_BUS_SendByte(&I2C_Bus1, reg & 0x7F) == I2C_NOACK)
  {
    res = 2;
    goto END;
  }

  /* Repeated start signal */
  I2C_BUS_SendRestart(&I2C_Bus1);

  /* Send slave device address */
  if (I2C_BUS_SendByte(&I2C_Bus1, I2C_RD(I2C_ADDRESS)) == I2C_NOACK)
  {
    res = 3;
    goto END;
  }

  /* Read reg data */
  *pBuf = I2C_BUS_ReceiveByte(&I2C_Bus1, I2C_NOACK);

END:
  /* Send STOP Condition */
  I2C_BUS_SendStop(&I2C_Bus1);
  return res;
}

uint8_t SPDIF_I2C_Write(uint8_t reg, uint8_t data)
{
  uint8_t res = 0;

  /* Send START condition */
  I2C_BUS_SendStart(&I2C_Bus1);

  /* Send slave device address */
  if (I2C_BUS_SendByte(&I2C_Bus1, I2C_WR(I2C_ADDRESS)) == I2C_NOACK)
  {
    res = 1;
    goto END;
  }

  /* Send the slave device reg address to write */
  if (I2C_BUS_SendByte(&I2C_Bus1, reg & 0x7F) == I2C_NOACK)
  {
    res = 2;
    goto END;
  }

  /* Send data byte to write */
  if (I2C_BUS_SendByte(&I2C_Bus1, data) == I2C_NOACK)
  {
    res = 3;
    goto END;
  }

END:
  /* Send STOP Condition */
  I2C_BUS_SendStop(&I2C_Bus1);
  return res;
}
