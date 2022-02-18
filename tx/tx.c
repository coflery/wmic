#include "delay.h"
#include "tx.h"

#define I2C_ADDRESS 0x21
#define TX_Chip_ID 0x21
#define INIT_FAIL_TIME 20

uint8_t analog_reg_val[12][4]; //Analog register values,
uint8_t reg_val[4];            //Temporatory register values
uint8_t bakup_reg_val[4];

BK bk9531;

uint8_t tx_reg_val[48][4] =
{
        {0x1C, 0x44, 0x0C, 0x88}, //REG0
        {0x04, 0xCF, 0x00, 0x57}, //REG1
        {0x89, 0x90, 0xE0, 0x2F}, //REG2
        {0xB4, 0x22, 0x46, 0xFF}, //REG3 	,update REG9[28],[31]=0-->1,140414;
        {0x51, 0x88, 0x00, 0x44}, //REG4
        {0x00, 0x28, 0x03, 0x80}, //REG5
        {0x5B, 0xED, 0xFB, 0x00}, //REG6
        {0x1C, 0x40, 0x00, 0x00}, //REG7
        {0x00, 0x08, 0x01, 0x00}, //REG8	,updated,140416
        {0x00, 0x00, 0x00, 0x03}, //REG9
        {0x05, 0x8C, 0x30, 0x30}, //REGA	,update REG9[27:25]=0-->2,140414; TX power
        {0x00, 0x06, 0xC3, 0xFF}, //REGB
        {0xE4, 0x16, 0x95, 0x21}, //REG10
        {0x34, 0xB0, 0x02, 0x91}, //REG11
        {0x00, 0x00, 0x00, 0x40}, //REG12
        {0x00, 0x00, 0x00, 0x00}, //REG13
        {0x00, 0x00, 0x00, 0x00}, //REG19
        {0x00, 0x00, 0x42, 0xC0}, //REG1A
        {0x53, 0x02, 0x00, 0x00}, //REG1B
        {0x04, 0x02, 0x03, 0xFF}, //REG1C
        {0x58, 0x02, 0x6C, 0x02}, //REG1D
        {0x0f, 0x0b, 0x32, 0x1b}, //REG1E
        {0x00, 0x00, 0x00, 0x80}, //REG1F
        {0x00, 0x00, 0x93, 0xA0}, //REG20
        {0x00, 0xD7, 0xD5, 0xF7}, //REG21
        {0x00, 0x00, 0x00, 0x00}, //REG22
        {0x7F, 0x7F, 0x00, 0xA0}, //REG23
        {0x00, 0x00, 0x70, 0x50}, //REG25
        {0x0F, 0x80, 0x1E, 0x04}, //REG26
        {0x40, 0x40, 0x40, 0x40}, //REG27
        {0x00, 0x00, 0x00, 0x40}, //REG28
        {0x03, 0xF0, 0x64, 0x00}, //REG29
        {0x01, 0x00, 0x00, 0x00}, //REG30
        {0x07, 0x05, 0x04, 0x02}, //REG31
        {0x12, 0x0F, 0x0C, 0x0A}, //REG32
        {0x22, 0x1E, 0x1A, 0x16}, //REG33
        {0x35, 0x30, 0x2B, 0x26}, //REG34
        {0x4B, 0x45, 0x40, 0x3A}, //REG35
        {0x63, 0x5D, 0x57, 0x51}, //REG36
        {0x7C, 0x76, 0x70, 0x69}, //REG37
        {0x96, 0x8F, 0x89, 0x83}, //REG38
        {0xAE, 0xA8, 0xA2, 0x9C}, //REG39
        {0xC5, 0xBF, 0xBA, 0xB4}, //REG3A
        {0xD9, 0xD4, 0xCF, 0xCA}, //REG3B
        {0xE9, 0xE5, 0xE1, 0xDD}, //REG3C
        {0xF5, 0xF3, 0xF0, 0xED}, //REG3D
        {0xFD, 0xFB, 0xFA, 0xF8}, //REG3E
        {0xFF, 0xFF, 0xFF, 0xFE}, //REG3F
};
uint8_t TX_I2C_Read(uint8_t reg, uint8_t *pBuf)
{
  uint8_t res = 0;
  if (reg <= 0x0B) //Analog registers not read directly from chip,recalled from analog_reg_val[][]
  {
    pBuf[0] = analog_reg_val[reg][0];
    pBuf[1] = analog_reg_val[reg][1];
    pBuf[2] = analog_reg_val[reg][2];
    pBuf[3] = analog_reg_val[reg][3];
    return 0;
  }
  else
  {
    /* Send START condition */
    if (I2C_BUS_SendStart(&bk9531.device))
    {
      /* Bus bus busy*/
      res = 1;
    }

    /* Send slave device address */
    if (I2C_BUS_SendByte(&bk9531.device, I2C_ADDRESS) == I2C_NOACK)
    {
      res = 2;
      goto END;
    }

    /* Send the slave device reg address to read */
    if (I2C_BUS_SendByte(&bk9531.device, I2C_RD((reg << 1))) == I2C_NOACK)
    {
      res = 3;
      goto END;
    }

    uint8_t len = 4;
    /* Read regs data */
    for (uint8_t i = 0; i < len; i++)
    {
      *pBuf = I2C_BUS_ReceiveByte(&bk9531.device, (i + 1) == len ? I2C_NOACK : I2C_ISACK);
      pBuf++;
    }
  }
END:
  /* Send STOP Condition */
  I2C_BUS_SendStop(&bk9531.device);
  return res;
}

uint8_t TX_I2C_Write(uint8_t reg, uint8_t *pBuf)
{
  uint8_t res = 0;
  if (reg <= 0x0B) //Analog registers saved by analog_reg_val[][] for read operation
  {
    analog_reg_val[reg][0] = pBuf[0];
    analog_reg_val[reg][1] = pBuf[1];
    analog_reg_val[reg][2] = pBuf[2];
    analog_reg_val[reg][3] = pBuf[3];
  }

  /* Send START condition */
  if (I2C_BUS_SendStart(&bk9531.device))
  {
    /* Bus bus busy*/
    res = 1;
  }

  /* Send slave device address */
  if (I2C_BUS_SendByte(&bk9531.device, I2C_ADDRESS) == I2C_NOACK)
  {
    res = 2;
    goto END;
  }

  /* Send the slave device reg address to write */
  if (I2C_BUS_SendByte(&bk9531.device, I2C_WR(reg << 1)) == I2C_NOACK)
  {
    res = 3;
    goto END;
  }

  /* Send data bytes to write */
  for (uint8_t i = 0; i < 4; i++)
  {
    if (I2C_BUS_SendByte(&bk9531.device, *pBuf) == I2C_NOACK)
    {
      res = 4;
      goto END;
    }
    pBuf++;
  }

END:
  /* Send STOP Condition */
  I2C_BUS_SendStop(&bk9531.device);
  return res;
}

/**
 * @brief 根据设备号选择所在的I2C总线
 *
 * @param index 设备号
 * @return true 失败
 * @return false 成功
 */
bool TX_Set_I2C_Bus(uint8_t index)
{
  switch (index)
  {
  case 1:
    bk9531.device = I2C_Bus1;
    break;
  default:
    return true;
  }
  return false;
}

uint8_t TX_Init(uint32_t freq)
{
  uint8_t i;
  uint8_t addr;

  //选择总线
  if (TX_Set_I2C_Bus(1))
  {
    return 1;
  }
  bk9531.bus_busy = true;

  for (i = 0; i <= INIT_FAIL_TIME; i++)
  {
    Delay_ms(20);
    TX_I2C_Read(0x10, reg_val);
    if (reg_val[3] != TX_Chip_ID)
      continue;
    break;
  }

  if (reg_val[3] != TX_Chip_ID)
    return 1;

  for (i = 1; i <= 48; i++)
  {
    if (i <= 12)
      addr = i - 1;
    else if (i <= 16)
      addr = 3 + i;
    else if (i <= 27)
      addr = 8 + i;
    else if (i <= 32)
      addr = 9 + i;
    else if (i <= 48)
      addr = 15 + i;
    TX_I2C_Write(addr, &tx_reg_val[i - 1][0]); //tx_reg_val=tx_48k_vband values
  }

  TX_Set_Band_And_Frequency(freq);
  TX_Reset_Chip();
  TX_Trigger();

  bk9531.bus_busy = false;
  return 0;
}

void TX_RF_UnLock_Check()
{
  TX_I2C_Read(0x19, reg_val);
  if (reg_val[3] & 0x0c) //TX unlock
  {
    TX_Trigger();
  }
}

/*
*    函 数 名: TX_Set_Band_And_Frequency
*    功能说明: 设置发射的频率
*    形    参: freq：要设置的频率(单位:kHz)
*    返 回 值: 0:成功 其他:频率超过范围
*/
uint8_t TX_Set_Band_And_Frequency(uint64_t freq)
{
  //Only for UHF
  if(freq < 500000 || freq > 980000)
    return 1;

  analog_reg_val[3][1] &= ~0x20; //REG3[21]=0
  analog_reg_val[3][1] |= 0x10;  //REG3[20]=1
  analog_reg_val[3][2] &= ~0xE0; //REG3[15:13]=0
  if(freq < 750800)
    analog_reg_val[3][2] |= 0x20;  //REG3[15:13]=1	// frequency < 750.8M reserved the line,else comment it.
  TX_I2C_Write(0x03, analog_reg_val[3]);

  uint32_t value;
  if (freq >= 160000 && freq <= 178000)
    value = (uint32_t)(freq * 24 * 0x800000 / 24576);
  else if (freq > 178000 && freq <= 270000)
    value = (uint32_t)(freq * 16 * 0x800000 / 24576);
  else if (freq >= 500000 && freq <= 710000)
    value = (uint32_t)(freq * 6 * 0x800000 / 24576);
  else if (freq > 710000 && freq <= 980000)
    value = (uint32_t)(freq * 4 * 0x800000 / 24576);
  else
    return 2;

  reg_val[0] = (value >> 24) & 0xff;
  reg_val[1] = (value >> 16) & 0xff;
  reg_val[2] = (value >> 8) & 0xff;
  reg_val[3] = (value >> 0) & 0xff;

  //TX freq set 700MHz
  // reg_val[0] = 0x55;
  // reg_val[1] = 0x73;
  // reg_val[2] = 0x00;
  // reg_val[3] = 0x00;
  TX_I2C_Write(0x1B, reg_val);
  return 0;
}

void TX_Reset_Chip()
{
  TX_I2C_Read(0x1a, reg_val);
  reg_val[3] &= ~0x80; //REG1A[7]=0
  TX_I2C_Write(0x1a, reg_val);
  reg_val[3] |= 0x80; //REG1A[7]=1
  TX_I2C_Write(0x1a, reg_val);
}

//TX mode,trigger chip to calibrate
void TX_Trigger()
{
  //Save default output power to backup
  bakup_reg_val[0] = analog_reg_val[10][0];
  bakup_reg_val[1] = analog_reg_val[10][1];
  bakup_reg_val[2] = analog_reg_val[10][2];
  bakup_reg_val[3] = analog_reg_val[10][3];

  //Set output power=low for tx_trigger
  analog_reg_val[10][3] = 0x00; //REGA[7:0]=0
  TX_I2C_Write(0x0a, analog_reg_val[10]);

  //Set ddf_en=0 for tx_trigger
  TX_I2C_Read(0x1f, reg_val);
  reg_val[3] &= ~0x80; //REG1F[7]=0
  TX_I2C_Write(0x1f, reg_val);

  //Set tx_en=0 for tx_trigger
  TX_I2C_Read(0x20, reg_val);
  reg_val[3] &= ~0x80; //REG20[7]=0
  TX_I2C_Write(0x20, reg_val);

  //Enable calibration clock
  analog_reg_val[7][0] |= 0x02;  //REG7[25]=1
  analog_reg_val[7][0] &= ~0x10; //REG7[28]=0
  TX_I2C_Write(0x07, analog_reg_val[7]);

  //Calibrate RF VCO
  analog_reg_val[3][1] &= ~0x40; //REG3[22]=0
  TX_I2C_Write(0x03, analog_reg_val[3]);
  analog_reg_val[3][1] |= 0x40; //REG_0x3[22]=1
  TX_I2C_Write(0x03, analog_reg_val[3]);
  Delay_ms(50); //At least delay 30ms

  //Calibrate Digital VCO
  analog_reg_val[4][0] &= ~0x02; //REG4[25]=0
  TX_I2C_Write(0x04, analog_reg_val[4]);
  analog_reg_val[4][0] |= 0x02; //REG4[25]=1
  TX_I2C_Write(0x04, analog_reg_val[4]);

  //Disable calibration clock
  analog_reg_val[7][0] &= ~0x02; //REG7[25]=0
  analog_reg_val[7][0] |= 0x10;  //REG7[28]=1
  TX_I2C_Write(0x07, analog_reg_val[7]);

  //Set ddf_en=1 for normal mode
  TX_I2C_Read(0x1f, reg_val);
  reg_val[3] |= 0x80; //REG1F[7]=1
  TX_I2C_Write(0x1f, reg_val);

  //Set tx_en=1 for normal mode
  TX_I2C_Read(0x20, reg_val);
  reg_val[3] |= 0x80; //REG20[7]=1
  TX_I2C_Write(0x20, reg_val);

  //Recall default output power
  TX_I2C_Write(0x0a, bakup_reg_val);
}

void TX_Write_ID(uint8_t id_dat)
{
  reg_val[0] = 0x00;
  reg_val[1] = 0x00;
  reg_val[2] = 0x00;
  reg_val[3] = id_dat;
  TX_I2C_Write(0x22, reg_val);
}

void TX_Write_UserData(uint8_t dat)
{
  reg_val[0] = 0x00;
  reg_val[1] = 0x00;
  reg_val[2] = dat;
  reg_val[3] = 0xA0;
  TX_I2C_Write(0x20, reg_val);
}

void TX_Write_28kHz_SampleMode(void)
{
  analog_reg_val[0][0] = 0x10; //REG0=0x10000008
  analog_reg_val[0][1] = 0X00;
  analog_reg_val[0][2] = 0X00;
  analog_reg_val[0][3] = 0X08;
  TX_I2C_Write(0x00, analog_reg_val[0]);

  analog_reg_val[8][1] |= 0x04; //REG8[18]=1
  TX_I2C_Write(0x08, analog_reg_val[8]);

  TX_I2C_Read(0x1A, reg_val);
  reg_val[2] |= 0x01; //REG1A[8]=1
  TX_I2C_Write(0x1A, reg_val);

  TX_I2C_Read(0x20, reg_val);
  reg_val[3] |= 0x08; //REG20[3]=1
  TX_I2C_Write(0x20, reg_val);
}