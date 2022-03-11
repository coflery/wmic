#include "delay.h"
#include "tx.h"

#define I2C_ADDRESS 0x25
#define TX_Chip_ID 0x9531
#define RETRY_COUNT 80

uint8_t analog_reg_val[12][4]; //Analog register values,
uint8_t reg_val[4];            //Temporatory register values
uint8_t bakup_reg_val[4];

BK bk9531;

const uint8_t tx_reg_val[35][4] =
    {
        {0x1E, 0x44, 0x0C, 0x88}, //REG0
        {0x04, 0xCF, 0x00, 0x57}, //REG1
        {0x89, 0x90, 0xE0, 0x2F}, //REG2
        {0x34, 0x12, 0x06, 0xFF}, //REG3
        {0x51, 0x88, 0x00, 0x44}, //REG4
        {0x00, 0x28, 0x03, 0x80}, //REG5
        {0x5B, 0xED, 0xFB, 0x00}, //REG6 ,update REG6[16:13]=0x6-->0xF,140425;
        {0x1E, 0x40, 0x00, 0x00}, //REG7
        {0x00, 0x08, 0x01, 0x00}, //REG8
        {0x00, 0x00, 0x7E, 0xFF}, //REG9
        {0x0F, 0x3A, 0x40, 0x40}, //REGA ,update U-band tx power for 10dBm,160104;
        {0x00, 0x06, 0xC3, 0xFF}, //REGB ,update REGB[19:16]=0-->6,140414;
        {0x00, 0x00, 0x00, 0x08}, //REGC
        {0x3A, 0x98, 0x00, 0x00}, //REGD
        {0x28, 0x28, 0x28, 0x28}, //REG30
        {0xD1, 0x00, 0x00, 0x28}, //REG31
        {0x10, 0x06, 0x00, 0x64}, //REG32
        {0x48, 0x80, 0x8D, 0x82}, //REG33
        {0x0F, 0x02, 0x11, 0x08}, //REG34
        {0x70, 0x50, 0x00, 0xC0}, //REG35
        {0x0F, 0x80, 0x1E, 0x04}, //REG36
        {0x00, 0x00, 0x00, 0x00}, //REG37
        {0x00, 0x00, 0x00, 0x00}, //REG38
        {0x03, 0xD7, 0xD5, 0xF7}, //REG39
        {0xC0, 0x25, 0x00, 0x74}, //REG3A
        {0x95, 0x25, 0x00, 0x3A}, //REG3B
        {0x95, 0x25, 0x00, 0x3B}, //REG3C
        {0x95, 0x25, 0x00, 0x3C}, //REG3D
        {0x00, 0xF8, 0x67, 0xC3}, //REG3E
        {0x80, 0x0F, 0x00, 0x00}, //REG3F
        {0x00, 0x00, 0x95, 0x31}, //REG70
        {0x18, 0xA4, 0x08, 0x10}, //REG71
        {0x00, 0x00, 0x00, 0x00}, //REG72
        {0x00, 0x07, 0x00, 0x51}, //REG77
        {0x00, 0x00, 0x00, 0x08}, //REG78
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
  uint8_t addr;

  //选择总线
  if (TX_Set_I2C_Bus(1))
  {
    return 1;
  }
  bk9531.bus_busy = true;

  //check if chip is online
  for (uint8_t i = 0; i < RETRY_COUNT; i++)
  {
    //read chipID
    if (TX_I2C_Read(0x70, reg_val))
      continue; //busy:chip reading i2c eeprom

    uint16_t chipID = reg_val[2] << 8 | reg_val[3];
    if (chipID == TX_Chip_ID)
      break;
    if (i + 1 == RETRY_COUNT)
      return 3;
    Delay_ms(50);
  }

  for (uint8_t i = 0; i < 35; i++)
  {
    if (i <= 0x0D)
      addr = i;
    else if (i <= 0x1D)
      addr = 34 + i;
    else if (i <= 0x20)
      addr = 82 + i;
    else if (i <= 0x22)
      addr = 86 + i;
    if (TX_I2C_Write(addr, (uint8_t *)tx_reg_val[i]))
    {
      return 4;
    }
  }

  TX_Set_Band_And_Frequency(freq);
  TX_Reset_Chip();
  TX_Trigger();

  bk9531.bus_busy = false;
  return 0;
}

void TX_RF_UnLock_Check()
{
  TX_I2C_Read(0x31, reg_val);
  if (reg_val[3] & 0x01) //TX unlock
  {
    TX_Trigger();
  }
}

/**
 * @brief 设置发射的频率(160-270MHz 500-980MHz)
 * @param freq 要设置的频率(单位:kHz)
 * @return uint8_t 0:成功 其他:频率超过范围
 */
uint8_t TX_Set_Band_And_Frequency(uint32_t freq)
{
  RF select;
  uint8_t R03_21_20;
  uint8_t R03_15_13;
  uint32_t R0D;

  if (freq >= 160000 && freq <= 178000)
    select = V160_178;
  else if (freq > 178000 && freq <= 270000)
    select = V178_270;
  else if (freq >= 500000 && freq <= 710000)
    select = U500_710;
  else if (freq > 710000 && freq <= 980000)
    select = U710_980;
  else
    return 1;

  //R03 R0D
  if (select == V160_178)
  {
    R03_21_20 = 0b10;
    R03_15_13 = 0b010;
    R0D = (uint32_t)((uint64_t)freq * 24 * 0x800000 / 24576);
  }
  else if (select == V178_270)
  {
    R03_21_20 = 0b10;
    R03_15_13 = 0b001;
    R0D = (uint32_t)((uint64_t)freq * 16 * 0x800000 / 24576);
  }
  else if (select == U500_710)
  {
    R03_21_20 = 0b01;
    R03_15_13 = 0b001;
    R0D = (uint32_t)((uint64_t)freq * 6 * 0x800000 / 24576);
  }
  else if (select == U710_980)
  {
    R03_21_20 = 0b01;
    R03_15_13 = 0b000;
    R0D = (uint32_t)((uint64_t)freq * 4 * 0x800000 / 24576);
  }

  analog_reg_val[3][1] &= ~0x30; //REG3[21:20]=0b00
  analog_reg_val[3][2] &= ~0xE0; //REG3[15:13]=0b000
  analog_reg_val[3][1] |= R03_21_20 << 4;
  analog_reg_val[3][2] |= R03_15_13 << 5;
  TX_I2C_Write(0x03, analog_reg_val[3]);

  reg_val[0] = R0D >> 24;
  reg_val[1] = R0D >> 16;
  reg_val[2] = R0D >> 8;
  reg_val[3] = R0D >> 0;
  TX_I2C_Write(0x0D, reg_val);
  return 0;
}

void TX_Reset_Chip()
{
  TX_I2C_Read(0x3F, reg_val);
  reg_val[1] &= ~0x08; //REG3F[19]=0
  TX_I2C_Write(0x3F, reg_val);
  reg_val[1] |= 0x08; //REG3F[19]=1
  TX_I2C_Write(0x3F, reg_val);
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
  TX_I2C_Write(0x0A, analog_reg_val[10]);

  //Set ddf_en=0 for tx_trigger
  TX_I2C_Read(0x35, reg_val);
  reg_val[3] &= ~0x80; //REG35[7]=0
  TX_I2C_Write(0x35, reg_val);

  //Set tx_en=0 for tx_trigger
  TX_I2C_Read(0x39, reg_val);
  reg_val[0] &= ~0x01; //REG39[24]=0
  TX_I2C_Write(0x39, reg_val);

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
  TX_I2C_Read(0x35, reg_val);
  reg_val[3] |= 0x80; //REG35[7]=1
  TX_I2C_Write(0x35, reg_val);

  //Set tx_en=1 for normal mode
  TX_I2C_Read(0x39, reg_val);
  reg_val[0] |= 0x01; //REG39[24]=1
  TX_I2C_Write(0x39, reg_val);

  //Recall default output power
  TX_I2C_Write(0x0A, bakup_reg_val);
}

void TX_Write_ID(uint8_t id_dat)
{
  reg_val[0] = 0x00;
  reg_val[1] = 0x00;
  reg_val[2] = 0x00;
  reg_val[3] = id_dat;
  TX_I2C_Write(0x38, reg_val);
}

void TX_Write_UserData(uint8_t dat)
{
  reg_val[0] = 0xC0;
  reg_val[1] = 0x25;
  reg_val[2] = 0x00;
  reg_val[3] = dat;
  TX_I2C_Write(0x3A, reg_val);
}
