#include <stdbool.h>
#include "stm32f0xx.h"
#include "delay.h"
#include "rx.h"
#include "i2c.h"
#include "gpio.h"

#define I2C_PORT I2C_PORT1
#define I2C_ADDRESS 0x22
#define RX_ChipID I2C_ADDRESS
#define TIMEOUT ((uint8_t)20)

//#define RX_FIFO_INT 0x19
#define RX_INTS_TRANS_PHASE_LOCK 0x04
#define RX_INTS_SYM_INT1 0x02
#define RX_NEED_PHASE_UNLOCK 10
#define RX_PHASELOCK_BIG_DELTAF 0
#define RX_INTM_TRANS_PHASE_LOCK 0x04

#define RX_GPIO4_Low 0x00
#define RX_GPIO4_High 0x02
#define RX_GPIO4_PhaseLock 0x40

uint8_t analog_reg_val[12][4]; //Analog register values,
uint8_t reg_val[4];            //Temporatory register values
uint8_t bakup_reg_val[4];

uint8_t CtuneBuf[4];
uint32_t deltaf_last;
uint8_t AfcCnt;

bool RX1_IRQ = false;
bool RX2_IRQ = false;

static uint8_t rx_reg_val[53][4] =
    {
        {0xDF, 0xFF, 0xFF, 0xF8}, //REG0
        {0x04, 0xd2, 0x80, 0x57}, //REG1
        {0x89, 0x90, 0xE0, 0x28}, //REG2
        {0x04, 0x22, 0x26, 0x9F}, //REG3		0x94,0x22,0x46,0x9F, for test mode
        {0x50, 0x88, 0x00, 0x44}, //REG4
        {0x00, 0x28, 0x03, 0x80}, //REG5
        {0x5B, 0xED, 0xFB, 0x00}, //REG6		update REG6[16:13]=0x6-->0xF,140425
        {0x1C, 0xA4, 0xC5, 0xAA}, //REG7		update REG7[8]=0-->1,140414
        {0xEF, 0xF1, 0x19, 0x4C}, //REG8
        {0x08, 0x51, 0x13, 0xA2}, //REG9		update REG9[7]=0-->1,140414
        {0x00, 0x6F, 0x00, 0x6F}, //REGA
        {0x1B, 0xD2, 0x58, 0x63}, //REGB
        {0xE4, 0x16, 0x95, 0x22}, //REG10		Software Ver No:2014/04/16;  chip id:9522
        {0x00, 0x00, 0x00, 0x00}, //REG11
        {0x00, 0x08, 0x00, 0xBD}, //REG16
        {0x00, 0x00, 0x00, 0x00}, //REG17
        {0x00, 0x00, 0x00, 0x00}, //REG18
        {0x00, 0x44, 0x6D, 0x60}, //REG19
        {0x00, 0x00, 0x32, 0xC0}, //REG1A		updated,140416
        {0x54, 0xb0, 0x51, 0xeb}, //REG1B
        {0x00, 0x17, 0xFC, 0xFA}, //REG1C		接收增益
        {0x00, 0x90, 0x00, 0x80}, //REG1D		default 0x4E,0x00,0x4D,0x00, GPIO3 指示
        {0x00, 0x00, 0x00, 0x00}, //REG1E
        {0x00, 0x00, 0x00, 0x00}, //REG1F
        {0x00, 0x00, 0x00, 0x00}, //REG20
        {0x00, 0x00, 0x00, 0x00}, //REG21
        {0x00, 0x00, 0x00, 0x00}, //REG22
        {0x00, 0x00, 0x00, 0x00}, //REG23
        {0x00, 0x00, 0x00, 0x00}, //REG24
        {0x00, 0x00, 0x00, 0x00}, //REG25
        {0x00, 0x00, 0x00, 0x00}, //REG26
        {0x00, 0x00, 0x00, 0x00}, //REG27
        {0x00, 0x00, 0x00, 0x00}, //REG28
        {0x00, 0x00, 0x00, 0x00}, //REG29
        {0x00, 0x00, 0x00, 0x00}, //REG2A
        {0x00, 0x00, 0x00, 0x00}, //REG2B
        {0x00, 0x00, 0x00, 0x00}, //REG2C
        {0x00, 0x00, 0x00, 0x00}, //REG2D
        {0x00, 0x00, 0x00, 0x00}, //REG2E
        {0x00, 0x00, 0x00, 0x00}, //REG2F
        {0x00, 0x00, 0x00, 0x00}, //REG30
        {0x00, 0x00, 0x00, 0x00}, //REG31
        {0x12, 0x82, 0x3D, 0x38}, //REG32,		auto-select ANT-Pin //0x22823D38 for  ANT@Pin5 //0x02823D38 for ANT@Pin6
        {0x00, 0xD7, 0xD5, 0xF7}, //REG33
        {0x00, 0x00, 0x00, 0x00}, //REG34
        {0x00, 0x00, 0x00, 0x00}, //REG35
        {0x01, 0x46, 0x02, 0x00}, //REG36,		updated,150708 CRC ERROR VERIFY
        {0x90, 0xF6, 0xAA, 0x26}, //REG37,		updated,140912  AGC Threshold
        {0x1F, 0x80, 0x1E, 0x07}, //REG38
        {0x40, 0x40, 0x40, 0x40}, //REG39
        {0x00, 0x00, 0x00, 0x40}, //REG3A
        {0x00, 0x78, 0x64, 0x00}, //REG3B
        {0x00, 0x00, 0x00, 0x00}, //REG3C
};

uint8_t init_rx(void)
{
  uint8_t i;
  uint8_t addr;

  for (i = 0; i < TIMEOUT; i++)
  {
    delay_ms(20);
    RX_I2C_Read(0x10, reg_val);
    if (reg_val[3] != RX_ChipID)
      continue;
    break;
  }

  if (i == TIMEOUT) //通信超时
    return 1;
  if (reg_val[3] != RX_ChipID) //ChipID不匹配
    return 2;

  for (i = 0; i <= 52; i++)
  {
    if (i <= 0x0b)
      addr = i;
    else if (i <= 0x0d)
      addr = 4 + i;
    else if (i <= 0x34)
      addr = 8 + i;
    RX_I2C_Write(addr, &rx_reg_val[i][0]); //tx_reg_val=tx_48k_vband values
  }

  delay_ms(20);
  RX_Set_Band_And_Frequency(805000);
  RX_Reset_Chip();
  RX_Trigger();

  //RX_GPIO4_Set(RX_GPIO4_PhaseLock);
  return 0;
}

void RX_RF_UnLock_Check(void)
{
  RX_I2C_Read(0x15, reg_val);
  if (reg_val[3] & 0x03) //RX unlock
  {
    RX_Trigger();
  }
}

/*
*    函 数 名: RX_Set_Band_And_Frequency
*    功能说明: 设置接收的频率
*    形    参: freq：要设置的频率(单位:kHz)
*    返 回 值: 0:成功 其他:频率超过范围
*/
uint8_t RX_Set_Band_And_Frequency(uint64_t freq)
{
  //Only for UHF
  if (freq < 500000 || freq > 980000)
    return 1;

  analog_reg_val[3][1] &= ~0x20; //REG3[21]=0
  analog_reg_val[3][1] |= 0x10;  //REG3[20]=1
  analog_reg_val[3][2] &= ~0xE0; //REG3[15:13]=0
  if (freq < 750800)
    analog_reg_val[3][2] |= 0x20; //REG3[15:13]=1	// frequency < 750.8M reserved the line,else comment it.
  RX_I2C_Write(0x03, analog_reg_val[3]);

  analog_reg_val[7][1] |= 0x0E; //REG7[19:17]=7
  RX_I2C_Write(0x07, analog_reg_val[7]);

  analog_reg_val[8][1] &= ~0x0C; //REG8[19:18]=0
  analog_reg_val[8][1] |= 0x30;  //REG8[21:20]=3
  RX_I2C_Write(0x08, analog_reg_val[8]);

  uint32_t value;
  if (freq >= 160000 && freq <= 178000)
    value = (uint32_t)((freq + 163.84) * 24 * 0x800000 / 24576);
  else if (freq > 178000 && freq <= 270000)
    value = (uint32_t)((freq + 163.84) * 16 * 0x800000 / 24576);
  else if (freq >= 500000 && freq <= 710000)
    value = (uint32_t)((freq + 163.84) * 6 * 0x800000 / 24576);
  else if (freq > 710000 && freq <= 980000)
    value = (uint32_t)((freq + 163.84) * 4 * 0x800000 / 24576);
  else
    return 2;

  reg_val[0] = (value >> 24) & 0xff;
  reg_val[1] = (value >> 16) & 0xff;
  reg_val[2] = (value >> 8) & 0xff;
  reg_val[3] = (value >> 0) & 0xff;

  //RX freq set 700MHz
  // reg_val[0] = 0x55;
  // reg_val[1] = 0x78;
  // reg_val[2] = 0x1E;
  // reg_val[3] = 0xB8;
  RX_I2C_Write(0x1B, reg_val);
  return 0;
}

void RX_Reset_Chip(void)
{
  RX_I2C_Read(0x1a, reg_val);
  reg_val[3] &= ~0x80; //REG1A[7]=0
  RX_I2C_Write(0x1a, reg_val);
  reg_val[3] |= 0x80; //REG1A[7]=1
  RX_I2C_Write(0x1a, reg_val);
}

//RX mode,trigger chip to calibrate
void RX_Trigger(void)
{
  //Enable calibration clock
  analog_reg_val[7][0] |= 0x02; //REG7[25]=1
  RX_I2C_Write(0x07, analog_reg_val[7]);

  //Calibrate RF VCO
  analog_reg_val[3][1] &= ~0x40; //REG3[22]=0
  RX_I2C_Write(0x03, analog_reg_val[3]);
  analog_reg_val[3][1] |= 0x40; //REG3[22]=1
  RX_I2C_Write(0x03, analog_reg_val[3]);
  delay_ms(5); //At least delay 2ms

  //Calibrate Digital VCO
  analog_reg_val[4][0] &= ~0x02; //REG4[25]=0
  RX_I2C_Write(0x04, analog_reg_val[4]);
  analog_reg_val[4][0] |= 0x02; //REG4[25]=1
  RX_I2C_Write(0x04, analog_reg_val[4]);

  //Disable calibration clock
  analog_reg_val[7][0] &= ~0x02; //REG7[25]=0
  RX_I2C_Write(0x07, analog_reg_val[7]);
}

void RX_Force_PhaseUnlock(void)
{
  RX_I2C_Read(0x32, reg_val);
  reg_val[3] &= ~0x08; //cfg_whiten_en = 0
  reg_val[2] &= 0x0f;  //sync_unlock_win = 0,for fast unlock
  RX_I2C_Write(0x32, reg_val);

  {
    delay_ms(2); //请确保该处延时大于或等于2ms
    RX_I2C_Read(0x32, reg_val);
    reg_val[3] &= ~0x10; //rx_en = 0
    reg_val[2] |= 0x30;  //sync_unlock_win = 3
    RX_I2C_Write(0x32, reg_val);
    reg_val[3] |= 0x18; //rx_en = 1,cfg_whiten_en=1
    RX_I2C_Write(0x32, reg_val);
    delay_ms(2); //请确保该处延时大于或等于2ms
  }
  //RX_PreventPop();
}

void RX_PreventPop(void)
{
  uint8_t FIFO_Num;

  RX_I2C_Read(0x16, reg_val);
  FIFO_Num = reg_val[1] & 0x3f; //read REG16[21:16]=audio FIFO count
  if (FIFO_Num >= 28)
  {
    RX_Force_PhaseUnlock(); //force Lock to UnLock, reset RX state
  }
}

void RX_Cali_FreqOffset(void)
{
  uint8_t fifo_num, ctune;
  uint32_t temp_val;
  int32_t deltaf, x_val;

  RX_I2C_Read(0x16, reg_val);
  RX_I2C_Read(0x1a, CtuneBuf);

  fifo_num = reg_val[1] & 0x3f; //read REG16[21:16]=audio FIFO count
  temp_val = reg_val[2];
  temp_val = temp_val << 8;
  temp_val += reg_val[3];
  temp_val = (temp_val + 16384) & 0x7fff; //remove bit15
  deltaf = temp_val - 16384;
  deltaf = deltaf / 128; //read REG16[14:0]=delta freq,unit is ppm

  if (((deltaf - deltaf_last)) >= 2 || ((deltaf - deltaf_last) <= -2))
  {
    deltaf_last = deltaf;
    deltaf = 0;
  }
  else
  {
    deltaf_last = deltaf;
  }

  if ((deltaf >= 8) || (deltaf <= -8))
  {
    x_val = deltaf * 3 / 2;
    AfcCnt = RX_NEED_PHASE_UNLOCK; //the first time enter fine tuning,force unlock
  }
  else
  {
    if (fifo_num >= 28)
    {
      x_val = 0;
      AfcCnt = RX_NEED_PHASE_UNLOCK; //full FIFO,force unlock
    }
    else if (AfcCnt == 4)
    {
      if (fifo_num > 20)
        x_val = 1;
      else if (fifo_num < 12 && deltaf <= 0)
        x_val = -1; //fine tuning,at most tuning 1ppm
      else
        x_val = 0;
      AfcCnt = 1; //default AfcCnt=1
    }
    else
    {
      x_val = 0;
      AfcCnt++;
    }
  }

  ctune = CtuneBuf[3] & 0x7f;
  x_val = ctune - x_val;

  if (x_val > 127)
    x_val = 0x7f;
  else if (x_val < 0)
    x_val = 0x0;

  CtuneBuf[3] = (CtuneBuf[3] & 0x80) | (x_val & 0x7f); //update crystal capacitors
}

void RX_Frequency_Tracking(void)
{
  RX_I2C_Read(0x19, reg_val);
  reg_val[3] = RX_INTS_SYM_INT1; //clear the symbot interrupt INT1
  RX_I2C_Write(0x19, reg_val);

  delay_ms(2); //请确保该处延时大于或等于2ms
  if (RX1_IRQ)
  {
    RX_Cali_FreqOffset();
    RX_I2C_Write(0x1a, CtuneBuf); //Update_Ctune
    RX1_IRQ = false;
  }
  else if (RX2_IRQ)
  {
    RX_Cali_FreqOffset();
    RX_I2C_Write(0x1a, CtuneBuf); //Update_Ctune
    RX2_IRQ = false;
  }
  else
  {
    AfcCnt = 1;
  }

  if (AfcCnt == RX_NEED_PHASE_UNLOCK)
  {
    AfcCnt = 1;
    RX_Force_PhaseUnlock();
  }
}

void RX_GPIO4_Set(uint8_t mode)
{
  switch (mode)
  {
  case RX_GPIO4_Low:
  case RX_GPIO4_High:
  case RX_GPIO4_PhaseLock:
    RX_I2C_Read(0x3A, reg_val);
    reg_val[3] = mode;
    RX_I2C_Write(0x3A, reg_val);
  }
}

void RX_Audio_Output_Set(bool Enable)
{
  RX_I2C_Read(0x1c, reg_val);
  if (Enable)
    reg_val[1] |= 0x10; //Reg0x1c [20]=1;audio_out_en=1;
  else
    reg_val[1] &= ~0x10; //Reg0x1c [20]=1;audio_out_en=1;
  RX_I2C_Write(0x1c, reg_val);
}

bool RX_Check_PhaseLock(void)
{
  uint8_t lock;

  RX_I2C_Read(0x19, reg_val);
  lock = reg_val[3] & 0x20;
  return (lock > 0);
}

uint8_t RX_Read_RSSI(void)
{
  RX_I2C_Read(0x19, reg_val);
  return (reg_val[1]);
}

void RX_Write_ID(uint8_t id_dat)
{
  reg_val[0] = 0x00;
  reg_val[1] = 0x00;
  reg_val[2] = 0x00;
  reg_val[3] = id_dat;
  RX_I2C_Write(0x34, reg_val);
}

uint8_t RX_Read_UserData(void)
{
  RX_I2C_Read(0x19, reg_val); //read specific data
  return (reg_val[2]);
}

void RX_Set_SampleRate_28kHz(void)
{
  analog_reg_val[9][3] |= 0x04; //REG9[2]=1
  RX_I2C_Write(0x09, analog_reg_val[9]);

  RX_I2C_Read(0x1A, reg_val);
  reg_val[2] |= 0x01; //REG1A[8]=1
  RX_I2C_Write(0x1A, reg_val);

  RX_I2C_Read(0x32, reg_val);
  reg_val[3] |= 0x02; //REG32[1]=1
  RX_I2C_Write(0x32, reg_val);
}

uint8_t RX_I2C_Write(uint8_t reg, uint8_t *pBuf)
{
  uint8_t res = 0;
  if (reg <= 0x0b) //Analog registers saved by analog_reg_val[][] for read operation
  {
    analog_reg_val[reg][0] = pBuf[0];
    analog_reg_val[reg][1] = pBuf[1];
    analog_reg_val[reg][2] = pBuf[2];
    analog_reg_val[reg][3] = pBuf[3];
  }

  /* Send START condition */
  I2C_Start(I2C_PORT);

  /* Send slave device address */
  I2C_WriteByte(I2C_PORT, I2C_ADDRESS);
  if (I2C_CheckAck(I2C_PORT) == NACK)
  {
    res = 1;
    goto END;
  }

  /* Send the slave device reg address to write */
  I2C_WriteByte(I2C_PORT, reg << 1);
  if (I2C_CheckAck(I2C_PORT) == NACK)
  {
    res = 2;
    goto END;
  }

  /* Send data bytes to write */
  for (uint8_t i = 0; i < 4; i++)
  {
    I2C_WriteByte(I2C_PORT, pBuf[i]);
    if (I2C_CheckAck(I2C_PORT) == NACK)
    {
      res = 3;
      goto END;
    }
  }

END:
  /* Send STOP Condition */
  I2C_Stop(I2C_PORT);
  return res;
}

uint8_t RX_I2C_Read(uint8_t reg, uint8_t *pBuf)
{
  uint8_t res = 0;
  if (reg <= 0x0b) //Analog registers not read directly from chip,recalled from analog_reg_val[][]
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
    I2C_Start(I2C_PORT);

    /* Send slave device address */
    I2C_WriteByte(I2C_PORT, I2C_ADDRESS);
    if (I2C_CheckAck(I2C_PORT) == NACK)
    {
      res = 1;
      goto END;
    }

    /* Send the slave device reg address to read */
    I2C_WriteByte(I2C_PORT, (reg << 1) | 0x01);
    if (I2C_CheckAck(I2C_PORT) == NACK)
    {
      res = 2;
      goto END;
    }

    /* Read regs data */
    for (uint8_t i = 0; i < 4; i++)
    {
      pBuf[i] = I2C_ReadByte(I2C_PORT);
      if (i + 1 < 4)
        /* Acknowledgement */
        I2C_Ack(I2C_PORT);
      else
        /* Disable Acknowledgement */
        I2C_NAck(I2C_PORT);
    }
  }
END:
  /* Send STOP Condition */
  I2C_Stop(I2C_PORT);
  return res;
}