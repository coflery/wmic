#include <stdbool.h>
#include "stm32f0xx.h"
#include "delay.h"
#include "rx.h"
#include "gpio.h"

#define I2C_ADDRESS 0x26
#define RX_ChipID   0x9532

#define RX_INT_SYMBOL        0x01
#define RX_INT_PLL_STATE     0x02
#define RX_INT_FRAME_ERR     0x04
#define RX_INT_RSSI_EN       0x08
#define RX_INT_SYMBOL_EN     0x10
#define RX_INT_PLL_STATE_EN  0x20
#define RX_INT_RSSI          0x80
#define RX_INT_FRAME_ERR_EN  0x01

#define RX_NEED_PHASE_UNLOCK 10

#define RX_GPIO4_Low         0x00
#define RX_GPIO4_High        0x02
#define RX_GPIO4_Function2   0x40

uint8_t analog_reg_val[12][4]; //Analog register values
uint8_t reg_val[4];            //Temporatory register values

uint8_t CtuneBuf[4];
uint32_t deltaf_last;
uint8_t AfcCnt;

BK bk9532;

const uint8_t rx_reg_val[38][4] =
    {
        {0xDF, 0xFF, 0xFF, 0xF8}, //REG0,|
        {0x04, 0xD2, 0x80, 0x57}, //REG1,| --->fixed
        {0x89, 0x90, 0xE0, 0x28}, //REG2,|
        {0x04, 0x12, 0x06, 0x9F}, //REG3, [21:20]=b10 for VHF,b01 for UHF;[15:13] for freq div select
        {0x50, 0x88, 0x00, 0x44}, //REG4,|
        {0x00, 0x28, 0x03, 0x80}, //REG5,| --->fixed
        {0x5B, 0xED, 0xFB, 0x00}, //REG6,|
        {0x1C, 0x2E, 0xC5, 0xAA}, //REG7, [23]=1 Enable XO2 output;[19:17]=b010 for VHF,b111 for UHF
        {0xEF, 0xF1, 0x19, 0x4C}, //REG8, 0xEDCD874C for VHF,0xEFF1194C for UHF
        {0x08, 0x51, 0x13, 0xA2}, //REG9, update REG9[7]=0-->1,140414
        {0x00, 0x6F, 0x00, 0x6F}, //REGA,|
        {0x1B, 0xD2, 0x58, 0x63}, //REGB,| --->fixed
        {0x00, 0x00, 0x00, 0x00}, //REG2C,|
        {0x00, 0x00, 0x00, 0xFF}, //REG2D,|
        {0xF1, 0x28, 0xA0, 0x00}, //REG2E,|--->reverb
        {0x00, 0x00, 0x2E, 0x91}, //REG2F,|
        {0x40, 0x40, 0x40, 0x40}, //REG30, GPIO3-O,GPIO2-O,GPIO1-I,GPIO0-I;0x40 for fun2 output,0x3C for fun3 pull-up input
        {0xD9, 0x02, 0x00, 0x40}, //REG31, [31:30]=b11 48kHz;[22:20]GPIO4-extbit;[19:17]GPIO3-RSSI;[16:14]GPIO2-PCM_DATA;[13:11]GPIO1-PCM_BCLK;[10:8]GPIO0-PCM_LRCK;[7:0]GPIO4-output
        {0x20, 0xFF, 0xEF, 0x07}, //REG32, [10]=1 auto-mute off
        {0x10, 0x00, 0x40, 0x00}, //REG33, [31:16] rssi high threshold;[15:0] rssi low threshold
        {0xFF, 0xFF, 0xFF, 0xFF}, //REG34, [30:24]=0xFFFF EQ15-EQ1 off;[15:0] howing threshold
        {0x00, 0x00, 0x00, 0x00}, //REG35, [23:0]=350 freq-shift at up 1Hz,enable it set REG36[3]=0
        {0x0C, 0x00, 0x60, 0xC8}, //REG36, [31]=0 MSB;[30:28]=b000 I2S mode;[27]=0 I2S-Master;[26]=1 I2S on;[13]=1 analog output on;[7]DUF on;[6]reverb off;[3]=0 freq-shift on for anti-howling
        {0x3E, 0x00, 0x97, 0x36}, //REG37, [31:25]LRCK-48kHz;[24:13]BCLK-3.072MHz;[12:8]24bit
        {0x40, 0xD7, 0xD5, 0xF7}, //REG38, [23:0] synchronize word
        {0x00, 0x00, 0x00, 0x00}, //REG39, [31:0] ID code,pair with transmitter
        {0x28, 0x02, 0x85, 0x64}, //REG3A,
        {0x6D, 0x00, 0x08, 0x00}, //REG3B,
        {0x00, 0x40, 0xFF, 0xFE}, //REG3C,
        {0x00, 0x00, 0x62, 0x29}, //REG3D,
        {0x1F, 0x55, 0x4E, 0x6E}, //REG3E,
        {0x85, 0x68, 0x00, 0x3F}, //REG3F, [26]AFC=ON;[16]=0 ANT1@pin6,[16]=1 ANT2@pin5;[17]=1 auto select ant;[4]=1 i2s_sda_pull_up enable;[3]=1 Hiz at lrck=1
        {0x00, 0x00, 0x00, 0x00}, //REG59,|
        {0x00, 0x00, 0x00, 0x00}, //REG5A,|
        {0x00, 0x00, 0x00, 0x00}, //REG5B,| --->reverb
        {0x00, 0x00, 0x00, 0x00}, //REG5C,|
        {0x1F, 0xFF, 0x3F, 0xFF}, //REG5D,
        {0x00, 0x00, 0x0F, 0x00}, //REG5E,
};

uint8_t RX_I2C_Read(uint8_t reg, uint8_t *pBuf)
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
    if (I2C_BUS_SendStart(&bk9532.device))
    {
      /* Bus bus busy*/
      res = 1;
    }

    /* Send slave device address */
    if (I2C_BUS_SendByte(&bk9532.device, I2C_ADDRESS) == I2C_NOACK)
    {
      res = 2;
      goto END;
    }

    /* Send the slave device reg address to read */
    if (I2C_BUS_SendByte(&bk9532.device, I2C_RD((reg << 1))) == I2C_NOACK)
    {
      res = 3;
      goto END;
    }

    uint8_t len = 4;
    /* Read regs data */
    for (uint8_t i = 0; i < len; i++)
    {
      *pBuf = I2C_BUS_ReceiveByte(&bk9532.device, (i + 1) == len ? I2C_NOACK : I2C_ISACK);
      pBuf++;
    }
  }
END:
  /* Send STOP Condition */
  I2C_BUS_SendStop(&bk9532.device);
  return res;
}

uint8_t RX_I2C_Write(uint8_t reg, uint8_t *pBuf)
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
  if (I2C_BUS_SendStart(&bk9532.device))
  {
    /* Bus bus busy*/
    res = 1;
  }

  /* Send slave device address */
  if (I2C_BUS_SendByte(&bk9532.device, I2C_ADDRESS) == I2C_NOACK)
  {
    res = 2;
    goto END;
  }

  /* Send the slave device reg address to write */
  if (I2C_BUS_SendByte(&bk9532.device, I2C_WR(reg << 1)) == I2C_NOACK)
  {
    res = 3;
    goto END;
  }

  /* Send data bytes to write */
  for (uint8_t i = 0; i < 4; i++)
  {
    if (I2C_BUS_SendByte(&bk9532.device, *pBuf) == I2C_NOACK)
    {
      res = 4;
      goto END;
    }
    pBuf++;
  }

END:
  /* Send STOP Condition */
  I2C_BUS_SendStop(&bk9532.device);
  return res;
}

/**
 * @brief 设置接收的频率(160-270MHz 500-980MHz)
 * @param freq 要设置的频率(单位:kHz)
 * @return uint8_t 0:成功 其他:频率超过范围
 */
uint8_t RX_Set_Band_And_Frequency(uint32_t freq)
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
    R0D = (uint32_t)((freq + 163.84) * 24 * 0x800000 / 24576);
  }
  else if (select == V178_270)
  {
    R03_21_20 = 0b10;
    R03_15_13 = 0b001;
    R0D = (uint32_t)((freq + 163.84) * 16 * 0x800000 / 24576);
  }
  else if (select == U500_710)
  {
    R03_21_20 = 0b01;
    R03_15_13 = 0b001;
    R0D = (uint32_t)((freq + 163.84) * 6 * 0x800000 / 24576);
  }
  else if (select == U710_980)
  {
    R03_21_20 = 0b01;
    R03_15_13 = 0b000;
    R0D = (uint32_t)((freq + 163.84) * 4 * 0x800000 / 24576);
  }

  analog_reg_val[3][1] &= ~0x30; //REG3[21:20]=0b00
  analog_reg_val[3][2] &= ~0xE0; //REG3[15:13]=0b000
  analog_reg_val[3][1] |= R03_21_20 << 4;
  analog_reg_val[3][2] |= R03_15_13 << 5;
  RX_I2C_Write(0x03, analog_reg_val[3]);

  reg_val[0] = R0D >> 24;
  reg_val[1] = R0D >> 16;
  reg_val[2] = R0D >> 8;
  reg_val[3] = R0D >> 0;
  RX_I2C_Write(0x0D, reg_val);

  //R07 R08 R3E
  if (select == V160_178 || select == V178_270)
  {
    analog_reg_val[7][1] &= ~0x0E; //REG7[19:17]=0b000
    analog_reg_val[7][1] |= 0x04;  //REG7[19:17]=0b010
    RX_I2C_Write(0x07, analog_reg_val[7]);

    analog_reg_val[8][0] = 0xED;
    analog_reg_val[8][1] = 0xCD;
    analog_reg_val[8][2] = 0x87;
    analog_reg_val[8][3] = 0x4C;
    RX_I2C_Write(0x08, analog_reg_val[8]);

    RX_I2C_Read(0x3E, reg_val);
    reg_val[0] &= ~0x0F; //REG3E[27:24]=0b0000
    reg_val[0] |= 0x0D;  //REG3E[27:24]=0b1101
    RX_I2C_Write(0x3E, reg_val);
  }
  else if (select == U500_710 || select == U710_980)
  {
    analog_reg_val[7][1] |= 0x0E; //REG7[19:17]=0b111
    RX_I2C_Write(0x07, analog_reg_val[7]);

    analog_reg_val[8][0] = 0xEF;
    analog_reg_val[8][1] = 0xF1;
    analog_reg_val[8][2] = 0x19;
    analog_reg_val[8][3] = 0x4C;
    RX_I2C_Write(0x08, analog_reg_val[8]);

    RX_I2C_Read(0x3E, reg_val);
    reg_val[0] |= 0x0F; //REG3E[27:24]=0b1111
    RX_I2C_Write(0x3E, reg_val);
  }
  return 0;
}

void RX_Reset_Chip(void)
{
  RX_I2C_Read(0x3F, reg_val);
  reg_val[3] &= ~0x20; //REG3F[5]=0
  RX_I2C_Write(0x3F, reg_val);
  reg_val[3] |= 0x20; //REG3F[5]=1
  RX_I2C_Write(0x3F, reg_val);
}

/**
 * @brief 接收模式,触发一次芯片完成校准
 *
 */
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
  Delay_ms(3); //At least delay 2ms

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

  Delay_ms(2); //请确保该处延时大于或等于2ms
  RX_I2C_Read(0x32, reg_val);
  reg_val[3] &= ~0x10; //rx_en = 0
  reg_val[2] |= 0x30;  //sync_unlock_win = 3
  RX_I2C_Write(0x32, reg_val);
  reg_val[3] |= 0x18; //rx_en = 1,cfg_whiten_en=1
  RX_I2C_Write(0x32, reg_val);
  Delay_ms(2); //请确保该处延时大于或等于2ms
}

void RX_PreventPop(void)
{
  uint8_t FIFO_Num;

  RX_I2C_Read(0x7B, reg_val);
  FIFO_Num = reg_val[0] & 0x3F; //read REG7B[10:0]=PLC FIFO count
  FIFO_Num <<= 8;
  FIFO_Num |= reg_val[1] & 0xF8;
  FIFO_Num >>= 3;

  if (FIFO_Num >= 28)
  {
    RX_Force_PhaseUnlock(); //force Lock to UnLock, reset RX state
  }
}

void RX_Cali_FreqOffset(void)
{
  uint16_t fifo_num, ctune;
  uint32_t temp_val;
  int32_t deltaf, x_val;

  RX_I2C_Read(0x7B, reg_val);
  RX_I2C_Read(0x38, CtuneBuf);

  fifo_num = reg_val[0] & 0x3F; //read REG7B[10:0]=PLC FIFO count
  fifo_num <<= 8;
  fifo_num |= reg_val[1] & 0xF8;
  fifo_num >>= 3;

  temp_val = reg_val[2];
  temp_val = temp_val << 8;
  temp_val += reg_val[3];
  temp_val = (temp_val + 16384) & 0x7FFF; //remove bit15
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

  ctune = CtuneBuf[0] & 0x7F;
  x_val = ctune - x_val;

  if (x_val > 127)
    x_val = 127;
  else if (x_val < 0)
    x_val = 0;

  CtuneBuf[0] = (CtuneBuf[0] & 0x80) | (x_val & 0x7F); //update crystal capacitors
}

void RX_Frequency_Tracking(void)
{
  RX_I2C_Read(0x31, reg_val);
  reg_val[0] |= RX_INT_SYMBOL; //clear the symbot interrupt
  RX_I2C_Write(0x31, reg_val);

  Delay_ms(2); //请确保该处延时大于或等于2ms
  if (BK1_IRQ())
  {
    RX_Cali_FreqOffset();
    RX_I2C_Write(0x38, CtuneBuf); //Update_Ctune
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
  case RX_GPIO4_Function2:
    RX_I2C_Read(0x31, reg_val);
    reg_val[3] = mode;
    RX_I2C_Write(0x31, reg_val);
  }
}

void RX_Audio_Output_Set(bool Enable)
{
  RX_I2C_Read(0x36, reg_val);
  if (Enable)
    reg_val[2] |= 0x20; //REG36h [13]=1,audio_out_en=1;
  else
    reg_val[2] &= ~0x20; //REG36h [13]=0,audio_out_en=0;
  RX_I2C_Write(0x36, reg_val);
}

void RX_Check_PhaseLock(uint8_t index)
{
  bk9532.bus_busy = true;
  //选择总线
  if (RX_Set_I2C_Bus(index))
    return;

  uint8_t lock;
  RX_I2C_Read(0x75, reg_val);
  lock = reg_val[2] & 0x08;
  //点亮对应的接收LED
  RX_GPIO4_Set(lock ? RX_GPIO4_High : RX_GPIO4_Low);
  bk9532.bus_busy = false;
}

uint8_t RX_Read_RSSI(void)
{
  RX_I2C_Read(0x75, reg_val);
  return reg_val[3];
}

void RX_Write_ID(uint8_t id_dat)
{
  reg_val[0] = 0x00;
  reg_val[1] = 0x00;
  reg_val[2] = 0x00;
  reg_val[3] = id_dat;
  RX_I2C_Write(0x39, reg_val);
}

uint8_t RX_Read_UserData(void)
{
  RX_I2C_Read(0x7C, reg_val);
  return reg_val[3];
}

/**
 * @brief 接收模式,设置采样率为32kHz
 *
 */
void RX_Set_SampleRate_32kHz(void)
{
  analog_reg_val[9][0] &= 0x7F; //REG9[31]=0
  RX_I2C_Write(0x09, analog_reg_val[9]);

  analog_reg_val[5][0] &= ~0x18; //REG5[28:27]=00
  analog_reg_val[5][0] |= 0x08;  //REG5[28:27]=01
  RX_I2C_Write(0x05, analog_reg_val[5]);

  RX_I2C_Read(0x31, reg_val);
  reg_val[0] &= ~0xC0; //REG31[31:30]=00
  reg_val[0] |= 0x80;  //REG31[31:30]=10
  RX_I2C_Write(0x31, reg_val);
}

void RX_RF_UnLock_Check(void)
{
  RX_I2C_Read(0x73, reg_val);
  if (reg_val[3] & 0x02) //RX unlock
  {
    RX_Trigger();
  }
}

void RX_IRQ(uint8_t index)
{
  if (bk9532.bus_busy || RX_Set_I2C_Bus(index))
  {
    return;
  }

  RX_I2C_Read(0x31, reg_val);
  if (reg_val[0] & RX_INT_SYMBOL)
  {
  }
  if (reg_val[0] & RX_INT_PLL_STATE)
  {
  }
  if (reg_val[0] & RX_INT_FRAME_ERR)
  {
  }
  if (reg_val[1] & RX_INT_RSSI)
  {
  }
  //RX_I2C_Write(0x31, reg_val);
}

/**
 * @brief 根据设备号选择所在的I2C总线
 *
 * @param index 设备号
 * @return true 失败
 * @return false 成功
 */
bool RX_Set_I2C_Bus(uint8_t index)
{
  switch (index)
  {
  case 1:
    bk9532.device = I2C_Bus1;
    break;
  case 2:
    bk9532.device = I2C_Bus2;
    break;
  case 3:
    bk9532.device = I2C_Bus3;
    break;
  case 4:
    bk9532.device = I2C_Bus4;
    break;
  default:
    return true;
  }
  return false;
}

/**
 * @brief 输出I2S数据时候的LRCK电平
 * @param DataOutAtLRCK true:高电平输出 false:低电平输出
 */
void RX_Set_LRCK_DataOut(bool DataOutAtLRCK)
{
  RX_I2C_Read(0x3F, reg_val);
  if (DataOutAtLRCK)
    reg_val[3] &= ~0x08;
  else
    reg_val[3] |= 0x08;
  RX_I2C_Write(0x3F, reg_val);
}

/**
 * @brief 初始化接收设备
 * @param index 设备号
 * @param freq 接收频率(单位:kHz)
 * @param DataOutAtLRCK 输出I2S数据时候的LRCK电平
 * @return uint8_t 成功 其他:失败
 */
uint8_t RX_Init(uint8_t index, uint32_t freq, bool DataOutAtLRCK)
{
  //选择总线
  if (RX_Set_I2C_Bus(index))
  {
    return 1;
  }
  bk9532.bus_busy = true;

  //check if chip is online
  for (uint8_t i = 0; i < 10; i++)
  {
    if (RX_I2C_Read(0x70, reg_val))
      return 2;

    uint16_t chipID = reg_val[2] << 8 | reg_val[3];
    if (chipID == RX_ChipID)
      break;
    if (i + 1 == 10)
      return 3;
    Delay_ms(25);
  }

  //chip software reset
  RX_Reset_Chip();

  //write regs table
  for (uint8_t addr, i = 0; i < 38; i++)
  {
    if (i <= 0x0B)
      addr = i;
    else if (i <= 0x1F)
      addr = 32 + i;
    else if (i <= 0x25)
      addr = 57 + i;
    if (RX_I2C_Write(addr, (uint8_t *)rx_reg_val[i]))
    {
      return 4;
    }
  }

  RX_Set_LRCK_DataOut(DataOutAtLRCK);

  //blink for write rx regs done
  for (uint8_t i = 0; i < 2; i++)
  {
    RX_GPIO4_Set(RX_GPIO4_High);
    Delay_ms(50);
    RX_GPIO4_Set(RX_GPIO4_Low);
    Delay_ms(50);
  }
  RX_GPIO4_Set(RX_GPIO4_Function2);

  if (RX_Set_Band_And_Frequency(freq))
  {
    return 5;
  }
  RX_Trigger();

  bk9532.bus_busy = false;
  return 0;
}