#include "stm32f0xx.h"
#include "i2c.h"
#include "delay.h"
#include "led.h"

#define PORT_I2C_SCL1 GPIOB
#define PORT_I2C_SCL2 GPIOA
#define PORT_I2C_SCL3 GPIOB
#define PORT_I2C_SCL4 GPIOB
#define PORT_I2C_SCL5 GPIOA
#define PORT_I2C_SDA1 GPIOB

#define PIN_I2C_SCL1 GPIO_Pin_4
#define PIN_I2C_SCL2 GPIO_Pin_3
#define PIN_I2C_SCL3 GPIO_Pin_2
#define PIN_I2C_SCL4 GPIO_Pin_10
#define PIN_I2C_SCL5 GPIO_Pin_15
#define PIN_I2C_SDA1 GPIO_Pin_6

#define RCC_I2C_SCL1 RCC_AHBPeriph_GPIOB
#define RCC_I2C_SCL2 RCC_AHBPeriph_GPIOA
#define RCC_I2C_SCL3 RCC_AHBPeriph_GPIOB
#define RCC_I2C_SCL4 RCC_AHBPeriph_GPIOB
#define RCC_I2C_SCL5 RCC_AHBPeriph_GPIOA
#define RCC_I2C_SDA1 RCC_AHBPeriph_GPIOB

#define LED_DATA_TRIG() led_data_trig()

/**
  * @brief  初始化IIC
  */
void i2c_init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHBPeriphClockCmd(RCC_I2C_SCL1, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_I2C_SCL2, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_I2C_SCL3, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_I2C_SCL4, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_I2C_SCL5, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_I2C_SDA1, ENABLE);

  //SCL1
  GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_SetBits(PORT_I2C_SCL1, PIN_I2C_SCL1);
  GPIO_Init(PORT_I2C_SCL1, &GPIO_InitStructure);
  //SCL2
  GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_SetBits(PORT_I2C_SCL2, PIN_I2C_SCL2);
  GPIO_Init(PORT_I2C_SCL2, &GPIO_InitStructure);
  //SCL3
  GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_SetBits(PORT_I2C_SCL3, PIN_I2C_SCL3);
  GPIO_Init(PORT_I2C_SCL3, &GPIO_InitStructure);
  //SCL4
  GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_SetBits(PORT_I2C_SCL4, PIN_I2C_SCL4);
  GPIO_Init(PORT_I2C_SCL4, &GPIO_InitStructure);
  //SCL5
  GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_SetBits(PORT_I2C_SCL5, PIN_I2C_SCL5);
  GPIO_Init(PORT_I2C_SCL5, &GPIO_InitStructure);
  //SDA1
  GPIO_InitStructure.GPIO_Pin = PIN_I2C_SDA1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_SetBits(PORT_I2C_SDA1, PIN_I2C_SDA1);
  GPIO_Init(PORT_I2C_SDA1, &GPIO_InitStructure);
}

/******************************** I2C1 ********************************/
void I2C_SetInputSDA1(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = PIN_I2C_SDA1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(PORT_I2C_SDA1, &GPIO_InitStructure);
}

void I2C_SetOutputSDA1(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = PIN_I2C_SDA1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(PORT_I2C_SDA1, &GPIO_InitStructure);
}

void I2C_SetInputSCL1(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(PORT_I2C_SCL1, &GPIO_InitStructure);
}

void I2C_SetOutputSCL1(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(PORT_I2C_SCL1, &GPIO_InitStructure);
}

/**
  * @brief  读取SDA线电平状态
  * @retval 执行的结果(false:低电平 true:高电平)
  */
bool I2C_ReadSDA1(void)
{
  return GPIO_ReadInputDataBit(PORT_I2C_SDA1, PIN_I2C_SDA1);
}

/**
  * @brief  读取SCL线电平状态
  * @retval 执行的结果(false:低电平 true:高电平)
  */
bool I2C_ReadSCL1(void)
{
  return GPIO_ReadInputDataBit(PORT_I2C_SCL1, PIN_I2C_SCL1);
}


/**
  * @brief  控制SDA线电平状态
  * @param  isHigh: 输出电平状态
  *     @arg false: 输出低电平
  *     @arg true: 输出高电平
  * @retval 执行的结果(false:成功 true:失败)
  */
void I2C_WriteSDA1(bool isHigh)
{
  GPIO_WriteBit(PORT_I2C_SDA1, PIN_I2C_SDA1, isHigh);
}

/**
  * @brief  控制SCL线电平状态
  * @param  isHigh: 输出电平状态
  *     @arg false: 输出低电平
  *     @arg true: 输出高电平
  * @retval 执行的结果(false:成功 true:失败)
  */
void I2C_WriteSCL1(bool isHigh)
{
  GPIO_WriteBit(PORT_I2C_SCL1, PIN_I2C_SCL1, isHigh);
}
void I2C_WriteSCL2(bool isHigh)
{
  GPIO_WriteBit(PORT_I2C_SCL2, PIN_I2C_SCL2, isHigh);
}
void I2C_WriteSCL3(bool isHigh)
{
  GPIO_WriteBit(PORT_I2C_SCL3, PIN_I2C_SCL3, isHigh);
}
void I2C_WriteSCL4(bool isHigh)
{
  GPIO_WriteBit(PORT_I2C_SCL4, PIN_I2C_SCL4, isHigh);
}
void I2C_WriteSCL5(bool isHigh)
{
  GPIO_WriteBit(PORT_I2C_SCL5, PIN_I2C_SCL5, isHigh);
}

/*********************** I2C_BUS ************************************/
/**
  * @brief  I2C总线发送开始信号
  * @param  isHigh: 输出电平状态
  * @retval 执行的结果(false:成功 true:失败)
  */
bool I2C_BUS_SendStart(const I2C_BUS *bus)
{
  bus->SetSDAIn();
  bus->SetSCLIn();
  if (bus->ReadSDA() == false)
    return true;
  if (bus->ReadSCL() == false)
    return true;
  bus->WriteSDA(true);
  bus->WriteSCL(true);
  bus->SetSDAOut();
  bus->SetSCLOut();
  Delay_us(bus->Timing->tSU_STA);
  bus->WriteSDA(false);
  Delay_us(bus->Timing->tHD_STA);
  return false;
}

/**
  * @brief  I2C总线发送重开始信号
  */
void I2C_BUS_SendRestart(const I2C_BUS *bus)
{
  bus->WriteSCL(false);
  Delay_us(bus->Timing->tHD_DAT);
  bus->WriteSDA(true);
  bus->SetSDAOut();
  Delay_us(bus->Timing->tSU_DAT);
  bus->WriteSCL(true);
  Delay_us(bus->Timing->tSU_STA);
  bus->WriteSDA(false);
  Delay_us(bus->Timing->tHD_STA);
}

/**
  * @brief  I2C总线发送停止信号
  */
void I2C_BUS_SendStop(const I2C_BUS *bus)
{
  bus->WriteSCL(false);
  bus->SetSCLOut(); //bug fix: force stop in bus busy
  Delay_us(bus->Timing->tHD_DAT);
  bus->WriteSDA(false);
  bus->SetSDAOut();
  Delay_us(bus->Timing->tSU_DAT);
  bus->WriteSCL(true);
  Delay_us(bus->Timing->tSU_STO);
  bus->WriteSDA(true);
  Delay_us(bus->Timing->tBUF);
}

/**
  * @brief  IIC总线发送数据
  * @param  data: 待发送的字节
  * @retval 执行的结果(I2C_ISACK:从机应答 I2C_NOACK:从机无应答)
  */
I2C_ACK I2C_BUS_SendByte(const I2C_BUS *bus, uint8_t data)
{
  //Send bit one by one
  for (uint8_t i = 0; i < 8; i++)
  {
    bus->WriteSCL(false);
    Delay_us(bus->Timing->tHD_DAT);
    bus->WriteSDA(data & 0x80);
    data <<= 1;
    bus->SetSDAOut();
    Delay_us(bus->Timing->tSU_DAT);
    bus->WriteSCL(true);
    Delay_us(bus->Timing->tHIGH);
  }
  //Read acknowledge
  bus->WriteSCL(false);
  Delay_us(bus->Timing->tHD_DAT);
  bus->SetSDAIn();
  Delay_us(bus->Timing->tSU_DAT);
  bus->WriteSCL(true);
  Delay_us(bus->Timing->tHIGH);
  LED_DATA_TRIG();

  return (bus->ReadSDA() == false ? I2C_ISACK : I2C_NOACK);
}

/**
  * @brief  IIC总线接收数据
  * @param  ack: 输出电平状态
  *     @arg I2C_ISACK: 发送应答
  *     @arg I2C_NOACK: 不发送应答
  * @retval 读取到的字节
  */
uint8_t I2C_BUS_ReceiveByte(const I2C_BUS *bus, I2C_ACK ack)
{
  uint8_t data = 0;
  //Receive bit one by one
  for (uint8_t i = 0; i < 8; i++)
  {
    bus->WriteSCL(false);
    Delay_us(bus->Timing->tHD_DAT);
    bus->SetSDAIn();
    Delay_us(bus->Timing->tSU_DAT);
    bus->WriteSCL(true);
    Delay_us(bus->Timing->tHIGH);
    data <<= 1;
    data |= (bus->ReadSDA() ? 0x01 : 0x00);
  }
  //Write acknowledge
  bus->WriteSCL(false);
  Delay_us(bus->Timing->tHD_DAT);
  bus->WriteSDA(ack == I2C_NOACK);
  bus->SetSDAOut();
  Delay_us(bus->Timing->tSU_DAT);
  bus->WriteSCL(true);
  Delay_us(bus->Timing->tHIGH);
  LED_DATA_TRIG();
  return data;
}

void DoNothing(void)
{
}
bool GetTrue(void)
{
  return true;
}

/**
  * @brief  I2C-100KHz时序(单位us)
  */
const I2C_TIMING I2C_Timing_100KHz =
    {
        .tHD_STA = 5, /* I2C specification: Min 4.0 us */
        .tSU_STA = 6, /* I2C specification: Min 4.7 us */
        .tHD_DAT = 1, /* I2C specification: Max 3.45 us */
        .tSU_DAT = 1, /* I2C specification: Min 0.25 us */
        .tSU_STO = 5, /* I2C specification: Min 4.0 us */
        .tHIGH = 5,   /* I2C specification: Min 4.0 us */
        .tBUF = 6,    /* I2C specification: Min 4.7 us */
};

/**
  * @brief  I2C总线接口
  */
const I2C_BUS I2C_Bus1 =
    {
        .SetSDAIn = I2C_SetInputSDA1,
        .SetSDAOut = I2C_SetOutputSDA1,
        .SetSCLIn = DoNothing,
        .SetSCLOut = DoNothing,
        .ReadSDA = I2C_ReadSDA1,
        .ReadSCL = GetTrue,
        .WriteSDA = I2C_WriteSDA1,
        .WriteSCL = I2C_WriteSCL1,
        .Timing = &I2C_Timing_100KHz,
};
const I2C_BUS I2C_Bus2 =
    {
        .SetSDAIn = I2C_SetInputSDA1,
        .SetSDAOut = I2C_SetOutputSDA1,
        .SetSCLIn = DoNothing,
        .SetSCLOut = DoNothing,
        .ReadSDA = I2C_ReadSDA1,
        .ReadSCL = GetTrue,
        .WriteSDA = I2C_WriteSDA1,
        .WriteSCL = I2C_WriteSCL2,
        .Timing = &I2C_Timing_100KHz,
};
const I2C_BUS I2C_Bus3 =
    {
        .SetSDAIn = I2C_SetInputSDA1,
        .SetSDAOut = I2C_SetOutputSDA1,
        .SetSCLIn = DoNothing,
        .SetSCLOut = DoNothing,
        .ReadSDA = I2C_ReadSDA1,
        .ReadSCL = GetTrue,
        .WriteSDA = I2C_WriteSDA1,
        .WriteSCL = I2C_WriteSCL3,
        .Timing = &I2C_Timing_100KHz,
};
const I2C_BUS I2C_Bus4 =
    {
        .SetSDAIn = I2C_SetInputSDA1,
        .SetSDAOut = I2C_SetOutputSDA1,
        .SetSCLIn = DoNothing,
        .SetSCLOut = DoNothing,
        .ReadSDA = I2C_ReadSDA1,
        .ReadSCL = GetTrue,
        .WriteSDA = I2C_WriteSDA1,
        .WriteSCL = I2C_WriteSCL4,
        .Timing = &I2C_Timing_100KHz,
};
const I2C_BUS I2C_Bus5 =
    {
        .SetSDAIn = I2C_SetInputSDA1,
        .SetSDAOut = I2C_SetOutputSDA1,
        .SetSCLIn = DoNothing,
        .SetSCLOut = DoNothing,
        .ReadSDA = I2C_ReadSDA1,
        .ReadSCL = GetTrue,
        .WriteSDA = I2C_WriteSDA1,
        .WriteSCL = I2C_WriteSCL5,
        .Timing = &I2C_Timing_100KHz,
};