#include <stm32f0xx.h>
#include "i2c.h"

#define SCL_USE_OD_MODE
#define SDA_USE_OD_MODE

#define PORT_I2C_SCL1 GPIOA
#define PORT_I2C_SCL2 GPIOA
#define PORT_I2C_SDA GPIOA

#define PIN_I2C_SCL1 GPIO_Pin_1
#define PIN_I2C_SCL2 GPIO_Pin_0
#define PIN_I2C_SDA GPIO_Pin_2

#define RCC_I2C_SCL1 RCC_AHBPeriph_GPIOA
#define RCC_I2C_SCL2 RCC_AHBPeriph_GPIOA
#define RCC_I2C_SDA RCC_AHBPeriph_GPIOA

static void I2C_Delay(void);
static void I2C_PinModeOutput(void);
static void I2C_PinModeInput(void);
static void I2C_SCL(enum i2c_port port, uint8_t stat);
static void I2C_SDA(uint8_t stat);
static uint8_t I2C_ReadSDA(void);



// void nvic_config(void)
// {
//     NVIC_InitTypeDef NVIC_InitStructure;

//     /* Reconfigure and enable I2C1 error interrupt to have the higher priority */
//     NVIC_InitStructure.NVIC_IRQChannel = I2C1_IRQn;
//     NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
//     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//     NVIC_Init(&NVIC_InitStructure);
// }
// void hw_i2c_gpio_config(void)
// {
//     GPIO_InitTypeDef GPIO_InitStructure;

//     RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

//     /* Configure the I2C clock source. The clock is derived from the HSI */
//     RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
//     /* I2C GPIO clock source */
//     RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

//     /* GPIO alternate as I2C */
//     GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_4);
//     GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_4);

//     /* Configure I2C pin: SCL */
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
//     GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//     GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//     GPIO_Init(GPIOA, &GPIO_InitStructure);

//     /* Configure I2C pin: SDA */
//     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//     GPIO_Init(GPIOA, &GPIO_InitStructure);
// }
// void hw_i2c_config(void)
// {
//     I2C_InitTypeDef I2C_InitStructure;

//     hw_i2c_gpio_config();

//     /* I2C configuration */
//     I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//     I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
//     I2C_InitStructure.I2C_DigitalFilter = 0x00;
//     I2C_InitStructure.I2C_OwnAddress1 = 0xAA;
//     I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//     I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//     I2C_InitStructure.I2C_Timing = 0x1045061D; //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

//     /* Apply I2C configuration after enabling it */
//     I2C_Init(I2C1, &I2C_InitStructure);

//     /* I2C Peripheral Enable */
//     I2C_Cmd(I2C1, ENABLE);

//     /* Enables the I2C SMBus Alert feature */
//     I2C_SMBusAlertCmd(I2C1, ENABLE);
//     I2C_ClearFlag(I2C1, I2C_FLAG_ALERT);

//     /* Enable SMBus Alert interrupt */
//     I2C_ITConfig(I2C1, I2C_IT_ERRI, ENABLE);
// }


/*
*    函 数 名: i2c_init
*    功能说明: 初始化IIC接口
*    形    参: 无
*    返 回 值: 无
*/
void i2c_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_I2C_SCL1, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_I2C_SCL2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_I2C_SDA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = PIN_I2C_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
#ifdef SDA_USE_OD_MODE
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
#endif
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_SetBits(PORT_I2C_SDA, PIN_I2C_SDA);
    GPIO_Init(PORT_I2C_SDA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL1;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
#ifdef SCL_USE_OD_MODE
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
#endif
    GPIO_SetBits(PORT_I2C_SCL1, PIN_I2C_SCL1);
    GPIO_Init(PORT_I2C_SCL1, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL2;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
#ifdef SCL_USE_OD_MODE
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
#endif
    GPIO_SetBits(PORT_I2C_SCL2, PIN_I2C_SCL2);
    GPIO_Init(PORT_I2C_SCL2, &GPIO_InitStructure);

    I2C_Stop(I2C_PORT1);
    I2C_Stop(I2C_PORT2);
}

/*
*    函 数 名: I2C_Delay
*    功能说明: 延时函数
*    形    参: 无
*    返 回 值: 无
*/
static void I2C_Delay(void)
{
#if 0
    uint8_t time = 1;
    while (time--)
    {
        ;
    }
#else
    __NOP();
#endif
}

/*
*    函 数 名: I2C_PinModeOutput
*    功能说明: 将SDA线的端口设置为输出
*    形    参: 无
*    返 回 值: 无
*/
static void I2C_PinModeOutput(void)
{
#ifndef SDA_USE_OD_MODE
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = PIN_I2C_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(PORT_I2C_SDA, &GPIO_InitStructure);
#endif
}

/*
*    函 数 名: I2C_PinModeInput
*    功能说明: 将SDA线的端口设置为输入
*    形    参: 无
*    返 回 值: 无
*/
static void I2C_PinModeInput(void)
{
#ifndef SDA_USE_OD_MODE
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = PIN_I2C_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(PORT_I2C_SDA, &GPIO_InitStructure);
#endif
}

/*
*    函 数 名: I2C_SCL
*    功能说明: 控制SCL线电平状态
*    形    参: stat：0 输出低电平，1 输出高电平
*    返 回 值: 无
*/
static void I2C_SCL(enum i2c_port port, uint8_t stat)
{
    if (port == I2C_PORT1)
        GPIO_WriteBit(PORT_I2C_SCL1, PIN_I2C_SCL1, stat);
    else if (port == I2C_PORT2)
        GPIO_WriteBit(PORT_I2C_SCL2, PIN_I2C_SCL2, stat);
}

/*
*    函 数 名: I2C_SDA
*    功能说明: 控制SDA线电平状态
*    形    参: stat：0 输出低电平，1 输出高电平
*    返 回 值: 无
*/
static void I2C_SDA(uint8_t stat)
{
    GPIO_WriteBit(PORT_I2C_SDA, PIN_I2C_SDA, stat);
}

/*
*    函 数 名: I2C_ReadSDA
*    功能说明: 读取SDA线电平状态
*    形    参: 无
*    返 回 值: 0 或 1
*/

static uint8_t I2C_ReadSDA(void)
{
    return GPIO_ReadInputDataBit(PORT_I2C_SDA, PIN_I2C_SDA);
}

/*
*    函 数 名: I2C_Start
*    功能说明: IIC总线起始信号
*    形    参: port：I2C_SCL1 I2C_SCL2
*    返 回 值: 无
*/
void I2C_Start(enum i2c_port port)
{
    I2C_PinModeOutput();

    I2C_SDA(1);
    I2C_Delay();
    I2C_SCL(port, 1);
    I2C_Delay();

    I2C_SDA(0);
    I2C_Delay();

    I2C_SCL(port, 0);
    I2C_Delay();
}

/*
*    函 数 名: I2C_Stop
*    功能说明: IIC总线停止信号
*    形    参: 无
*    返 回 值: 无
*/
void I2C_Stop(enum i2c_port port)
{
    I2C_PinModeOutput();

    I2C_SCL(port, 0);
    I2C_SDA(0);
    I2C_Delay();

    I2C_SCL(port, 1);
    I2C_Delay();
    I2C_SDA(1);
    I2C_Delay();
}

/*
*    函 数 名: I2C_WriteByte
*    功能说明: IIC总线写数据
*    形    参: byte：写入的一个字节数据
*    返 回 值: 无
*/
void I2C_WriteByte(enum i2c_port port, uint8_t byte)
{
    uint8_t i;

    I2C_PinModeOutput();

    I2C_SCL(port, 0);
    I2C_Delay();

    for (i = 0; i < 8; i++)
    {
        if (byte & 0x80)
        {
            I2C_SDA(1);
        }
        else
        {
            I2C_SDA(0);
        }
        //I2C_Delay();

        byte = byte << 1;

        I2C_SCL(port, 1);
        I2C_Delay();
        I2C_SCL(port, 0);
        I2C_Delay();
    }

    I2C_SDA(1);
}

/*
*    函 数 名: I2C_Write
*    功能说明: IIC总线写数据
*    形    参: byte：写入的一个字节数据
*    返 回 值: 无
*/
void I2C_Write(enum i2c_port port, uint8_t byte, uint8_t SlaveAddr)
{
    I2C_WriteByte(port, byte);
}

/*
*    函 数 名: I2C_ReadByte
*    功能说明: IIC总线读数据
*    形    参: 无
*    返 回 值: recv：读取的一个字节数据
*/
uint8_t I2C_ReadByte(enum i2c_port port)
{
    uint8_t i;
    uint8_t recv = 0;

    I2C_PinModeOutput();

    I2C_SDA(1);
    I2C_Delay();

    I2C_PinModeInput();

    for (i = 0; i < 8; i++)
    {
        recv = recv << 1;

        I2C_SCL(port, 1);
        I2C_Delay();

        if (I2C_ReadSDA())
        {
            recv |= 0x01;
        }
        else
        {
            recv |= 0x00;
        }

        I2C_SCL(port, 0);
        I2C_Delay();
    }

    return recv;
}

/*
*    函 数 名: I2C_Ack
*    功能说明: IIC总线主机主动应答
*    形    参: 无
*    返 回 值: 无
*/
void I2C_Ack(enum i2c_port port)
{
    I2C_PinModeOutput();

    I2C_SCL(port, 0);
    I2C_SDA(0);
    I2C_Delay();

    I2C_SCL(port, 1);
    I2C_Delay();

    I2C_SCL(port, 0);
}

/*
*    函 数 名: I2C_NAck
*    功能说明: IIC总线主机主动非应答
*    形    参: 无
*    返 回 值: 无
*/
void I2C_NAck(enum i2c_port port)
{
    I2C_PinModeOutput();

    I2C_SCL(port, 0);
    I2C_SDA(1);
    I2C_Delay();

    I2C_SCL(port, 1);
    I2C_Delay();

    I2C_SCL(port, 0);
}

/*
*    函 数 名: I2C_CheckAck
*    功能说明: IIC总线检测应答信号
*    形    参: 无
*    返 回 值: 0 应答信号，1 非应答信号 
*/
enum ack_bit I2C_CheckAck(enum i2c_port port)
{
    uint8_t time = 0;

    I2C_PinModeOutput();

    I2C_SDA(1);
    I2C_Delay();
    I2C_SCL(port, 1);
    I2C_Delay();

    I2C_PinModeInput();

    while (I2C_ReadSDA())
    {
        time++;
        if (time >= 100)
        {
            return NACK;
        }
    }

    I2C_SCL(port, 0);

    return ACK;
}
