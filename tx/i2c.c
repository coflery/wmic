#include <stm32f0xx.h>
#include "i2c.h"

#define PORT_I2C_SCL GPIOA
#define PORT_I2C_SDA GPIOA

#define PIN_I2C_SCL GPIO_Pin_9
#define PIN_I2C_SDA GPIO_Pin_10

#define RCC_I2C_SCL RCC_AHBPeriph_GPIOA
#define RCC_I2C_SDA RCC_AHBPeriph_GPIOA

//#define I2C_HW

#ifdef I2C_HW
#define I2C_TIMEOUT 10
#define I2CPORT I2C1
#endif

static void I2C_Delay(void);
static void I2C_PinModeOutput(void);
static void I2C_PinModeInput(void);
static void I2C_SCL(uint8_t stat);
static void I2C_SDA(uint8_t stat);
static uint8_t I2C_ReadSDA(void);


/*
*    函 数 名: I2C_Init_GPIO
*    功能说明: 初始化IIC GPIO口
*    形    参: 无
*    返 回 值: 无
*/
void I2C_Init_GPIO(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_I2C_SCL, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_I2C_SDA, ENABLE);
#ifndef I2C_HW
    GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_SetBits(PORT_I2C_SCL, PIN_I2C_SCL);
    GPIO_Init(PORT_I2C_SCL, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_I2C_SDA;
    GPIO_SetBits(PORT_I2C_SDA, PIN_I2C_SDA);
    GPIO_Init(PORT_I2C_SDA, &GPIO_InitStructure);

    I2C_Stop();
#else
    RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_PinAFConfig(PORT_I2C_SCL, GPIO_PinSource9, GPIO_AF_4);
    GPIO_PinAFConfig(PORT_I2C_SDA, GPIO_PinSource10, GPIO_AF_4);

    GPIO_InitStructure.GPIO_Pin = PIN_I2C_SCL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(PORT_I2C_SCL, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = PIN_I2C_SDA;
    GPIO_Init(PORT_I2C_SCL, &GPIO_InitStructure);
#endif
}

/*
*    函 数 名: i2c_init
*    功能说明: 初始化IIC接口
*    形    参: 无
*    返 回 值: 无
*/
void i2c_init(void)
{
    I2C_Init_GPIO();
#ifdef I2C_HW
    I2C_InitTypeDef I2C_InitStructure;

    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_AnalogFilter = I2C_AnalogFilter_Enable;
    I2C_InitStructure.I2C_DigitalFilter = 0x00;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_Timing = 0x00210507;

    I2C_Init(I2CPORT, &I2C_InitStructure);
    I2C_Cmd(I2CPORT, ENABLE);
#endif
}

/*
*    函 数 名: I2C_Delay
*    功能说明: 延时函数
*    形    参: 无
*    返 回 值: 无
*/
static void I2C_Delay(void)
{
    uint8_t time = 10;
    while (time--)
    {
        ;
    }
}

/*
*    函 数 名: I2C_PinModeOutput
*    功能说明: 将SDA线的端口设置为输出
*    形    参: 无
*    返 回 值: 无
*/
static void I2C_PinModeOutput(void)
{
#ifndef I2C_HW
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
#ifndef I2C_HW
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
static void I2C_SCL(uint8_t stat)
{
#ifndef I2C_HW
    GPIO_WriteBit(PORT_I2C_SCL, PIN_I2C_SCL, stat);
#endif
}

/*
*    函 数 名: I2C_SDA
*    功能说明: 控制SDA线电平状态
*    形    参: stat：0 输出低电平，1 输出高电平
*    返 回 值: 无
*/
static void I2C_SDA(uint8_t stat)
{
#ifndef I2C_HW
    GPIO_WriteBit(PORT_I2C_SDA, PIN_I2C_SDA, stat);
#endif
}

/*
*    函 数 名: I2C_ReadSDA
*    功能说明: 读取SDA线电平状态
*    形    参: 无
*    返 回 值: 0 或 1
*/

static uint8_t I2C_ReadSDA(void)
{
#ifndef I2C_HW
    return GPIO_ReadInputDataBit(PORT_I2C_SDA, PIN_I2C_SDA);
#endif
    return 1;
}

/*
*    函 数 名: I2C_Start
*    功能说明: IIC总线起始信号
*    形    参: 无
*    返 回 值: 无
*/
void I2C_Start(void)
{
    I2C_PinModeOutput();

    I2C_SDA(1);
    I2C_Delay();
    I2C_SCL(1);
    I2C_Delay();

    I2C_SDA(0);
    I2C_Delay();

    I2C_SCL(0);
    I2C_Delay();
}

/*
*    函 数 名: I2C_Stop
*    功能说明: IIC总线停止信号
*    形    参: 无
*    返 回 值: 无
*/
void I2C_Stop(void)
{
    I2C_PinModeOutput();

    I2C_SCL(0);
    I2C_SDA(0);
    I2C_Delay();

    I2C_SCL(1);
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
void I2C_WriteByte(uint8_t byte)
{
#ifndef I2C_HW
    uint8_t i;

    I2C_PinModeOutput();

    I2C_SCL(0);
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

        byte = byte << 1;

        I2C_SCL(1);
        I2C_Delay();
        I2C_SCL(0);
        I2C_Delay();
    }

    I2C_SDA(1);
#else
    I2C_SendData(I2CPORT, byte);
#endif
}

/*
*    函 数 名: I2C_ReadByte
*    功能说明: IIC总线读数据
*    形    参: 无
*    返 回 值: recv：读取的一个字节数据
*/
uint8_t I2C_ReadByte(void)
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

        I2C_SCL(1);
        I2C_Delay();

        if (I2C_ReadSDA())
        {
            recv |= 0x01;
        }
        else
        {
            recv |= 0x00;
        }

        I2C_SCL(0);
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
void I2C_Ack(void)
{
    I2C_PinModeOutput();

    I2C_SCL(0);
    I2C_SDA(0);
    I2C_Delay();

    I2C_SCL(1);
    I2C_Delay();

    I2C_SCL(0);
}

/*
*    函 数 名: I2C_NAck
*    功能说明: IIC总线主机主动非应答
*    形    参: 无
*    返 回 值: 无
*/
void I2C_NAck(void)
{
    I2C_PinModeOutput();

    I2C_SCL(0);
    I2C_SDA(1);
    I2C_Delay();

    I2C_SCL(1);
    I2C_Delay();

    I2C_SCL(0);
}

/*
*    函 数 名: I2C_CheckAck
*    功能说明: IIC总线检测应答信号
*    形    参: 无
*    返 回 值: 0 应答信号，1 非应答信号 
*/
uint8_t I2C_CheckAck(void)
{
    uint8_t time = 0;

    I2C_PinModeOutput();

    I2C_SDA(1);
    I2C_Delay();
    I2C_SCL(1);
    I2C_Delay();

    I2C_PinModeInput();

    while (I2C_ReadSDA())
    {
        time++;
        if (time >= 100)
        {
            return 1;
        }
    }

    I2C_SCL(0);

    return 0;
}