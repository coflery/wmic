#include "stm32f0xx.h"
#include "spi.h"
#include "gpio.h"

void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef   SPI_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	GPIO_PinAFConfig(SPI1_PORT, GPIO_PinSource5, GPIO_AF_0);
	GPIO_PinAFConfig(SPI1_PORT, GPIO_PinSource6, GPIO_AF_0);
	GPIO_PinAFConfig(SPI1_PORT, GPIO_PinSource7, GPIO_AF_0);

	GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN | SPI1_MOSI_PIN | SPI1_MISO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    //没有上下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;   //中速
	GPIO_Init(SPI1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = SPI1_CSN_PIN;       //配置CSN
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;      //输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;    //没有上下拉
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;   //中速
	GPIO_SetBits(SPI1_PORT, SPI1_CSN_PIN);
	GPIO_Init(SPI1_PORT, &GPIO_InitStructure);

	SPI_I2S_DeInit(SPI1);                                  //将寄存器重设为缺省值

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode      = SPI_Mode_Master;     //主机模式
	SPI_InitStructure.SPI_DataSize  = SPI_DataSize_8b;     //8 位帧结构
	SPI_InitStructure.SPI_CPOL      = SPI_CPOL_Low;        //通信空闲时 SCK 为低电平
	SPI_InitStructure.SPI_CPHA      = SPI_CPHA_1Edge;      //第一个时钟沿捕获

	SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;              //软件控制 NSS
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;   //SPI 速度 8 分频(48/8=6MHz)
	SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;          //数据传输从 MSB 开始
	SPI_InitStructure.SPI_CRCPolynomial     = 7;                         //CRC 校验
	SPI_Init(SPI1, &SPI_InitStructure);
	// SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);          //重要:把应答数据位设置为8位
	SPI_Cmd(SPI1, ENABLE);
}

uint8_t SPI1_ReadByte(void)
{
	SPI1_CSN_L();
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	uint8_t data = SPI_ReceiveData8(SPI1);
	SPI1_CSN_L();
	return data;
}

void SPI1_WriteByte(uint8_t data)
{
	SPI1_CSN_L();
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_SendData8(SPI1, data);
	SPI1_CSN_H();
}

uint8_t SPI1_Transfer(uint8_t data)
{
	SPI1_WriteByte(data);
	return(SPI1_ReadByte());
}

/**
 * @brief 
 * 
 * @param isHiZ 
 */
void SwitchSpiHiZ(bool isHiZ)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	if (isHiZ)
	{
		GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN | SPI1_MOSI_PIN | SPI1_MISO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	  //输入
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; //中速
		GPIO_Init(SPI1_PORT, &GPIO_InitStructure);
	}
	else
	{
		GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN | SPI1_MOSI_PIN | SPI1_MISO_PIN;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	  //复用
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	  //推挽
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; //中速
		GPIO_Init(SPI1_PORT, &GPIO_InitStructure);
	}
}