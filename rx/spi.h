#ifndef _SPI_H
#define _SPI_H
#include <stdbool.h>

#define SPI1_PORT       GPIOA
#define SPI1_CSN_PIN    GPIO_Pin_4
#define SPI1_SCK_PIN    GPIO_Pin_5
#define SPI1_MISO_PIN   GPIO_Pin_6
#define SPI1_MOSI_PIN   GPIO_Pin_7
#define SPI1_CSN_L()    (SPI1_PORT->BRR  = SPI1_CSN_PIN)
#define SPI1_CSN_H()    (SPI1_PORT->BSRR = SPI1_CSN_PIN)

void SPI1_Init(void);
uint8_t SPI1_ReadByte(void);
void SPI1_WriteByte(uint8_t data);
uint8_t SPI1_Transfer(uint8_t data);
void SwitchSpiHiZ(bool isHiZ);
#endif
