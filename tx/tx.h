#ifndef _TX_H
#define _TX_H

uint8_t TX_Init(uint32_t freq);
void TX_Write_ID(uint8_t id_dat);
void TX_Write_UserData(uint8_t dat);
void TX_Write_28kHz_SampleMode(void);
uint8_t TX_Set_Band_And_Frequency(uint64_t freq);
void TX_Trigger(void);
void TX_Reset_Chip(void);
void TX_RF_UnLock_Check(void);

void TX_I2C_Write(uint8_t reg,uint8_t *pBuf);
void TX_I2C_Read(uint8_t reg,uint8_t *pBuf);

#endif