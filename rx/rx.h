#ifndef _RX_H
#define _RX_H

uint8_t init_rx(void);

void RX_Write_ID(uint8_t id_dat);
void RX_GPIO4_Set(uint8_t mode);
uint8_t RX_Read_UserData(void);
void RX_Set_SampleRate_28kHz(void);
uint8_t RX_Set_Band_And_Frequency(uint64_t freq);
void RX_Trigger(void);
void RX_Reset_Chip(void);
void RX_RF_UnLock_Check(void);
void RX_Frequency_Tracking(void);

uint8_t RX_I2C_Write(uint8_t reg, uint8_t *pBuf);
uint8_t RX_I2C_Read(uint8_t reg, uint8_t *pBuf);

#endif