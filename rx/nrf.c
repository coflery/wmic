#include "nrf.h"
#include "i2c.h"

#define NRF_ADDRESS_SADR0	0x52
#define NRF_ADDRESS_SADR1	0xD2

#define SLAVEBUFSIZE 64

// Rx I2S Control
#define TXSTA      0x01
#define TXFMT      0x51
#define TXMOD      0x5A
#define I2SCNF_IN  0x54
#define I2SRAT     0x55
// Rx I2S Control
#define I2SCNF_OUT 0x44
#define RXMOD      0x4A
// Link Status
#define LNKSTA     0x03
#define LNKQ       0x04
#define LNKERR     0x05
#define LNKMOD     0x36
#define LNKWTH     0x37
#define LNKETH     0x38
// Rx Status
#define RXSTAT     0x06
// Rx GPIO
#define RXPIN      0x07//input
#define RXPIO      0x41//output
#define RXPWME     0x42//pwm control
#define RXPWMD     0x43//pwm cycle
// Tx GPIO
#define TXDD       0x75
// RF Channel
#define CH0        0x0C
#define CH1        0x0D
#define CH2        0x0E
#define CH3        0x0F
#define CH4        0x10
#define CH5        0x11
#define CH6        0x12
#define CH7        0x13
#define CH8        0x14
#define CH9        0x15
#define CH10       0x16
#define CH11       0x17
#define CH12       0x18
#define CH13       0x19
#define CH14       0x1A
#define CH15       0x1B
#define CH16       0x1C
#define CH17       0x1D
#define CH18       0x1E
#define CH19       0x1F
#define CH20       0x20
#define CH21       0x21
#define CH22       0x22
#define CH23       0x23
#define CH24       0x24
#define CH25       0x25
#define CH26       0x26
#define CH27       0x27
#define CH28       0x28
#define CH29       0x29
#define CH30       0x2A
#define CH31       0x2B
#define CH32       0x2C
#define CH33       0x2D
#define CH34       0x2E
#define CH35       0x2F
#define CH36       0x30
#define CH37       0x31
// RF Channel Config
#define BCHD       0x32
#define NBCH       0x33
#define NACH       0x34
#define NLCH       0x35
// ID
#define ADDR0      0x39
#define ADDR1      0x3A
#define ADDR2      0x3B
#define ADDR3      0x3C
#define ADDR4      0x3D
// Remote Register Update
#define LINKCSTATE 0x3E
#define RXCSTATE   0x4B
#define TXCSTATE   0x5B
// RF Power
#define RXPWR      0x49
#define TXPWR      0x56
// Timer
#define RXSTI0     0x46
#define RXSTI1     0x47
#define RXWTI      0x48
#define RXLTI      0x4C
#define TXLTI      0x4D
#define TXSTI0     0x57
#define TXSTI1     0x58
#define TXWTI      0x59
// Interrupts
#define INTSTA     0x02
#define INTCF      0x53
#define RXWAKE     0x45
// Reset Output
#define RXRESO     0x40
#define TXRESO     0x50
// Tx Link Latency
#define TXLAT      0x52
 // Rx Audio Buffer
#define RXBUF0     0x60
#define RXBUF1     0x61
#define RXBUF2     0x62
#define RXBUF3     0x63
#define RXBUF4     0x64
#define RXBUF5     0x65
#define RXBUF6     0x66
#define RXBUF7     0x67
#define RXBUF8     0x68
#define RXBUF9     0x69
#define RXBUF10    0x6A
#define RXBUF11    0x6B
#define RXBUF12    0x6C
#define RXBUF13    0x6D
#define RXBUF14    0x6E
#define RXBUF15    0x6F
// Rx Audio Control
#define RXDCMD     0x70
#define RXWCNT     0x71
#define RXRCNT     0x72
#define RXEXEC_ID  0x73
#define RXEXEC     0x74
// Test
#define REV        0x7D
#define TESTREG    0x7E
#define TESTCH     0x7F

uint8_t ADDRESS;

uint8_t NRF_I2C_Read(uint8_t reg, uint8_t *pBuf, uint8_t length)
{
  uint8_t res = 0;
  /* Send START condition */
  if (I2C_BUS_SendStart(&I2C_Bus5))
  {
    /* Bus bus busy*/
    res = 1;
  }

  /* Send slave device address */
  if (I2C_BUS_SendByte(&I2C_Bus5, I2C_WR(ADDRESS)) == I2C_NOACK)
  {
    res = 2;
    goto END;
  }

  /* Send the slave device reg address to read */
  if (I2C_BUS_SendByte(&I2C_Bus5, reg) == I2C_NOACK)
  {
    res = 3;
    goto END;
  }

  /* Send RESTART condition */
  I2C_BUS_SendRestart(&I2C_Bus5);

  /* Send slave device address */
  if (I2C_BUS_SendByte(&I2C_Bus5, I2C_RD(ADDRESS)) == I2C_NOACK)
  {
    res = 4;
    goto END;
  }

  /* Read regs data */
  for (uint8_t i = 0; i < length; i++)
  {
    *pBuf = I2C_BUS_ReceiveByte(&I2C_Bus5, (i + 1) == length ? I2C_NOACK : I2C_ISACK);
    pBuf++;
  }
END:
  /* Send STOP Condition */
  I2C_BUS_SendStop(&I2C_Bus5);
  return res;
}

uint8_t NRF_I2C_Write(uint8_t reg, uint8_t *pBuf, uint8_t length)
{
  uint8_t res = 0;

  /* Send START condition */
  if (I2C_BUS_SendStart(&I2C_Bus5))
  {
    /* Bus bus busy*/
    res = 1;
  }

  /* Send slave device address */
  if (I2C_BUS_SendByte(&I2C_Bus5, I2C_WR(ADDRESS)) == I2C_NOACK)
  {
    res = 2;
    goto END;
  }

  /* Send the slave device reg address to write */
  if (I2C_BUS_SendByte(&I2C_Bus5, reg) == I2C_NOACK)
  {
    res = 3;
    goto END;
  }

  /* Send data bytes to write */
  for (uint8_t i = 0; i < length; i++)
  {
    if (I2C_BUS_SendByte(&I2C_Bus5, *pBuf) == I2C_NOACK)
    {
      res = 4;
      goto END;
    }
    pBuf++;
  }

END:
  /* Send STOP Condition */
  I2C_BUS_SendStop(&I2C_Bus5);
  return res;
}

uint8_t NRF_I2C_ReadByte(uint8_t reg, uint8_t *pBuf)
{
  return NRF_I2C_Read(reg, pBuf, 1);
}

uint8_t NRF_I2C_WriteByte(uint8_t reg, uint8_t pBuf)
{
  return NRF_I2C_Write(reg, &pBuf, 1);
}

// Wait until a flag indicates that registers are ready. Time out if it takes too long!
uint8_t NRF_flagready(uint8_t flag)
{
  uint8_t inbyte;
  int counter = MAXPOLLITER;

  while (counter-- > 0)
  {                                      // Only run permitted number of times
    if (NRF_I2C_ReadByte(flag, &inbyte)) // Read the flag through slave interface
      return BUSY;
    if (inbyte == FLAGREADY) // Flag is ready,
      return OKAY;           // all is well
    if (inbyte == Z1TIMEOUT) // 0x02 means internal Z1 timeout
      return TIMEOUT;        // Here if Z1 wasn't able to complete a transfer
  }
  return TIMEOUT; // Sorry, this function timed out!
}

// Write 0x01 to a flag in order to initiate a transfer etc.
uint8_t NRF_setflag(uint8_t flag)
{
  if (NRF_I2C_WriteByte(flag, 0x01))
    return BUSY;
  return OKAY;
}

// Check if z1 has a link, this is a simple binary test!
uint8_t NRF_haslink(void)
{
  uint8_t flag;
  NRF_I2C_ReadByte(LNKSTA, &flag);
  if (flag == LINKPRESENT) // Flag is ready,
    return 1;              // all is well
  else
    return 0;
}

// Reads a single byte from a Microchip 25AA640 or compatible SPI EEPROM on the ARX
uint8_t NRF_read_eeprom(uint8_t adrh, uint8_t adrl, uint8_t *data) {
	// Maybe rather use pointers for data and return TIMEOUT/OKAY?
	// How to read from external EEPROM:
	// Put command bytes into Z1 ATX's RXBUF write buffer, max 16 bytes
	// Update RXWCNT with the number of command bytes
	// Update RXRCNT with the number of bytes to be received, max 16 bytes
	// RXRCNT must be 1 if the EEPROM doesn't have auto increment RXRCNT
	// Execute the transfer by writing to the RXEXEC flag
	// Get the result back from RXBUF register

  if (NRF_flagready(RXEXEC) != OKAY)
    return BUSY;

  NRF_I2C_WriteByte(RXDCMD, 0x8c);  // Use ARX's master SPI interface at 250kbps
  NRF_I2C_WriteByte(RXBUF0, 0x03); // Buffer 0x03 for read command to 25AA640
  NRF_I2C_WriteByte(RXBUF1, adrh); // Buffer most significant address to 25AA640
  NRF_I2C_WriteByte(RXBUF2, adrl); // Buffer least significant address to 25AA640
  NRF_I2C_WriteByte(RXWCNT, 0x03);  // We are going to send 3 bytes to 25AA640
  NRF_I2C_WriteByte(RXRCNT, 0x01);  // We are going to read 1 byte from 25AA640
  NRF_setflag(RXEXEC);              // Execute the read/write transfer

  if (NRF_flagready(RXEXEC) != OKAY) // Must wait a while before the answer is ready
    return TIMEOUT;

  NRF_I2C_ReadByte(RXBUF0, data); // Return the one byte from the ATX's read buffer
  return OKAY;
}

// Write a single byte to a Microchip 25AA640 or compatible SPI EEPROM on the ARX
uint8_t NRF_write_eeprom(uint8_t adrh, uint8_t adrl, uint8_t data) {
	// How to write to external EEPROM:
	// Put command bytes into Z1 ATX's RXBUF write buffer, max 16 bytes
	// Update RXWCNT with the number of command bytes
	// Repeat for write enable command and then the actual write
	// Execute the transfer by writing to the RXEXEC flag

	if (NRF_flagready(RXEXEC)==OKAY) {
		NRF_I2C_WriteByte(RXDCMD, 0x8c);				// Use ARX's master SPI interface at 250kbps
		NRF_I2C_WriteByte(RXBUF0, 0x06);				// Buffer 0x06 for write enable command to 25AA640
		NRF_I2C_WriteByte(RXWCNT, 0x01);				// We are going to send 1 byte to 25AA640
		NRF_I2C_WriteByte(RXRCNT, 0x00);				// We are going to read 0 bytes from 25AA640
		NRF_setflag(RXEXEC);							    // Execute the write enable command
		
		if (NRF_flagready(RXEXEC)==OKAY) {		// If the write enable came through, go for the write
			NRF_I2C_WriteByte(RXBUF0, 0x02);			// Buffer 0x02 for write command to 25AA640
			NRF_I2C_WriteByte(RXBUF1, adrh);			// Buffer most significant address to 25AA640
			NRF_I2C_WriteByte(RXBUF2, adrl);			// Buffer least significant address to 25AA640
			NRF_I2C_WriteByte(RXBUF3, data);			// Buffer the data byte to 25AA640
			NRF_I2C_WriteByte(RXWCNT, 0x04);			// We are going to send 3 bytes to 25AA640
			NRF_I2C_WriteByte(RXRCNT, 0x00);			// We are going to read 0 bytes from 25AA640
			NRF_setflag(RXEXEC);						    // Execute the read/write transfer
			return OKAY;
		}
	}
	return TIMEOUT;
}

uint8_t NRF_init(uint8_t index)
{
  uint8_t rev;

  if (index == 1)
    ADDRESS = NRF_ADDRESS_SADR0;
  else if (index == 2)
    ADDRESS = NRF_ADDRESS_SADR1;
  else
    return 1;

  //检查通信状态以及软件版本
  if (NRF_I2C_ReadByte(REV, &rev) || rev != 0x20)
    return 2;

  // Channel
  for (uint8_t i = 0; i < 38; i++)
  {
    //From 2404MHz to 2478MHz, setp 2MHz
    NRF_I2C_WriteByte(CH0 + i, 4 + 2 * i);
  }
  NRF_I2C_WriteByte(NBCH, 9);//黑名单频道长度
  NRF_I2C_WriteByte(NACH, 0);//音频流传输默认频道
  NRF_I2C_WriteByte(NLCH, 13);//链接模式传输默认频道
  NRF_I2C_WriteByte(LNKWTH, 80);//链路警告阈值
  NRF_I2C_WriteByte(LNKETH, 160);//链路错误阈值
  NRF_I2C_WriteByte(ADDR0, index); //设备地址ID
  NRF_I2C_WriteByte(RXPWR, 3);//接收端射频功率0dBm
  NRF_I2C_WriteByte(TXPWR, 3);//发送端射频功率0dBm
  NRF_I2C_WriteByte(RXMOD, 0x20);//打开射频收发器
  NRF_I2C_WriteByte(TXFMT, 0);//16Bit音频数据
  NRF_I2C_WriteByte(TXLAT, 0);//音频延迟6ms
  NRF_I2C_WriteByte(INTCF, 0);//关闭所有中断
  NRF_I2C_WriteByte(I2SCNF_IN, 0);//I2S从机,16bit
  NRF_I2C_WriteByte(TXMOD, 0x80);//打开射频收发器,MCLK输出关
  NRF_I2C_WriteByte(LNKMOD, 0x10); //使能当前配置参数
  NRF_I2C_WriteByte(TXCSTATE, 1);  //把TX的I2S配置信息发送到RX
  //NRF_I2C_WriteByte(RXCSTATE, 1);  //把RX的I2S配置信息发送到TX
  return 0;
}

// Set up the DAC
uint8_t NRF_dac_init(void)
{
  extern uint8_t slaveoutbuf[SLAVEBUFSIZE]; // Global data buffer from MCU to Z1 slave
  extern uint8_t volume;
  int n; // Index to the data buffer

  uint8_t temp;

  if (volume == 0xFF)
    volume = 0x11; // If volume wasn't initiated, set it conveniently

  temp = NRF_flagready(RXEXEC);
  if (temp == OKAY)
  { // Is TX ready to convey data?
    // Set up a transfer from MCU through ATX and ARX to DAC
    slaveoutbuf[n = 0] = 0x02;         // Data for RXDCMD, 2-wire at RX, 2-wire start/stop, 100kb/s
    slaveoutbuf[++n] = 10;             // Data for RXWCNT, sending 10 bytes to DAC
    slaveoutbuf[++n] = 0;              // Data for RXRCNT, reading 0 bytes from DAC
    NRF_I2C_WriteByte(RXDCMD, RXRCNT); // Send configuration data
    slaveoutbuf[n = 0] = 0x20;         // Data for RXBUF_0, 2-w hardware adr. of MAX9850
    slaveoutbuf[++n] = 0x02;           // Data for RXBUF_1, internal address of setup register
    slaveoutbuf[++n] = volume;         // Data for RXBUF_2, MAX9850 address 0x02, "Volume"
    slaveoutbuf[++n] = 0x00;           // Data for RXBUF_3, MAX9850 address 0x03, "General Purpose"
    slaveoutbuf[++n] = 0x00;           // Data for RXBUF_4, MAX9850 address 0x04, "Interrupt Enable"
    slaveoutbuf[++n] = 0xfd;           // Data for RXBUF_5, MAX9850 address 0x05, "Enable"
    slaveoutbuf[++n] = 0x00;           // Data for RXBUF_6, MAX9850 address 0x06, "Clock"
    slaveoutbuf[++n] = 0x00;           // Data for RXBUF_7, MAX9850 address 0x07, "Charge Pump"
    slaveoutbuf[++n] = 0x80;           // Data for RXBUF_8, MAX9850 address 0x08, "LRCK MSB"
    slaveoutbuf[++n] = 0x10;           // Data for RXBUF_9, MAX9850 address 0x09, "LRCK LSB"
    NRF_I2C_WriteByte(RXBUF0, RXBUF9); // Move data through to DAC
    NRF_setflag(RXEXEC);               // Execute transfer to RX
  }
  return temp; // Return whatever NRF_flagready() said
}

// Transmit volume information
uint8_t NRF_dac_setvolume(void)
{
  extern uint8_t slaveoutbuf[SLAVEBUFSIZE]; // Global data buffer from MCU to Z1 slave
  extern uint8_t volume;
  uint8_t n; // Index to the data buffer
  uint8_t temp;

  temp = NRF_flagready(RXEXEC); // Is Z1 ready to push data to DAC?

  if (temp == OKAY)
  { // Is TX ready to convey data?
    // Set up a transfer from MCU through ATX and ARX to DAC
    slaveoutbuf[n = 0] = 0x02;         // Data for RXDCMD, 2-wire at RX, 2-wire start/stop, 100kb/s
    slaveoutbuf[++n] = 0x03;           // Data for RXWCNT, sending 3 bytes to DAC
    slaveoutbuf[++n] = 0x00;           // Data for RXRCNT, reading 0 bytes from DAC
    NRF_I2C_WriteByte(RXDCMD, RXRCNT); // Send configuration data

    slaveoutbuf[n = 0] = 0x20;         // Data for RXBUF_0, 2-w hardware adr. of MAX9850 FIX: define?
    slaveoutbuf[++n] = 0x02;           // Data for RXBUF_1, internal address of setup register
    slaveoutbuf[++n] = volume;         // Data for RXBUF_2, MAX9850 address 0x02, "Volume"
    NRF_I2C_WriteByte(RXBUF0, RXBUF2); // Move data through to DAC
    NRF_setflag(RXEXEC);               // Execute transfer to RX
  }
  return temp; // Return whatever NRF_flagready() said
}

// Increase volume
uint8_t NRF_dac_volume_up(void)
{
  extern uint8_t volume; // Global variable

  if (volume > 0) // On MAX9850, 0x00 is highest volume, 0x27 is lowest
    volume--;
  return (NRF_dac_setvolume()); // Write the new volume setting to DAC
}

// Decrease volume
uint8_t NRF_dac_volume_down(void)
{
  extern uint8_t volume; // Global variable

  if (volume < 0x27) // On MAX9850, 0x00 is highest volume, 0x27 is lowest
    volume++;
  return (NRF_dac_setvolume()); // Write the new volume setting to DAC
}