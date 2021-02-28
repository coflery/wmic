#include <stdbool.h>
#include <stddef.h>
#include "stm32f0xx.h"
#include "delay.h"
#include "fm.h"
#include "i2c.h"
#include "gpio.h"
#include "bitmap.h"

#define I2C_PORT I2C_PORT1
#define I2C_ADDRESS 0xC6
#define TIMEOUT ((uint8_t)20)

/* --------------------------------------------------------- */

//	TX commands
#define CMD_POWER_UP 0x01        // Power up device and mode selection. Modes include FM transmit and analog/digital audio interface configuration.
#define CMD_GET_REV 0x10         // Returns revision information on the device.
#define CMD_POWER_DOWN 0x11      // Power down device.
#define CMD_SET_PROPERTY 0x12    // Sets the value of a property.
#define CMD_GET_PROPERTY 0x13    // Retrieves a property's value.
#define CMD_GET_INT_STATUS 0x14  // Read interrupt status bits.
#define CMD_PATCH_ARGS 0x15      // Reserved command used for patch file downloads.
#define CMD_PATCH_DATA 0x16      // Reserved command used for patch file downloads.
#define CMD_GPIO_CTL 0x80        // Configures GPO1, 2, and 3 as output or Hi-Z.
#define CMD_GPIO_SET 0x81        // Sets GPO1, 2, and 3 output level (low or high).
#define CMD_TX_TUNE_FREQ 0x30    // Tunes to given transmit frequency.
#define CMD_TX_TUNE_POWER 0x31   // Sets the output power level and tunes the antenna capacitor.
#define CMD_TX_TUNE_MEASURE 0x32 // Measure the received noise level at the specified frequency.
#define CMD_TX_TUNE_STATUS 0x33  // Queries the status of a previously sent TX Tune Freq, TX Tune Power, or TX Tune Measure command.
#define CMD_TX_ASQ_STATUS 0x34   // Queries the TX status and input audio signal metrics.
#define CMD_TX_RDS_BUFF 0x35     // Queries the status of the RDS Group Buffer and loads new data into buffer.
#define CMD_TX_RDS_PS 0x36       // Set up default PS strings.

//	TX properties
#define PROP_GPO_IEN 0x0001                   //0x0000 Enables interrupt sources.
#define PROP_DIGITAL_INPUT_FORMAT 0x0101      //0x0000 Configures the digital input format.
#define PROP_DIGITAL_INPUT_SAMPLE_RATE 0x0103 //0x0000 Configures the digital input sample rate in 1 Hz steps. Default is 0.
#define PROP_REFCLK_FREQ 0x0201               //0x8000 Sets frequency of reference clock in Hz. The range is 31130 to 34406 Hz, or 0 to disable the AFC. Default is 32768 Hz.
#define PROP_REFCLK_PRESCALE 0x0202           //0x0001 Sets the prescaler value for RCLK input.
#define PROP_TX_COMPONENT_ENABLE 0x2100       //0x0003 Enable transmit multiplex signal components. Default has pilot and L-R enabled.
#define PROP_TX_AUDIO_DEVIATION 0x2101        //0x1AA9 Configures audio frequency deviation level. Units are in 10 Hz increments. Default is 6825 (68.25 kHz).
#define PROP_TX_PILOT_DEVIATION 0x2102        //0x02A3 Configures pilot tone frequency deviation level. Units are in 10 Hz increments. Default is 675 (6.75 kHz)
#define PROP_TX_RDS_DEVIATION 0x2103          //0x00C8 Si4721 Only. Configures the RDS/RBDS frequency deviation level. Units are in 10 Hz increments. Default is 2 kHz.
#define PROP_TX_LINE_INPUT_LEVEL 0x2104       //0x327C Configures maximum analog line input level to the LIN/RIN pins to reach the maximum deviation level programmed into the audio deviation property TX Audio Deviation. Default is 636 mVPK.
#define PROP_TX_LINE_INPUT_MUTE 0x2105        //0x0000 Sets line input mute. L and R inputs may be independently muted. Default is not muted.
#define PROP_TX_PREEMPHASIS 0x2106            //0x0000 Configures pre-emphasis time constant. Default is 0 (75 μS).
#define PROP_TX_PILOT_FREQUENCY 0x2107        //0x4A38 Configures the frequency of the stereo pilot. Default is 19000 Hz.
#define PROP_TX_ACOMP_ENABLE 0x2200           //0x0002 Enables audio dynamic range control and limiter. Default is 2 (limiter is enabled, audio dynamic range control is disabled).
#define PROP_TX_ACOMP_THRESHOLD 0x2201        //0xFFD8 Sets the threshold level for audio dynamic range control. Default is -40 dB.
#define PROP_TX_ACOMP_ATTACK_TIME 0x2202      //0x0000 Sets the attack time for audio dynamic range control. Default is 0 (0.5 ms).
#define PROP_TX_ACOMP_RELEASE_TIME 0x2203     //0x0004 Sets the release time for audio dynamic range control. Default is 4 (1000 ms).
#define PROP_TX_ACOMP_GAIN 0x2204             //0x000F Sets the gain for audio dynamic range control. Default is 15 dB.
#define PROP_TX_LIMITER_RELEASE_TIME 0x2205   //0x0066 Sets the limiter release time. Default is 102 (5.01 ms)
#define PROP_TX_ASQ_INTERRUPT_SOURCE 0x2300   //0x0000 Configures measurements related to signal quality metrics. Default is none selected.
#define PROP_TX_ASQ_LEVEL_LOW 0x2301          //0x0000 Configures low audio input level detection threshold. This threshold can be used to detect silence on the incoming audio.
#define PROP_TX_ASQ_DURATION_LOW 0x2302       //0x0000 Configures the duration which the input audio level must be below the low threshold in order to detect a low audio condition.
#define PROP_TX_ASQ_LEVEL_HIGH 0x2303         //0x0000 Configures high audio input level detection threshold. This threshold can be used to detect activity on the incoming audio.
#define PROP_TX_ASQ_DURATION_HIGH 0x2304      //0x0000 Configures the duration which the input audio level must be above the high threshold in order to detect a high audio condition.
#define PROP_TX_RDS_INTERRUPT_SOURCE 0x2C00   //0x0000 Si4721 Only. Configure RDS interrupt sources. Default is none selected.
#define PROP_TX_RDS_PI 0x2C01                 //0x40A7 Si4721 Only. Sets transmit RDS program identifier.
#define PROP_TX_RDS_PS_MIX 0x2C02             //0x0003 Si4721 Only. Configures mix of RDS PS Group with RDS Group Buffer.
#define PROP_TX_RDS_PS_MISC 0x2C03            //0x1008 Si4721 Only. Miscellaneous bits to transmit along with RDS_PS Groups.
#define PROP_TX_RDS_PS_REPEAT_COUNT 0x2C04    //0x0003 Si4721 Only. Number of times to repeat transmission of a PS message before transmitting the next PS message.
#define PROP_TX_RDS_PS_MESSAGE_COUNT 0x2C05   //0x0001 Si4721 Only. Number of PS messages in use.
#define PROP_TX_RDS_PS_AF 0x2C06              //0xE0E0 Si4721 Only. RDS Program Service Alternate Frequency. This provides the ability to inform the receiver of a single alternate frequency using AF Method A coding and is transmitted along with the RDS_PS Groups.
#define PROP_TX_RDS_FIFO_SIZE 0x2C07          //0x0000 Si4721 Only. Number of blocks reserved for the FIFO. Note that the value written must be one larger than the desired FIFO size.

//	TX configuration
//#define GPO_IEN			0x0000
#define F_CLK 3072000
#define REFCLK_FREQ 32000
#define REFCLK_PRESCALE (F_CLK / REFCLK_FREQ)
#define REFCLK_RCLKSEL 1                // clock source from DCLK
#define DIGITAL_INPUT_FORMAT 10         // Audio format I2S@24bits
#define DIGITAL_INPUT_SAMPLE_RATE 48000 // Audio sample rate
//#define TX_COMPONENT_ENABLE		0x0003
#define TX_AUDIO_DEVIATION 7500 // 75 KHz
//#define TX_PILOT_DEVIATION		0x02A3
//#define TX_RDS_DEVIATION		0x00C8
//#define TX_LINE_INPUT_LEVEL		0x327C
//#define TX_LINE_INPUT_MUTE		0x0000
#define TX_PREEMPHASIS 1 // 50 μs - Asia
//#define TX_PILOT_FREQUENCY		0x4A38

//#define TX_ACOMP_ENABLE		0x0003	// Example 1 (minimal compression):
//#define TX_ACOMP_THRESHOLD		0xFFD8	// TX_ACOMP_THRESHOLD = -40 dBFS
//#define TX_ACOMP_ATTACK_TIME		0x0009	// TX_ACOMP_ATTACK_TIME = 5 ms
//#define TX_ACOMP_RELEASE_TIME		0x0000	// TX_ACOMP_RELEASE_TIME = 100 ms
//#define TX_ACOMP_GAIN			0x000F	// TX_ACOMP_GAIN = 15 dB

#define TX_ACOMP_ENABLE 0x0003       // Example 2 (aggressive compression):
#define TX_ACOMP_THRESHOLD 0xFFF1    // TX_ACOMP_THRESHOLD = -15 dBFS
#define TX_ACOMP_ATTACK_TIME 0x0000  // TX_ACOMP_ATTACK_TIME = 0.5 ms
#define TX_ACOMP_RELEASE_TIME 0x0004 // TX_ACOMP_RELEASE_TIME = 1000 ms
#define TX_ACOMP_GAIN 0x0005         // TX_ACOMP_GAIN = 5 dB

//#define TX_LIMITER_RELEASE_TIME	0x0066
//#define TX_ASQ_INTERRUPT_SOURCE	0x0000
//#define TX_ASQ_LEVEL_LOW		0x0000
//#define TX_ASQ_DURATION_LOW		0x0000
//#define TX_ASQ_LEVEL_HIGH		0x0000
//#define TX_ASQ_DURATION_HIGH		0x0000
//#define TX_RDS_INTERRUPT_SOURCE	0x0000
//#define TX_RDS_PI			0x40A7
//#define TX_RDS_PS_MIX			0x0003
//#define TX_RDS_PS_MISC		0x1008
//#define TX_RDS_PS_REPEAT_COUNT	0x0003
//#define TX_RDS_PS_MESSAGE_COUNT	0x0001
//#define TX_RDS_PS_AF			0xE0E0
//#define TX_RDS_FIFO_SIZE		0x0000

/* --------------------------------------------------------- */

// si741x/2x power up
#define FUNC_RX 1         // 1 = Receive
#define FUNC_TX 2         // 2 = Transmit
#define OPMODE_ANALOG 80  // 01010000 = Analog audio inputs (LIN/RIN)
#define OPMODE_DIGITAL 15 // 00001111 = Digital audio inputs (DIN/DFS/DCLK)

/* --------------------------------------------------------- */

// misc
#define TX_POWER 120 // may be set as high as 120 dBμV
#define ANTCAP 0     // auto
#define GPIO_CTL 2   // (0 << 3) | (0 << 2) | (1 << 1)
#define INTACK 1

/* --------------------------------------------------------- */
typedef struct
{
  uint8_t Status;
  uint8_t PartNum;
  char FirewareRevMajor;
  char FirewareRevMinor;
  uint16_t PatchID;
  char ComponentRevMajor;
  char ComponentRevMinor;
  char DieRev;
} REV;

typedef struct
{
  uint8_t Status;
  uint8_t Reserved0;
  uint16_t Frequency;
  uint8_t Reserved1;
  uint8_t RFPower;
  uint8_t ANTCapacitor;
  uint8_t RXNoiseLevel;
} TXTUNE;

uint8_t get_status(uint8_t *status);
uint8_t wait_stc();
uint8_t wait_cts();
/* --------------------------------------------------------- */

uint8_t FM_I2C_Read(uint8_t *pBuf, uint8_t len)
{
  uint8_t res = 0;
  /* Send START condition */
  I2C_Start(I2C_PORT);

  /* Send slave device address */
  I2C_WriteByte(I2C_PORT, I2C_ADDRESS | 0x01);
  if (I2C_CheckAck(I2C_PORT) == NACK)
  {
    res = 1;
    goto END;
  }

  /* Read reg data */
  for (uint8_t i = 0; i < len; i++)
  {
    *pBuf = I2C_ReadByte(I2C_PORT);
    if (len == 1)
      I2C_NAck(I2C_PORT);
    else
      I2C_Ack(I2C_PORT);
    pBuf++;
  }

END:
  /* Send STOP Condition */
  I2C_Stop(I2C_PORT);
  return res;
}

uint8_t FM_I2C_Write(uint8_t cmd, uint8_t *pBuf, uint8_t len)
{
  uint8_t res = 0;

  /* Send START condition */
  I2C_Start(I2C_PORT);

  /* Send slave device address */
  I2C_WriteByte(I2C_PORT, I2C_ADDRESS & 0xFE);
  if (I2C_CheckAck(I2C_PORT) == NACK)
  {
    res = 1;
    goto END;
  }

  /* Send the write command */
  I2C_WriteByte(I2C_PORT, cmd);
  if (I2C_CheckAck(I2C_PORT) == NACK)
  {
    res = 2;
    goto END;
  }

  /* Send data bytes to write */
  for (uint8_t i = 0; i < len; i++)
  {
    I2C_WriteByte(I2C_PORT, *pBuf);
    if (I2C_CheckAck(I2C_PORT) == NACK)
    {
      res = 3;
      goto END;
    }
    pBuf++;
  }

END:
  /* Send STOP Condition */
  I2C_Stop(I2C_PORT);
  return res;
}

/* --------------------------------------------------------- */
uint8_t wait_stc()
{
  uint8_t res;
  uint8_t status;
  do
  {
    delay_ms(100);
    res = FM_I2C_Read(&status, sizeof(status));
  } while (res == 0 && !(status & B00000001));
  if (status & B01000000)
    return 2;
  return res;
}

//si47xx is responding else system reset
uint8_t wait_cts()
{
  uint8_t res;
  uint8_t status;
  do
  {
    delay_ms(100);
    res = FM_I2C_Read(&status, sizeof(status));
  } while (res == 0 && !(status & B10000000));
  if (status & B01000000)
    return 2;
  return res;
}
/* --------------------------------------------------------- */

uint8_t power_up()
{
  uint8_t res = 0;
  uint8_t buff[2];
  buff[0] = FUNC_TX;
  buff[1] = OPMODE_DIGITAL;
  res += FM_I2C_Write(CMD_POWER_UP, buff, sizeof(buff));
  res += wait_cts();
  return res;
}

uint8_t power_down()
{
  uint8_t res = 0;
  res += FM_I2C_Write(CMD_POWER_DOWN, NULL, 0);
  res += wait_cts();
  return res;
}

uint8_t set_property(uint16_t property, uint16_t value)
{
  uint8_t res = 0;
  uint8_t buff[5];
  buff[0] = 0;
  buff[1] = property >> 8;
  buff[2] = property;
  buff[3] = value >> 8;
  buff[4] = value;
  res += FM_I2C_Write(CMD_SET_PROPERTY, buff, sizeof(buff));
  res += wait_cts();
  return res;
}

uint8_t get_status(uint8_t *status)
{
  uint8_t res = 0;
  res += FM_I2C_Write(CMD_GET_INT_STATUS, NULL, 0);
  res += FM_I2C_Read(status, sizeof(status));
  return res;
}

uint8_t get_rev()
{
  REV rev;
  uint8_t res = 0;
  res += FM_I2C_Write(CMD_GET_REV, NULL, 0);
  res += FM_I2C_Read((uint8_t *)(&rev), 9);
  res += !(rev.Status & 0x80); //response check
  res += !(rev.PartNum == 20); //chipID check
  return res;
}

uint8_t gpio_ctl()
{
  uint8_t res = 0;
  uint8_t buff[1];
  buff[0] = GPIO_CTL;
  res += FM_I2C_Write(CMD_GPIO_CTL, buff, sizeof(buff));
  res += wait_cts();
  return res;
}

uint8_t tune_power()
{
  uint8_t res = 0;
  uint8_t buff[4];
  buff[0] = 0;
  buff[1] = 0;
  buff[2] = TX_POWER;
  buff[3] = ANTCAP;
  res = FM_I2C_Write(CMD_TX_TUNE_POWER, buff, sizeof(buff));
  wait_stc();
  return res;
}

uint8_t tune_freq(uint16_t freq)
{
  uint8_t res = 0;
  uint8_t buff[3];
  buff[0] = 0;
  buff[1] = freq >> 8;
  buff[2] = freq;
  res += FM_I2C_Write(CMD_TX_TUNE_FREQ, buff, sizeof(buff));
  wait_stc();
  return res;
}

uint8_t tune_measure(uint16_t freq)
{
  uint8_t res = 0;
  uint8_t buff[4];
  buff[0] = 0;
  buff[1] = freq >> 8;
  buff[2] = freq;
  buff[3] = ANTCAP;
  res += FM_I2C_Write(CMD_TX_TUNE_MEASURE, buff, sizeof(buff));
  wait_stc();
  return res;
}

uint8_t tune_status()
{
  TXTUNE status;
  uint8_t res = 0;
  uint8_t buff[1];
  buff[0] = INTACK;
  res += FM_I2C_Write(CMD_TX_TUNE_STATUS, buff, sizeof(buff));
  res += FM_I2C_Read((uint8_t *)(&status), 8);
  wait_cts();
  return res;
}

uint8_t configure()
{
  uint8_t res = 0;

#if defined(GPO_IEN) && GPO_IEN != 0x0000
  res += set_property(PROP_GPO_IEN, GPO_IEN); // default 0x0000
#endif
#if defined(REFCLK_FREQ) && REFCLK_FREQ != 0x8000
  res += set_property(PROP_REFCLK_FREQ, REFCLK_FREQ); // default 0x8000
#endif
#if defined(REFCLK_PRESCALE) && REFCLK_PRESCALE != 0x0001
  res += set_property(PROP_REFCLK_PRESCALE, (REFCLK_RCLKSEL) << 12 | REFCLK_PRESCALE); // default 0x0001
#endif
#if defined(DIGITAL_INPUT_FORMAT) && DIGITAL_INPUT_FORMAT != 0x0000
  res += set_property(PROP_DIGITAL_INPUT_FORMAT, DIGITAL_INPUT_FORMAT); // default 0x0000
#endif
#if defined(DIGITAL_INPUT_SAMPLE_RATE) && DIGITAL_INPUT_SAMPLE_RATE != 0x0000
  res += set_property(PROP_DIGITAL_INPUT_SAMPLE_RATE, DIGITAL_INPUT_SAMPLE_RATE); // default 0x0000
#endif
#if defined(TX_COMPONENT_ENABLE) && TX_COMPONENT_ENABLE != 0x0003
  res += set_property(PROP_TX_COMPONENT_ENABLE, TX_COMPONENT_ENABLE); // default 0x0003
#endif
#if defined(TX_AUDIO_DEVIATION) && TX_AUDIO_DEVIATION != 0x1AA9
  res += set_property(PROP_TX_AUDIO_DEVIATION, TX_AUDIO_DEVIATION); // default 0x1AA9
#endif
#if defined(TX_PILOT_DEVIATION) && TX_PILOT_DEVIATION != 0x02A3
  res += set_property(PROP_TX_PILOT_DEVIATION, TX_PILOT_DEVIATION); // default 0x02A3
#endif
#if defined(TX_RDS_DEVIATION) && TX_RDS_DEVIATION != 0x00C8
  res += set_property(PROP_TX_RDS_DEVIATION, TX_RDS_DEVIATION); // default 0x00C8
#endif
#if defined(TX_LINE_INPUT_LEVEL) && TX_LINE_INPUT_LEVEL != 0x327C
  res += set_property(PROP_TX_LINE_INPUT_LEVEL, TX_LINE_INPUT_LEVEL); // default 0x327C
#endif
#if defined(TX_LINE_INPUT_MUTE) && TX_LINE_INPUT_MUTE != 0x0000
  res += set_property(PROP_TX_LINE_INPUT_MUTE, TX_LINE_INPUT_MUTE); // default 0x0000
#endif
#if defined(TX_PREEMPHASIS) && TX_PREEMPHASIS != 0x0000
  res += set_property(PROP_TX_PREEMPHASIS, TX_PREEMPHASIS); // default 0x0000
#endif
#if defined(TX_PILOT_FREQUENCY) && TX_PILOT_FREQUENCY != 0x4A38
  res += set_property(PROP_TX_PILOT_FREQUENCY, TX_PILOT_FREQUENCY); // default 0x4A38
#endif
#if defined(TX_ACOMP_ENABLE) && TX_ACOMP_ENABLE != 0x0002
  res += set_property(PROP_TX_ACOMP_ENABLE, TX_ACOMP_ENABLE); // default 0x0002
#endif
#if defined(TX_ACOMP_THRESHOLD) && TX_ACOMP_THRESHOLD != 0xFFD8
  res += set_property(PROP_TX_ACOMP_THRESHOLD, TX_ACOMP_THRESHOLD); // default 0xFFD8
#endif
#if defined(TX_ACOMP_ATTACK_TIME) && TX_ACOMP_ATTACK_TIME != 0x0000
  res += set_property(PROP_TX_ACOMP_ATTACK_TIME, TX_ACOMP_ATTACK_TIME); // default 0x0000
#endif
#if defined(TX_ACOMP_RELEASE_TIME) && TX_ACOMP_RELEASE_TIME != 0x0004
  res += set_property(PROP_TX_ACOMP_RELEASE_TIME, TX_ACOMP_RELEASE_TIME); // default 0x0004
#endif
#if defined(TX_ACOMP_GAIN) && TX_ACOMP_GAIN != 0x000F
  res += set_property(PROP_TX_ACOMP_GAIN, TX_ACOMP_GAIN); // default 0x000F
#endif
#if defined(TX_LIMITER_RELEASE_TIME) && TX_LIMITER_RELEASE_TIME != 0x0066
  res += set_property(PROP_TX_LIMITER_RELEASE_TIME, TX_LIMITER_RELEASE_TIME); // default 0x0066
#endif
#if defined(TX_ASQ_INTERRUPT_SOURCE) && TX_ASQ_INTERRUPT_SOURCE != 0x0000
  res += set_property(PROP_TX_ASQ_INTERRUPT_SOURCE, TX_ASQ_INTERRUPT_SOURCE); // default 0x0000
#endif
#if defined(TX_ASQ_LEVEL_LOW) && TX_ASQ_LEVEL_LOW != 0x0000
  res += set_property(PROP_TX_ASQ_LEVEL_LOW, TX_ASQ_LEVEL_LOW); // default 0x0000
#endif
#if defined(TX_ASQ_DURATION_LOW) && TX_ASQ_DURATION_LOW != 0x0000
  res += set_property(PROP_TX_ASQ_DURATION_LOW, TX_ASQ_DURATION_LOW); // default 0x0000
#endif
#if defined(TX_ASQ_LEVEL_HIGH) && TX_ASQ_LEVEL_HIGH != 0x0000
  res += set_property(PROP_TX_ASQ_LEVEL_HIGH, TX_ASQ_LEVEL_HIGH); // default 0x0000
#endif
#if defined(TX_ASQ_DURATION_HIGH) && TX_ASQ_DURATION_HIGH != 0x0000
  res += set_property(PROP_TX_ASQ_DURATION_HIGH, TX_ASQ_DURATION_HIGH); // default 0x0000
#endif
#if defined(TX_RDS_INTERRUPT_SOURCE) && TX_RDS_INTERRUPT_SOURCE != 0x0000
  res += set_property(PROP_TX_RDS_INTERRUPT_SOURCE, TX_RDS_INTERRUPT_SOURCE); // default 0x0000
#endif
#if defined(TX_RDS_PI) && TX_RDS_PI != 0x40A7
  res += set_property(PROP_TX_RDS_PI, TX_RDS_PI); // default 0x40A7
#endif
#if defined(TX_RDS_PS_MIX) && TX_RDS_PS_MIX != 0x0003
  res += set_property(PROP_TX_RDS_PS_MIX, TX_RDS_PS_MIX); // default 0x0003
#endif
#if defined(TX_RDS_PS_MISC) && TX_RDS_PS_MISC != 0x1008
  res += set_property(PROP_TX_RDS_PS_MISC, TX_RDS_PS_MISC); // default 0x1008
#endif
#if defined(TX_RDS_PS_REPEAT_COUNT) && TX_RDS_PS_REPEAT_COUNT != 0x0003
  res += set_property(PROP_TX_RDS_PS_REPEAT_COUNT, TX_RDS_PS_REPEAT_COUNT); // default 0x0003
#endif
#if defined(TX_RDS_PS_MESSAGE_COUNT) && TX_RDS_PS_MESSAGE_COUNT != 0x0001
  res += set_property(PROP_TX_RDS_PS_MESSAGE_COUNT, TX_RDS_PS_MESSAGE_COUNT); // default 0x0001
#endif
#if defined(TX_RDS_PS_AF) && TX_RDS_PS_AF != 0xE0E0
  res += set_property(PROP_TX_RDS_PS_AF, TX_RDS_PS_AF); // default 0xE0E0
#endif
#if defined(TX_RDS_FIFO_SIZE) && TX_RDS_FIFO_SIZE != 0x0000
  res += set_property(PROP_TX_RDS_FIFO_SIZE, TX_RDS_FIFO_SIZE); // default 0x0000
#endif
  return res;
}

uint8_t fm_init()
{
  uint8_t res = 0;
  res += power_down();
  res += power_up();
  res += get_rev();
  res += gpio_ctl();
  res += configure();
  res += tune_freq(10300);
  res += tune_power();
  res += tune_status();
  return res;
}
