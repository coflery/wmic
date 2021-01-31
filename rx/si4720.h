///
/// \file SI4721.h
/// \brief Library header file for the radio library to control the SI4721 radio chip.
///
/// \author N Poole, nickpoole.me
/// \author Matthias Hertel, http://www.mathertel.de
/// \copyright Copyright (c) 2014 by Matthias Hertel.\n
/// This work is licensed under a BSD style license.\n
/// See http://www.mathertel.de/License.aspx
///
/// This library enables the use of the Radio Chip SI4721.
///
/// More documentation and source code is available at http://www.mathertel.de/Arduino
///
/// ChangeLog:
/// ----------
/// * 01.12.2019 created.


#ifndef SI4721_h
#define SI4721_h

#define SI4721_ADR 0x63  ///< The I2C address of SI4721 is 0x61 or 0x63

// #include <Arduino.h>

// // The wire library is used for the communication with the radio chip.
// #include <Wire.h>

// // Include the radio library that is extended by the SI4721 library.
// #include <radio.h>

// A structure for storing ASQ Status and Audio Input Metrics
typedef struct ASQ_STATUS {
  uint8_t asq;
  uint8_t audioInLevel;
};

// A structure for storing TX Tuning Status
typedef struct TX_STATUS {
  uint16_t frequency;
  uint8_t dBuV;
  uint8_t antennaCap;
  uint8_t noiseLevel;
};

// ----- library definition -----

/// Library to control the SI4721 radio chip.
class SI4721 : public RADIO {
public:
  const uint8_t MAXVOLUME = 15;   ///< max volume level for radio implementations.
  const uint8_t MAXVOLUMEX = 63;  ///< max volume level for the SI4721 specific implementation.

  SI4721();

  bool   init(TwoWire &wirePort = Wire, uint8_t deviceAddress = SI4721_ADR);  ///< Initialize the library and the chip.
  void   term();  ///< Terminate all radio functions in the chip.

  // ----- Audio functions -----

  void    setVolume(uint8_t newVolume);   ///< Control the volume output of the radio chip in the range 0..15.

  void    setVolumeX(uint8_t newVolume);  ///< Control the volume output of the radio chip in the range 0..63.
  uint8_t getVolumeX();                   ///< Retrieve the current output volume in the range 0..63.

  void    setMute(bool switchOn);         ///< Control the mute mode of the radio chip.
  void    setSoftMute(bool switchOn);     ///< Control the softmute mode (mute on low signals) of the radio chip.

  // Overwrite audio functions that are not supported.
  void    setBassBoost(bool switchOn);    ///< regardless of the given parameter, the Bass Boost will never switch on.

  // ----- Radio receiver functions -----

  void    setMono(bool switchOn);         ///< Control the mono/stereo mode of the radio chip.

  void    setBand(RADIO_BAND newBand);    ///< Control the band of the radio chip.

  void    setFrequency(RADIO_FREQ newF);  ///< Control the frequency.
  RADIO_FREQ getFrequency(void);

  void seekUp(bool toNextSender = true);   // start seek mode upwards
  void seekDown(bool toNextSender = true); // start seek mode downwards

  void checkRDS(); // read RDS data from the current station and process when data available.

  void getRadioInfo(RADIO_INFO *info);
  void getAudioInfo(AUDIO_INFO *info);

  // ----- debug Helpers send information to Serial port

  void  debugScan();               // Scan all frequencies and report a status
  void  debugStatus();             // Report Info about actual Station
  
  // ----- transmit functions
  
  void setModeReceive();
  void setModeTransmit();
  void beginRDS(uint16_t programID = 0xBEEF);
  void setRDSstation(char *s);
  void setRDSbuffer(char *s);  
  void setTXpower(uint8_t pwr);
  
  ASQ_STATUS getASQ();
  TX_STATUS getTuneStatus();
  
  // ----- regional compatibility
  
  void setDeemphasis(uint8_t uS); // set the deemphasis (50 for Europe, 75 for USA)

private:
  // ----- local variables

  uint8_t _realVolume; ///< The real volume set to the chip.
  
  bool _transmitMode = false; ///< Remember which mode we're in 
  
  uint8_t _fmDeemphasis = 50; ///< RX Deemphasis and TX Preemphasis in uS

  // store the current status values
  uint8_t _status;        ///< the status after sending a command

  uint8_t tuneStatus[8];
  uint8_t rsqStatus[1 + 7];
  uint8_t rdsStatusx[1 + 12];
  uint8_t agcStatus[1 + 2];

  /// structure used to read status information from the SI4721 radio chip.
  union {
    // use structured access 
    struct {
      uint8_t  status;
      uint8_t  resp1;
      uint8_t  resp2;
      uint8_t  rdsFifoUsed;
      uint8_t  blockAH; uint8_t  blockAL;
      uint8_t  blockBH; uint8_t  blockBL;
      uint8_t  blockCH; uint8_t  blockCL;
      uint8_t  blockDH; uint8_t  blockDL;
      uint8_t  blockErrors;
    };
    // use the the byte while receiving and sending.
    uint8_t buffer[1 + 7];
  } tuneStatus2; // union RDSSTATUS


  /// structure used to read RDS information from the SI4721 radio chip.
  union {
    // use structured access 
    struct {
      uint8_t  status;
      uint8_t  resp1;
      uint8_t  resp2;
      uint8_t  rdsFifoUsed;
      uint8_t  blockAH; uint8_t  blockAL;
      uint8_t  blockBH; uint8_t  blockBL;
      uint8_t  blockCH; uint8_t  blockCL;
      uint8_t  blockDH; uint8_t  blockDL;
      uint8_t  blockErrors;
    };
    // use the the byte while receiving and sending.
    uint8_t buffer[1 + 12];
  } rdsStatus; // union RDSSTATUS


  // ----- low level communication to the chip using I2C bus

  /// send a command
  void _sendCommand(int cnt, int cmd, ...);

  /// set a property
  void _setProperty(uint16_t prop, uint16_t value);

  /// read the interrupt status.
  uint8_t _readStatus();

  /// read status information into a buffer
  void _readStatusData(uint8_t cmd, uint8_t param, uint8_t *values, uint8_t len);

  void _seek(bool seekUp = true);
  void _waitEnd();
  
  TwoWire *_i2cPort;
  uint8_t _i2caddr;
  
};

#endif
