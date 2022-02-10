#ifndef _NRF_H
#define _NRF_H

// Primitive I/O operations
#define MAXPOLLITER 1000 // Flag poll cycles before timeout is declared
#define FLAGREADY   0x00 // 0x00 indicates that a flag is ready
#define Z1TIMEOUT   0x02 // 0x02 indicates something else... timeout? FIX!
#define OKAY        0x00 // Error/result reports that all is well
#define TIMEOUT     0xFF // Error/result reports that something timed out
#define BUSY        0xFE // Error/result reports that busy

uint8_t NRF_init(uint8_t index);
uint8_t NRF_flagready(uint8_t flag);
uint8_t NRF_setflag(uint8_t flag);

// Link and address operations
#define LINKPRESENT 0x01 // A link is there!
uint8_t NRF_haslink(void);
void NRF_force_relink(void);
void NRF_setprivateadr(void);
void NRF_setinitialadr(void);
uint8_t NRF_randombyte(void);

// ARX EEPROM access
uint8_t NRF_read_eeprom(uint8_t adrh, uint8_t adrl, uint8_t *data);
uint8_t NRF_write_eeprom(uint8_t adrh, uint8_t adrl, uint8_t data);


#endif