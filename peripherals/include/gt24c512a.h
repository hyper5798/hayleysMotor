#ifndef __GT24C512A_H__
#define __GT24C512A_H__
#include <stdint.h>
uint8_t GTEepromWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size );
uint8_t GTEepromReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size );
void GTEepromSetDeviceAddr( uint8_t addr );
uint8_t GTEepromGetDeviceAddr( void );


#endif // __GT24C512A_H__
