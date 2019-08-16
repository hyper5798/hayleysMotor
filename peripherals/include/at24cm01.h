#ifndef __AT24CM01_H__
#define __AT24CM01_H__
#include <stdint.h>
uint8_t AT24CM01EepromWriteBuffer( uint32_t addr, uint8_t *buffer, uint32_t size );
uint8_t AT24CM01EepromReadBuffer( uint32_t addr, uint8_t *buffer, uint32_t size );
void AT24CM01EepromSetDeviceAddr( uint8_t addr );
uint8_t AT24CM01EepromGetDeviceAddr( void );


#endif // __AT24CM01_H__
