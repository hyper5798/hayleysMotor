#ifndef __EEPROM_BOARD_H__
#define __EEPROM_BOARD_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

void InternalEepromReadBytes(uint32_t EepromAddr, uint8_t *Buffer, uint16_t Length);
uint8_t InternalEepromWriteBuffer( uint32_t EepromAddrStart, uint8_t *Buffer, uint16_t BufferSize );
uint8_t InternalEepromWriteBufferWord( uint32_t EepromAddrStart, uint8_t *Buffer, uint16_t BufferSize );
#ifdef __cplusplus
}
#endif
#endif /*__EEPROM_BOARD_H__*/
