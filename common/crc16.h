#ifndef __CRC16_H__
#define __CRC16_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint32_t crc16(unsigned char *puchMsg, uint16_t DataLen);
uint16_t crcStd16(const uint8_t *inputKey, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* __CRC16_H__ */

