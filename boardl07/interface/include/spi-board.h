#ifndef __SPI_BOARD_H__
#define __SPI_BOARD_H__

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>

typedef enum
{
	SPIInterface1 = 0x0,
    SPIInterface2, 
    SPIInterface3,
}SPI_Number;
void SpiInit(SPI_Number spi);
void SpiDeInit(void);
uint16_t SpiInOut(uint16_t outData );
void SpiWriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );
void SpiReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size );
//Jason add for SPI1 at 2019.08.19
void Spi1WriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size, uint8_t nssSwitch );
void Spi1ReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size, uint8_t nssSwitch );
#ifdef __cplusplus
}
#endif
#endif /*__SPI_BOARD_H__*/
