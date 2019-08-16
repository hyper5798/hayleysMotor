#ifndef __I2C_BOARD_H__
#define __I2C_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif
    
#include <stdint.h>
#include <stdbool.h>
#include "board.h"
#include "stm32l0xx.h"


	/*!
 * Operation Mode for the I2C
 */
typedef enum
{
    MODE_I2C = 0,
    MODE_SMBUS_DEVICE,
    MODE_SMBUS_HOST
}I2cMode;

/*!
 * I2C signal duty cycle
 */
typedef enum
{
    I2C_DUTY_CYCLE_2 = 0,
    I2C_DUTY_CYCLE_16_9
}I2cDutyCycle;

/*!
 * I2C select if the acknowledge in after the 7th or 10th bit
 */
typedef enum
{
    I2C_ACK_ADD_7_BIT = 0,
    I2C_ACK_ADD_10_BIT
}I2cAckAddrMode;

/*!
 * Internal device address size
 */
typedef enum
{
    I2C_ADDR_SIZE_8 = 0,
    I2C_ADDR_SIZE_16,
}I2cAddrSize;
    
void I2C1Init(void);
void I2C1DeInit(void);
void I2cSetAddrSize(I2cAddrSize addrSize );
uint8_t I2cWriteBuffer( uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size );
uint8_t I2cWrite(uint8_t deviceAddr, uint16_t addr, uint8_t data );
uint8_t I2cReadBuffer(uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size );
uint8_t I2cRead(uint8_t deviceAddr, uint16_t addr, uint8_t *data );
void I2cResetBus(void );
uint8_t I2cWaitStandbyState(uint8_t deviceAddr );
#ifdef __cplusplus
}
#endif
#endif /*__I2C_BOARD_H__*/
