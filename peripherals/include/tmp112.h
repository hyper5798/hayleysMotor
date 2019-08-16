/*
    (C)2014 Gemtek

Description: Driver for the TMP112 Temperature Sensor

Maintainer: Hanson Chen
*/


#ifndef __TMP112_H__
#define __TMP112_H__

#include <stdint.h>
#include <stdbool.h>
#include "gpio-board.h"
#include "i2c-board.h"

#define TMP112_REG_MSB															0
#define TMP112_REG_LSB															1

#define EM_DISABLED_SHIFT														4
#define MSB_CHECK_SHIFT															7
#define MSBYTE_SHIFT																8

#define TMP112_RES_UNIT															( (float) 0.0625 )
    
#define TMP112_SD_SHIFT                                                         0
#define TMP112_TM_SHIFT															1
#define TMP112_POL_SHIFT														2
#define TMP112_OS_SHIFT                                                         7

#define TMP112_SD_MASK                                                          0x01
#define TMP112_TM_MASK                                                          0x02
#define TMP112_OS_MASK                                                          0x80

/*
 * TMP112 I2C address
 */ 
#define TMP112_I2C_ADDRESS                          0x49

/*
 * TMP112 Registers
 */ 

#define TEMP_REG																		0x00
#define CONFIG_REG																	0x01
#define	LOW_LIMIT_REG																0x02
#define HIGH_LIMIT_REG															0x03

typedef enum
{
    TMP112COMPARATOR_MODE = 0,
    TMP112INTERRUPT_MODE
}TMP112ThermostatMode_e;

typedef enum
{
    TMP112POLACTIVELOW = 0,
	TMP112POLACTIVEHIGH
}TMP112Polarity_e;

typedef enum
{
    TMP112LIMITLOW = 0,
	TMP112LIMITHIGH
}TMP112Limit_e;

/*!
 * \brief Initializes the device
 *
 * \retval status [SUCCESS, FAIL]
 */
uint8_t TMP112Init( void );

/*!
 * \brief Resets the device
 *
 * \retval status [SUCCESS, FAIL]
 */
uint8_t TMP112Reset( void );


/*!
 * \brief Settings for TMP112's interrupt
 *
 * \retval status [SUCCESS, FAIL]
 */
uint8_t TMP112SetInterrupt( TMP112Polarity_e ActiveState, TMP112ThermostatMode_e TmMode );

/*!
 * \brief Writes a byte at specified address in the device
 *
 * \param [IN]:    addr
 * \param [IN]:    data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t TMP112Write( uint8_t addr, uint8_t data );

/*!
 * \brief Writes a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [IN]: data
 * \param [IN]: size
 * \retval status [SUCCESS, FAIL]
 */
uint8_t TMP112WriteBuffer( uint8_t addr, uint8_t *data, uint8_t size );

/*!
 * \brief Reads a byte at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t TMP112Read( uint8_t addr, uint8_t *data );

/*!
 * \brief Reads a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \param [IN]: size
 * \retval status [SUCCESS, FAIL]
 */
uint8_t TMP112ReadBuffer( uint8_t addr, uint8_t *data, uint8_t size );

/*!
 * \brief Sets the I2C device slave address
 *
 * \param [IN]: addr
 */
void TMP112SetDeviceAddr( uint8_t addr );

/*!
 * \brief Gets the I2C device slave address
 *
 * \retval: addr Current device slave address
 */
uint8_t TMP112GetDeviceAddr( void );


/*!
 * \brief Reads the Temperature from the TMP112
 *
 * \retval temperature Measured temperature
 */
float TMP112ReadTemperature( void );

/*!
 * \brief Reads the Temperature from the TMP112 when t-sensor shutdown
 *
 * \retval temperature Measured temperature
 */
uint8_t TMP112OneShot( void );


/*!
 * \brief Test the Value from the conversion of TMP112
 *
 * \retval temperature Measured temperature
 */
float TMP112ReadTemperatureForTest( uint8_t *TestTempVal );

uint8_t TMP112LimitTemperature(TMP112Limit_e limit, float temp);

/*!
 * \brief Enable/Disable TMP112
 *
 * \param [IN]: 1 is shutdown, 0 is not shutdown
 */
uint8_t TMP112Enable(uint8_t enable);

#endif  // __TMP112_H__
