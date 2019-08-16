/*
    (C)2014 Gemtek

Description: Driver for the PCA9539 IO expander

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Hanson Chen
*/
#ifndef __PCA9539_H__
#define __PCA9539_H__

#include <stdbool.h>

#define PCA9539_I2C_ADDRESS                          0x74		

/*!
 * PCA9539 registers addresses
 */
 
#define REG_IN_PORT_0																	0x00
#define REG_IN_PORT_1																	0x01
#define REG_OUT_PORT_0																0x02
#define REG_OUT_PORT_1																0x03
#define REG_POL_INV_0																	0x04
#define REG_POL_INV_1																	0x05
#define REG_CONFIG_0																	0x06
#define REG_CONFIG_1																	0x07

#define REG_RESET_VAL																	0xFF

#define REG_MASK_BIT_0																0x01
#define REG_MASK_BIT_1	 															0x02
#define REG_MASK_BIT_2	 															0x04
#define REG_MASK_BIT_3	 															0x08
#define REG_MASK_BIT_4	 															0x10
#define REG_MASK_BIT_5	 															0x20
#define REG_MASK_BIT_6	 															0x40
#define REG_MASK_BIT_7	 															0x80

#define GPIO_INT_STA_SIZE															2
#define GPIO_CONF_STA_SIZE														GPIO_INT_STA_SIZE

//extern uint8_t GpioIntStatus[GPIO_INT_STA_SIZE];

typedef enum
{
		REG_IDX_INT_STA_0 = 0,
		REG_IDX_INT_STA_1,
	
}GpioIntStaRegIdx;

typedef enum
{
		ISG0 = 0, ISG1, ISG2, ISG3, ISG4, ISG5, ISG6, ISG7, 	\
		ISG8, ISG9, ISG10, ISG11, ISG12, ISG13, ISG14, ISG15,	\
	
}GpioIntStaPinIdx;


typedef enum
{
		PCA9539IOLOW = 0,
		PCA9539IOHIGH
	
}PCA9539IOStatus;

typedef enum
{
		PCA9539IOINPUT = 0,
		PCA9539IOOUTPUT
}PCA9539DIRStatus;

typedef enum
{
		PCA9539IORISING = 0,
		PCA9539IOFALLING
}PCA9539IRQMode;


typedef void( PCA9539IrqHandler )(uint32_t status );

void PCA9539SetCallback(PCA9539IrqHandler *irqHandler );

/*!
 * \brief Initializes the device
 */
void PCA9539Init( void );

/*!
 * \brief Resets the device
 *
 * \retval status [SUCCESS, FAIL]
 */
void PCA9539Reset( void );

//void PCA9539SetInterrupt( void );

/*!
 * \brief Writes a byte at specified address in the device
 *
 * \param [IN]: addr
 * \param [IN]: data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t PCA9539Write( uint8_t addr, uint8_t data );

/*!
 * \brief Writes a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [IN]: data
 * \param [IN]: size
 * \retval status [SUCCESS, FAIL]
 */
uint8_t PCA9539WriteBuffer( uint8_t addr, uint8_t *data, uint8_t size );

/*!
 * \brief Reads a byte at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t PCA9539Read( uint8_t addr, uint8_t *data );

/*!
 * \brief Reads a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \param [IN]: size
 * \retval status [SUCCESS, FAIL]
 */
uint8_t PCA9539ReadBuffer( uint8_t addr, uint8_t *data, uint8_t size );

/*!
 * \brief Sets the I2C device slave address
 *
 * \param [IN]: addr
 */
void PCA9539SetDeviceAddr( uint8_t addr );

/*!
 * \brief Gets the I2C device slave address
 *
 * \retval: addr Current device slave address
 */
uint8_t PCA9539GetDeviceAddr( void );

void PCA9539SetDIRStatus(GpioIntStaPinIdx pin, PCA9539DIRStatus dir, PCA9539IOStatus status);
void PCA9539SetIOStatus(GpioIntStaPinIdx pin, PCA9539IOStatus status);
PCA9539IOStatus PCA9539GetIOStatus(GpioIntStaPinIdx pin);
void PCA9539SetInterrupt( GpioIntStaPinIdx pin, PCA9539IRQMode irqMode );
bool PCA9539CheckInterruptStatus( GpioIntStaPinIdx pin, uint32_t *GpioIntStatus );
void PCA9539InterruptSignal( void );

#endif  // __PCA9539_H__
