/*
    (C)2014 Gemtek

Description: Driver for the STMPE1801 IO expander

Maintainer: Hanson Chen
*/

#ifndef __STMPE1801_H__
#define __STMPE1801_H__
#include <stdint.h>
#include <stdbool.h>
//extern uint8_t MpLow;
//extern uint8_t MpMid;
//extern uint8_t MpHigh;

//extern uint8_t DirLow;
//extern uint8_t DirMid;
//extern uint8_t DirHigh;

//extern uint8_t ReLow;
//extern uint8_t ReMid;
//extern uint8_t ReHigh;

//extern uint8_t FeLow;
//extern uint8_t FeMid;
//extern uint8_t FeHigh;

//extern uint8_t PuLow;
//extern uint8_t PuMid;
//extern uint8_t PuHigh;

//extern uint8_t IntCtrlLow;
//extern uint8_t IntEnMaskLow;
//extern uint8_t IntEnGpioMaskLow;
//extern uint8_t IntEnGpioMaskMid;
//extern uint8_t IntEnGpioMaskHigh;

#define STMPE1801_I2C_ADDRESS                         0x40

#define CHIP_ID																				0x00
#define VERSION_ID																		0x01
#define SYS_CTRL																			0x02

#define INT_CTRL_LOW																	0x04
#define INT_CTRL_HIGH																	0x05
#define INT_EN_MASK_LOW																0x06
#define INT_EN_MASK_HIGH															0x07
#define INT_STA_LOW																		0x08
#define INT_STA_HIGH																	0x09
#define INT_EN_GPIO_MASK_LOW													0x0A
#define INT_EN_GPIO_MASK_MID													0x0B
#define INT_EN_GPIO_MASK_HIGH													0x0C
#define INT_STA_GPIO_LOW															0x0D
#define INT_STA_GPIO_MID															0x0E
#define INT_STA_GPIO_HIGH															0x0F
#define GPIO_SET_LOW																	0x10
#define GPIO_SET_MID																	0x11
#define GPIO_SET_HIGH																	0x12
#define GPIO_CLR_LOW																	0x13
#define GPIO_CLR_MID																	0x14
#define GPIO_CLR_HIGH																	0x15
#define GPIO_MP_LOW																		0x16
#define GPIO_MP_MID																		0x17
#define GPIO_MP_HIGH																	0x18
#define GPIO_SET_DIR_LOW															0x19
#define GPIO_SET_DIR_MID															0x1A
#define GPIO_SET_DIR_HIGH															0x1B
#define GPIO_RE_LOW																		0x1C
#define GPIO_RE_MID																		0x1D
#define GPIO_RE_HIGH																	0x1E
#define GPIO_FE_LOW																		0x1F
#define GPIO_FE_MID																		0x20
#define GPIO_FE_HIGH																	0x21
#define GPIO_PULL_UP_LOW															0x22
#define GPIO_PULL_UP_MID															0x23
#define GPIO_PULL_UP_HIGH															0x24

#define KPC_ROW																				0x30
#define KPC_COL_LOW																		0x31
#define KPC_COL_HIGH																	0x32
#define KPC_CTRL_LOW																	0x33
#define KPC_CTRL_MID																	0x34
#define KPC_CTRL_HIGH																	0x35
#define KPC_CMD																				0x36
#define KPC_COMB_KEY_0																0x37
#define KPC_COMB_KEY_1																0x38
#define KPC_COMB_KEY_2																0x39
#define KPC_DATA_BYTE0																0x3A
#define KPC_DATA_BYTE1																0x3B
#define KPC_DATA_BYTE2																0x3C
#define KPC_DATA_BYTE3																0x3D
#define KPC_DATA_BYTE4																0x3E

#define REG_INT_STA_IS0_MASK													0x01
#define REG_INT_STA_IS1_MASK													0x02
#define REG_INT_STA_IS2_MASK													0x04
#define REG_INT_STA_IS3_MASK													0x08
#define REG_INT_STA_IS4_MASK													0x10

#define STMPE1801_RST_MASK													0x80

//#define REG_MASK_BIT_0																0x01
//#define REG_MASK_BIT_1	 															0x02
//#define REG_MASK_BIT_2	 															0x04
//#define REG_MASK_BIT_3	 															0x08
//#define REG_MASK_BIT_4	 															0x10
//#define REG_MASK_BIT_5	 															0x20
//#define REG_MASK_BIT_6	 															0x40
//#define REG_MASK_BIT_7	 															0x80

#define GPIO_INT_STA_SIZE															3

typedef enum
{
		ISG0 = 0, ISG1, ISG2, ISG3, ISG4, ISG5, ISG6, ISG7, 	\
		ISG8, ISG9, ISG10, ISG11, ISG12, ISG13, ISG14, ISG15,	\
		ISG16, ISG17,
	
}GpioIntStaPinIdx;

typedef enum
{
		REG_IDX_INT_STA_GPIO_LOW = 0,
		REG_IDX_INT_STA_GPIO_MID,
		REG_IDX_INT_STA_GPIO_HIGH
	
}GpioIntStaRegIdx;

typedef enum
{
		STMPE1801IOLOW = 0,
		STMPE1801IOHIGH
	
}STMPE1801IOStatus;

typedef enum
{
		STMPE1801IOINPUT = 0,
		STMPE1801IOOUTPUT
}STMPE1801DIRStatus;

typedef enum
{
		STMPE1801IORISING = 0,
		STMPE1801IOFALLING
}STMPE1801IRQMode;

//extern uint8_t GpioIntStatus[GPIO_INT_STA_SIZE];
typedef void( STMPE1801IrqHandler )(uint32_t status );

void STMPE1801SetCallback(STMPE1801IrqHandler *irqHandler );

/*!
 * \brief Initializes the device
 */
void STMPE1801Init( void);


/*!
 * \brief Resets the device
 *
 */
void STMPE1801Reset( void );


void STMPE1801SetInitInterrupt( void );


/*!
 * \brief Writes a byte at specified address in the device
 *
 * \param [IN]: addr
 * \param [IN]: data
 * \retval status [OK, FAIL]
 */
uint8_t STMPE1801Write( uint8_t addr, uint8_t data );

/*!
 * \brief Writes a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [IN]: data
 * \param [IN]: size
 * \retval status [OK, FAIL]
 */
uint8_t STMPE1801WriteBuffer( uint8_t addr, uint8_t *data, uint8_t size );

/*!
 * \brief Reads a byte at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \retval status [OK, FAIL]
 */
uint8_t STMPE1801Read( uint8_t addr, uint8_t *data );

/*!
 * \brief Reads a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \param [IN]: size
 * \retval status [OK, FAIL]
 */
uint8_t STMPE1801ReadBuffer( uint8_t addr, uint8_t *data, uint8_t size );


/*!
 * \brief Sets the I2C device slave address
 *
 * \param [IN]: addr
 */
void STMPE1801SetDeviceAddr( uint8_t addr );

/*!
 * \brief Gets the I2C device slave address
 *
 * \retval: addr Current device slave address
 */
uint8_t STMPE1801GetDeviceAddr( void );

void ReadSTMPE1801Status( void );

void STMPE1801SetIOStatus(GpioIntStaPinIdx pin, STMPE1801IOStatus status);
void STMPE1801SetDIRStatus(GpioIntStaPinIdx pin, STMPE1801DIRStatus dir, STMPE1801IOStatus status);
STMPE1801IOStatus STMPE1801GetIOStatus(GpioIntStaPinIdx pin);
void STMPE1801SetInterrupt( GpioIntStaPinIdx pin, STMPE1801IRQMode irqMode );
bool STMPE1801CheckInterruptStatus( GpioIntStaPinIdx pin, uint32_t *GpioIntStatus );
void STMPE1801InterruptSignal( void );

#endif  // __STMPE1801_H__
