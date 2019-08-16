/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: BOSCH G-Sensor BMA250E driver implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: 
*/
#include <math.h>
#include "board.h"
#include "i2c-board.h"
#include "bma250e.h"
#include <stddef.h>

static uint8_t I2cDeviceAddr = 0;

static bool BMA250Initialized = false;

static uint8_t BMA250E_RESOLUTION = BMA2x2_10_RESOLUTION;

static BMA250EIrqHandler *BMA250ECallback1 = NULL;
static BMA250EIrqHandler *BMA250ECallback2 = NULL;

void BMA250EInterruptSignal1( void )
{
    if(BMA250ECallback1 != NULL)
        BMA250ECallback1();
}

void BMA250EInterruptSignal2( void )
{
    if(BMA250ECallback2 != NULL)
        BMA250ECallback2();
}

void BMA250ESetCallback(BMA250EIrqHandler *irqHandler1, BMA250EIrqHandler *irqHandler2 )
{
    BMA250ECallback1 = irqHandler1;
    BMA250ECallback2 = irqHandler2;
}

uint8_t BMA250EReset( )
{
    if( BMA250E_I2C_Write( BMA2x2_RST_REG, BMA2x2_ENABLE_SOFT_RESET_VALUE, 1 ) == SUCCESS )
    {
        return SUCCESS;
    }
    return FAIL;
}

void BMA250ESetDeviceAddr( uint8_t addr )
{
    	I2cDeviceAddr = addr;
}

uint8_t BMA250EGetDeviceAddr( void )
{
    	return I2cDeviceAddr;
}

uint8_t BMA250EInit( void )
{
//	uint8_t regVal = 0;
	BMA250ESetDeviceAddr( BMA2x2_I2C_ADDRESS );

	if( BMA250Initialized == false )
	{   
		BMA250Initialized = true;
        
        if(BMA250EReset( ) == FAIL)
            return FAIL;
        
		//read BMA250E id
//		BMA250E_I2C_Read( BMA2x2_CHIP_ID_REG, &regVal,  1);
//		if( regVal != BMA250E_ID )
//		{
//		    return FAIL;
//		}

//		DelayMs(10);

//		BMA250E_Range_Set(BMA250E_RANGE);
//		BMA250E_Bandwidth_Set(BMA250E_BANDWIDTH);
//		BMA250E_ShadowDis_Disable(true);
//		BMA250E_INT_Latched_Mode_Set(true);
		
		//BMA250E_INT_Level_Set(BMA2x2_INTR1_LEVEL, true);
		//BMA250E_INT_Level_Set(BMA2x2_INTR2_LEVEL, true);		
	}
	return SUCCESS;
}

void BMA250EDeInit( void )
{
    BMA250Initialized = false;
}

uint8_t BMA250E_I2C_Write( uint8_t addr, uint8_t data, uint8_t size )
{
	return I2cWriteBuffer( I2cDeviceAddr << 1, addr, &data, size );
}

uint8_t BMA250E_I2C_Read( uint8_t addr, uint8_t *data, uint8_t size)
{
	return I2cReadBuffer( I2cDeviceAddr << 1, addr, data, size );
}

int16_t BMA250EConvertToNumerical( uint16_t AxisRaw, uint8_t Res, float base_g )
{
    uint16_t rawdata;
    uint8_t sign_flag = 0;
    int16_t GVal = 0;
    if( AxisRaw == 0x0 )
    {
        return 0;
    }
    
    switch( Res )
    {
        case BMA2x2_10_RESOLUTION:
        {
            rawdata = (AxisRaw >> BMA250E_10_BIT_DATA_SHIFT);
            sign_flag = (rawdata >> BMA250E_10_BIT_SIGN_SHIFT) & 0x01;
                
            if(sign_flag == 0)
                GVal = rawdata * base_g;
            else
            {
                AxisRaw = ~rawdata;
                AxisRaw = (AxisRaw & BMA250E_10_BIT_MASK) + 1;
                GVal = -AxisRaw * base_g;
            }
            break;
        }
        case BMA2x2_12_RESOLUTION:
            break;
        case BMA2x2_14_RESOLUTION:
            break;
    }
    return ( GVal );
}

/*!
 *	@brief This API is used to set shadow dis
 *	in the register 0x13 bit 6
 *
 *  @param  isDisable : The value of shadow dis disable/enable *
 *       ----------------- | ------------------
 *              0          | enable MSB Lock
 *              1          | disable MSB Lock
 */
void BMA250E_ShadowDis_Disable(bool isDisable)
{
	uint8_t shadow_dis_data;

	BMA250E_I2C_Read(BMA2x2_DATA_CTRL_REG, &shadow_dis_data, 1);

	if(isDisable) {
		shadow_dis_data = shadow_dis_data | BMA2x2_ENABLE_SHADOW_DIS;
	} else {
		shadow_dis_data = shadow_dis_data & ~BMA2x2_ENABLE_SHADOW_DIS;
	}

	BMA250E_I2C_Write(BMA2x2_DATA_CTRL_REG, shadow_dis_data, 1);
}

/*!
 *	@brief This API is used to set the bandwidth of the sensor in the register
 *	0x10 bit from 0 to 4
 *
 *
 *  @param bandwidth : The value of bandwidth
 *		  bandwidth          |   result			| updateTime
 *       ----------------- | --------------
 *              0x08       | BMA2x2_BW_7_81HZ	| 64ms
 *              0x09       | BMA2x2_BW_15_63HZ	| 32ms
 *              0x0A       | BMA2x2_BW_31_25HZ	| 16ms
 *              0x0B       | BMA2x2_BW_62_50HZ	| 8ms
 *              0x0C       | BMA2x2_BW_125HZ	| 4ms
 *              0x0D       | BMA2x2_BW_250HZ	| 2ms
 *              0x0E       | BMA2x2_BW_500HZ		| 1ms
 *              0x0F       | BMA2x2_BW_1000HZ	| 0.5ms
 *
*/
void BMA250E_Bandwidth_Set(uint8_t bandwidth)
{
	uint8_t bandwidth_data;

	BMA250E_I2C_Read(BMA2x2_BW_SELECT_REG, &bandwidth_data, 1);

	switch (bandwidth) {
		case BMA2x2_BW_7_81HZ:
			bandwidth_data = (bandwidth_data & 0x0) | BMA2x2_BW_7_81HZ;
			/*  7.81 Hz      64000 uS   */
			break;
		case BMA2x2_BW_15_63HZ:
			bandwidth_data = (bandwidth_data & 0x0)  | BMA2x2_BW_15_63HZ;
			/*  15.63 Hz     32000 uS   */
			break;
		case BMA2x2_BW_31_25HZ:
			bandwidth_data = (bandwidth_data & 0x0) | BMA2x2_BW_31_25HZ;
			/*  31.25 Hz     16000 uS   */
			break;
		case BMA2x2_BW_62_50HZ:
			bandwidth_data = (bandwidth_data & 0x0) | BMA2x2_BW_7_81HZ;
			/*  62.50 Hz     8000 uS   */
			break;
		case BMA2x2_BW_125HZ:
			bandwidth_data = (bandwidth_data & 0x0) | BMA2x2_BW_7_81HZ;
			/*  125 Hz       4000 uS   */
			break;
		case BMA2x2_BW_250HZ:
			bandwidth_data = (bandwidth_data & 0x0) | BMA2x2_BW_7_81HZ;
			/*  250 Hz       2000 uS   */
			break;
		case BMA2x2_BW_500HZ:
			bandwidth_data = (bandwidth_data & 0x0) | BMA2x2_BW_7_81HZ;
			/*!  500 Hz       1000 uS   */
			break;
		case BMA2x2_BW_1000HZ:
			bandwidth_data = (bandwidth_data & 0x0) | BMA2x2_BW_1000HZ;
			/*  1000 Hz      500 uS   */
			break;
		default:
			break;
	}

	BMA250E_I2C_Write(BMA2x2_BW_SELECT_REG, bandwidth_data, 1);

}

void BMA250E_Range_Set(uint8_t range)
{
	uint8_t range_data;

	BMA250E_I2C_Read(BMA2x2_RANGE_SELECT_REG, &range_data, 1);
	
	switch (range) {
		case BMA2x2_RANGE_2G:
			range_data = (range_data & 0x0) | BMA2x2_RANGE_2G;
			break;
		case BMA2x2_RANGE_4G:
			range_data = (range_data & 0x0) | BMA2x2_RANGE_4G;
			break;
		case BMA2x2_RANGE_8G:
			range_data = (range_data & 0x0) | BMA2x2_RANGE_8G;
			break;
		case BMA2x2_RANGE_16G:
			range_data = (range_data & 0x0) | BMA2x2_RANGE_16G;
			break;
		default:
			break;
	}

	BMA250E_I2C_Write(BMA2x2_RANGE_SELECT_REG, range_data, 1);
}

/*!
 *	@brief This API is used to set
 *  interrupt enable bits of the sensor in the registers 0x16 and 0x17
 *	@note It reads the flat enable, orient enable,
 *	@note single tap enable, double tap enable
 *	@note slope-x enable, slope-y enable, slope-z enable,
 *	@note fifo watermark enable,
 *	@note fifo full enable, data enable, low-g enable,
 *	@note high-z enable, high-y enable
 *	@note high-z enable
 *
 *
 *
 *  @param v_intr_type_u8: The value of interrupts
 *        int_type   |   result
 *       ----------------- | ------------------
 *              0          | BMA2x2_LOW_G_INTR
 *              1          | BMA2x2_HIGH_G_X_INTR
 *              2          | BMA2x2_HIGH_G_Y_INTR
 *              3          | BMA2x2_HIGH_G_Z_INTR
 *              4          | BMA2x2_DATA_ENABLE
 *              5          | SLOPE_X_INTR
 *              6          | SLOPE_Y_INTR
 *              7          | SLOPE_Z_INTR
 *              8          | SINGLE_TAP_INTR
 *              9          | SINGLE_TAP_INTR
 *              10         | ORIENT_INT
 *              11         | FLAT_INT
 *
 *  @param v_value_u8 : The value of interrupts enable
 *        isEnable       |   result
 *       ----------------- | ------------------
 *              false       | INTR_DISABLE
 *              true       | INTR_ENABLE
 *
 */
void BMA250E_INT_Enable_Set(uint8_t int_type, bool isEnable)
{
	uint8_t int_1_data;
	uint8_t int_2_data;

	BMA250E_I2C_Read(BMA2x2_INTR_ENABLE1_REG, &int_1_data, 1);
	BMA250E_I2C_Read(BMA2x2_INTR_ENABLE2_REG, &int_2_data, 1);

	switch (int_type) {
		case BMA2x2_LOW_G_INTR:
			/* Low G Interrupt  */
			if(isEnable) {
				int_2_data = int_2_data | 0x8;
			} else {
				int_2_data = int_2_data & ~0x8;
			}

			break;
		case BMA2x2_HIGH_G_X_INTR:
			/* High G X Interrupt */
			if(isEnable) {
				int_2_data = int_2_data | 0x1;
			} else {
				int_2_data = int_2_data & ~0x1;
			}

			break;
		case BMA2x2_HIGH_G_Y_INTR:
			/* High G Y Interrupt */
			if(isEnable) {
				int_2_data = int_2_data | 0x2;
			} else {
				int_2_data = int_2_data & ~0x2;
			}

			break;
		case BMA2x2_HIGH_G_Z_INTR:
			/* High G Z Interrupt */
			if(isEnable) {
				int_2_data = int_2_data | 0x4;
			} else {
				int_2_data = int_2_data & ~0x4;
			}

			break;
		case BMA2x2_DATA_ENABLE:
			/*Data En Interrupt  */
			if(isEnable) {
				int_2_data = int_2_data | 0x10;
			} else {
				int_2_data = int_2_data & ~0x10;
			}

			break;
		case BMA2x2_SLOPE_X_INTR:
			/* Slope X Interrupt */
			if(isEnable) {
				int_1_data = int_1_data | 0x1;
			} else {
				int_1_data = int_1_data & ~0x1;
			}

			break;
		case BMA2x2_SLOPE_Y_INTR:
			/* Slope Y Interrupt */
			if(isEnable) {
				int_1_data = int_1_data | 0x2;
			} else {
				int_1_data = int_1_data & ~0x2;
			}

			break;
		case BMA2x2_SLOPE_Z_INTR:
			/* Slope Z Interrupt */
			if(isEnable) {
				int_1_data = int_1_data | 0x4;
			} else {
				int_1_data = int_1_data & ~0x4;
			}

			break;
		case BMA2x2_SINGLE_TAP_INTR:
			/* Single Tap Interrupt */
			if(isEnable) {
				int_1_data = int_1_data | 0x20;
			} else {
				int_1_data = int_1_data & ~0x20;
			}

			break;
		case BMA2x2_DOUBLE_TAP_INTR:
			/* Double Tap Interrupt */
			if(isEnable) {
				int_1_data = int_1_data | 0x10;
			} else {
				int_1_data = int_1_data & ~0x10;
			}

			break;
		case BMA2x2_ORIENT_INTR:
			/* Orient Interrupt  */
			if(isEnable) {
				int_1_data = int_1_data | 0x40;
			} else {
				int_1_data = int_1_data & ~0x40;
			}

			break;
		case BMA2x2_FLAT_INTR:
			/* Flat Interrupt */
			if(isEnable) {
				int_1_data = int_1_data | 0x80;
			} else {
				int_1_data = int_1_data & ~0x80;
			}

			break;
		default:

			break;
	}

	BMA250E_I2C_Write(BMA2x2_INTR_ENABLE1_REG, int_1_data, 1);
	BMA250E_I2C_Write(BMA2x2_INTR_ENABLE2_REG, int_2_data, 1);

}

/*!
 *	@brief This API is used to set
 *	the source data status of source data,
 *	source slow no motion, source slope, source high
 *	and source low in the register 0x1E bit from 0 to 5
 *
 *
 *
 *  @param channel : The value of source select
 *       channel     |    result
 *       -----------------| ------------------
 *               0        | BMA2x2_ACCEL_SOURCE_LOW_G
 *               1        | BMA2x2_ACCEL_SOURCE_HIGH_G
 *               2        | BMA2x2_ACCEL_SOURCE_SLOPE
 *               3        | BMA2x2_ACCEL_SOURCE_SLOW_NO_MOTION
 *               4        | BMA2x2_ACCEL_SOURCE_TAP
 *               5        | BMA2x2_ACCEL_SOURCE_DATA
 *
 *	@param isEnable: The source status enable value
 *       isEnable         |    result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 */
void BMA250E_INT_Source_Set(uint8_t channel, uint8_t isEnable)
{
	uint8_t int_source_data;
	BMA250E_I2C_Read(BMA2x2_INTR_SOURCE_REG, &int_source_data, 1);

	switch (channel) {
		/* write the source interrupt register*/
		case BMA2x2_SOURCE_LOW_G:
			if(isEnable) {
				int_source_data = int_source_data | 0x1;
			} else {
				int_source_data = int_source_data & ~0x1;
			}
			break;
		case BMA2x2_SOURCE_HIGH_G:
			if(isEnable) {
				int_source_data = int_source_data | 0x2;
			} else {
				int_source_data = int_source_data & ~0x2;
			}
			break;
		case BMA2x2_SOURCE_SLOPE:
			if(isEnable) {
				int_source_data = int_source_data | 0x4;
			} else {
				int_source_data = int_source_data & ~0x4;
			}
			break;
		case BMA2x2_SOURCE_SLOW_NO_MOTION:
			if(isEnable) {
				int_source_data = int_source_data | 0x8;
			} else {
				int_source_data = int_source_data & ~0x8;
			}
			break;
		case BMA2x2_SOURCE_TAP:
			if(isEnable) {
				int_source_data = int_source_data | 0x10;
			} else {
				int_source_data = int_source_data & ~0x10;
			}
			break;
		case BMA2x2_SOURCE_DATA:
			if(isEnable) {
				int_source_data = int_source_data | 0x20;
			} else {
				int_source_data = int_source_data & ~0x20;
			}
			break;
		default:
			break;
	}
	BMA250E_I2C_Write(BMA2x2_INTR_SOURCE_REG, int_source_data, 1);
}

/*!
 *	@brief This API is used to set
 *	the interrupt output type in the register 0x20.
 *	@note INTR1 -> bit 1
 *	@note INTR2 -> bit 3
 *
 *  @param channel: The value of output type select
 *         channel   |    result
 *       -----------------| ------------------
 *               0        | BMA2x2_ACCEL_INTR1_OUTPUT
 *               1        | BMA2x2_ACCEL_INTR2_OUTPUT
 *
 *	@param isOpenDrain: The value of output type select
 *       isOpenDrain         |    result
 *       ------------------------ | ------------------
 *              true              | OPEN_DRAIN
 *              false              | PUSS_PULL
 */
void BMA250E_INT_OutputType_Set(uint8_t channel, bool isOpenDrain)
{
	uint8_t output_data;
	BMA250E_I2C_Read(BMA2x2_INTR_SET_REG, &output_data, 1);	
	switch (channel) {
		/* write the active level */
		case BMA2x2_INTR1_OUTPUT:
			if(isOpenDrain) {
				output_data = output_data | 0x2;
			} else {
				output_data = output_data & ~0x2;
			}
			break;
		case BMA2x2_INTR2_OUTPUT:
			if(isOpenDrain) {
				output_data = output_data | 0x8;
			} else {
				output_data = output_data & ~0x8;
			}
			break;
		default:
			break;
	}
	BMA250E_I2C_Write(BMA2x2_INTR_SET_REG, output_data, 1);	
}

/*!
 * @brief This API is used to set
 * the interrupt status of new data in the register 0x19
 * @note INTR1_data -> register 0x19 bit 0
 * @note INTR2_data -> register 0x19 bit 7
 *
 *
 *
 *  @param channel: The value of new data interrupt select
 *        channel     |   result
 *       ----------------- | ------------------
 *              0          | BMA2x2_ACCEL_INTR1_NEWDATA
 *              1          | BMA2x2_ACCEL_INTR2_NEWDATA
 *
 *	@param isEnable: The new data interrupt enable value
 *       isEnable          |    result
 *       ------------------------ | ------------------
 *              0x00              | INTR_DISABLE
 *              0x01              | INTR_ENABLE
 *
 */
void BMA250E_INT_NewData_Set(uint8_t channel, bool isEnable)
{
	uint8_t new_data;


	BMA250E_I2C_Read(BMA2x2_INTR_DATA_SELECT_REG, &new_data, 1);	
	switch (channel) {
		/* write the active level */
		case BMA2x2_INTR1_NEWDATA:
			if(isEnable) {
				new_data = new_data | 0x1;
			} else {
				new_data = new_data & ~0x1;
			}
			break;
		case BMA2x2_INTR2_NEWDATA:
			if(isEnable) {
				new_data = new_data | 0x80;
			} else {
				new_data = new_data & ~0x80;
			}
			break;
		default:
			break;
	}
	BMA250E_I2C_Write(BMA2x2_INTR_DATA_SELECT_REG, new_data, 1);		
}

/*!
 *	@brief This API is used to set
 *	Active Level status in the register 0x20
 *	@note INTR1 -> bit 0
 *	@note INTR2 -> bit 2
 *
 *  @param channel: The value of Active Level select
 *       channel     |    result
 *       -----------------| ------------------
 *               0        | BMA2x2_ACCEL_INTR1_LEVEL
 *               1        | BMA2x2_ACCEL_INTR2_LEVEL
 *
 *  @param isActiveHigh: The Active Level status enable value
 *       isActiveHigh          |    result
 *       ------------------------ | ------------------
 *              true              | ACTIVE_HIGH
 *              false              | ACTIVE_LOW
 *
 */
void BMA250E_INT_Level_Set(uint8_t channel, bool isActiveHigh)
{
	uint8_t level_data;
	BMA250E_I2C_Read(BMA2x2_INTR_SET_REG, &level_data, 1);	
	switch (channel) {
		/* write the active level */
		case BMA2x2_INTR1_LEVEL:
			if(isActiveHigh) {
				level_data = level_data | 0x1;
			} else {
				level_data = level_data & ~0x1;
			}
			break;
		case BMA2x2_INTR2_LEVEL:
			if(isActiveHigh) {
				level_data = level_data | 0x4;
			} else {
				level_data = level_data & ~0x4;
			}
			break;
		default:
			break;
	}
	BMA250E_I2C_Write(BMA2x2_INTR_SET_REG, level_data, 1);	
}

/*!
 *	@brief This API is used to set
 *	the set interrupt latched mode in the register 0x21 bit 0~3 
 *
 *
 *
 *  @param  isEnable: The value of enable/disable interrupt latched mode
 *          isReset         |  result
 *       ------------------------ | ------------------
 *              true              | enable interrupt latched mode
 *              false             | disable interrupt latched mode

 */
void BMA250E_INT_Latched_Mode_Set(bool isEnable)
{
	uint8_t latched_data;

	BMA250E_I2C_Read(BMA2x2_INTR_CTRL_REG, &latched_data, 1);
	if(isEnable)
		latched_data = latched_data | 0x0F;
	else
		latched_data = latched_data & ~0x0F;

	BMA250E_I2C_Write(BMA2x2_INTR_CTRL_REG, latched_data, 1);
}

/*!
 *	@brief This API is used to set
 *	the status of fast offset compensation(cal_rdy) in the register 0x36
 *	bit 4(Read Only Possible)

 *  @param  channel: compensation axis(x/y/z)
 *          channel         |  result
 *       ------------------------ | ------------------
 *              0x1              |  x axis
 *              0x2              |  y axis
  *             0x3              |  z axis
 */
void BMA250E_INT_Compensation_Set(uint8_t channel)
{
	uint8_t offset_ctrl_data;
	uint8_t cal_rdy = 0x0;		

	//offset compensation must not be triggered when cal_rdy is '0'
	while(cal_rdy == 0x0) {
		BMA250E_I2C_Read(BMA2x2_OFFSET_CTRL_REG, &offset_ctrl_data, 1);
		cal_rdy = (offset_ctrl_data & 0x10) >> BMA250E_FFSET_COMPENSATION_CAL_RTY_SHIFT;
//		DelayMs(10);
	}
	offset_ctrl_data = (offset_ctrl_data & BMA250E_FFSET_COMPENSATION_CAL_TRIGGER_MASK) \
        | ((channel & 0x3) << BMA250E_FFSET_COMPENSATION_CAL_TRIGGER_SHIFT);
    
	BMA250E_I2C_Write(BMA2x2_OFFSET_CTRL_REG, offset_ctrl_data, 1);

    while(cal_rdy == 0x0) {
		BMA250E_I2C_Read(BMA2x2_OFFSET_CTRL_REG, &offset_ctrl_data, 1);
		cal_rdy = (offset_ctrl_data & 0x10) >> BMA250E_FFSET_COMPENSATION_CAL_RTY_SHIFT;
//		DelayMs(10);
	}	
}

/*!
 *	@brief This API is used to set
 *	the reset interrupt in the register 0x21 bit 7
 *
 *
 *
 *  @param  isReset: The value of reset interrupt
 *          isReset         |  result
 *       ------------------------ | ------------------
 *              true              | clear any latch interrupt
 *              false              | keep latch interrupt active

 */
void BMA250E_INT_RST(bool isReset)
{
	uint8_t reset_data;

	BMA250E_I2C_Read(BMA2x2_INTR_CTRL_REG, &reset_data, 1);
	if(isReset)
		reset_data = reset_data | 0x80;
	else
		reset_data = reset_data & ~0x80;

	BMA250E_I2C_Write(BMA2x2_INTR_CTRL_REG, reset_data, 1);
}

/*!
 * @brief This API is used to set
 * the interrupt enable of slope interrupt in the register 0x19 and 0x1B
 * @note INTR1_slope -> register 0x19 bit 2
 * @note INTR2_slope -> register 0x1B bit 2
 *
 *
 *
 * @param channel: the value of slope channel select
 *        channel     |   result
 *       ----------------- | ------------------
 *              0          | BMA2x2_INTR1_SLOPE
 *              1          | BMA2x2_INTR2_SLOPE
 *
 * @param isEnable : The slope value enable value
 *        isEnable         |   result
 *       ------------------------ | ------------------
 *              false              | INTR_DISABLE
 *              true              | INTR_ENABLE
 *
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMA250E_INT_Slope_Set(uint8_t channel, bool isEnable)
{
	uint8_t slope_data;
	
	switch (channel) {
		/* write the active level */
		case BMA2x2_INTR1_SLOPE:
			BMA250E_I2C_Read(BMA2x2_INTR1_PAD_SELECT_REG, &slope_data, 1);	
			if(isEnable) {
				slope_data = slope_data | 0x4;
			} else {
				slope_data = slope_data & ~0x4;
			}
			BMA250E_I2C_Write(BMA2x2_INTR1_PAD_SELECT_REG, slope_data, 1);	
			break;
		case BMA2x2_INTR2_SLOPE:
			BMA250E_I2C_Read(BMA2x2_INTR2_PAD_SELECT_REG, &slope_data, 1);	
			if(isEnable) {
				slope_data = slope_data | 0x4;
			} else {
				slope_data = slope_data & ~0x4;
			}
			BMA250E_I2C_Write(BMA2x2_INTR2_PAD_SELECT_REG, slope_data, 1);	
			break;
        case BMA2x2_INTR2_LOW_G:
			BMA250E_I2C_Read(BMA2x2_INTR2_PAD_SELECT_REG, &slope_data, 1);	
			if(isEnable) {
				slope_data = slope_data | 0x1;
			} else {
				slope_data = slope_data & ~0x1;
			}
			BMA250E_I2C_Write(BMA2x2_INTR2_PAD_SELECT_REG, slope_data, 1);	
			break;
		default:
			break;
	}	
}

/*!
 * @brief This API is used to set the threshold of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note LOW_THRES		-> register 0x23 bit form 0 to 7
 *	@note HIGH_THRES		-> register 0x26 bit form 0 to 7
 *	@note SLOPE_THRES		-> register 0x28 bit form 0 to 7
 *	@note SLO_NO_MOT_THRES -> register 0x29 bit form 0 to 7
 *
 *  @param channel: The value of threshold selection
 *     channel   | result
 *   -----------------| ------------------
 *               0    | BMA2x2_ACCEL_LOW_THRES
 *               1    | BMA2x2_ACCEL_HIGH_THRES
 *               2    | BMA2x2_ACCEL_SLOPE_THRES
 *               3    | BMA2x2_ACCEL_SLOW_NO_MOTION_THRES
 *
 *  @param v_thres_u8: The threshold value of selected interrupts
 *
 *	@note : LOW-G THRESHOLD
 *     Threshold                    |    result
 * ---------------------------------| ------------------
 * BMA2x2_ACCEL_LOW_THRES           | Low-threshold interrupt trigger
 *                                  | according to(v_thres_u8 * 7.81) mg
 *                                  | range from 0g to 1.992g
 *                                  | default is 375mg
 *	@note : HIGH-G THRESHOLD
 *	@note Threshold of high-g interrupt according to accel g range
 *    g-range           |      High-g threshold
 *  --------------------|----------------------------
 *     2g               |    (v_thres_u8 * 7.81) mg
 *     4g               |    (v_thres_u8 * 15.63) mg
 *     8g               |    (v_thres_u8 * 31.25) mg
 *     16g              |    (v_thres_u8 * 62.5) mg
 *
 *	@note : SLOPE THRESHOLD
 *	@note Threshold of slope interrupt according to accel g range
 *    g-range           |      Slope threshold
 *  --------------------|----------------------------
 *     2g               |    (v_thres_u8 * 3.19) mg
 *     4g               |    (v_thres_u8 * 7.81) mg
 *     8g               |    (v_thres_u8 * 15.63) mg
 *     16g              |    (v_thres_u8 * 31.25) mg
 *
 *	@note : SLOW NO MOTION THRESHOLD
 *	@note Threshold of slow no motion interrupt according to accel g range
 *    g-range           |   slow no motion threshold
 *  --------------------|----------------------------
 *     2g               |    (v_thres_u8 * 3.19) mg
 *     4g               |    (v_thres_u8 * 7.81) mg
 *     8g               |    (v_thres_u8 * 15.63) mg
 *     16g              |    (v_thres_u8 * 31.25) mg
 *
 */
void BMA250E_INT_Slope_Threshold_Set(uint8_t channel, uint8_t threshold)
{
	uint8_t threshold_data;

	switch (channel) {
		/* write the threshold value*/
		case BMA2x2_LOW_THRES:
			/*LOW THRESHOLD*/
			threshold_data = threshold;
			BMA250E_I2C_Write(BMA2x2_LOW_THRES_REG, threshold_data, 1);
			break;
		case BMA2x2_HIGH_THRES:
			/*HIGH THRESHOLD*/
			threshold_data = threshold;
			BMA250E_I2C_Write(BMA2x2_HIGH_THRES_REG, threshold_data, 1);
			break;
		case BMA2x2_SLOPE_THRES:
			/*SLOPE THRESHOLD*/
			threshold_data = threshold;
			BMA250E_I2C_Write(BMA2x2_SLOPE_THRES_REG, threshold_data, 1);
			break;
		case BMA2x2_SLOW_NO_MOTION_THRES:
			/*SLO NO MOT THRESHOLD*/
			threshold_data = threshold;
			BMA250E_I2C_Write(BMA2x2_SLOW_NO_MOTION_THRES_REG, threshold_data, 1);
			break;
		default:
			break;
	}
}

/*!
 *	@brief This API is used to set the duration of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note LOW_DURN		-> register 0x22 bit form 0 to 7
 *	@note HIGH_DURN		-> register 0x25 bit form 0 to 7
 *	@note SLOPE_DURN		-> register 0x27 bit form 0 to 1
 *	@note SLO_NO_MOT_DURN -> register 0x27 bit form 2 to 7
 *
 *  @param channel: The value of duration select
 *     channel   | result
 *   -----------------| ------------------
 *               0    | BMA2x2_ACCEL_LOW_DURN
 *               1    | BMA2x2_ACCEL_HIGH_DURN
 *               2    | BMA2x2_ACCEL_SLOPE_DURN
 *               3    | BMA2x2_ACCEL_SLOW_NO_MOTION_DURN
 *
 *	@param v_durn_u8: The value of duration
 *
 *	@note :
 *     Duration           |    result
 * -----------------------| ------------------
 * BMA2x2_ACCEL_LOW_DURN  | Low-g interrupt trigger
 *         -              | delay according to([v_durn_u8 +1]*2)ms
 *         -              | range from 2ms to 512ms. default is 20ms
 * BMA2x2_ACCEL_HIGH_DURN | high-g interrupt trigger
 *         -              | delay according to([v_durn_u8 +1]*2)ms
 *         -              | range from 2ms to 512ms. default is 32ms
 * BMA2x2_ACCEL_SLOPE_DURN| slope interrupt trigger
 *         -              | if[v_durn_u8<1:0>+1] consecutive data points
 *         -              | are above the slope interrupt threshold
 * SLO_NO_MOT_DURN        | Refer data sheet for clear information
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMA250E_INT_Slope_Duration_Set(uint8_t channel, uint8_t duration)
{
	uint8_t duration_data;

	/* write duration data */
	switch (channel)   {
		case BMA2x2_LOW_DURN:
			/*LOW DURATION*/
			duration_data = duration;
			BMA250E_I2C_Write(BMA2x2_LOW_DURN_REG, duration_data, 1);
			break;
		case BMA2x2_HIGH_DURN:
			/*HIGH DURATION*/
			duration_data = duration;
			BMA250E_I2C_Write(BMA2x2_HIGH_DURN_REG, duration_data, 1);
			break;
		case BMA2x2_SLOPE_DURN:
			/*SLOPE DURATION*/
			BMA250E_I2C_Write(BMA2x2_SLOPE_DURN_REG, duration_data, 1);
			duration_data = duration_data |duration;
			BMA250E_I2C_Write(BMA2x2_SLOPE_DURN_REG, duration_data, 1);
		break;
		case BMA2x2_SLOW_NO_MOTION_DURN:
			/*SLO NO MOT DURATION*/
			//BMA250E_I2C_Write(BMA2x2_SLOPE_DURN_REG, duration_data, 1);
			//duration_data = duration_data |duration;
			//BMA250E_I2C_Write(BMA2x2_SLOPE_DURN_REG, duration_data, 1);
			break;
		default:
			break;
	}

}

void BMA250E_INT_Slope_Hysteresis_Set(uint8_t channel, uint8_t hysteresis)
{
	uint8_t hysteresis_data;

	/* write hysteresis data */
	switch (channel)   {
		case BMA2x2_LOW_HYST:
			/*LOW HYSTERESIS*/
//			hysteresis_data = hysteresis;
//			BMA250E_I2C_Write(BMA2x2_LOW_HYST_REG, hysteresis_data, 1);
            BMA250E_I2C_Read(BMA2x2_LOW_HYST_REG, &hysteresis_data, 1);
//            hysteresis_data = hysteresis_data | 0x4;
            hysteresis_data = hysteresis_data | (1<<2);
//			if(isEnable) {
//				slope_data = slope_data | 0x4;
//			} else {
//				slope_data = slope_data & ~0x4;
//			}
//			BMA250E_I2C_Write(BMA2x2_INTR1_PAD_SELECT_REG, slope_data, 1);
			break;
		default:
			break;
	}

}

/*!
 * @brief This API is used to set
 * the interrupt status of orient interrupt in the register 0x19 and 0x1B
 * @note INTR1_orient -> register 0x19 bit 6
 * @note INTR2_orient -> register 0x1B bit 6
 *
 *
 * @param channel: The value of orient interrupt select
 *        channel     |   result
 *       ----------------- | ------------------
 *              0          | BMA2x2_INTR1_ORIENT
 *              1          | BMA2x2_INTR2_ORIENT
 *
 *  @param isEnable: The value of orient interrupt enable
 *       isEnable         |   result
 *       ------------------------ | ------------------
 *              false              | INTR_DISABLE
 *              true              | INTR_ENABLE
*/
void BMA250E_INT_Orient_Set(uint8_t channel, uint8_t isEnable)
{
	uint8_t orient_data;
	
	switch (channel) {
		/* write the active level */
		case BMA2x2_INTR1_ORIENT:
			BMA250E_I2C_Read(BMA2x2_INTR1_PAD_SELECT_REG, &orient_data, 1);	
			if(isEnable) {
				orient_data = orient_data | 0x40;
			} else {
				orient_data = orient_data & ~0x40;
			}
			BMA250E_I2C_Write(BMA2x2_INTR1_PAD_SELECT_REG, orient_data, 1);	
			break;
		case BMA2x2_INTR2_ORIENT:
			BMA250E_I2C_Read(BMA2x2_INTR2_PAD_SELECT_REG, &orient_data, 1);	
			if(isEnable) {
				orient_data = orient_data | 0x40;
			} else {
				orient_data = orient_data & ~0x40;
			}
			BMA250E_I2C_Write(BMA2x2_INTR2_PAD_SELECT_REG, orient_data, 1);	
			break;
		default:
			break;
	}	
}

/*!
 *	@brief This API is used to set
 *	the orient mode in the register 0x2C bit 0 and 1
 *
 *
 *
 *  @param mode : The value of orient mode
 *     mode |    result
 *  --------------------|------------------
 *     ORIENT_MODE_SYMMETRICAL             | symmetrical
 *     ORIENT_MODE_HIGH_ASYMMETRICAL             | high asymmetrical
 *     ORIENT_MODE_LOW_ASYMMETRICAL             | low asymmetrical
 *     ORIENT_MODE_SYMMETRICAL2             | symmetrical
 *
*/
void BMA250E_INT_Orient_Mode_Set(uint8_t mode)
{
	uint8_t orient_mode_data;
	BMA250E_I2C_Read(BMA2x2_ORIENT_PARAM_REG, &orient_mode_data, 1);	
	switch (mode) {
		case ORIENT_MODE_SYMMETRICAL:
			orient_mode_data = (orient_mode_data & 0xFC) | ORIENT_MODE_SYMMETRICAL;
			break;
		case ORIENT_MODE_HIGH_ASYMMETRICAL:
			orient_mode_data = (orient_mode_data & 0xFC)  | ORIENT_MODE_HIGH_ASYMMETRICAL;
			break;
		case ORIENT_MODE_LOW_ASYMMETRICAL:
			orient_mode_data = (orient_mode_data & 0xFC) | ORIENT_MODE_LOW_ASYMMETRICAL;
			break;
		case ORIENT_MODE_SYMMETRICAL2:
			orient_mode_data = (orient_mode_data & 0xFC) | ORIENT_MODE_SYMMETRICAL2;
			break;
		
		default:
			break;
	}
	BMA250E_I2C_Write(BMA2x2_ORIENT_PARAM_REG, orient_mode_data, 1);
}

/*!
 *	@brief This API is used to set
 *	the orient block in the register 0x2C bit 2 and 3
 *
 *
 *
 *	@param mode : The value of orient block
 *     mode |    result
 *  --------------------|------------------
 *     0x00             | no blocking
 *     0x01             | theta blocking or
 *                      | acceleration slope in any axis > 1.5g
 *     0x02             | theta blocking or
 *                      | acceleration slope in any axis > 0.2g
 *                      | acceleration in any axis > 1.5g
 *     0x03             | theta blocking or
 *                      | acceleration slope in any axis > 0.4g
 *                      | acceleration in any axis > 1.5g
 *                      | value of orient is not stable for at lease 100ms
 *
 *
 *	@return results of bus communication function
 *	@retval 0 -> Success
 *	@retval -1 -> Error
 *
 *
 */
void BMA250E_INT_Orient_Block_Mode_Set(uint8_t mode)
{
	uint8_t block_mode_data;

	BMA250E_I2C_Read(BMA2x2_ORIENT_PARAM_REG, &block_mode_data, 1);	
	switch (mode) {
		case 0x0:
			block_mode_data = (block_mode_data & 0xF3) | 0x0;
			break;
		case 0x1:
			block_mode_data = (block_mode_data & 0xF3)  | 0x4;
			break;
		case 0x2:
			block_mode_data = (block_mode_data & 0xF3) | 0x8;
			break;
		case 0x3:
			block_mode_data = (block_mode_data & 0xF3) | 0xC;
			break;
		
		default:
			break;
	}
	BMA250E_I2C_Write(BMA2x2_ORIENT_PARAM_REG, block_mode_data, 1);
}


/*!
 *	@brief  This API is used to set
 *	the theta value of orient interrupts
 *	@note FLAT_THETA   -> register 0x2E bit 0 to 5
 *
 *  @param deg_angle: The value of theta deg
 * @note ORIENT_THETA : Defines threshold for detection of flat position
 *                in range from 0 deg to 44.8 deg
 *
 */
void BMA250E_INT_Orient_Block_Threshold_Set(double deg_angle)
{	
	double rad_angle = 0;
	uint8_t orient_threshold_data;
	uint8_t threshold_theta;	
	double tan_angle = 0;
	
	if(deg_angle > 0 && deg_angle <= 44.8) {
		//degree to radian
		rad_angle = deg_angle * 3.14 / 180;		
	} else {
		//degree to radian
		rad_angle = 44.8 * 3.14 / 180;		
	}

	tan_angle = tan(rad_angle);

	threshold_theta =  (uint8_t)((8 * tan_angle) *  (8 * tan_angle));

	BMA250E_I2C_Read(BMA2x2_THETA_BLOCK_REG, &orient_threshold_data, 1);	
	orient_threshold_data = (orient_threshold_data & 0xC0) | threshold_theta;
	BMA250E_I2C_Write(BMA2x2_THETA_BLOCK_REG, orient_threshold_data, 1);
}

/*!
 *	@brief This API is used to set
 *	the interrupt enable of flat hysteresis("flat_hy)
 *	in the register 0x2C bit 4 to 6
 *
 *  @param flat_hy : The value of flat hysteresis
 */
void BMA250E_INT_Orient_Hyst_Set(uint8_t flat_hy)
{
	uint8_t orient_hyst_data;

	BMA250E_I2C_Read(BMA2x2_ORIENT_PARAM_REG, &orient_hyst_data, 1);	
	orient_hyst_data = (orient_hyst_data & 0x8F) | ((flat_hy & 0x7) << 4);
	BMA250E_I2C_Write(BMA2x2_ORIENT_PARAM_REG, orient_hyst_data, 1);	
}

/*!
 * @brief This API is used to set
 * the interrupt status of orient interrupt in the register 0x19 and 0x1B
 * @note INTR1_flat-> register 0x19 bit 7
 * @note INTR2_flat-> register 0x1B bit 7
 *
 *
 * @param channel: The value of orient interrupt select
 *        channel     |   result
 *       ----------------- | ------------------
 *              0          | BMA2x2_INTR1_FLAT
 *              1          | BMA2x2_INTR2_FLAT
 *
 *  @param isEnable: The value of orient interrupt enable
 *       isEnable         |   result
 *       ------------------------ | ------------------
 *              false              | INTR_DISABLE
 *              true              | INTR_ENABLE
*/
void BMA250E_INT_Flat_Set(uint8_t channel, uint8_t isEnable)
{
	uint8_t flat_data;
	
	switch (channel) {
		/* write the active level */
		case BMA2x2_INTR1_FLAT:
			BMA250E_I2C_Read(BMA2x2_INTR1_PAD_SELECT_REG, &flat_data, 1);	
			if(isEnable) {
				flat_data = flat_data | 0x80;
			} else {
				flat_data = flat_data & ~0x80;
			}
			BMA250E_I2C_Write(BMA2x2_INTR1_PAD_SELECT_REG, flat_data, 1);	
			break;
		case BMA2x2_INTR2_FLAT:
			BMA250E_I2C_Read(BMA2x2_INTR2_PAD_SELECT_REG, &flat_data, 1);	
			if(isEnable) {
				flat_data = flat_data | 0x80;
			} else {
				flat_data = flat_data & ~0x80;
			}
			BMA250E_I2C_Write(BMA2x2_INTR2_PAD_SELECT_REG, flat_data, 1);	
			break;
		default:
			break;
	}	
}

/*!
 *	@brief  This API is used to set
 *	the theta value of flat interrupts
 *	@note FLAT_THETA   -> register 0x2E bit 0 to 5
 *
 *  @param deg_angle: The value of theta angle
 * @note FLAT_THETA : Defines a blocking angle between 0 deg to 44.8 deg
 *
 */
void BMA250E_INT_Flat_Block_Threshold_Set(double deg_angle)
{	
	double rad_angle = 0;
	uint8_t flat_threshold_data;
	uint8_t threshold_theta;	
	double tan_angle = 0;
	
	if(deg_angle > 0 && deg_angle <= 44.8) {
		//degree to radian
		rad_angle = deg_angle * 3.14 / 180;		
	} else {
		//degree to radian
		rad_angle = 44.8 * 3.14 / 180;		
	}

	tan_angle = tan(rad_angle);

	threshold_theta =  (uint8_t)((8 * tan_angle) *  (8 * tan_angle));

	BMA250E_I2C_Read(BMA2x2_THETA_FLAT_REG, &flat_threshold_data, 1);	
	flat_threshold_data = (flat_threshold_data & 0xC0) | threshold_theta;
	BMA250E_I2C_Write(BMA2x2_THETA_FLAT_REG, flat_threshold_data, 1);
}

/*!
 *	@brief This API is used to set
 *  the interrupt enable of flat hold time(flat_hold_time)
 *	in the register 0x2F bit 4 and 5
 *
 *
 *  @param  hold_time : The value of flat hold time
 *     hold_time    |    result
 *  ------------------------- |------------------
 *     0x00                   | 0ms
 *     0x01                   | 512ms
 *     0x02                   | 1024ms
 *     0x03                   | 2048ms
 *
 *
 */
 void BMA250E_INT_Flat_Hold_Time_Set(uint8_t hold_time)
{
	uint8_t flat_hold_time_data;

	BMA250E_I2C_Read(BMA2x2_FLAT_HOLD_TIME_REG, &flat_hold_time_data, 1);	
	flat_hold_time_data = (flat_hold_time_data & 0xCF) | ((hold_time & 0x3) << 4);
	BMA250E_I2C_Write(BMA2x2_FLAT_HOLD_TIME_REG, flat_hold_time_data, 1);	
}

/*!
 *	@brief This API is used to set
 *	the interrupt enable of flat hysteresis("flat_hy)
 *	in the register 0x2F bit 0 to 2
 *
 *  @param flat_hy : The value of flat hysteresis
 */
void BMA250E_INT_Flat_Hyst_Set(uint8_t flat_hy)
{
	uint8_t flat_hyst_data;

	BMA250E_I2C_Read(BMA2x2_FLAT_HOLD_TIME_REG, &flat_hyst_data, 1);	
	flat_hyst_data = (flat_hyst_data & 0xF8) | (flat_hy & 0x7);
	BMA250E_I2C_Write(BMA2x2_FLAT_HOLD_TIME_REG, flat_hyst_data, 1);	
}

/*!
 *	@brief This API read interrupt status0 & status2 of flat, orient, single tap,
 *	double tap, slow no motion, slope, highg and lowg from location 09h/0Bh
 *
 *
 *
 *	@param  state : The value of interrupt status
*/

void BMA250E_INT_Status0_Get(void)
{
    uint8_t status0_data;
	BMA250E_I2C_Read(BMA2x2_STAT1_REG, &status0_data, 1);
}

void BMA250E_INT_Status2_Get(void)
{
    uint8_t status2_data;
	BMA250E_I2C_Read(BMA2x2_STAT_TAP_SLOPE_REG, &status2_data, 1);
}

void BMA250E_Polling_Data(struct BMA250E_Accel_Data *data)
{
	uint8_t	data_lsb[2] = {0, 0};
//	uint8_t	data_msb = 0;
    int16_t rawdata = 0;
	uint8_t 	range = BMA250E_RANGE;
	float 	base_g = 0;

	switch(range){
		
		case BMA2x2_RANGE_2G:
			base_g = 3.91;
			break;
			
		case BMA2x2_RANGE_4G:
			base_g = 7.81;
			break;
			
		case BMA2x2_RANGE_8G:
			base_g = 15.63;
			break;
			
		case BMA2x2_RANGE_16G:
			base_g = 31.25;
			break;

	}

	switch (BMA250E_RESOLUTION) {
		/* This case used for the resolution bit 12*/
		case BMA2x2_12_RESOLUTION:
			BMA250E_I2C_Read(BMA2x2_ACCEL_X12_LSB__REG, data_lsb, 2);
			data->x_lsb = (int16_t)((((int32_t)((uint8_t)data_lsb[1])) << 8) |(data_lsb[0] & RESOLUTION_12_MASK));
			data->x_lsb= (int16_t)((data->x_lsb >> 4) * base_g);

			BMA250E_I2C_Read(BMA2x2_ACCEL_Y12_LSB__REG, data_lsb, 2);
			data->y_lsb = (int16_t)((((int32_t)((uint8_t)data_lsb[1])) << 8) |(data_lsb[0] & RESOLUTION_12_MASK));
			data->y_lsb= (int16_t)((data->y_lsb >> 4) * base_g);

			BMA250E_I2C_Read(BMA2x2_ACCEL_Z12_LSB__REG, data_lsb, 2);
			data->z_lsb = (int16_t)((((int32_t)((uint8_t)data_lsb[1])) << 8) |(data_lsb[0] & RESOLUTION_12_MASK));
			data->z_lsb= (int16_t)((data->z_lsb >> 4) * base_g);
			
			break;
		/* This case used for the resolution bit 10*/
		case BMA2x2_10_RESOLUTION:
			BMA250E_I2C_Read(BMA2x2_ACCEL_X10_LSB__REG, data_lsb, 2);
            rawdata = (int16_t)((((int32_t)((uint8_t)data_lsb[1])) << 8) |(data_lsb[0] & RESOLUTION_10_MASK));
            data->new_data_x = data_lsb[0] & BMA250E_10_NEW_DATA_MASK;
			data->x_lsb = BMA250EConvertToNumerical(rawdata, BMA2x2_10_RESOLUTION, base_g);

			BMA250E_I2C_Read(BMA2x2_ACCEL_Y10_LSB__REG, data_lsb, 2);
            rawdata = (int16_t)((((int32_t)((uint8_t)data_lsb[1])) << 8) |(data_lsb[0] & RESOLUTION_10_MASK));
            data->new_data_y = data_lsb[0] & BMA250E_10_NEW_DATA_MASK;
			data->y_lsb = BMA250EConvertToNumerical(rawdata, BMA2x2_10_RESOLUTION, base_g);

            BMA250E_I2C_Read(BMA2x2_ACCEL_Z10_LSB__REG, data_lsb, 2);
            rawdata = (int16_t)((((int32_t)((uint8_t)data_lsb[1])) << 8) |(data_lsb[0] & RESOLUTION_10_MASK));
            data->new_data_z = data_lsb[0] & BMA250E_10_NEW_DATA_MASK;
			data->z_lsb = BMA250EConvertToNumerical(rawdata, BMA2x2_10_RESOLUTION, base_g);
            
			break;

		/* This case used for the resolution bit 14*/
		case BMA2x2_14_RESOLUTION:
			BMA250E_I2C_Read(BMA2x2_ACCEL_X14_LSB__REG, data_lsb, 2);
			data->x_lsb = (int16_t)((((int32_t)((uint8_t)data_lsb[1])) << 8) |(data_lsb[0] & RESOLUTION_14_MASK));
			data->x_lsb= (int16_t)((data->x_lsb >> 2) * base_g);

			BMA250E_I2C_Read(BMA2x2_ACCEL_Y14_LSB__REG, data_lsb, 2);
			data->y_lsb = (int16_t)((((int32_t)((uint8_t)data_lsb[1])) << 8) |(data_lsb[0] & RESOLUTION_14_MASK));
			data->y_lsb= (int16_t)((data->y_lsb >> 2) * base_g);

			BMA250E_I2C_Read(BMA2x2_ACCEL_Z14_LSB__REG, data_lsb, 2);
			data->z_lsb = (int16_t)((((int32_t)((uint8_t)data_lsb[1])) << 8) |(data_lsb[0] & RESOLUTION_14_MASK));
			data->z_lsb= (int16_t)((data->z_lsb >> 2) * base_g);

			break;

		default:
			break;
	}
}

void BMA250E_Polling_Compensation_Offset(uint8_t *ofx, uint8_t *ofy, uint8_t *ofz)
{
	BMA250E_I2C_Read(BMA2x2_OFFSET_X_AXIS_REG, ofx, 1);
	BMA250E_I2C_Read(BMA2x2_OFFSET_Y_AXIS_REG, ofy, 1);
	BMA250E_I2C_Read(BMA2x2_OFFSET_Z_AXIS_REG, ofz, 1);
}

void BMA250E_Mode_Setting(uint8_t mode)
{
    uint8_t reg_data;
    BMA250E_I2C_Read(BMA250E_PMU_LPW_REG, &reg_data, 1);
    reg_data = (reg_data & BMA250E_MODE_MASK) | mode;
    BMA250E_I2C_Write(BMA250E_PMU_LPW_REG, reg_data, 1);
}

void BMA250E_Sleep_During_Setting(uint8_t time)
{
    uint8_t reg_data;
    BMA250E_I2C_Read(BMA250E_PMU_LPW_REG, &reg_data, 1);
    reg_data = (reg_data & BMA250E_SLEEP_DURING_MASK) | time;
    BMA250E_I2C_Write(BMA250E_PMU_LPW_REG, reg_data, 1);
}

void BMA250E_Low_Power_Mode(uint8_t mode)
{
    uint8_t reg_data;
    BMA250E_I2C_Read(BMA250E_PMU_LOW_POWER_REG, &reg_data, 1);
    reg_data = (reg_data & BMA250E_LOW_POWER_MODE_MASK) | (mode << BMA250E_LOW_POWER_MODE_BIT);
    BMA250E_I2C_Write(BMA250E_PMU_LOW_POWER_REG, reg_data, 1);
}

void BMA250E_Low_Power_Sleep_Timer_Mode(uint8_t mode)
{
    uint8_t reg_data;
    BMA250E_I2C_Read(BMA250E_PMU_LOW_POWER_REG, &reg_data, 1);
    reg_data = (reg_data & BMA250E_LOW_POWER_SLEEP_TIMER_MODE_MASK) | (mode << BMA250E_LOW_POWER_SLEEP_TIMER_MODE_BIT);
    BMA250E_I2C_Write(BMA250E_PMU_LOW_POWER_REG, reg_data, 1);
}

bool BMA250E_Flat_Status(void)
{
    uint8_t reg_data;
    BMA250E_I2C_Read(BMA250E_STAT_FLAT_ORIENT_HIGH_REG, &reg_data, 1);
    if((reg_data&BMA250E_INT_STATUS_FLAT_MASK) == BMA250E_INT_STATUS_FLAT_MASK)
        return true;
    else
        return false;
}

void BMA250E_Setting_Fast_Compensation_Target(uint8_t x, uint8_t y, uint8_t z)
{
    uint8_t reg_data;
    BMA250E_I2C_Read(BMA250E_OFC_SETTING_REG, &reg_data, 1);
    reg_data = reg_data & BMA250E_FFSET_COMPENSATION_MASK;
    reg_data = (reg_data | x << BMA250E_FFSET_COMPENSATION_X_BIT);
    reg_data = (reg_data | y << BMA250E_FFSET_COMPENSATION_Y_BIT);
    reg_data = (reg_data | z << BMA250E_FFSET_COMPENSATION_Z_BIT);
    BMA250E_I2C_Write(BMA250E_OFC_SETTING_REG, reg_data, 1);
}
