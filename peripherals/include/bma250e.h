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
#ifndef __BMA250E_H__
#define __BMA250E_H__
#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define BMA250E_ID 							0xF9
#define BMA2x2_I2C_ADDRESS                  		0x18

//Gemtek default value settings
#define SLOPE_THRESHOLD_VALUE 			0x06
#define SLOPE_DURATION_VALUE 				0x0
//#define BMA250E_RANGE 						BMA2x2_RANGE_8G
#define BMA250E_BANDWIDTH 				BMA2x2_BW_7_81HZ
#define BMA250E_RANGE 						BMA2x2_RANGE_2G
//#define BMA250E_BANDWIDTH 				BMA2x2_BW_7_81HZ

#define BMA2x2_CHIP_ID_REG                      	0x00
#define BMA2x2_X_AXIS_LSB_REG                   	0x02
#define BMA2x2_X_AXIS_MSB_REG                   	0x03
#define BMA2x2_Y_AXIS_LSB_REG                   	0x04
#define BMA2x2_Y_AXIS_MSB_REG                   	0x05
#define BMA2x2_Z_AXIS_LSB_REG                   	0x06
#define BMA2x2_Z_AXIS_MSB_REG                   	0x07
#define BMA2x2_STAT1_REG						0x09
#define BMA2x2_STAT_TAP_SLOPE_REG				0x0B
#define BMA250E_STAT_FLAT_ORIENT_HIGH_REG   	0x0C
#define BMA2x2_RANGE_SELECT_REG                 0x0F
#define BMA2x2_BW_SELECT_REG                    0x10
#define BMA250E_PMU_LPW_REG                     0x11
#define BMA250E_PMU_LOW_POWER_REG               0x12
#define BMA2x2_DATA_CTRL_REG                    0x13
#define BMA2x2_RST_REG                          		0x14

#define BMA2x2_INTR_ENABLE1_REG                 0x16
#define BMA2x2_INTR_ENABLE2_REG                 0x17

#define BMA2x2_INTR1_PAD_SELECT_REG             0x19
#define BMA2x2_INTR_DATA_SELECT_REG             0x1A
#define BMA2x2_INTR2_PAD_SELECT_REG             0x1B
#define BMA2x2_INTR_SOURCE_REG                  0x1E

#define BMA2x2_INTR_SET_REG                     0x20
#define BMA2x2_INTR_CTRL_REG                    0x21

#define BMA2x2_LOW_DURN_REG                     0x22
#define BMA2x2_LOW_THRES_REG                    0x23
#define BMA2x2_LOW_HYST_REG                     0x24

#define BMA2x2_HIGH_DURN_REG                    0x25
#define BMA2x2_HIGH_THRES_REG                   0x26

#define BMA2x2_SLOPE_DURN_REG                   0x27
#define BMA2x2_SLOPE_THRES_REG                  0x28
#define BMA2x2_SLOW_NO_MOTION_THRES_REG         0x29
#define BMA2x2_TAP_THRES_REG                    0x2B

#define BMA2x2_ORIENT_PARAM_REG                 0x2C
#define BMA2x2_THETA_BLOCK_REG                  0x2D

#define BMA2x2_THETA_FLAT_REG                   0x2E
#define BMA2x2_FLAT_HOLD_TIME_REG               0x2F

#define BMA2x2_OFFSET_CTRL_REG                  0x36
#define BMA250E_OFC_SETTING_REG                 0x37
#define BMA2x2_OFFSET_X_AXIS_REG                0x38
#define BMA2x2_OFFSET_Y_AXIS_REG                0x39
#define BMA2x2_OFFSET_Z_AXIS_REG                0x3A

#define BMA2x2_INTR1_OUTPUT      0
#define BMA2x2_INTR2_OUTPUT      1
#define BMA2x2_INTR1_LEVEL       0
#define BMA2x2_INTR2_LEVEL       1
#define BMA2x2_INTR1_ORIENT            0
#define BMA2x2_INTR2_ORIENT            1
#define BMA2x2_INTR1_SLOPE             0
#define BMA2x2_INTR2_SLOPE             1
#define BMA2x2_INTR2_LOW_G             2
#define BMA2x2_INTR1_FLAT         		0
#define BMA2x2_INTR2_FLAT            	1


#define ORIENT_MODE_SYMMETRICAL			0x0
#define ORIENT_MODE_HIGH_ASYMMETRICAL	0x1
#define ORIENT_MODE_LOW_ASYMMETRICAL	0x2
#define ORIENT_MODE_SYMMETRICAL2			0x3


/***************************************************/
/**\name SOFT RESET VALUE   */
/***************************************************/
#define BMA2x2_ENABLE_SOFT_RESET_VALUE        0xB6

#define BMA2x2_ENABLE_SHADOW_DIS       0x40

/******************************************/
/**\name  INTERRUPT TYPE SELECTION     */
/******************************************/
#define BMA2x2_LOW_G_INTR       0
/**< enable/disable low-g interrupt*/
#define BMA2x2_HIGH_G_X_INTR    1
/**< enable/disable high_g X interrupt*/
#define BMA2x2_HIGH_G_Y_INTR    2
/**< enable/disable high_g Y interrupt*/
#define BMA2x2_HIGH_G_Z_INTR    3
/**< enable/disable high_g Z interrupt*/
#define BMA2x2_DATA_ENABLE      4
/**< enable/disable data interrupt*/
#define BMA2x2_SLOPE_X_INTR     5
/**< enable/disable slope X interrupt*/
#define BMA2x2_SLOPE_Y_INTR     6
/**< enable/disable slope X interrupt*/
#define BMA2x2_SLOPE_Z_INTR     7
/**< enable/disable slope X interrupt*/
#define BMA2x2_SINGLE_TAP_INTR  8
/**< enable/disable single tap interrupt*/
#define BMA2x2_DOUBLE_TAP_INTR  9
/**< enable/disable double tap interrupt*/
#define BMA2x2_ORIENT_INTR      10
/**< enable/disable orient interrupt*/
#define BMA2x2_FLAT_INTR        11
/**< enable/disable flat interrupt*/
#define BMA2x2_FIFO_FULL_INTR   12
/**< enable/disable fifo full interrupt*/
#define BMA2x2_FIFO_WM_INTR     13
/**< enable/disable fifo water mark interrupt*/
/**< enable flat  interrupt*/
#define BMA2x2_INTR1_NEWDATA           0
/**< disable data  interrupt*/
#define BMA2x2_INTR2_NEWDATA           1

/**************************************************************/
/**\name	ACCEL RESOLUTION DEFINITION   */
/**************************************************************/
#define BMA2x2_12_RESOLUTION                    0
#define BMA2x2_10_RESOLUTION                    1
#define BMA2x2_14_RESOLUTION                    2
#define RESOLUTION_12_MASK		0xF0
#define RESOLUTION_10_MASK		0xC0
#define RESOLUTION_14_MASK		0xFC
/* Definitions used for accel resolution bit shifting*/
#define BMA2x2_14_BIT_SHIFT		0xFC
/**< It refers 14bit accel resolution*/
#define BMA2x2_10_BIT_SHIFT		0xC0
/**< It refers 10bit accel resolution*/
#define BMA2x2_12_BIT_SHIFT		0xF0

#define BMA250E_10_BIT_SIGN_SHIFT       9
#define BMA250E_10_BIT_MASK        0x3ff
#define BMA250E_10_NEW_DATA_MASK        0x01
#define BMA250E_10_BIT_DATA_SHIFT       6

/******************************************/
/**\name  SOURCE REGISTER     */
/******************************************/
#define BMA2x2_SOURCE_LOW_G            0
#define BMA2x2_SOURCE_HIGH_G           1
#define BMA2x2_SOURCE_SLOPE            2
#define BMA2x2_SOURCE_SLOW_NO_MOTION   3
#define BMA2x2_SOURCE_TAP              4
#define BMA2x2_SOURCE_DATA             5

/******************************************/
/**\name  THRESHOLD     */
/******************************************/
#define BMA2x2_LOW_THRES                0
#define BMA2x2_HIGH_THRES               1
#define BMA2x2_SLOPE_THRES              2
#define BMA2x2_SLOW_NO_MOTION_THRES     3


/******************************************/
/**\name  DURATION     */
/******************************************/
#define BMA2x2_LOW_DURN                0
#define BMA2x2_HIGH_DURN               1
#define BMA2x2_SLOPE_DURN              2
#define BMA2x2_SLOW_NO_MOTION_DURN     3

/******************************************/
/**\name  HYSTERESIS     */
/******************************************/
#define BMA2x2_LOW_HYST                0


/******************************/
/**\name DATA REGISTER-X  */
/******************************/
#define BMA2x2_NEW_DATA_X__POS          0
#define BMA2x2_NEW_DATA_X__LEN          1
#define BMA2x2_NEW_DATA_X__MSK          0x01
#define BMA2x2_NEW_DATA_X__REG          BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACCEL_X14_LSB__POS           2
#define BMA2x2_ACCEL_X14_LSB__LEN           6
#define BMA2x2_ACCEL_X14_LSB__MSK           0xFC
#define BMA2x2_ACCEL_X14_LSB__REG           BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACCEL_X12_LSB__POS           4
#define BMA2x2_ACCEL_X12_LSB__LEN           4
#define BMA2x2_ACCEL_X12_LSB__MSK           0xF0
#define BMA2x2_ACCEL_X12_LSB__REG           BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACCEL_X10_LSB__POS           6
#define BMA2x2_ACCEL_X10_LSB__LEN           2
#define BMA2x2_ACCEL_X10_LSB__MSK           0xC0
#define BMA2x2_ACCEL_X10_LSB__REG           BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACCEL_X8_LSB__POS           0
#define BMA2x2_ACCEL_X8_LSB__LEN           0
#define BMA2x2_ACCEL_X8_LSB__MSK           0x00
#define BMA2x2_ACCEL_X8_LSB__REG           BMA2x2_X_AXIS_LSB_REG

#define BMA2x2_ACCEL_X_MSB__POS           0
#define BMA2x2_ACCEL_X_MSB__LEN           8
#define BMA2x2_ACCEL_X_MSB__MSK           0xFF
#define BMA2x2_ACCEL_X_MSB__REG           BMA2x2_X_AXIS_MSB_REG
/******************************/
/**\name DATA REGISTER-Y  */
/******************************/
#define BMA2x2_NEW_DATA_Y__POS          0
#define BMA2x2_NEW_DATA_Y__LEN          1
#define BMA2x2_NEW_DATA_Y__MSK          0x01
#define BMA2x2_NEW_DATA_Y__REG          BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACCEL_Y14_LSB__POS           2
#define BMA2x2_ACCEL_Y14_LSB__LEN           6
#define BMA2x2_ACCEL_Y14_LSB__MSK           0xFC
#define BMA2x2_ACCEL_Y14_LSB__REG           BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACCEL_Y12_LSB__POS           4
#define BMA2x2_ACCEL_Y12_LSB__LEN           4
#define BMA2x2_ACCEL_Y12_LSB__MSK           0xF0
#define BMA2x2_ACCEL_Y12_LSB__REG           BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACCEL_Y10_LSB__POS           6
#define BMA2x2_ACCEL_Y10_LSB__LEN           2
#define BMA2x2_ACCEL_Y10_LSB__MSK           0xC0
#define BMA2x2_ACCEL_Y10_LSB__REG           BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACCEL_Y8_LSB__POS           0
#define BMA2x2_ACCEL_Y8_LSB__LEN           0
#define BMA2x2_ACCEL_Y8_LSB__MSK           0x00
#define BMA2x2_ACCEL_Y8_LSB__REG           BMA2x2_Y_AXIS_LSB_REG

#define BMA2x2_ACCEL_Y_MSB__POS           0
#define BMA2x2_ACCEL_Y_MSB__LEN           8
#define BMA2x2_ACCEL_Y_MSB__MSK           0xFF
#define BMA2x2_ACCEL_Y_MSB__REG           BMA2x2_Y_AXIS_MSB_REG
/******************************/
/**\name DATA REGISTER-Z  */
/******************************/
#define BMA2x2_NEW_DATA_Z__POS          0
#define BMA2x2_NEW_DATA_Z__LEN          1
#define BMA2x2_NEW_DATA_Z__MSK          0x01
#define BMA2x2_NEW_DATA_Z__REG          BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACCEL_Z14_LSB__POS           2
#define BMA2x2_ACCEL_Z14_LSB__LEN           6
#define BMA2x2_ACCEL_Z14_LSB__MSK           0xFC
#define BMA2x2_ACCEL_Z14_LSB__REG           BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACCEL_Z12_LSB__POS           4
#define BMA2x2_ACCEL_Z12_LSB__LEN           4
#define BMA2x2_ACCEL_Z12_LSB__MSK           0xF0
#define BMA2x2_ACCEL_Z12_LSB__REG           BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACCEL_Z10_LSB__POS           6
#define BMA2x2_ACCEL_Z10_LSB__LEN           2
#define BMA2x2_ACCEL_Z10_LSB__MSK           0xC0
#define BMA2x2_ACCEL_Z10_LSB__REG           BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACCEL_Z8_LSB__POS           0
#define BMA2x2_ACCEL_Z8_LSB__LEN           0
#define BMA2x2_ACCEL_Z8_LSB__MSK           0x00
#define BMA2x2_ACCEL_Z8_LSB__REG           BMA2x2_Z_AXIS_LSB_REG

#define BMA2x2_ACCEL_Z_MSB__POS           0
#define BMA2x2_ACCEL_Z_MSB__LEN           8
#define BMA2x2_ACCEL_Z_MSB__MSK           0xFF
#define BMA2x2_ACCEL_Z_MSB__REG           BMA2x2_Z_AXIS_MSB_REG

/****************************************************/
/**\name  RANGE AND BANDWIDTH SELECT     */
/***************************************************/
#define BMA2x2_RANGE_2G                 3
/**< sets range to +/- 2G mode */
#define BMA2x2_RANGE_4G                 5
/**< sets range to +/- 4G mode */
#define BMA2x2_RANGE_8G                 8
/**< sets range to +/- 8G mode */
#define BMA2x2_RANGE_16G                12
/**< sets range to +/- 16G mode */

#define BMA2x2_BW_7_81HZ        0x08
 /**< sets bandwidth to LowPass 7.81HZ  */
#define BMA2x2_BW_15_63HZ       0x09
/**< sets bandwidth to LowPass 15.63HZ  */
#define BMA2x2_BW_31_25HZ       0x0A
/**< sets bandwidth to LowPass 31.25HZ  */
#define BMA2x2_BW_62_50HZ       0x0B
 /**< sets bandwidth to LowPass 62.50HZ  */
#define BMA2x2_BW_125HZ         0x0C
 /**< sets bandwidth to LowPass 125HZ  */
#define BMA2x2_BW_250HZ         0x0D
/**< sets bandwidth to LowPass 250HZ  */
#define BMA2x2_BW_500HZ         0x0E
/**< sets bandwidth to LowPass 500HZ  */
#define BMA2x2_BW_1000HZ        0x0F
 /**< sets bandwidth to LowPass 1000HZ  */

/****************************************************/
/**\name  MODE SELECT AND SLEEP DURING TIME    */
/***************************************************/
#define BMA250E_MODE_MASK           0x1f
#define BMA250E_MODE_Normal         0x00
#define BMA250E_MODE_DeepSuspend    0x20
#define BMA250E_MODE_LowPower       0x40
#define BMA250E_MODE_Suspend        0x80

#define BMA250E_SLEEP_DURING_MASK   0xe0
#define BMA250E_SLEEP_DURING_0_5ms  0x00
#define BMA250E_SLEEP_DURING_1ms    0x0c
#define BMA250E_SLEEP_DURING_2ms    0x0e
#define BMA250E_SLEEP_DURING_4ms    0x10
#define BMA250E_SLEEP_DURING_6ms    0x12
#define BMA250E_SLEEP_DURING_10ms   0x14
#define BMA250E_SLEEP_DURING_25ms   0x16
#define BMA250E_SLEEP_DURING_50ms   0x18
#define BMA250E_SLEEP_DURING_100ms  0x1a
#define BMA250E_SLEEP_DURING_500ms  0x1c
#define BMA250E_SLEEP_DURING_1s     0x1e

/****************************************************/
/**\name  LOW POWER MODE SELECT AND SLEEP TIMER MODE*/
/***************************************************/
#define BMA250E_LOW_POWER_MODE_MASK           0xBf
#define BMA250E_LOW_POWER_MODE_BIT              6
#define BMA250E_LOW_POWER_SLEEP_TIMER_MODE_MASK 0xDf
#define BMA250E_LOW_POWER_SLEEP_TIMER_MODE_BIT  5

/****************************************************/
/**\name INTERRUPT STATUS FLAT ORIENT HIGH*/
/***************************************************/
#define BMA250E_INT_STATUS_FLAT_MASK           0x80

/****************************************************/
/**\name OFFSET COMPENSATION SETTING                */
/***************************************************/
#define BMA250E_FFSET_COMPENSATION_MASK           0x81
#define BMA250E_FFSET_COMPENSATION_Z_BIT            5
#define BMA250E_FFSET_COMPENSATION_Y_BIT            3
#define BMA250E_FFSET_COMPENSATION_X_BIT            1
#define BMA250E_FFSET_COMPENSATION_CAL_RTY_SHIFT    4
#define BMA250E_FFSET_COMPENSATION_CAL_TRIGGER_SHIFT    5
#define BMA250E_FFSET_COMPENSATION_CAL_TRIGGER_MASK 0x9F

struct BMA250E_Accel_Data{
	int8_t new_data_x;
	int16_t x_lsb;
	int8_t new_data_y;	
	int16_t y_lsb;
	int8_t new_data_z;	
	int16_t z_lsb;
};

struct BMA250E_Compensation_Data{
	int8_t x_offset;
	int8_t y_offset;	
	int8_t z_offset;		
};

typedef void( BMA250EIrqHandler )(void );

void BMA250ESetCallback(BMA250EIrqHandler *irqHandler1, BMA250EIrqHandler *irqHandler2 );

void BMA250EInterruptSignal1( void );

void BMA250EInterruptSignal2( void );

uint8_t BMA250EInit( void );

void BMA250EDeInit( void );

uint8_t BMA250E_I2C_Write( uint8_t addr, uint8_t data, uint8_t size );

uint8_t BMA250E_I2C_Read( uint8_t addr, uint8_t *data, uint8_t size);

uint8_t BMA250EGetDeviceAddr( void );

/*!
 *	@brief This API is used to set shadow dis
 *	in the register 0x13 bit 6
 *
 *  @param  isDisable : The value of shadow dis disable/enable *
 *       ----------------- | ------------------
 *              0          | enable MSB Lock
 *              1          | disable MSB Lock
 */
void BMA250E_ShadowDis_Disable(bool isDisable);

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
void BMA250E_Bandwidth_Set(uint8_t bandwidth);

void BMA250E_Range_Set(uint8_t range);

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
void BMA250E_INT_Enable_Set(uint8_t int_type, bool isEnable);

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
void BMA250E_INT_Source_Set(uint8_t channel, uint8_t isEnable);

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
void BMA250E_INT_OutputType_Set(uint8_t channel, bool isOpenDrain);

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
void BMA250E_INT_NewData_Set(uint8_t channel, bool isEnable);

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
void BMA250E_INT_Level_Set(uint8_t channel, bool isActiveHigh);

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
void BMA250E_INT_Latched_Mode_Set(bool isEnable);

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
void BMA250E_INT_Compensation_Set(uint8_t channel);

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
void BMA250E_INT_RST(bool isReset);

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
void BMA250E_INT_Slope_Set(uint8_t channel, bool isEnable);

/*!
 * @brief This API is used to set the threshold of
 *	Low, High, Slope and slow no motion interrupts in the registers
 *	@note LOW_THRES		-> register 0x23 bit form 0 to 7
 *	@note HIGH_THRES		-> register 0x26 bit form 0 to 7
 *	@note SLOPE_THRES		-> register 0x28 bit form 0 to 7
 *	@note SLO_NO_MOT_THRES -> register 0x29 bit form 0 to 7
 *
 *  @param v_channel_u8: The value of threshold selection
 *     v_channel_u8   | result
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
void BMA250E_INT_Slope_Threshold_Set(uint8_t channel, uint8_t threshold);


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
void BMA250E_INT_Slope_Duration_Set(uint8_t channel, uint8_t duration);

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
void BMA250E_INT_Orient_Set(uint8_t channel, uint8_t isEnable);

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
void BMA250E_INT_Orient_Mode_Set(uint8_t mode);

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
void BMA250E_INT_Orient_Block_Mode_Set(uint8_t mode);

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
void BMA250E_INT_Orient_Block_Threshold_Set(double deg_angle);

/*!
 *	@brief This API is used to set
 *	the interrupt enable of flat hysteresis("flat_hy)
 *	in the register 0x2C bit 4 to 6
 *
 *  @param flat_hy : The value of flat hysteresis
 */
void BMA250E_INT_Orient_Hyst_Set(uint8_t flat_hy);

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
void BMA250E_INT_Flat_Set(uint8_t channel, uint8_t isEnable);

/*!
 *	@brief  This API is used to set
 *	the theta value of flat interrupts
 *	@note FLAT_THETA   -> register 0x2E bit 0 to 5
 *
 *  @param deg_angle: The value of theta angle
 * @note FLAT_THETA : Defines a blocking angle between 0 deg to 44.8 deg
 *
 */
void BMA250E_INT_Flat_Block_Threshold_Set(double deg_angle);

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
 void BMA250E_INT_Flat_Hold_Time_Set(uint8_t hold_time);

/*!
 *	@brief This API is used to set
 *	the interrupt enable of flat hysteresis("flat_hy)
 *	in the register 0x2F bit 0 to 2
 *
 *  @param flat_hy : The value of flat hysteresis
 */
void BMA250E_INT_Flat_Hyst_Set(uint8_t flat_hy);

/*!
 *	@brief This API read interrupt status0 & status2 of flat, orient, single tap,
 *	double tap, slow no motion, slope, highg and lowg from location 09h/0Bh
 *
 *
 *
 *	@param  state : The value of interrupt status
*/
void BMA250E_INT_Status0_Get(void);
void BMA250E_INT_Status2_Get(void);

void BMA250E_Polling_Data(struct BMA250E_Accel_Data *data);
void BMA250E_Polling_Compensation_Offset(uint8_t *ofx, uint8_t *ofy, uint8_t *ofz);
void BMA250E_Mode_Setting(uint8_t mode);
/*!
 *	@brief This API set operation mode 
 *
 *	@param  mode : 
 *           register 11 bit from 7 to 5
 *           0x80      suspend mode 
 *           0x40      low power
 *           0x20      deep suspend
 *           0x00      normal
*/
void BMA250E_Sleep_During_Setting(uint8_t time);
/*!
 *	@brief This API set low power mode sleep time 
 *
 *	@param  mode : 
 *           register 11 bit from 4 to 1
 *           0x00      0.5ms 
 *           0x0c      1ms
 *           0x0e      2ms
 *           0x10      4ms
 *           0x12      6ms
 *           0x14      10ms
 *           0x16      25ms
 *           0x18      50ms
 *           0x1a      100ms
 *           0x1c      500ms
 *           0x1e      1s
*/

void BMA250E_Low_Power_Mode(uint8_t mode);
/*!
 *	@brief This API set low power mode 1 or 2
 *
 *	@param  mode : 0 is low power mode1, 1 is low power mode2
*/

void BMA250E_Low_Power_Sleep_Timer_Mode(uint8_t mode);
/*!
 *	@brief This API set low power sleep timer mode EDT or EST
 *
 *	@param  mode : 0 is EDT, 1 is EST
*/

bool BMA250E_Flat_Status(void);
/*!
 *	@brief This API read flat status is 
 *  flat position or not flat position.
 *
 *	@return : ture is flat position, false is not
*/

void BMA250E_Setting_Fast_Compensation_Target(uint8_t x, uint8_t y, uint8_t z);
/*!
 *	@brief This API set x/y/z target for 
 *  fast compensation.
 *
 *	@@param  0x00 is 0g, 0x01 is 1g, 0x02 is -1g, 0x3 is 0g
*/

void BMA250E_INT_Slope_Hysteresis_Set(uint8_t channel, uint8_t hysteresis);
#ifdef __cplusplus
}
#endif
#endif // __BMA250E_H__
