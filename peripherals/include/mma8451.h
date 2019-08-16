/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Driver for the MMA8451 Accelerometer

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#ifndef __MMA8451_H__
#define __MMA8451_H__

#include <stdint.h>
#include <stdbool.h>

/*
 * MMA8451 I2C address
 */ 
#define MMA8451_I2C_ADDRESS                         0x1C

/*
 * MMA8451 Registers
 */ 
#define MMA8451_DATA_STATUS                         0x00
#define MMA8451_OUT_X_MSB                           0x01
#define MMA8451_OUT_X_LSB                           0x02
#define MMA8451_OUT_Y_MSB                           0x03
#define MMA8451_OUT_Y_LSB                           0x04
#define MMA8451_OUT_Z_MSB                           0x05
#define MMA8451_OUT_Z_LSB                           0x06
#define MMA8451_FIFO_SETUP                          0x09
#define MMA8451_TRIGGER_CONFIG                      0x0A
#define MMA8451_SYS_MODE                            0x0B
#define MMA8451_SYS_INT_STATUS                      0x0C
#define MMA8451_ID                                  0x0D
#define MMA8451_ID_XYZ_DATA_CFG                     0x0E

#define MMA8451_HP_FILTER_CUTOFF                    0x0F
#define MMA8451_PL_STATUS                           0x10
#define MMA8451_PL_CFG                              0x11
#define MMA8451_PL_COUNT                            0x12
#define MMA8451_PL_BF_ZCOMP                         0x13
#define	MMA8451_PL_THS_REG                          0x14
#define	MMA8451_FF_MF_CFG                           0x15
#define	MMA8451_FF_MF_SRC                           0x16
#define	MMA8451_FF_MF_THS                           0x17
#define	MMA8451_FF_MF_COUNT                         0x18

#define	MMA8451_TRANSIENT_CFG                       0x1D
#define	MMA8451_TRANSIENT_SRC                       0x1E
#define	MMA8451_TRANSIENT_THS                       0x1F
#define	MMA8451_TRANSIENT_COUNT                     0x20

#define	MMA8451_PULSE_CFG                           0x21
#define	MMA8451_PULSE_SRC                           0x22
#define	MMA8451_PULSE_THSX                          0x23
#define	MMA8451_PULSE_THSY                          0x24
#define	MMA8451_PULSE_THSZ                          0x25
#define	MMA8451_PULSE_TMLT                          0x26
#define	MMA8451_PULSE_LTCY                          0x27
#define	MMA8451_PULSE_WIND                          0x28

#define MMA8451_CTRL_REG_1                          0x2A
#define MMA8451_CTRL_REG_2                          0x2B
#define MMA8451_CTRL_REG_3                          0x2C
#define MMA8451_CTRL_REG_4                          0x2D
#define MMA8451_CTRL_REG_5                          0x2E

#define MMA8451_INT_SRC_DRDY                        0x01
#define MMA8451_INT_SRC_FF_MT                       0x04
#define MMA8451_INT_SRC_PULSE                       0x08
#define MMA8451_INT_SRC_LNDPRT                      0x10
#define MMA8451_INT_SRC_TRANS                       0x20
#define MMA8451_INT_SRC_FIFO                        0x40
#define MMA8451_INT_SRC_ASLP                        0x80

#define MMA8451_INT_EN_DRDY                         0x01
#define MMA8451_INT_EN_FF_MT                        0x04
#define MMA8451_INT_EN_PULSE                        0x08
#define MMA8451_INT_EN_LNDPRT                       0x10
#define MMA8451_INT_EN_TRANS                        0x20
#define MMA8451_INT_EN_FIFO                         0x40
#define MMA8451_INT_EN_ASLP                         0x80

/* Fraction Values */
#define FRAC_2d1    5000
#define FRAC_2d2    2500
#define FRAC_2d3    1250
#define FRAC_2d4    625
#define FRAC_2d5    313
#define FRAC_2d6    156
#define FRAC_2d7    78
#define FRAC_2d8    39
#define FRAC_2d9    20
#define FRAC_2d10   10
#define FRAC_2d11   5
#define FRAC_2d12   2

#define UINT2               2+1
#define UINT4               4+1
#define UINT8               8+1
#define UINT16              16+1

#define RES_14_SCALE_2G                     0.00024414  // ~= 0.25 mg
#define RES_14_SCALE_4G                     0.00048828  // ~= 0.5 mg
#define RES_14_SCALE_8G                     0.000976    // ~= 1.0 mg

#define RES_8_SCALE_2G                      0.0156      // 15.6 mg
#define RES_8_SCALE_4G                      0.03125     // 31.25 mg
#define RES_8_SCALE_8G                      0.0625      // 62.5 mg

#define RES_14_SIGNED_MASK                  (uint16_t) 0x7FFF
#define RES_8_SIGNED_MASK                   (uint16_t) 0x7F

#define RES_14_FRAC_MASK                    (uint16_t) 0x3FFF // this is for 2G SCALE
#define RES_8_FRAC_MASK                     (uint16_t) 0x3F // this is for 2G SCALE

#define RES_14_SIGNED_COMP_MASK             0x8000 // don't use ~RES_14_SIGNED_MASK directly, will cause error
#define	RES_8_SIGNED_COMP_MASK              0x80

#define RES_14_INT_MASK                     0x4000
#define RES_8_INT_MASK                      0x40

#define RES_14_UNSIGNED_OFFSET              ( RES_14_SIGNED_MASK >> 2 ) //0x1FFF  //already shifted
#define RES_14_SIGNED_OFFSET                0x0

#define MMA8451_FF_MF_ELE                   0x80 /*event latch enable*/
#define MMA8451_FF_MF_MASK                  0x40 /*bit6 OAE FF is 0, MF is 1*/
#define MMA8451_FF_MF_ZEFE                  0x20
#define MMA8451_FF_MF_YEFE                  0x10
#define MMA8451_FF_MF_XEFE                  0x08

#define MMA8451_REG_MASK_BIT_0              0x01
#define MMA8451_REG_MASK_BIT_1              0x02
#define MMA8451_REG_MASK_BIT_2              0x04
#define MMA8451_REG_MASK_BIT_3              0x08
#define MMA8451_REG_MASK_BIT_4              0x10
#define MMA8451_REG_MASK_BIT_5              0x20
#define MMA8451_REG_MASK_BIT_6              0x40
#define MMA8451_REG_MASK_BIT_7              0x80

#define MMA8451_REG_MASK_BIT_8              0x0100
#define MMA8451_REG_MASK_BIT_9              0x0200
#define MMA8451_REG_MASK_BIT_10             0x0400
#define MMA8451_REG_MASK_BIT_11             0x0800
#define MMA8451_REG_MASK_BIT_12             0x1000
#define MMA8451_REG_MASK_BIT_13             0x2000
#define MMA8451_REG_MASK_BIT_14             0x4000
#define MMA8451_REG_MASK_BIT_15             0x8000

#define RES_14_SIGNED_BIT                   MMA8451_REG_MASK_BIT_15

/*int1/2 tyep*/
typedef enum {
    MMA8451_INT1 = 0,
    MMA8451_INT2,
} MMA8451_INT_Type;

/*Montion/Freefall tyep*/
typedef enum {
    MMA8451_FF = 0,
    MMA8451_MF,
} MMA8451_FF_MT_Type;

/* Orientation type*/
typedef enum {
    X_AXIS = 0,
    Y_AXIS,
    Z_AXIS
} Axis_Type;

/*Active Modes*/
typedef enum {
    FULL_SCALE_2G = 0,
    FULL_SCALE_4G,
    FULL_SCALE_8G
} Full_Scale;

typedef enum {
    MMA8451_STANDBY_MODE = 0,
    MMA8451_ACTIVE_MODE,
    MMA8451_ACTIVE_WAKE,
    MMA8451_ACTIVE_SLEEP
} MMA8451_System_Mode;

typedef enum {
    MMA8451_SR_2G = 0,
    MMA8451_SR_4G,
    MMA8451_SR_8G
} MMA8451_Scale_Range;

typedef enum {
    MMA8451_DR_800_HZ = 0,
    MMA8451_DR_400_Hz,
    MMA8451_DR_200_Hz,
    MMA8451_DR_100_Hz,
    MMA8451_DR_50_Hz,
    MMA8451_DR_12_5_Hz,
    MMA8451_DR_6_25_Hz,
    MMA8451_DR_1_563_Hz
} MMA8451_Data_Rate;

typedef enum {
    MMA8451_NORMAL = 0,
    MMA8451_LOW_NOISE_LOW_POWER,
    MMA8451_HIGH_RESOLUTION,
    MMA8451_LOW_POWER
} MMA8451_Power_Mode;

typedef enum {
    MMA8451_MSB = 0,
    MMA8451_LSB,
    
    MMA8451_X_MSB = 0,
    MMA8451_X_LSB,
    MMA8451_Y_MSB,
    MMA8451_Y_LSB,
    MMA8451_Z_MSB,
    MMA8451_Z_LSB,
} MMA8451DataType;

typedef enum {
    MMA8451_RES_8_BIT = 0,
    MMA8451_RES_14_BIT
} MMA8451ResType;


/*!
 * \brief Initializes the device
 *
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MMA8451Init( void );

/*!
 * \brief Resets the device
 *
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MMA8451Reset( void );

/*!
 * \brief Writes a byte at specified address in the device
 *
 * \param [IN]: addr
 * \param [IN]: data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MMA8451Write( uint8_t addr, uint8_t data );

/*!
 * \brief Writes a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [IN]: data
 * \param [IN]: size
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MMA8451WriteBuffer( uint8_t addr, uint8_t *data, uint8_t size );

/*!
 * \brief Reads a byte at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MMA8451Read( uint8_t addr, uint8_t *data );

/*!
 * \brief Reads a buffer at specified address in the device
 *
 * \param [IN]: addr
 * \param [OUT]: data
 * \param [IN]: size
 * \retval status [SUCCESS, FAIL]
 */
uint8_t MMA8451ReadBuffer( uint8_t addr, uint8_t *data, uint8_t size );

/*!
 * \brief Sets the I2C device slave address
 *
 * \param [IN]: addr
 */
void MMA8451SetDeviceAddr( uint8_t addr );

/*!
 * \brief Gets the I2C device slave address
 *
 * \retval: addr Current device slave address
 */
uint8_t MMA8451GetDeviceAddr( void );

/*
    14-bit for X, Y and X
*/
struct MMA8451_output_data{
    uint8_t x_val_hi;
    uint8_t x_val_lo;
    uint8_t y_val_hi;
    uint8_t y_val_lo;
    uint8_t z_val_hi;
    uint8_t z_val_lo;
};

uint8_t MMA8451ReadN( uint8_t addr, uint8_t *data, uint8_t size );
void MMA8451_ModeSetting( uint8_t mode );
void MMA8451_RangeSetting( uint8_t range );
void MMA8451_FastReadModeSetting( bool status );
void MMA8451_DataRateSetting( uint8_t drate );
void MMA8451_OversamplingModeSetting( uint8_t status, uint8_t mode );
void MMA8451_Polling_Data( struct MMA8451_output_data *data );
void SettingsForMMA8451( void );
int16_t MMA8451ConvertToNumerical( uint16_t AxisRaw, MMA8451ResType Res, MMA8451_Scale_Range Scale );
void MMA8451GetLatestValue( int16_t *MMA8451NumericalData_X,  int16_t *MMA8451NumericalData_Y, int16_t *MMA8451NumericalData_Z );
void MMA8451GetLatestValue_withRAW(int16_t *MMA8451NumericalData_X,  int16_t *MMA8451NumericalData_Y, int16_t *MMA8451NumericalData_Z, 
        int16_t *MMA8451NumericalData_X_raw,  int16_t *MMA8451NumericalData_Y_raw, int16_t *MMA8451NumericalData_Z_raw);
void MMA8451_OrientationDetectSettings( void );

/*!
 * \brief set motion/fallfree detection 
 *  always use 12.5hz normal mode 
 *  Event flag latch disabled
 * \param [IN]: int_num(int1/2)
 * \param [IN]: type
 * \param [IN]: force(max 8g)
 * \param [IN]: type(5.1sec)
 * \retval void
 */
void MMA8451_FF_MF_Settings(MMA8451_INT_Type int_num, MMA8451_FF_MT_Type type, float force, float sec);
void MMA8451_DRDY_Settings(void);
void MMA8451_FIFO_Settings(void);

/*!
 * \brief set transient detection 
 *  always use 12.5hz normal mode
 *  Event flag latch disabled
 * \param [IN]: int_num(int1/2)
 * \param [IN]: type
 * \param [IN]: force(max 8g)
 * \param [IN]: type(0.319sec)
 * \retval void
 */
void MMA8451_Transient_Settings(MMA8451_INT_Type int_num, float force, float sec);

typedef void(MMA8451IrqHandler)(void);
void MMA8451SetCallback_1(MMA8451IrqHandler *irqHandler);
void MMA8451SetCallback_2(MMA8451IrqHandler *irqHandler);
void MMA8451InterruptSignal_1(void);
void MMA8451InterruptSignal_2(void);

#endif  // __MMA8451_H__
