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
#include "board.h"
#include "i2c-board.h"
#include "mma8451.h"
#include <stdio.h>
#include <string.h>

static uint8_t I2cDeviceAddr = 0;

static MMA8451IrqHandler *MMA8451Callback_1 = NULL;
static MMA8451IrqHandler *MMA8451Callback_2 = NULL;

void MMA8451SetCallback_1(MMA8451IrqHandler *irqHandler)
{
    MMA8451Callback_1 = irqHandler;
}

void MMA8451SetCallback_2(MMA8451IrqHandler *irqHandler)
{
    MMA8451Callback_2 = irqHandler;
}

void MMA8451InterruptSignal_1(void)
{
    if(MMA8451Callback_1 != NULL)
        MMA8451Callback_1();
}

void MMA8451InterruptSignal_2(void)
{
    if(MMA8451Callback_2 != NULL)
        MMA8451Callback_2();
}

uint8_t MMA8451Init( void )
{
    uint8_t regVal = 0;

    MMA8451SetDeviceAddr( MMA8451_I2C_ADDRESS );

    MMA8451Read( MMA8451_ID, &regVal );

    if( regVal != 0x1A )   // Fixed Device ID Number = 0x1A
    {
        return FAIL;
    }
    /*The I2C communication system is reset to avoid accidental corrupted data access.*/
    //MMA8451Reset( );
    return SUCCESS;
}


uint8_t MMA8451Reset( )
{
    if( MMA8451Write( MMA8451_CTRL_REG_2, 0x40 ) == SUCCESS ) // Reset the MMA8451 with CTRL_REG2
    {
        return SUCCESS;
    }
    return FAIL;
}

uint8_t MMA8451Write( uint8_t addr, uint8_t data )
{
    return MMA8451WriteBuffer( addr, &data, 1 );
}

uint8_t MMA8451WriteBuffer( uint8_t addr, uint8_t *data, uint8_t size )
{
    return I2cWriteBuffer( I2cDeviceAddr << 1, addr, data, size );
}

uint8_t MMA8451Read( uint8_t addr, uint8_t *data )
{
    return MMA8451ReadBuffer( addr, data, 1 );
}

uint8_t MMA8451ReadN( uint8_t addr, uint8_t *data, uint8_t size )
{
    return MMA8451ReadBuffer( addr, data, size );
}


uint8_t MMA8451ReadBuffer( uint8_t addr, uint8_t *data, uint8_t size )
{
    return I2cReadBuffer( I2cDeviceAddr << 1, addr, data, size );
}

void MMA8451SetDeviceAddr( uint8_t addr )
{
    I2cDeviceAddr = addr;
}

uint8_t MMA8451GetDeviceAddr( void )
{
    return I2cDeviceAddr;
}

void MMA8451_ModeSetting( uint8_t mode )
{
    uint8_t regAdd = 0;
    uint8_t regVal = 0;

    regAdd = MMA8451_CTRL_REG_1;

    MMA8451Read( regAdd, &regVal );

    if( mode == MMA8451_STANDBY_MODE ) {
        //Enable Standby mode
        MMA8451Write( regAdd, regVal & ~MMA8451_REG_MASK_BIT_0 );
    }
    else if( mode == MMA8451_ACTIVE_MODE ) {
        //Enable Active mode
        MMA8451Write( regAdd, regVal | MMA8451_REG_MASK_BIT_0 );
    }

}
void MMA8451_FastReadModeSetting( bool status )
{
    uint8_t regVal = 0;
    uint8_t regAdd = 0;

    regAdd = MMA8451_CTRL_REG_1;

    MMA8451Read( regAdd, &regVal );

    if(status)
        MMA8451Write( regAdd, regVal | MMA8451_REG_MASK_BIT_1 );
    else
        MMA8451Write( regAdd, regVal & ~MMA8451_REG_MASK_BIT_1 );

    return;
}

void MMA8451_RangeSetting( uint8_t range )
{
    uint8_t regAdd = 0;
    uint8_t regVal = 0;

    regAdd = MMA8451_ID_XYZ_DATA_CFG;

    MMA8451Read( regAdd, &regVal );

    if(range == MMA8451_SR_2G){
        MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_0);
        MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_1);
    }else if(range == MMA8451_SR_4G){
        MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_0);
        MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_1);
    }else if(range == MMA8451_SR_8G){
        MMA8451Write(regAdd, regVal & ~ MMA8451_REG_MASK_BIT_0);
        MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_1);
    }
}

void MMA8451_DataRateSetting( uint8_t drate )
{
    uint8_t regVal = 0;
    uint8_t regAdd = 0;

    regAdd = MMA8451_CTRL_REG_1;

    MMA8451Read( regAdd, &regVal );

    switch(drate){
        case MMA8451_DR_800_HZ:
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_3);
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_4);
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_5);
            break;
        case MMA8451_DR_400_Hz:
            MMA8451Write(regAdd, regVal |  MMA8451_REG_MASK_BIT_3);
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_4);
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_5);
            break;
        case MMA8451_DR_200_Hz:
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_3);
            MMA8451Write(regAdd, regVal |  MMA8451_REG_MASK_BIT_4);
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_5);
            break;
        case MMA8451_DR_100_Hz:
            MMA8451Write(regAdd, regVal |  MMA8451_REG_MASK_BIT_3);
            MMA8451Write(regAdd, regVal |  MMA8451_REG_MASK_BIT_4);
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_5);
            break;
        case MMA8451_DR_50_Hz:
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_3);
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_4);
            MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_5);
            break;
        case MMA8451_DR_12_5_Hz:
            MMA8451Write(regAdd, regVal |  MMA8451_REG_MASK_BIT_3);
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_4);
            MMA8451Write(regAdd, regVal |  MMA8451_REG_MASK_BIT_5);
            break;
        case MMA8451_DR_6_25_Hz:
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_3);
            MMA8451Write(regAdd, regVal |  MMA8451_REG_MASK_BIT_4);
            MMA8451Write(regAdd, regVal |  MMA8451_REG_MASK_BIT_5);
            break;
        case MMA8451_DR_1_563_Hz:
            MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_3);
            MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_4);
            MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_5);
            break;
        default:
            break;
    }
}

void MMA8451_OversamplingModeSetting( uint8_t status, uint8_t mode )
{
    uint8_t regVal = 0;
    uint8_t regAdd = 0;

    regAdd = MMA8451_CTRL_REG_2;
    MMA8451Read( regAdd, &regVal );

    if(mode == MMA8451_NORMAL){
        if(status == MMA8451_ACTIVE_SLEEP){
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_3);
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_4);
        }else if(status == MMA8451_ACTIVE_WAKE){
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_0);
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_1);
        }
    }else if(mode == MMA8451_LOW_NOISE_LOW_POWER){
        if(status == MMA8451_ACTIVE_SLEEP){
            MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_3);
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_4);
        }else if(status == MMA8451_ACTIVE_WAKE){
            MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_0);
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_1);
        }
    }else if(mode == MMA8451_HIGH_RESOLUTION){
        if(status == MMA8451_ACTIVE_SLEEP){
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_3);
            MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_4);
        }else if(status == MMA8451_ACTIVE_WAKE){
            MMA8451Write(regAdd, regVal & ~MMA8451_REG_MASK_BIT_0);
            MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_1);
        }
    }else if(mode == MMA8451_LOW_POWER){
        if(status == MMA8451_ACTIVE_SLEEP){
            MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_3);
            MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_4);
        }else if(status == MMA8451_ACTIVE_WAKE){
            MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_0);
            MMA8451Write(regAdd, regVal | MMA8451_REG_MASK_BIT_1);
        }
    }
}

void MMA8451_Polling_Data( struct MMA8451_output_data *data )
{
    uint8_t regVal = 0;
    char bin_str[8];
    int i = 0;

    MMA8451Read( MMA8451_DATA_STATUS, &regVal );

    strcpy(bin_str,"");

    for(i=7;i>=0;i--) {
        sprintf(bin_str,"%s%d",bin_str,(regVal&(1<<i))>>i);
    }

    if(bin_str[4] == '1')
    { //ZYXDR Bit

        MMA8451Read(MMA8451_OUT_X_MSB,&regVal);
        data->x_val_hi = regVal;
        MMA8451Read(MMA8451_OUT_Y_MSB,&regVal);
        data->y_val_hi = regVal;
        MMA8451Read(MMA8451_OUT_Z_MSB,&regVal);
        data->z_val_hi = regVal;
        MMA8451Read(MMA8451_OUT_X_LSB,&regVal);
        data->x_val_lo = regVal;
        MMA8451Read(MMA8451_OUT_Y_LSB,&regVal);
        data->y_val_lo = regVal;
        MMA8451Read(MMA8451_OUT_Z_LSB,&regVal);
        data->z_val_lo = regVal;
    }
    else
    {
            data->x_val_lo = 0xaa;
            data->y_val_lo = 0xbb;
            data->z_val_lo = 0xcc;
    }

    return;
}

void SettingsForMMA8451( )
{
    MMA8451_ModeSetting( MMA8451_STANDBY_MODE );
    MMA8451_DataRateSetting( MMA8451_DR_12_5_Hz );

    //MMA8451_OrientationDetectSettings( );

    MMA8451_RangeSetting(MMA8451_SR_2G);
    MMA8451_ModeSetting(MMA8451_ACTIVE_MODE);
    MMA8451_FastReadModeSetting( false );
}


/*
    axis_data : The 3-axis data.
    data_hi   : The most significant bit set for 14-bit output.
    scale     : Dynamically selectable full-scale. (2g , 4g or 8g)
*/

//int16_t MMA8451ConvertToNumerical( uint16_t AxisRaw, MMA8451ResType Res, MMA8451_Scale_Range Scale )
//{
//    uint16_t GValOffset;
//    float GValPerCount = 0;
//    float GVal = 0;
//
//    if( AxisRaw == 0x0 )
//    {
//            return 0;
//    }
//
//    if( Res == MMA8451_RES_8_BIT )
//    {
//            // TBD
//
//            // 8-Bit Resolution
////              GValOffset = ( AxisRaw & RES_8_SIGNED_MASK ) >> 1;
////
////              switch( Scale )
////              {
////                      case MMA8451_SR_2G:
////                              GValPerCount = RES_8_SCALE_2G;
////                              break;
////
////                      case MMA8451_SR_4G:
////                              GValPerCount = RES_8_SCALE_4G;
////                              break;
////
////                      case MMA8451_SR_8G:
////                              GValPerCount = RES_8_SCALE_8G;
////                              break;
////              }
//
//
//    }
//    else
//    {
//        // 14-Bit Resolution
//
//        switch( Scale )
//        {
//            case MMA8451_SR_2G:
//
//                    GValOffset = ( AxisRaw & RES_14_FRAC_MASK ) >> 2 ;
//
//                    GValPerCount = RES_14_SCALE_2G;
//
//                    GVal = ( ( AxisRaw & RES_14_SIGNED_COMP_MASK ) == RES_14_SIGNED_COMP_MASK ) ? -2.0 : 0.0 ;
//                    GVal += ( ( AxisRaw & RES_14_INT_MASK ) == RES_14_INT_MASK ) ? 1.0 : 0.0 ;
//
//                    break;
//
//            case MMA8451_SR_4G:
//
////                              GValOffset = ( AxisRaw & RES_14_FRAC_MASK ) >> 2 ;
////                              GValPerCount = RES_14_SCALE_4G;
////                              GVal = ( ( AxisRaw & ~RES_14_SIGNED_MASK ) == ~RES_14_SIGNED_MASK ) ? -4.0 : 3.9995 ;
//
//                    break;
//
//            case MMA8451_SR_8G:
//
////                              GValOffset = ( AxisRaw & RES_14_FRAC_MASK ) >> 2 ;
////                              GValPerCount = RES_14_SCALE_8G;
////                              GVal = ( ( AxisRaw & ~RES_14_SIGNED_MASK ) == ~RES_14_SIGNED_MASK ) ? -8.0 : 7.999 ;
//
//                    break;
//            }
//
//
//            GVal += ( GValOffset * GValPerCount );
//
//    }
//
//    return ( GVal * 1000.0 );
//}

int16_t MMA8451ConvertToNumerical( uint16_t AxisRaw, MMA8451ResType Res, MMA8451_Scale_Range Scale )
{
    int16_t result, divider = 1;
    
    if(AxisRaw == 0 || Res != MMA8451_RES_14_BIT) //8bit need to implement
        return 0;
    
    if (Scale == MMA8451_SR_8G) divider = 1024;
    if (Scale == MMA8451_SR_4G) divider = 2048;
    if (Scale == MMA8451_SR_2G) divider = 4096;
    
    result = AxisRaw >> 2;
    if((AxisRaw & RES_14_SIGNED_BIT) == RES_14_SIGNED_BIT)
        result |= 0xC000;
    
    result = ( ( (float)result ) / divider ) * 1000;
    return result;
}

void MMA8451GetLatestValue( int16_t *MMA8451NumericalData_X,  int16_t *MMA8451NumericalData_Y, int16_t *MMA8451NumericalData_Z)
{
    int16_t MMA8451NumericalData_X_raw, MMA8451NumericalData_Y_raw, MMA8451NumericalData_Z_raw;
    
    MMA8451GetLatestValue_withRAW(MMA8451NumericalData_X, MMA8451NumericalData_Y, MMA8451NumericalData_Z,
        &MMA8451NumericalData_X_raw, &MMA8451NumericalData_Y_raw, &MMA8451NumericalData_Z_raw);
    
    return;
}

void MMA8451GetLatestValue_withRAW( int16_t *MMA8451NumericalData_X,  int16_t *MMA8451NumericalData_Y, int16_t *MMA8451NumericalData_Z, 
    int16_t *MMA8451NumericalData_X_raw,  int16_t *MMA8451NumericalData_Y_raw, int16_t *MMA8451NumericalData_Z_raw)
{
//    char bin_str[8];
    uint8_t regVal = 0;
//    int8_t idx = 0;
    uint8_t RawData[6];
    int16_t ReadyToConvert;

    MMA8451Read( MMA8451_DATA_STATUS, &regVal );

//    strcpy( bin_str, "" );
//
//    for( idx = 7 ; idx >= 0 ; idx-- )
//    {
//        sprintf( bin_str, "%s%d", bin_str, ( regVal&( 1 << idx ) ) >> idx );
//    }
//
//    if( ( bin_str[4] == '1' ) && ( bin_str[0] == '1') )
    if( ((regVal & MMA8451_REG_MASK_BIT_7) == MMA8451_REG_MASK_BIT_7) ||  // ZYXRO == 1
        ((regVal & MMA8451_REG_MASK_BIT_3) == MMA8451_REG_MASK_BIT_3) )   // ZYXDR == 1
    {
        //read all axis data
        MMA8451ReadBuffer( MMA8451_OUT_X_MSB, RawData, sizeof( RawData ) );
        
        ReadyToConvert = ( RawData[MMA8451_X_MSB] << 8 ) | RawData[MMA8451_X_LSB];
        // in g, accurates to the 3rd decimal place
        *MMA8451NumericalData_X_raw = ReadyToConvert;
        *MMA8451NumericalData_X = MMA8451ConvertToNumerical( ReadyToConvert, MMA8451_RES_14_BIT, MMA8451_SR_2G );

        ReadyToConvert = ( RawData[MMA8451_Y_MSB] << 8 ) | RawData[MMA8451_Y_LSB];
        // in g, accurates to the 3rd decimal place
        *MMA8451NumericalData_Y_raw = ReadyToConvert;
        *MMA8451NumericalData_Y = MMA8451ConvertToNumerical( ReadyToConvert, MMA8451_RES_14_BIT, MMA8451_SR_2G );

        ReadyToConvert = ( RawData[MMA8451_Z_MSB] << 8 ) | RawData[MMA8451_Z_LSB];
        // in g, accurates to the 3rd decimal place
        *MMA8451NumericalData_Z_raw = ReadyToConvert;
        *MMA8451NumericalData_Z = MMA8451ConvertToNumerical( ReadyToConvert, MMA8451_RES_14_BIT, MMA8451_SR_2G );
    }
}

void MMA8451_OrientationDetectSettings( void )
{
        uint8_t regVal = 0;

        MMA8451Read( MMA8451_PL_CFG, &regVal );
        MMA8451Write( MMA8451_PL_CFG, regVal | 0x40 );

        MMA8451Read( MMA8451_PL_BF_ZCOMP, &regVal );
        regVal &= 0x3F;
        //choose between 0x00, 0x40, 0x80, 0xC0, front back detection
        // Z-tilt, 90, 60, 45, 30 degrees
        MMA8451Write( MMA8451_PL_BF_ZCOMP, regVal | 0x80 );

        MMA8451Read( MMA8451_PL_BF_ZCOMP, &regVal );
        regVal &= 0xF8;
        // choose between Z-lockout angle
        MMA8451Write( MMA8451_PL_BF_ZCOMP, regVal | 0x00 );     //set to 14 degrees
//      MMA8451Write( MMA8451_PL_BF_ZCOMP, regVal | 0x01 );     //set to 18 degrees
//      MMA8451Write( MMA8451_PL_BF_ZCOMP, regVal | 0x02 );     //set to 21 degrees
//      MMA8451Write( MMA8451_PL_BF_ZCOMP, regVal | 0x03 );     //set to 25 degrees
//      MMA8451Write( MMA8451_PL_BF_ZCOMP, regVal | 0x04 );     //set to 29 degrees
//      MMA8451Write( MMA8451_PL_BF_ZCOMP, regVal | 0x05 );     //set to 33 degrees
//      MMA8451Write( MMA8451_PL_BF_ZCOMP, regVal | 0x06 );     //set to 37 degrees
//      MMA8451Write( MMA8451_PL_BF_ZCOMP, regVal | 0x07 );     //set to 42 degrees

        MMA8451Read( MMA8451_PL_THS_REG, &regVal );
        regVal &= 0x07;
        //Sets trip threshold to 20 degrees
        MMA8451Write( MMA8451_PL_THS_REG, regVal | (0x09) << 3 );

        MMA8451Read( MMA8451_PL_THS_REG, &regVal );
        regVal &= 0xF8;
        // Make sure that there isn't a resulting trip value greater than 90 or less than 0
        //Sets threshold to (+/-)4 degrees
        MMA8451Write( MMA8451_PL_THS_REG, regVal | 0x01 );

        //Interrupt Settings
        MMA8451Read( MMA8451_CTRL_REG_4, &regVal );
        MMA8451Write( MMA8451_CTRL_REG_4, regVal | 0x10 );

//      MMA8451Read( MMA8451_CTRL_REG_5, &regVal );
        // Only route LNDPRT to INT2
        MMA8451Write( MMA8451_CTRL_REG_5, 0xFF );   // route to INT2

        MMA8451Write( MMA8451_PL_COUNT, 0x4b );     // Sets the debounce counter to 1.5s at 12.5 Hz

}

void MMA8451_FF_MF_Settings(MMA8451_INT_Type int_num, MMA8451_FF_MT_Type type, float force, float sec)
{
    uint8_t regVal = 0;
    uint8_t threshold = 0;
    uint8_t decount = 0;

    MMA8451_ModeSetting(MMA8451_STANDBY_MODE);

    MMA8451_OversamplingModeSetting(MMA8451_ACTIVE_WAKE, MMA8451_NORMAL);

    MMA8451_DataRateSetting(MMA8451_DR_12_5_Hz);

    MMA8451Read( MMA8451_FF_MF_CFG, &regVal );
    if(type == MMA8451_FF)
        regVal &= ~MMA8451_FF_MF_MASK;
    else
        regVal |= MMA8451_FF_MF_MASK;

    /*enable x, y and z*/
    regVal |= (MMA8451_FF_MF_ZEFE|MMA8451_FF_MF_YEFE|MMA8451_FF_MF_XEFE);

    MMA8451Write( MMA8451_FF_MF_CFG, regVal );

    threshold = force/RES_8_SCALE_8G;
    MMA8451Write( MMA8451_FF_MF_THS, threshold );

    decount = sec/0.02;/*12.5hz time step 20ms*/

    MMA8451Write( MMA8451_FF_MF_COUNT, decount );

    MMA8451Read( MMA8451_CTRL_REG_4, &regVal );

    regVal |= MMA8451_INT_SRC_FF_MT;

    MMA8451Write( MMA8451_CTRL_REG_4, regVal);

    MMA8451Read( MMA8451_CTRL_REG_5, &regVal );

    if(int_num == MMA8451_INT1)
        regVal |= MMA8451_INT_EN_FF_MT;
    else if(int_num == MMA8451_INT2)
        regVal &= ~MMA8451_INT_EN_FF_MT;

    MMA8451Write( MMA8451_CTRL_REG_5, regVal);

    MMA8451_ModeSetting(MMA8451_ACTIVE_MODE);

    return;
}

void MMA8451_DRDY_Settings(void)
{
    uint8_t regVal = 0;
    MMA8451_ModeSetting(MMA8451_STANDBY_MODE);
    MMA8451_OversamplingModeSetting(MMA8451_ACTIVE_WAKE, MMA8451_NORMAL);

    MMA8451Write(MMA8451_FIFO_SETUP, 0x00);

    MMA8451Read( MMA8451_CTRL_REG_1, &regVal );

    regVal &= ~MMA8451_REG_MASK_BIT_1;

    MMA8451Write( MMA8451_CTRL_REG_1, regVal);

    MMA8451Read( MMA8451_CTRL_REG_4, &regVal );

    regVal |= MMA8451_INT_SRC_DRDY;

    MMA8451Write( MMA8451_CTRL_REG_4, regVal);

    MMA8451Read( MMA8451_CTRL_REG_5, &regVal );

    regVal |= MMA8451_INT_EN_DRDY;

    MMA8451Write( MMA8451_CTRL_REG_5, regVal);

    MMA8451_ModeSetting(MMA8451_ACTIVE_MODE);

}

void MMA8451_FIFO_Settings(void)
{
    uint8_t regVal = 0;
    MMA8451_ModeSetting(MMA8451_STANDBY_MODE);
    MMA8451_OversamplingModeSetting(MMA8451_ACTIVE_WAKE, MMA8451_NORMAL);

    MMA8451Write( MMA8451_CTRL_REG_1, 0x18 );

    MMA8451Write( MMA8451_FIFO_SETUP, 0x40 );

    MMA8451Read( MMA8451_CTRL_REG_4, &regVal );

    regVal |= MMA8451_INT_SRC_FIFO;

    MMA8451Write( MMA8451_CTRL_REG_4, regVal);

    MMA8451Read( MMA8451_CTRL_REG_5, &regVal );

    regVal |= MMA8451_INT_EN_FIFO;

    MMA8451Write( MMA8451_CTRL_REG_5, regVal);

    MMA8451Write( MMA8451_ID_XYZ_DATA_CFG, 0x11);

    MMA8451_ModeSetting(MMA8451_ACTIVE_MODE);

}

void MMA8451_Transient_Settings(MMA8451_INT_Type int_num, float force, float sec)
{
    uint8_t regVal = 0;
    uint8_t threshold = 0;
    uint8_t decount = 0;

    /*Disable MMA8451*/
    MMA8451_ModeSetting(MMA8451_STANDBY_MODE);
    MMA8451_OversamplingModeSetting(MMA8451_ACTIVE_WAKE, MMA8451_NORMAL);
    /*set ODR 800Hz*/
    MMA8451_DataRateSetting(MMA8451_DR_12_5_Hz);
    /*Event flag enable on X/Y/Z transient. Latch enable.*/
    MMA8451Write(MMA8451_TRANSIENT_CFG, 0x16);
    /*Setup threshold 0.0625g/LSB.*/
    threshold = force/RES_8_SCALE_8G;
    MMA8451Write(MMA8451_TRANSIENT_THS, threshold);
    /*Setup debounce.*/
    decount = sec/0.02; /*12.5hz 20ms step*/
    MMA8451Write(MMA8451_TRANSIENT_COUNT, decount);

    /*Enable transient interrupt.*/
    MMA8451Read( MMA8451_CTRL_REG_4, &regVal );

    regVal |= MMA8451_INT_SRC_TRANS;

    MMA8451Write( MMA8451_CTRL_REG_4, regVal);

    MMA8451Read( MMA8451_CTRL_REG_5, &regVal );

    if(int_num == MMA8451_INT1)
        regVal |= MMA8451_INT_EN_TRANS;
    else if(int_num == MMA8451_INT2)
        regVal &= ~MMA8451_INT_EN_TRANS;

    MMA8451Write( MMA8451_CTRL_REG_5, regVal);

    /*Enable MMA8451*/
    MMA8451_ModeSetting(MMA8451_ACTIVE_MODE);
}

