/*
    (C)2014 Gemtek

Description: Driver for the TMP112 Temperature Sensor

Maintainer: Hanson Chen
*/


#include "board.h"
#include "tmp112.h"

#define TIMEOUT_MAX                                 0x8000

static uint8_t I2cDeviceAddr = 0;
static bool TMP112Initialized = false;

static bool ExtendedModeEnabled;

uint8_t TMP112Enable(uint8_t enable)
{
    uint8_t Tmp112ConfigReg[2];
    TMP112ReadBuffer( CONFIG_REG, Tmp112ConfigReg, sizeof(Tmp112ConfigReg) );
    if(enable == 0)
    {
        Tmp112ConfigReg[0] = Tmp112ConfigReg[0] | (1 << TMP112_SD_SHIFT);
    }
    else
    {
        Tmp112ConfigReg[0] = Tmp112ConfigReg[0] & ~(1 << TMP112_SD_SHIFT);
    }
    
    TMP112WriteBuffer( CONFIG_REG, Tmp112ConfigReg, sizeof(Tmp112ConfigReg) );
    
    return SUCCESS;
}

uint8_t TMP112Init( void )
{
    uint8_t HighLimitTemp[2];
    uint8_t LowLimitTemp[2];
    uint8_t Tmp112ConfigReg[2];
    TMP112SetDeviceAddr( TMP112_I2C_ADDRESS );

    if( TMP112Initialized == false )
    {          
        TMP112Reset( );
    
        TMP112Initialized = true;
    }

//		    TMP112LimitTemperature(TMP112LIMITLOW, -55);
//    TMP112LimitTemperature(TMP112LIMITHIGH, 38);
//		// 33 degrees in Celsius for T_High
//		HighLimitTemp[TMP112_REG_MSB] = 0x21;
//		HighLimitTemp[TMP112_REG_LSB] = 0x00;
//		
//		// 10 degrees in Celsius for T_Low
//		LowLimitTemp[TMP112_REG_MSB] = 0x0A;
//		LowLimitTemp[TMP112_REG_LSB] = 0x00;
//		
//		TMP112WriteBuffer( HIGH_LIMIT_REG, HighLimitTemp, sizeof(HighLimitTemp) );
//		TMP112WriteBuffer( LOW_LIMIT_REG, LowLimitTemp, sizeof(LowLimitTemp) );
		
		TMP112ReadBuffer( HIGH_LIMIT_REG, HighLimitTemp, sizeof(HighLimitTemp) );
		TMP112ReadBuffer( LOW_LIMIT_REG, LowLimitTemp, sizeof(LowLimitTemp) );
		TMP112ReadBuffer( CONFIG_REG, Tmp112ConfigReg, sizeof(Tmp112ConfigReg) );

    ExtendedModeEnabled = Tmp112ConfigReg[0] & TMP112_TM_MASK; 
		
    return SUCCESS;
}


uint8_t TMP112Reset( )
{
    /*  Issues a General Call Command to reset to power-up values. However, this will affect
		 *  other Slave devices so do not use this if it's not the first peripheral to init
		 */
//    if( I2cWriteBuffer( &I2c, 0x00, 0x06, 0x00, 1 ) == SUCCESS )
//    {
//        return SUCCESS;
//    }
//    return FAIL;
		
		return SUCCESS;
}

uint8_t TMP112SetInterrupt(TMP112Polarity_e ActiveState, TMP112ThermostatMode_e TmMode )
{
    uint8_t Tmp112ConfigReg[2];
	TMP112ReadBuffer( CONFIG_REG, Tmp112ConfigReg, sizeof(Tmp112ConfigReg) );
    
    if(ActiveState == TMP112POLACTIVEHIGH)
    {
        Tmp112ConfigReg[0] = Tmp112ConfigReg[0] | (1 << TMP112_POL_SHIFT);
    }
    else
    {
        Tmp112ConfigReg[0] = Tmp112ConfigReg[0] & ~(1 << TMP112_POL_SHIFT);
    }
    
    if(TmMode == TMP112COMPARATOR_MODE)
    {
        Tmp112ConfigReg[0] = Tmp112ConfigReg[0] & ~(1 << TMP112_TM_SHIFT);
    }
    else
    {
        Tmp112ConfigReg[0] = Tmp112ConfigReg[0] | (1 << TMP112_TM_SHIFT);
    }
    
    TMP112WriteBuffer( CONFIG_REG, Tmp112ConfigReg, sizeof(Tmp112ConfigReg) );
    
    return SUCCESS;
}


uint8_t TMP112Write( uint8_t addr, uint8_t data )
{
    return TMP112WriteBuffer( addr, &data, 1 );
}

uint8_t TMP112WriteBuffer( uint8_t addr, uint8_t *data, uint8_t size )
{
    return I2cWriteBuffer( I2cDeviceAddr << 1, addr, data, size );
}

uint8_t TMP112Read( uint8_t addr, uint8_t *data )
{
    return TMP112ReadBuffer( addr, data, 1 );
}

uint8_t TMP112ReadBuffer( uint8_t addr, uint8_t *data, uint8_t size )
{
    return I2cReadBuffer( I2cDeviceAddr << 1, addr, data, size );
}

void TMP112SetDeviceAddr( uint8_t addr )
{
    I2cDeviceAddr = addr;
}

uint8_t TMP112GetDeviceAddr( void )
{
    return I2cDeviceAddr;
}

uint8_t TMP112LimitTemperature(TMP112Limit_e limit, float temp)
{
    uint8_t TempRegVal[2];
    uint16_t TempVal;
    if((temp > 128) || (temp < -128))
        return FAIL;
    if( ExtendedModeEnabled == false )
    {
        if(temp < 0)
        {
            TempVal = (temp/(-TMP112_RES_UNIT));
            TempVal -= 1;
            TempVal = ~TempVal;
            TempRegVal[TMP112_REG_MSB] = TempVal >> (8 - EM_DISABLED_SHIFT);
            TempRegVal[TMP112_REG_LSB] = TempVal << EM_DISABLED_SHIFT;
        }
        else
        {
            TempVal = (temp/TMP112_RES_UNIT);
            TempRegVal[TMP112_REG_MSB] = TempVal >> (8 - EM_DISABLED_SHIFT);
            TempRegVal[TMP112_REG_LSB] = TempVal << EM_DISABLED_SHIFT;
        }
    }
    if(limit == TMP112LIMITLOW)
    {
        TMP112WriteBuffer( LOW_LIMIT_REG, TempRegVal, sizeof(TempRegVal) );
    }
    else if(limit == TMP112LIMITHIGH)
    {
        TMP112WriteBuffer( HIGH_LIMIT_REG, TempRegVal, sizeof(TempRegVal) );
    }
    return SUCCESS;
}

float TMP112ReadTemperature( void )
{
    uint8_t TempRegVal[2] = {0};
    uint8_t TempValMsb;		
    uint8_t TempValLsb;
    float TempVal;
		
    if(TMP112OneShot() == FAIL)
        return 0;
    if(TMP112ReadBuffer( TEMP_REG, TempRegVal, sizeof( TempRegVal ) ) == FAIL)
        return 0;

    if( ExtendedModeEnabled == false )
    {	
            if( ( TempRegVal[TMP112_REG_MSB] >> MSB_CHECK_SHIFT ) )
            {
                    TempRegVal[TMP112_REG_MSB] = ~TempRegVal[TMP112_REG_MSB];
                    TempRegVal[TMP112_REG_LSB] = ~TempRegVal[TMP112_REG_LSB];
                    
                    // for negative digital data 
                    TempValMsb = TempRegVal[TMP112_REG_MSB]  >> EM_DISABLED_SHIFT;
                    TempValLsb = ( TempRegVal[TMP112_REG_MSB] << EM_DISABLED_SHIFT ) + ( TempRegVal[TMP112_REG_LSB] >> EM_DISABLED_SHIFT );
                    TempVal = ( TempValMsb << MSBYTE_SHIFT ) + TempValLsb + 1;
                    
                    return ( TempVal * (-TMP112_RES_UNIT) );		
            }
            else
            {
                    // for positive digital data
                    TempValMsb = TempRegVal[TMP112_REG_MSB] >> EM_DISABLED_SHIFT;
                    TempValLsb = ( TempRegVal[TMP112_REG_MSB] << EM_DISABLED_SHIFT ) + ( TempRegVal[TMP112_REG_LSB] >> EM_DISABLED_SHIFT );
                    TempVal = ( TempValMsb << MSBYTE_SHIFT ) + TempValLsb;
                
                    return ( TempVal * TMP112_RES_UNIT );
            }
    }
    else
    {
            //To-Do: Add 13-bit digital conversion
            return 0;
    }		
}

uint8_t TMP112OneShot( void )
{
    uint8_t Tmp112ConfigReg[2] = {0};
    uint32_t OneShotTimeOut = TIMEOUT_MAX;
    TMP112ReadBuffer( CONFIG_REG, Tmp112ConfigReg, sizeof(Tmp112ConfigReg) );
    Tmp112ConfigReg[0] = Tmp112ConfigReg[0] | (1 << TMP112_OS_SHIFT);
    TMP112WriteBuffer( CONFIG_REG, Tmp112ConfigReg, sizeof(Tmp112ConfigReg) );
    
    while(1)
    {
        if((TMP112ReadBuffer( CONFIG_REG, Tmp112ConfigReg, sizeof(Tmp112ConfigReg) )) == FAIL)
        {
            if( ( OneShotTimeOut-- ) == 0 )
            {
                return( FAIL );
            }
            else
            {
                continue;
            }
        }
        if((Tmp112ConfigReg[0] & TMP112_OS_MASK) == 0x00)
            break;
        if( ( OneShotTimeOut-- ) == 0 )
        {
            return( FAIL );
        }
    }
    
    return SUCCESS;
}


float TMP112ReadTemperatureForTest( uint8_t *TestTempVal )
{
//    uint8_t TempRegVal[2];
		uint8_t TempValMsb;		
		uint8_t TempValLsb;
		float TempVal;
	
		if( ExtendedModeEnabled == false )
		{	
				if( ( TestTempVal[TMP112_REG_MSB] >> MSB_CHECK_SHIFT ) )
				{
						TestTempVal[TMP112_REG_MSB] = ~TestTempVal[TMP112_REG_MSB];
						TestTempVal[TMP112_REG_LSB] = ~TestTempVal[TMP112_REG_LSB];
						
						// for negative digital data 
						TempValMsb = TestTempVal[TMP112_REG_MSB]  >> EM_DISABLED_SHIFT;
						TempValLsb = ( TestTempVal[TMP112_REG_MSB] << EM_DISABLED_SHIFT ) + ( TestTempVal[TMP112_REG_LSB] >> EM_DISABLED_SHIFT );
						TempVal = ( TempValMsb << MSBYTE_SHIFT ) + TempValLsb + 1;
						
						return ( TempVal * (-TMP112_RES_UNIT) );		
				}
				else
				{
						// for positive digital data
						TempValMsb = TestTempVal[TMP112_REG_MSB] >> EM_DISABLED_SHIFT;
						TempValLsb = ( TestTempVal[TMP112_REG_MSB] << EM_DISABLED_SHIFT ) + ( TestTempVal[TMP112_REG_LSB] >> EM_DISABLED_SHIFT );
						TempVal = ( TempValMsb << MSBYTE_SHIFT ) + TempValLsb;
					
						return ( TempVal * TMP112_RES_UNIT );
				}
		}
		else
		{
				//To-Do: Add 13-bit digital conversion
				return 0;
		}		
}
