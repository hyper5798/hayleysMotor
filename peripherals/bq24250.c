#include "bq24250.h"
#include "i2c-board.h"

BQ2425State_e BQ24250CheckCharge()
{
    uint8_t data;
    
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_STAT_REG, &data) == OK)
    {
        data &= BQ24250_CHARGE_MASK;
        data >>= BQ24250_CHARGE_SHIFT;
//        if(data == BQ24250_CHARGE_FAULT)
//            return false;
//        else
//            return true;
        return (BQ2425State_e)data;
    }
    return BQ2425State_I2CError;
}

BQ2425StFaultReason_e BQ24250GetFaultReason()
{
    uint8_t data;
    
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_STAT_REG, &data) == OK)
    {
        data &= BQ24250_FAULT_MASK;
        //data >>= BQ24250_FAULT_SHIFT; //shift 0 bit
        return (BQ2425StFaultReason_e)data;
    }
    return BQ2425StFaultReason_I2CError;
}

void BQ24250Init(void)
{
    uint8_t data = 0;
    
    //if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_CHARGE_REG, &data) == OK)
    {
        data |= BQ24250_RESET_MASK;
        I2cWrite( BQ24250_I2C_ADDRESS << 1, BQ24250_CHARGE_REG, data);
    }
}

void BQ24250HighImpedance(bool state)
{
    uint8_t data;
    
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_CHARGE_REG, &data) == OK)
    {
        data &= (~BQ24250_RESET_MASK);
        data &= (~BQ24250_HZ_MODE_MAST);
        data |= (state << BQ24250_HZ_MODE_SHIFT);
        I2cWrite( BQ24250_I2C_ADDRESS << 1, BQ24250_CHARGE_REG, data);
    }
}

bool BQ24250SetChargeVoltage(uint16_t voltage)
{
    uint8_t data;
    int16_t set_vol = 0;
    int16_t temp_vol = 0;
    int8_t index = 0;
    if(voltage < 350)
        return FAIL;
    set_vol = voltage;
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_VOLTAGE_REG, &data) == OK)
    {
        data &= BQ24250_USB_DET_MASK;
        set_vol -= BQ24250_VBATREG_DEFAULT_VALUE;
        for(index = 6; index > 0; index--)
        {
            temp_vol = set_vol - (1<<index);
            if(temp_vol > 0)
            {
                data |= (1<<(index+1));
                set_vol = temp_vol;
            }
            else if(temp_vol == 0)
            {
                data |= (1<<(index+1));
                break;
            }
        }
        I2cWrite( BQ24250_I2C_ADDRESS << 1, BQ24250_VOLTAGE_REG, data);
    }
    return OK;
}

void BQ24250SYSOFF(bool state)
{
    uint8_t data;
    
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_REG6, &data) == OK)
    {
        data &= (~BQ24250_SYSOFF_MASK);
        data |= (state << BQ24250_SYSOFF_SHIFT);
        I2cWrite( BQ24250_I2C_ADDRESS << 1, BQ24250_REG6, data);
    }
}

uint8_t BQ24250GetSYSOFF(void)
{
    uint8_t data;
    
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_REG6, &data) == OK)
    {
        data &= BQ24250_SYSOFF_MASK;
        data >>= BQ24250_SYSOFF_SHIFT;
        
        return data;
    }
    return 1;
}

void BQ24250SetLimitCurrent(BQ24250CURRENT_LIMIT_e value)
{
    uint8_t data;
    
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_CHARGE_REG, &data) == OK)
    {
        data &= (~BQ24250_RESET_MASK);
        data &= (~BQ24250_ILIMIT_MASK);
        data |= (value << BQ24250_ILIMIT_SHIFT);
        I2cWrite( BQ24250_I2C_ADDRESS << 1, BQ24250_CHARGE_REG, data);
    }
}

void BQ24250EnableWD(bool state)
{
    uint8_t data;
    
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_STAT_REG, &data) == OK)
    {
        data &= (~BQ24250_WD_MASK);
        data |= (state << BQ24250_WD_SHIFT);
        I2cWrite( BQ24250_I2C_ADDRESS << 1, BQ24250_STAT_REG, data);
    }
}

void BQ24250SetIterm(uint8_t value)
{
    uint8_t data;
    
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_CURRENT_REG, &data) == OK)
    {
        data &= (~BQ24250_ITERM_MASK);
        data |= (value << BQ24250_ITERM_SHIFT);
        I2cWrite( BQ24250_I2C_ADDRESS << 1, BQ24250_CURRENT_REG, data);
    }
}
void BQ24250SetIchg(uint8_t value)
{
    uint8_t data;
    
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_CURRENT_REG, &data) == OK)
    {
        data &= (~BQ24250_ICHG_MASK);
        data |= (value << BQ24250_ICHG_SHIFT);
        I2cWrite( BQ24250_I2C_ADDRESS << 1, BQ24250_CURRENT_REG, data);
    }
}

void BQ24250DisableCharge(uint8_t state)
{
    uint8_t data;
    
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_CHARGE_REG, &data) == OK)
    {
        data &= (~BQ24250_RESET_MASK);
        data &= (~BQ24250_CHARGE_ENABLE_MASK);
        data |= (state << BQ24250_CHARGE_ENABLE_SHIFT);
        I2cWrite( BQ24250_I2C_ADDRESS << 1, BQ24250_CHARGE_REG, data);
    }
}

void BQ24250SetTSEN(uint8_t value)
{
    uint8_t data;
    
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_REG6, &data) == OK)
    {
        data &= (~BQ24250_TSEN_MASK);
        data |= (value << BQ24250_TSEN_SHIFT);
        I2cWrite( BQ24250_I2C_ADDRESS << 1, BQ24250_REG6, data);
    }
}

/*!
 * Read TS_EN value
 * Author: Crux
 * \param [IN]      N/A
 * \param [OUT]     N/A
 * \param [return]  TS_EN value
 */
uint8_t BQ24250GetTSEN(void)
{
    uint8_t data;
    
    if(I2cRead( BQ24250_I2C_ADDRESS << 1, BQ24250_REG6, &data) == OK)
    {
        data &= BQ24250_TSEN_MASK;
        data >>= BQ24250_TSEN_SHIFT;
        
        return data;
    }
    return 2;
}
