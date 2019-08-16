/*

    (C)2014 Gemtek

Description: Driver for the PCA9539 IO expander

Maintainer: Hanson Chen
*/
#include "i2c-board.h"
#include "pca9539.h"
#include "gpio-board.h"
#include <stddef.h>
#include <string.h>


static uint8_t I2cDeviceAddr = 0;

static bool PCA9539Initialized = false;

uint8_t PCA9539ChipId;
uint8_t PCA9539VersionId;
static PCA9539IrqHandler *PCA9539Callback = NULL;



void PCA9539InterruptSignal( void )
{
    uint32_t IntStat;
		uint8_t ConfigStatus[GPIO_CONF_STA_SIZE];
		uint8_t GpioIntStatus[GPIO_INT_STA_SIZE];
	
		PCA9539ReadBuffer( REG_CONFIG_0, ConfigStatus, sizeof( ConfigStatus ) );
		PCA9539ReadBuffer( REG_IN_PORT_0, GpioIntStatus, sizeof( GpioIntStatus ) );
    
		// Because the value of Input Reg we read will be affect by Output Reg's value, so
		// we mask the Input Reg with Config Reg to prevent misunderstanding.
		IntStat |= ( GpioIntStatus[REG_IDX_INT_STA_0] & ConfigStatus[REG_IDX_INT_STA_0] );
		IntStat |= ( ( GpioIntStatus[REG_IDX_INT_STA_1] & ConfigStatus[REG_IDX_INT_STA_1] ) << 8 );
	
		
//		//hanson mod, test only
//		if( PCA9539CheckInterruptStatus( IRQ_2_MMA8451, &IntStat ) == true )
//		{
//				PCA9539SetDIRStatus( IRQ_2_MMA8451, PCA9539IOOUTPUT, PCA9539IOLOW );
////				vTaskDelay(1/portTICK_PERIOD_MS);
//		}
		
		
		if(PCA9539Callback != NULL)
				PCA9539Callback( IntStat );
		//SendQueueToStateFromISR(StateIOExpander, &IntStat);
    
//		// Clears and Read The Register Value First
//		GpioIoeReadInterruptStatus( );
//	
//		
//		// Then get the specific index's value
//		if( GpioGetInterruptStatus( &IrqTmp112 ) == true )
//		{
//				if( Tmp112IrqCnt == 0xFFFF )
//						Tmp112IrqCnt = 0;
//				else
//						Tmp112IrqCnt++;

//		}
//		
//		if( GpioGetInterruptStatus( &Irq2Mma8451 ) == true )
//		{
//				MMA8451Read( MMA8451_SYS_INT_STATUS, &IntStat );
//				
//				if( ( IntStat & MMA8451_INT_SRC_LNDPRT ) == MMA8451_INT_SRC_LNDPRT )
//				{
//						MMA8451Read( MMA8451_PL_STATUS, &RegVal );
//				}
//		}

}

void PCA9539SetCallback(PCA9539IrqHandler *irqHandler )
{
    PCA9539Callback = irqHandler;
}

void PCA9539Init( void )
{
    if( PCA9539Initialized == false )
    {   
				//Gpio_t obj;
				uint8_t ConfigStatus[GPIO_CONF_STA_SIZE];
				uint8_t GpioIntStatus[GPIO_INT_STA_SIZE];
				
				PCA9539SetDeviceAddr( PCA9539_I2C_ADDRESS );
        PCA9539Initialized = true;
        
				PCA9539Reset();
			
//				PCA9539SetInitInterrupt();
//				PCA9539Read( CHIP_ID, &STMPE1801ChipId );
//        PCA9539Read( VERSION_ID, &STMPE1801VersionId );
				
				// Read to clear/reset the interrupt pin first
				PCA9539ReadBuffer( REG_CONFIG_0, ConfigStatus, sizeof( ConfigStatus ) );
				PCA9539ReadBuffer( REG_IN_PORT_0, GpioIntStatus, sizeof( GpioIntStatus ) );
			
//#ifdef MODBUS_BOARD			
        //GpioSetInterrupt( &obj, IRQ_FALLING_EDGE, IRQ_VERY_LOW_PRIORITY, &PCA9539InterruptSignal );
//#else
//				GpioInit( &obj, WKUP1, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
//				GpioSetInterrupt( &obj, IRQ_FALLING_EDGE, IRQ_VERY_LOW_PRIORITY, &PCA9539InterruptSignal );
//#endif
    }
		
}

void PCA9539Reset( void )
{		
		PCA9539Write( REG_OUT_PORT_0, REG_RESET_VAL );
		PCA9539Write( REG_OUT_PORT_1, REG_RESET_VAL );
		PCA9539Write( REG_CONFIG_0, REG_RESET_VAL );
		PCA9539Write( REG_CONFIG_1, REG_RESET_VAL );
}

uint8_t PCA9539Write( uint8_t addr, uint8_t data )
{
    return PCA9539WriteBuffer( addr, &data, 1 );
}

uint8_t PCA9539WriteBuffer( uint8_t addr, uint8_t *data, uint8_t size )
{
		return I2cWriteBuffer( I2cDeviceAddr << 1, addr, data, size );
}

uint8_t PCA9539Read( uint8_t addr, uint8_t *data )
{
    return PCA9539ReadBuffer( addr, data, 1 );
}

uint8_t PCA9539ReadBuffer( uint8_t addr, uint8_t *data, uint8_t size )
{
		return I2cReadBuffer( I2cDeviceAddr << 1, addr, data, size );
}

void PCA9539SetDeviceAddr( uint8_t addr )
{
    I2cDeviceAddr = addr;
}

uint8_t PCA9539GetDeviceAddr( void )
{
    return I2cDeviceAddr;
}


void PCA9539SetDIRStatus(GpioIntStaPinIdx pin, PCA9539DIRStatus dir, PCA9539IOStatus status)
{
    uint8_t regAdd = 0;
    uint8_t regVal = 0;
    uint16_t pinIndex = 0;
    
    pinIndex = ( 0x01 << pin % 16 );
	
    if( ( pin % 16 ) > ISG7 )
    {
        regAdd = REG_CONFIG_1;
        pinIndex = ( pinIndex >> 8 ) & 0x00FF;
    }
    else
    {
				regAdd = REG_CONFIG_0;
        pinIndex &= 0x00FF;
    }
		
    PCA9539Read( regAdd, &regVal );

    if( dir == PCA9539IOINPUT )
    {
				regVal |= pinIndex;
    }
    else
    {
        regVal &= ~pinIndex;
    }
		
    PCA9539Write( regAdd, regVal );
		
		//Sets the value to specified pin, and will only affects output direction
		if( dir == PCA9539IOOUTPUT )
		{
				PCA9539SetIOStatus( pin, status );
		}
}


void PCA9539SetIOStatus(GpioIntStaPinIdx pin, PCA9539IOStatus status)
{
    uint8_t regAdd = 0;
    uint16_t pinIndex = 0;
		uint8_t regVal = 0;
		
    /* Sets specified pin's value, the bit we wrote should all be '1' and will 
			 only affects output port
		 */
	
		pinIndex = ( 0x01 << ( pin ) % 16 );
		
		if( ( pin % 16 ) > ISG7 )
		{
				regAdd = REG_OUT_PORT_1;
				pinIndex = ( pinIndex >> 8 ) & 0x00FF;
		}
		else
		{
				regAdd = REG_OUT_PORT_0;
				pinIndex &= 0x00FF;
		}
		
		PCA9539Read( regAdd, &regVal );
		
		if( status == PCA9539IOLOW )
		{
				regVal = regVal & ~pinIndex;
		}
		else
		{
				regVal = regVal | pinIndex;
		}
			
		PCA9539Write( regAdd, regVal );
}


PCA9539IOStatus PCA9539GetIOStatus( GpioIntStaPinIdx pin )
{
		/*	The Input Port registers (registers 0 and 1) reflect the incoming logic levels of the pins, regardless
				of whether the pin is defined as an input or an output by the Configuration register.
		*/
		uint8_t regAdd = 0;
    uint8_t regVal = 0;
    uint16_t pinIndex = 0;
		pinIndex = ( 0x01 << ( pin ) % 16 );
	
		if( ( pin % 16 ) > ISG7 )
    {
				regAdd = REG_IN_PORT_1;
        pinIndex = ( pinIndex >> 8 ) & 0x00FF;
    }
    else
    {
				regAdd = REG_IN_PORT_0;
        pinIndex = ( pinIndex ) & 0x00FF;
    }
		
    PCA9539Read( regAdd, &regVal );

    if( ( regVal & pinIndex ) == 0x00 )
    {
        return PCA9539IOLOW;
    }
    else
    {
        return PCA9539IOHIGH;
    } 
}


void PCA9539SetInterrupt( GpioIntStaPinIdx pin, PCA9539IRQMode irqMode )
{
		// Interrupt is generated by "any rising or falling edge" of the port inputs in the input mode.
		// So settings for each sensors will be sufficient, there's no need to set IO Expander.		
}

bool PCA9539CheckInterruptStatus( GpioIntStaPinIdx pin, uint32_t *GpioIntStatus )
{
    uint8_t regIdx;
    uint16_t pinIndex;
    uint8_t IntStatus[GPIO_INT_STA_SIZE];
    
    memcpy(IntStatus, GpioIntStatus, GPIO_INT_STA_SIZE);
		
    pinIndex = ( 0x01 << ( pin ) % 16 );										//0x00,	0x01,	0x02, ... index

    if( ( pin % 16 ) > 0x07 )
    {
        pinIndex = ( pinIndex >> 8 ) & 0x00FF;
				regIdx = REG_IDX_INT_STA_1;
    }
    else
    {
        pinIndex &= 0x00FF;
				regIdx = REG_IDX_INT_STA_0;
    }
    
		if( IntStatus[regIdx] & pinIndex )
		{	
				// We assumed that each IO-Expander's pin should be set to falling edge.
				return false;
		}
		else 
		{
				// Therefore, if the pin status doesn't stay at HIGH, it means that the 
				// alert occurred to the corresponding pin
				return true;
		}
}



