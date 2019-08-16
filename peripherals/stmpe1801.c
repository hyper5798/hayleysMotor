/*
    (C)2014 Gemtek

Description: Driver for the STMPE1801 IO expander

Maintainer: Hanson Chen
*/

#include "i2c-board.h"
#include "stmpe1801.h"
#include <stddef.h>
#include <string.h>
//#include "state_task.h"

//These are for ReadSTMPE1801Status()
//uint8_t MpLow;
//uint8_t MpMid;
//uint8_t MpHigh;

//uint8_t DirLow;
//uint8_t DirMid;
//uint8_t DirHigh;

//uint8_t ReLow;
//uint8_t ReMid;
//uint8_t ReHigh;

//uint8_t FeLow;
//uint8_t FeMid;
//uint8_t FeHigh;

//uint8_t PuLow;
//uint8_t PuMid;
//uint8_t PuHigh;

//uint8_t IntCtrlLow;
//uint8_t IntEnMaskLow;
//uint8_t IntEnGpioMaskLow;
//uint8_t IntEnGpioMaskMid;
//uint8_t IntEnGpioMaskHigh;


static uint8_t I2cDeviceAddr = 0;

static bool STMPE1801Initialized = false;

uint8_t STMPE1801ChipId;
uint8_t STMPE1801VersionId;
static STMPE1801IrqHandler *STMPE1801Callback = NULL;
//uint8_t GpioIntStatus[GPIO_INT_STA_SIZE];

void STMPE1801InterruptSignal( void )
{
    uint32_t IntStat;
//    uint8_t InterruptStatus;
//    uint8_t GpioIntStatus[GPIO_INT_STA_SIZE];
//    
//    
//    STMPE1801Read( INT_STA_LOW, &InterruptStatus );
//    if( InterruptStatus & REG_INT_STA_IS3_MASK )
//    {
//        IntStat = InterruptStatus << 24;
//        STMPE1801ReadBuffer( INT_STA_GPIO_LOW, GpioIntStatus, sizeof( GpioIntStatus ) );
//        IntStat = IntStat | GpioIntStatus[REG_IDX_INT_STA_GPIO_LOW];
//        IntStat = IntStat | (GpioIntStatus[REG_IDX_INT_STA_GPIO_MID] << 8);
//        IntStat = IntStat | (GpioIntStatus[REG_IDX_INT_STA_GPIO_HIGH] << 16);
        
        if(STMPE1801Callback != NULL)
            STMPE1801Callback(IntStat);
        //SendQueueToStateFromISR(StateIOExpander, &IntStat);
//    }
    
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

void STMPE1801SetCallback(STMPE1801IrqHandler *irqHandler )
{
    STMPE1801Callback = irqHandler;
}

void STMPE1801Init( void )
{
    if( STMPE1801Initialized == false )
    {
        STMPE1801SetDeviceAddr( STMPE1801_I2C_ADDRESS );
        STMPE1801Initialized = true;

        STMPE1801Reset();
    
        STMPE1801SetInitInterrupt();
    
        STMPE1801Read( CHIP_ID, &STMPE1801ChipId );
        STMPE1801Read( VERSION_ID, &STMPE1801VersionId );
    }
}


void STMPE1801Reset( void )
{
//    uint8_t readData = 1;
//		STMPE1801Write( SYS_CTRL, STMPE1801_RST_MASK );
//    
//    do
//    {
//        
//        if(STMPE1801Read( SYS_CTRL, &readData ) != OK)
//            continue;
//    }while(STMPE1801_RST_MASK == (readData&STMPE1801_RST_MASK));
    
    
	
		//Set Default GPIO direction to Input
		STMPE1801Write( GPIO_SET_DIR_LOW,  0x0 );
		STMPE1801Write( GPIO_SET_DIR_MID,  0x0 );
		STMPE1801Write( GPIO_SET_DIR_HIGH, 0x0 );
	
		//Set Default GPIO edge dectection to falling edge
		STMPE1801Write( GPIO_FE_LOW, 	0x00 );
		STMPE1801Write( GPIO_FE_MID, 	0x00 );
		STMPE1801Write( GPIO_FE_HIGH, 0x00 );
	
		//Set Default GPIO edge dectection to rising edge
		STMPE1801Write( GPIO_RE_LOW, 	0x00 );
		STMPE1801Write( GPIO_RE_MID, 	0x00 );
		STMPE1801Write( GPIO_RE_HIGH, 0x00 );
	
		//Set Default GPIO pin to pull up
        /*for track board*/
		STMPE1801Write( GPIO_PULL_UP_LOW, 	0x61 );/*g-sensor int1,2, charge*/
		STMPE1801Write( GPIO_PULL_UP_MID, 	0xc0 );/*BT disconnet connect*/
		STMPE1801Write( GPIO_PULL_UP_HIGH, 0x00 );
}

void STMPE1801SetInitInterrupt( void )
{
		uint8_t InterruptStatus = 0x00;
		
		// IC2 = 0 (Active Low/Falling Edge), IC1 = 0 (Level Interrupt), IC0 = 1 (Global interrupt mask)
		// IC2 is related to Wkup1's interrupt settings
		STMPE1801Write( INT_CTRL_LOW, 0x01 );

    // Only enables GPIO interrupt IE3:GPIO controller interrupt mask
		STMPE1801Write( INT_EN_MASK_LOW, 0x08 );

		// Clears all the corresponding GPIO interrupt status
        STMPE1801Write( INT_EN_GPIO_MASK_LOW, InterruptStatus );
		STMPE1801Write( INT_EN_GPIO_MASK_MID, InterruptStatus );
		STMPE1801Write( INT_EN_GPIO_MASK_HIGH, InterruptStatus );
		STMPE1801Read( INT_STA_LOW, &InterruptStatus );
		STMPE1801Read( INT_STA_GPIO_LOW, &InterruptStatus );
		STMPE1801Read( INT_STA_GPIO_MID, &InterruptStatus );
		STMPE1801Read( INT_STA_GPIO_HIGH, &InterruptStatus );
}



uint8_t STMPE1801Write( uint8_t addr, uint8_t data )
{
    return STMPE1801WriteBuffer( addr, &data, 1 );
}

uint8_t STMPE1801WriteBuffer( uint8_t addr, uint8_t *data, uint8_t size )
{
    return I2cWriteBuffer( I2cDeviceAddr << 1, addr, data, size );
}

uint8_t STMPE1801Read( uint8_t addr, uint8_t *data )
{
    return STMPE1801ReadBuffer( addr, data, 1 );
}

uint8_t STMPE1801ReadBuffer( uint8_t addr, uint8_t *data, uint8_t size )
{
    return I2cReadBuffer( I2cDeviceAddr << 1, addr, data, size );
}


void STMPE1801SetDeviceAddr( uint8_t addr )
{
    I2cDeviceAddr = addr;
}

uint8_t STMPE1801GetDeviceAddr( void )
{
    return I2cDeviceAddr;
}

//hanson mod, For Debug Only
void ReadSTMPE1801Status( void )
{
    //These are for ReadSTMPE1801Status()
uint8_t MpLow;
uint8_t MpMid;
uint8_t MpHigh;

uint8_t DirLow;
uint8_t DirMid;
uint8_t DirHigh;

uint8_t ReLow;
uint8_t ReMid;
uint8_t ReHigh;

uint8_t FeLow;
uint8_t FeMid;
uint8_t FeHigh;

uint8_t PuLow;
uint8_t PuMid;
uint8_t PuHigh;

uint8_t IntCtrlLow;
uint8_t IntEnMaskLow;
uint8_t IntEnGpioMaskLow;
uint8_t IntEnGpioMaskMid;
uint8_t IntEnGpioMaskHigh;
			
//		STMPE1801Init();
	
		STMPE1801Read( GPIO_MP_LOW, &MpLow );
		STMPE1801Read( GPIO_MP_MID, &MpMid );
		STMPE1801Read( GPIO_MP_HIGH, &MpHigh );

		STMPE1801Read( GPIO_SET_DIR_LOW, &DirLow );
		STMPE1801Read( GPIO_SET_DIR_MID, &DirMid );
		STMPE1801Read( GPIO_SET_DIR_HIGH, &DirHigh );

		STMPE1801Read( GPIO_RE_LOW, &ReLow );
		STMPE1801Read( GPIO_RE_MID, &ReMid );
		STMPE1801Read( GPIO_RE_HIGH, &ReHigh );

		STMPE1801Read( GPIO_FE_LOW, &FeLow );
		STMPE1801Read( GPIO_FE_MID, &FeMid );
		STMPE1801Read( GPIO_FE_HIGH, &FeHigh );

		STMPE1801Read( GPIO_PULL_UP_LOW, &PuLow );
		STMPE1801Read( GPIO_PULL_UP_MID, &PuMid );
		STMPE1801Read( GPIO_PULL_UP_HIGH, &PuHigh );

		STMPE1801Read( INT_CTRL_LOW, &IntCtrlLow );
		STMPE1801Read( INT_EN_MASK_LOW, &IntEnMaskLow );
		STMPE1801Read( INT_EN_GPIO_MASK_LOW, &IntEnGpioMaskLow );
		STMPE1801Read( INT_EN_GPIO_MASK_MID, &IntEnGpioMaskMid );
		STMPE1801Read( INT_EN_GPIO_MASK_HIGH, &IntEnGpioMaskHigh );

}

void STMPE1801SetDIRStatus(GpioIntStaPinIdx pin, STMPE1801DIRStatus dir, STMPE1801IOStatus status)
{
    uint8_t regAdd = 0;
    uint8_t regVal = 0;
    uint16_t pinIndex = 0;
    
    pinIndex = ( 0x01 << pin % 16 );
    
    if( ( pin % 16 ) > ISG7 )
    {
        regAdd = GPIO_SET_DIR_MID;
        pinIndex = ( pinIndex >> 8 ) & 0x00FF;
    }
    else
    {
        if( pin < ISG16 )
        {
                regAdd = GPIO_SET_DIR_LOW;
        }
        else
        {
                regAdd = GPIO_SET_DIR_HIGH;
        }
        
        pinIndex &= 0x00FF;
    }
    
    //Sets the value to specified pin
    STMPE1801SetIOStatus( pin, status );
    
    //Sets the direction to specified pin
    STMPE1801Read( regAdd, &regVal );
    if( dir == STMPE1801IOINPUT )
    {
        regVal &= ~pinIndex;
    }
    else
    {
        regVal |= pinIndex;
    }
    STMPE1801Write( regAdd, regVal );
}


void STMPE1801SetIOStatus(GpioIntStaPinIdx pin, STMPE1801IOStatus status)
{
    uint8_t regAdd = 0;
    uint16_t pinIndex = 0;
		
    /* Sets specified pin's value, the bit we wrote should all be '1' and the value we set
		 * will depends on what register we use, i.e. GPIO_CLR_xxx for 0, GPIO_SET_xxx for 1
		 */
	
		pinIndex = ( 0x01 << ( pin ) % 16 );
		
		if( ( pin % 16 ) > ISG7 )
		{
				if( status == STMPE1801IOLOW ) {
						regAdd = GPIO_CLR_MID;
				}
				else {
						regAdd = GPIO_SET_MID;
				}
				
				pinIndex = ( pinIndex >> 8 ) & 0x00FF;
		}
		else
		{
				if( pin < ISG16 )
				{	
						if( status == STMPE1801IOLOW ) {
								regAdd = GPIO_CLR_LOW;
						}
						else {
								regAdd = GPIO_SET_LOW;
						}
				}
				else 
				{
						if( status == 0 ) {
								regAdd = GPIO_CLR_HIGH;
						}
						else {
								regAdd = GPIO_SET_HIGH;
						}
				}
				
				pinIndex &= 0x00FF;
		}
			
		STMPE1801Write( regAdd, pinIndex );
}

STMPE1801IOStatus STMPE1801GetIOStatus(GpioIntStaPinIdx pin)
{
		uint8_t regAdd = 0;
    uint8_t regVal = 0;
    uint16_t pinIndex = 0;
		pinIndex = ( 0x01 << ( pin ) % 16 );
		if( ( pin % 16 ) > ISG7 )
		{
				regAdd = GPIO_MP_MID;
            pinIndex = ( pinIndex >> 8 ) & 0x00FF;
		}
		else
		{
				if( pin < ISG16 )
				{	
						regAdd = GPIO_MP_LOW;
				}
				else 
				{
						regAdd = GPIO_MP_HIGH;
				}
                pinIndex &= 0x00FF;
		}
		
    STMPE1801Read( regAdd, &regVal );

    if( ( regVal & pinIndex ) == 0x00 )
    {
        return STMPE1801IOLOW;
    }
    else
    {
        return STMPE1801IOHIGH;
    } 
}

void STMPE1801SetInterrupt( GpioIntStaPinIdx pin, STMPE1801IRQMode irqMode )
{
		uint8_t regAdd = 0;
    uint8_t regAddirq = 0;
    uint8_t regVal = 0;
    uint16_t pinIndex = 0;
	
		// To-do: Take STMPE1801's Rising/Falling edge settings into account.
		// We now set the default irqModes to Falling edge.
	
    pinIndex = ( 0x01 << ( pin ) % 16 );										//0x00,	0x01,	0x02, ... index
	
	if( ( pin % 16 ) > 0x07 )
    {
		regAdd = INT_EN_GPIO_MASK_MID;
        if(irqMode == STMPE1801IORISING)
            regAddirq = GPIO_RE_MID;
        else
            regAddirq = GPIO_FE_MID;
        pinIndex = ( pinIndex >> 8 ) & 0x00FF;
    }
    else
    {
        if( pin < ISG16 )
        {
            regAdd = INT_EN_GPIO_MASK_LOW;
            if(irqMode == STMPE1801IORISING)
                regAddirq = GPIO_RE_LOW;
            else
            regAddirq = GPIO_FE_LOW;
        }
        else
        {
            regAdd = INT_EN_GPIO_MASK_HIGH;
            if(irqMode == STMPE1801IORISING)
                regAddirq = GPIO_RE_HIGH;
            else
                regAddirq = GPIO_FE_HIGH;
        }
        pinIndex &= 0x00FF;
    }
		
    STMPE1801Read( regAdd, &regVal );

    regVal |= pinIndex;
		
    STMPE1801Write( regAdd, regVal );
    
    STMPE1801Read( regAddirq, &regVal );

    regVal |= pinIndex;
		
    STMPE1801Write( regAddirq, regVal );
}

bool STMPE1801CheckInterruptStatus( GpioIntStaPinIdx pin, uint32_t *GpioIntStatus )
{
    uint8_t regIdx;
    uint16_t pinIndex;
    uint8_t IntStatus[GPIO_INT_STA_SIZE];
    
    memcpy(IntStatus, GpioIntStatus, GPIO_INT_STA_SIZE);
    pinIndex = ( 0x01 << ( pin ) % 16 );										//0x00,	0x01,	0x02, ... index

    if( ( pin % 16 ) > 0x07 )
    {
        pinIndex = ( pinIndex >> 8 ) & 0x00FF;
        regIdx = REG_IDX_INT_STA_GPIO_MID;
    }
    else
    {
        if( pin < ISG16 )
        {
            regIdx = REG_IDX_INT_STA_GPIO_LOW;
        }
        else
        {
            regIdx = REG_IDX_INT_STA_GPIO_HIGH;
        }
        pinIndex &= 0x00FF;
    }
    
    if( IntStatus[regIdx] & pinIndex )
    {
        return true;
    }
    else 
    {
        return false;
    }	
}
