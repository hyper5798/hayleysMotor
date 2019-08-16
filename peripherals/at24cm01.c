/*
(C)2015 Gemtek

Description: Driver for 128K external EEPROM

Maintainer: HM Tsai
*/
#include "board.h"
#include "i2c-board.h"
#include "at24cm01.h"
#if defined(STM32L073xx) || defined(STM32L071xx)
#include <stm32l0xx.h>
#else
#include <stm32l1xx.h>
#endif
#define DEVICE_I2C_ADDRESS                          0xA8

static uint8_t I2cDeviceAddr = DEVICE_I2C_ADDRESS;

#define EE_PAGE_SIZE                                256
#define EE_PAGE_BIT									0x02

uint8_t AT24CM01EepromWriteBuffer( uint32_t addr, uint8_t *buffer, uint32_t size )
{
    uint8_t nbPage = 0;
    uint8_t nbBytes = 0;
    uint8_t nbBytesRemaining = 0;
    uint16_t lAddr = 0;
	uint32_t EndAddr = 0;
	
	EndAddr = addr + size - 1;
	
	/*start add and end add are different page*/
	if(((addr >> 16) == 0) && ((EndAddr >> 16) == 1))
		return FAIL;
		
	if((addr >> 16) == 0)
		I2cDeviceAddr = DEVICE_I2C_ADDRESS & ~EE_PAGE_BIT;
	else
		I2cDeviceAddr = DEVICE_I2C_ADDRESS | EE_PAGE_BIT;
    
    lAddr = addr % EE_PAGE_SIZE;
    nbBytesRemaining = EE_PAGE_SIZE - lAddr;
    nbPage =  size / EE_PAGE_SIZE;
    nbBytes = size % EE_PAGE_SIZE;
    
    I2cSetAddrSize(I2C_ADDR_SIZE_16 );
    /*!< If lAddr is EE_PAGE_SIZE aligned  */
    if( lAddr == 0 )
    {
        /*!< If size < EE_PAGE_SIZE */
        if( nbPage == 0 ) 
        {
            if( I2cWriteBuffer( I2cDeviceAddr, addr, buffer, size ) == FAIL )
            {
                I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                return FAIL;
            }
            if( I2cWaitStandbyState( I2cDeviceAddr ) == FAIL )
            {
                I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                return FAIL;
            }
        }
        /*!< If size > EE_PAGE_SIZE */
        else  
        {
            while( nbPage-- )
            {
                if( I2cWriteBuffer( I2cDeviceAddr, addr, buffer, EE_PAGE_SIZE ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
                if( I2cWaitStandbyState( I2cDeviceAddr ) == FAIL )
                {
                    I2cSetAddrSize(I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
                addr +=  EE_PAGE_SIZE;
                buffer += EE_PAGE_SIZE;
            }
    
            if( nbBytes != 0 )
            {
                if( I2cWriteBuffer( I2cDeviceAddr, addr, buffer, nbBytes ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
                if( I2cWaitStandbyState( I2cDeviceAddr ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
            }
        }
    }
    /*!< If addr is not EE_PAGE_SIZE aligned  */
    else 
    {
        /*!< If size < EE_PAGE_SIZE */
        if( nbPage== 0 ) 
        {
            /*!< If the number of data to be written is more than the remaining space 
            in the current page: */
            if ( size > nbBytesRemaining )
            {
                if( I2cWriteBuffer(I2cDeviceAddr, addr, buffer, nbBytesRemaining ) == FAIL )
                {
                    I2cSetAddrSize(I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
                if( I2cWaitStandbyState( I2cDeviceAddr ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
                
                if( I2cWriteBuffer( I2cDeviceAddr, ( addr + nbBytesRemaining ),
                                                   ( uint8_t* )( buffer + nbBytesRemaining ),
                                                   ( size - nbBytesRemaining ) ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
                if( I2cWaitStandbyState(I2cDeviceAddr ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
            }      
            else      
            {
                if( I2cWriteBuffer( I2cDeviceAddr, addr, buffer, nbBytes ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
                if( I2cWaitStandbyState( I2cDeviceAddr ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
            }     
        }
        /*!< If size > EE_PAGE_SIZE */
        else
        {
            size -= nbBytesRemaining;
            nbPage =  size / EE_PAGE_SIZE;
            nbBytes = size % EE_PAGE_SIZE;
            
            if( nbBytesRemaining != 0 )
            {  
                if( I2cWriteBuffer( I2cDeviceAddr, addr, buffer, nbBytesRemaining ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
                if( I2cWaitStandbyState( I2cDeviceAddr ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
                addr += nbBytesRemaining;
                buffer += nbBytesRemaining;
            } 
            
            while( nbPage-- )
            {
                if( I2cWriteBuffer( I2cDeviceAddr, addr, buffer, EE_PAGE_SIZE ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
                if( I2cWaitStandbyState( I2cDeviceAddr ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
                addr +=  EE_PAGE_SIZE;
                buffer += EE_PAGE_SIZE;  
            }
            if( nbBytes != 0 )
            {
                if( I2cWriteBuffer( I2cDeviceAddr, addr, buffer, nbBytes ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
                if( I2cWaitStandbyState( I2cDeviceAddr ) == FAIL )
                {
                    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
                    return FAIL;
                }
            }
        }
    } 
    I2cSetAddrSize( I2C_ADDR_SIZE_8 );
    return OK;
}

uint8_t AT24CM01EepromReadBuffer( uint32_t addr, uint8_t *buffer, uint32_t size )
{
    uint8_t status = FAIL;
	
	uint32_t EndAddr = 0;
	uint16_t readdr = 0;
	uint16_t resize = 0;
	
	EndAddr = addr + size -1;

    I2cSetAddrSize( I2C_ADDR_SIZE_16 );
	
	if(((addr >> 16) == 1) && (EndAddr < 0x20000))
	{
		I2cDeviceAddr = DEVICE_I2C_ADDRESS | EE_PAGE_BIT;
		readdr = (uint16_t)addr;
		resize = (uint16_t)size;
		status = I2cReadBuffer( I2cDeviceAddr, readdr, buffer, resize );
	}
	else if((addr >> 16) == 0)
	{
		if((EndAddr >> 16) == 0)
		{
			I2cDeviceAddr = DEVICE_I2C_ADDRESS & ~EE_PAGE_BIT;
			readdr = (uint16_t)addr;
			resize = (uint16_t)size;
			status = I2cReadBuffer( I2cDeviceAddr, readdr, buffer, resize );
		}
		else
		{
			I2cDeviceAddr = DEVICE_I2C_ADDRESS & ~EE_PAGE_BIT;
			readdr = (uint16_t)addr;
			resize =  0x10000 - readdr;
			status = I2cReadBuffer( I2cDeviceAddr, readdr, buffer, resize );
			if(status == OK)
			{
				uint16_t readedLen = resize;
				I2cDeviceAddr = DEVICE_I2C_ADDRESS | EE_PAGE_BIT;
				readdr = (uint16_t)0x10000;
				resize =  EndAddr - 0x10000;
				status = I2cReadBuffer( I2cDeviceAddr, readdr, buffer+readedLen, resize );
			}
		}
	}
//    status = I2cReadBuffer( I2cDeviceAddr, addr, buffer, size );
    I2cSetAddrSize( I2C_ADDR_SIZE_8 );

    return status;
}

void AT24CM01EepromSetDeviceAddr( uint8_t addr )
{
    I2cDeviceAddr = addr;
}

uint8_t AT24CM01EepromGetDeviceAddr( void )
{
    return I2cDeviceAddr;
}
