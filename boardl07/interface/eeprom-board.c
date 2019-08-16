#include "stm32l0xx.h"
#include "board.h"
#include "eeprom-board.h"

/* Private define ------------------------------------------------------------*/
#define DATA_EEPROM_PAGE_SIZE      0x8
#define DATA_32                    0x12345678
#define FAST_DATA_32               0x55667799

#define MCU_EEPROM_SIZE             0x1000
#define MCU_EEPROM_START_ADDR      0x08080000

void InternalEepromReadBytes(uint32_t EepromAddr, uint8_t *Buffer, uint16_t Length)
{
    /* Add by Gavin, Date: 2017/07/04, Log: Align address to word for avoiding hard fault */
    if(EepromAddr % 4 != 0)
    {
        EepromAddr += 4 - (EepromAddr %4);
    }

    uint8_t *wAddr;
    wAddr=(uint8_t *)(EepromAddr);
    while(Length--){
        *Buffer++=*wAddr++;
    }
}

uint8_t InternalEepromWriteBuffer( uint32_t EepromAddrStart, uint8_t *Buffer, uint16_t BufferSize )
{
	 return InternalEepromWriteBufferWord( EepromAddrStart, Buffer, BufferSize );
	/*
		uint16_t AddrIdx = 0;
        uint32_t EndAddr = 0;
		HAL_StatusTypeDef Write_Result;
		EndAddr = EepromAddrStart + BufferSize;
        if(EndAddr >= (MCU_EEPROM_SIZE + MCU_EEPROM_START_ADDR))
            return FAIL;
//		EepromAddrStart = EEPROM_SERIAL_NUM_ADDR;

		__HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR |FLASH_FLAG_SIZERR);

		HAL_FLASH_Unlock();
		HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram();

		for( AddrIdx = 0 ; AddrIdx < BufferSize ; AddrIdx++ ){
//				Write_Result = DATA_EEPROM_FastProgramByte( EepromAddrStart+AddrIdx , Buffer[AddrIdx] );
				Write_Result = DATA_EEPROM_ProgramByte( EepromAddrStart+AddrIdx , Buffer[AddrIdx] );

				if( Write_Result != HAL_OK )
						break;
		}

		HAL_FLASH_Lock();
		HAL_FLASHEx_DATAEEPROM_DisableFixedTimeProgram();

        if(Write_Result == HAL_OK)
            return OK;
        return FAIL;
				*/
}

uint8_t InternalEepromWriteBufferWord( uint32_t EepromAddrStart, uint8_t *Buffer, uint16_t BufferSize )
{
    uint32_t tmpBuffer;
    uint16_t AddrIdx = 0;
    uint32_t EndAddr = 0;
    HAL_StatusTypeDef Write_Result;
    EndAddr = EepromAddrStart + BufferSize;
    if(EndAddr >= (MCU_EEPROM_SIZE + MCU_EEPROM_START_ADDR))
        return FAIL;

    /* Add by Gavin, Date: 2017/06/28, Log: Align address to word for avoiding hard fault */
    if(EepromAddrStart % 4 != 0)
    {
        EepromAddrStart += 4 - (EepromAddrStart %4);
    }

    __HAL_FLASH_CLEAR_FLAG( FLASH_FLAG_EOP | FLASH_FLAG_PGAERR | FLASH_FLAG_WRPERR |FLASH_FLAG_SIZERR);
    HAL_FLASH_Unlock();
    HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram();

    if(BufferSize >= 4)
    {
        for( AddrIdx = 0 ; AddrIdx < BufferSize; AddrIdx =  AddrIdx +4)
        {
            tmpBuffer = Buffer[AddrIdx+3] << 24 | Buffer[AddrIdx+2] << 16
                | Buffer[AddrIdx+1] << 8 |Buffer[AddrIdx];
            Write_Result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, EepromAddrStart+AddrIdx, tmpBuffer);
            if( Write_Result != HAL_OK )
                break;
        }
        AddrIdx = AddrIdx -4;
    }
    if(BufferSize % 4 != 0)
    {
        uint8_t leftBytes = BufferSize - AddrIdx;
        tmpBuffer = 0;
        while(leftBytes >0)
        {
            leftBytes --;
            tmpBuffer = tmpBuffer | (Buffer[AddrIdx + leftBytes] << (8*leftBytes) );
        }
        Write_Result = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, EepromAddrStart+AddrIdx, tmpBuffer);
    }

    HAL_FLASH_Lock();
    HAL_FLASHEx_DATAEEPROM_DisableFixedTimeProgram();

    if(Write_Result == HAL_OK)
        return OK;
    return FAIL;
}
