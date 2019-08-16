#include "crc16.h"
#include <string.h>

/*This Crc is for Modbus version*/
uint32_t crc16(uint8_t *puchMsg, uint16_t DataLen)
{
    uint32_t temp1 = 0xFFFF;
    uint32_t temp2, k, q;
    for(k = 0 ; k < DataLen ; k++)
    {
        temp1 = temp1^puchMsg[k];
        for(q = 0 ; q < 8 ; q++)
        {
            temp2 = temp1;
            temp1 = temp1 >> 1;
            if(temp2 & 0x01)
            {
                temp1 = temp1^0xA001;
            }
        }
    }
    return temp1;
}

/*This Crc is for standard version*/
#define		CRC_POLY_16		0xA001
#define		CRC_START_16		0x0000

uint16_t crcStd16(const uint8_t *puchMsg, uint8_t DataLen)
{
    uint16_t temp1 = CRC_START_16;
    uint16_t temp2, k, q;
    for(k = 0 ; k < DataLen ; k++)
    {
        temp1 = temp1^puchMsg[k];
        for(q = 0 ; q < 8 ; q++)
        {
            temp2 = temp1;
            temp1 = temp1 >> 1;
            if(temp2 & 0x01)
            {
                temp1 = temp1^CRC_POLY_16;
            }
        }
    }
    return temp1;
}

