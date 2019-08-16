/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Helper functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "utilities.h"

/*!
 * Redefinition of rand() and srand() standard C functions.
 * These functions are redefined in order to get the same behavior across
 * different compiler toolchains implementations.
 */
// Standard random functions redefinition start
#define RAND_LOCAL_MAX 2147483647

static unsigned long next = 1;

int rand1( void )
{
    return ( ( next = next * 1103515245 + 12345 ) % RAND_LOCAL_MAX );
}

void srand1( unsigned int seed )
{
    next = seed;
}
// Standard random functions redefinition end

int32_t randr( int32_t min, int32_t max )
{
    return ( int32_t )rand1( ) % ( max - min + 1 ) + min;
}

/* random integer from 0 to n-1 */
int irand( int n )
{
	int r, rand_max = RAND_MAX - (RAND_MAX % n);
	/* reroll until r falls in a range that can be evenly
	 * distributed in n bins.  Unless n is comparable to
	 * to RAND_MAX, it's not *that* important really. */
	while ( ( r = rand1( ) ) >= rand_max );
	return r / (rand_max / n);
}


/*Durstenfeld's method (swapping selected item and last item in each iteration instead of literally shifting everything)*/
void ShufflePosition( uint8_t *list, uint8_t len )
{
    uint32_t j;
    int tmp;
    while(len) {
        j = irand(len);
        if (j != len - 1) {
            tmp = list[j];
            list[j] = list[len - 1];
            list[len - 1] = tmp;
        }
        len--;
    }
}


int8_t Nibble2HexChar( uint8_t a )
{
    if( a < 10 )
    {
        return '0' + a;
    }
    else if( a < 16 )
    {
        return 'A' + ( a - 10 );
    }
    else
    {
        return '?';
    }
}

//#ifdef __GNUC__
///* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
//   set to 'Yes') calls __io_putchar() */
//int __io_putchar( int c )
//#else /* __GNUC__ */
//int fputc( int c, FILE *stream )
//#endif
//{
//   return( ITM_SendChar( c ) );
//}
uint32_t TransferMacFromStrToNum( char *MacAddrStr )
{
    uint32_t MacAddrNum = 0;
//    uint32_t MacAddrNumFragment;
//    uint8_t i;

    char MacAddrStrTmp[9];
//    char *MacAddrEnd;

//    MacAddrNum = 0;
//    MacAddrNumFragment = 0;
    strncpy( MacAddrStrTmp, MacAddrStr, sizeof(MacAddrStrTmp) );

    // Re-locate the pointer address
//    MacAddrStr = MacAddrStrTmp;

//    for(i=0; i<4; i++)
//    {
//        MacAddrNumFragment = strtol(MacAddrStr, &MacAddrEnd, 16);
//        MacAddrStr = MacAddrEnd+1;
//        MacAddrNum <<= 8;
//        MacAddrNum |= MacAddrNumFragment;
//    }
    MacAddrNum = strtoul(MacAddrStrTmp, NULL, 16);

    return MacAddrNum;
}

void TransferKeyFromStrToNum( uint8_t *KeyNum, uint8_t *KeyStr , uint8_t len)
{
    uint8_t KeyStrFragment[3];
    uint8_t idx;

    for( idx = 0; idx < len ; idx++ )
    {
        memset( KeyStrFragment, '\0', sizeof( KeyStrFragment ) );

        strncpy( (char *)KeyStrFragment, (char *)KeyStr + ( idx * 2 ), 2 );

        *( KeyNum + idx ) = strtol( (char *)KeyStrFragment, NULL, 16 );
    }
}

void TransferKeyFromNumToKey( uint8_t *KeyNum, uint8_t *KeyStr , uint8_t len)
{
    uint8_t idx;

    for( idx = 0; idx < len ; idx++ )
    {
        uint8_t first, second;
        uint8_t number = KeyNum[idx];
        first = (number/16);
        second = (number%16);

        if(first <= 9)
        {
            first+=48;/*0 is 0x30*/
        }
        else
        {
            first+=55;/*A is 0x41*/
        }
        *(KeyStr + ( idx * 2 )) = first;

        if(second <= 9)
        {
            second+=48;
        }
        else
        {
            second+=55;
        }
        *(KeyStr + ( idx * 2 ) + 1) = second;
    }
}

void RemoveEndChar(uint8_t *str)
{
    uint8_t i;

    //replace '\r' and '\n' and "\r\n" with '\0'
    for(i=0; i<strlen((char*)str); i++)
    {
        if(*(str + i) == '\r' || *(str + i) == '\n')
            *(str + i) = '\0';
    }

    return;
}

void memcpyr( uint8_t *dst, const uint8_t *src, uint16_t size )
{
    dst = dst + ( size - 1 );
    while( size-- )
    {
        *dst-- = *src++;
    }
}

uint8_t strtol_char(const char ch, int base)
{
    char buf[2] = {0};
    buf[0] = ch;
    return (uint8_t)strtol(buf, NULL, base);
}
