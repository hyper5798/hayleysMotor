#include "tracker_utilities.h"

/* board */
#include "rtc-board.h"
#include "eeprom-board.h"

/* peripherals */
#include "at24cm01.h"

/* FreeRTOS */
#include "FreeRTOS.h"

/* common */
#include "tracker_common.h"
#include "jsmn.h"

/* std. */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <string.h>

uint16_t heap_free;

/* Check string value is binary, oct, or hex
    - Input:
        str: string
        base: 2,10,16
        length: string length
    - Output:
       Correct: OK
       Incorrect: FAIL
   - Example:
      checkVauleFormate("04000000", 16, 8) ==> OK
      checkVauleFormate("0400000G", 16, 8) ==> FAIL
      checkVauleFormate("04", 16, 3) ==> FAIL
*/
uint8_t checkVauleFormate(uint8_t* str, uint8_t base, uint8_t length)
{
    char* ptr = NULL;
    if(base != 0)
    {
        strtol((const char*)str, &ptr, base);
        if(*ptr != '\0')
        {
            return FAIL;
        }
    }
    if(length != 0)
    {
        if(strlen((const char*)str) != length)
        {
            return FAIL;
        }
    }
    return OK;
}

uint16_t LogBank_nextIndex = 0;
bool LogBank_initialized = false;

/* log bank init, read LogBank_nextIndex from external EEPROM */
void LogBank_init()
{
    uint8_t ret = FAIL;
    
    ret = AT24CM01EepromReadBuffer(LOGBANK_EEPROM_STAR_ADDR, (uint8_t*)&LogBank_nextIndex, sizeof(LogBank_nextIndex));
    
    if(ret != FAIL)
    {
        LogBank_initialized = true;
        
        if(LogBank_nextIndex == 0xffff)
        {
            //reset index
            LogBank_nextIndex = 0;
            AT24CM01EepromWriteBuffer(LOGBANK_EEPROM_STAR_ADDR, (uint8_t*)&LogBank_nextIndex, sizeof(LogBank_nextIndex));
        }
    }
    else
    {
        LogBank_initialized = false;
    }
    
    return;
}

/* push log to bank */
uint8_t LogBank_push(const char *format, ...)
{
    uint8_t ret = FAIL;
    
    if(LogBank_initialized == true)
    {
        uint32_t EEPROM_WRITE_ADDR;
        uint8_t *buf, hour, min, sec, len;
        va_list args;
        
        buf = pvPortMalloc(LOGBANK_BUFF_LEN);
        memset(buf, 0, LOGBANK_BUFF_LEN);
        
        //prepare log
        RtcGetTime(&hour, &min, &sec);
        snprintf((char*)buf, 16, "[%d][%d:%02d:%02d] ", LogBank_nextIndex, hour, min, sec);
        len = strlen((char*)buf);
        
        va_start(args, format);
        vsnprintf((char*)&buf[len], (LOGBANK_BUFF_LEN-1)-len, format, args);
        va_end(args);
        
        buf[LOGBANK_BUFF_LEN-1] = '\0'; //force string end at the end of buffer
        
        //save to EEPROM
        EEPROM_WRITE_ADDR = LOGBANK_EEPROM_LOG_ADDR + (LogBank_nextIndex % LOGBANK_MAX_SIZE) * (LOGBANK_BUFF_LEN);
        ret = AT24CM01EepromWriteBuffer(EEPROM_WRITE_ADDR, buf, LOGBANK_BUFF_LEN);
        
        vPortFree(buf);
        
        //update index
        LogBank_nextIndex++;
        AT24CM01EepromWriteBuffer(LOGBANK_EEPROM_STAR_ADDR, (uint8_t*)&LogBank_nextIndex, sizeof(LogBank_nextIndex));
    }
    
    return ret;
}

/* get now LogBank_nextIndex */
uint16_t LogBank_getNextIndex()
{
    AT24CM01EepromReadBuffer(LOGBANK_EEPROM_STAR_ADDR, (uint8_t*)&LogBank_nextIndex, sizeof(LogBank_nextIndex));
    return LogBank_nextIndex;
}

/* pop a log with index = logIndex */
void LogBank_pop(uint16_t logIndex, uint8_t *buff, uint16_t len, uint8_t force)
{
    uint16_t targetIndex;
    uint8_t targetBank;
    
    if(force == 0)
    {
        if(LogBank_nextIndex == 0)
        {
            targetIndex = 0;
        }
        else if(logIndex >= LogBank_nextIndex)
        {
            targetIndex = LogBank_nextIndex - 1;
        }
        else
        {
            targetIndex = logIndex;
        }
    }
    else
    {
        targetIndex = logIndex;
    }
    
    targetBank = targetIndex % LOGBANK_MAX_SIZE;
    AT24CM01EepromReadBuffer(LOGBANK_EEPROM_LOG_ADDR + (targetBank * LOGBANK_BUFF_LEN), buff, len);
    return;
}

/* clean all log and reset LogBank_nextIndex */
void LogBank_clean()
{
    uint8_t *buf, i;
    
    buf = pvPortMalloc(LOGBANK_BUFF_LEN);
    memset(buf, 0, LOGBANK_BUFF_LEN);
    
    for(i=0; i<=LOGBANK_MAX_SIZE; i++)
    {
        AT24CM01EepromWriteBuffer(LOGBANK_EEPROM_STAR_ADDR + (i*LOGBANK_BUFF_LEN), buf, LOGBANK_BUFF_LEN);
    }
    
    //update next index
    AT24CM01EepromReadBuffer(LOGBANK_EEPROM_STAR_ADDR, (uint8_t*)&LogBank_nextIndex, sizeof(LogBank_nextIndex));
    return;
}

/* save error code to external EEPROM */
void sysErrorCode_save(uint8_t err)
{
    InternalEepromWriteBuffer(EEPROM_ERROR_CODE_ADDR, (uint8_t*)&err, sizeof(err));
    return;
}

/* read error code from external EEPROM */
uint8_t sysErrorCode_read()
{
    uint8_t ret;
    InternalEepromReadBytes(EEPROM_ERROR_CODE_ADDR, &ret, sizeof(ret));
    return ret;
}

/* check value format before strtoul() */
uint8_t tracker_strtoul(uint8_t *input_str, uint32_t *output_int, uint8_t base)
{
    uint8_t len;
    
    if(input_str == NULL || output_int == NULL)
        return FAIL;
    
    len = strlen((char*)input_str);
    if(len == 0)
        return FAIL;
    
    if(checkVauleFormate(input_str, base, len) == FAIL)
        return FAIL;
    
    *output_int = strtoul((char*)input_str, NULL, base);
    return OK;
}

/* check value format before atoi() */
uint8_t tracker_atoi(uint8_t *input_str, uint32_t *output_int)
{
    uint8_t len;
    
    if(input_str == NULL || output_int == NULL)
        return FAIL;
    
    len = strlen((char*)input_str);
    if(len == 0)
        return FAIL;
    
    if(checkVauleFormate(input_str, 10, len) == FAIL)
        return FAIL;
    
    *output_int = (uint32_t)atoi((char*)input_str);
    return OK;
}

/* trans hex string to byte array */
uint8_t hexStr2ByteArr(uint8_t *hexStr, uint8_t *byteArr, uint8_t hexStrlen)
{
    uint8_t i, buff[3] = {0};
    
    if(hexStr == NULL || byteArr == NULL || hexStrlen == 0)
        return FAIL;
    
    i = strlen((char*)hexStr);
    if(i < hexStrlen || hexStrlen%2 != 0)
        return FAIL;
    
    if(checkVauleFormate(hexStr, 16, hexStrlen) == FAIL)
        return FAIL;
    
    /* Modified by Crux, Date: 2017/03/09, Log: Fix length not correct problem */
    for(i=0; i<hexStrlen/2; i++)
    {
        buff[0] = *(hexStr + i*2);
        buff[1] = *(hexStr + i*2 + 1);
        *(byteArr + i) = strtol((char*)buff, NULL, 16);
        memset(buff, 0, 3);
    }
    return OK;
}

static jsmn_parser js_pars;
static jsmntok_t js_tok[32];
static int8_t jsoneq(char *json, jsmntok_t *tok, const char *s)
{
    if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
            strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}

uint8_t getJsonValue(uint8_t *jsonStr, uint8_t *name, uint8_t *value, uint8_t value_len)
{
    int16_t json_ret, idx;
    
    jsmn_init(&js_pars);
    json_ret = jsmn_parse(&js_pars, (char*)jsonStr, strlen((char*)jsonStr), js_tok, 32);
    
    if (json_ret < 0) 
    {
        //Failed to parse JSON
        return FAIL;
    }
    
    //Assume the top-level element is an object
    if (json_ret < 1 || js_tok[0].type != JSMN_OBJECT)
    {
        return FAIL;
    }
    
    for(idx = 1; idx < json_ret; idx ++)
    {
        if(jsoneq((char*)jsonStr, &js_tok[idx], (char*)name) == 0)
        {
            snprintf((char*)value, value_len, "%.*s", 
                js_tok[idx+1].end-js_tok[idx+1].start, jsonStr+js_tok[idx+1].start);
            return OK;
        }
    }
    
    return FAIL;
}

uint8_t jsonMultiParser(uint8_t *json_src, json_multi_parser_buff_t *parser_buff)
{
    uint8_t ret, i = 0;
    
    while(parser_buff[i].keyword != NULL)
    {
        parser_buff[i].result = getJsonValue(
                                    json_src, 
                                    (uint8_t*)parser_buff[i].keyword, 
                                    (uint8_t*)parser_buff[i].value,
                                    JSON_PARSE_VALUE_MAX_LEN);
        i++;
    }
    
    if(i == 0)
        ret = FAIL;
    else
        ret = OK;
    return ret;
}

