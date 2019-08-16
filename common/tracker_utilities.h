#ifndef __TRACKER_UTILITIES_H__
#define __TRACKER_UTILITIES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//#if defined(USE_BAND_868)
//    #define FW_COUNTRY_TAG      "EU_868"
//#elif defined (USE_BAND_915)
//    #define FW_COUNTRY_TAG      "US_915"
//#elif defined (USE_BAND_780)
//    #define FW_COUNTRY_TAG      "CN_780"
//#elif defined (USE_BAND_433)
//    #define FW_COUNTRY_TAG      "EU_433"
//#else
//    #error "Please define USE_BAND"
//#endif

#define LOGBANK_MAX_SIZE            (0x20) //32 bytes
#define LOGBANK_BUFF_LEN            (0x100) //256 bytes
#define LOGBANK_EEPROM_STAR_ADDR    (0x1D000)
#define LOGBANK_EEPROM_LOG_ADDR     (LOGBANK_EEPROM_STAR_ADDR + 0x100)

#define JSON_STR_MAX_LEN            UART1_TX_BUFF_LEN
#define JSON_PARSE_VALUE_MAX_LEN    (64)

enum {
    sysErrorCode_noError = 0,
    sysErrorCode_hardFault,
    sysErrorCode_memManage,
    sysErrorCode_busFault,
    sysErrorCode_usageFault,
    sysErrorCode_mallocFail,
    sysErrorCode_stackOverflow,
    sysErrorCode_lowBattery,
};

extern uint16_t heap_free;

/* Check string value is binary, oct, or hex */
uint8_t checkVauleFormate(uint8_t* str, uint8_t base, uint8_t length);

/* log bank init, read LogBank_nextIndex from external EEPROM */
void LogBank_init(void);

/* push log to bank */
uint8_t LogBank_push(const char *format, ...);

/* get now LogBank_nextIndex */
uint16_t LogBank_getNextIndex(void);

/* pop a log with index = logIndex */
void LogBank_pop(uint16_t logIndex, uint8_t *buff, uint16_t len, uint8_t force);

/* clean all log and reset LogBank_nextIndex */
void LogBank_clean(void);

/* read/write error code to EEPROM */
void sysErrorCode_save(uint8_t err);
uint8_t sysErrorCode_read(void);

/* check value format before strtoul() */
uint8_t tracker_strtoul(uint8_t *input_str, uint32_t *output_int, uint8_t base);

/* check value format before atoi() */
uint8_t tracker_atoi(uint8_t *input_str, uint32_t *output_int);

/* trans hex string to byte array */
uint8_t hexStr2ByteArr(uint8_t *hexStr, uint8_t *byteArr, uint8_t hexStrlen);

/*!
 * Get value by name from JSON string.
 * Author: Crux
 * \param [IN]  jsonStr     input JSON string
 * \param [IN]  name        name
 * \param [OUT] value       value buffer
 * \param [OUT] value_len   value buffer length
 * \param [return] result   OK: parse sucessfully, FAIL: parse failed
 */
uint8_t getJsonValue(uint8_t *jsonStr, uint8_t *name, uint8_t *value, uint8_t value_len);

/*!
 * Parse multi value in JSON.
 *
 * example:
 *
    uint8_t ret, json_str[] = "{\"name1\":\"1\",\"name2\":\"22\",\"name3\":\"333\"}";
    
    json_multi_parser_buff_t json_parse_buff[] = 
    {
        {"name1", NULL, FAIL},
        {"name2", NULL, FAIL},
        {"name3", NULL, FAIL},
        {NULL, NULL, NULL} //MUST end with NULL
    };
    
    ret = jsonMultiParser(json_str, json_parse_buff);
    
    //result:
    // json_parse_buff[] = 
    // {
    //     {"name1", "1", OK},
    //     {"name2", "22", OK},
    //     {"name3", "333", OK},
    //     {NULL, NULL}
    // };
 *
 * Author: Crux
 * \param [IN]  json_src     input JSON string
 * \param [OUT] parser_buff  parse result
 * \param [return] result    OK: parse sucessfully, FAIL: parse failed
 */
typedef struct
{
    char *keyword;
    char value[JSON_PARSE_VALUE_MAX_LEN];
    uint8_t result;
}json_multi_parser_buff_t;

uint8_t jsonMultiParser(uint8_t *json_src, json_multi_parser_buff_t *parser_buff);

#ifdef __cplusplus
}
#endif

#endif /*__TRACKER_UTILITIES_H__*/
