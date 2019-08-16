#ifndef AT_COMMAND_APP_H
#define AT_COMMAND_APP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "board.h"

#ifdef STM32L073xx
#define ATCOMMAND_BUFFER_SIZE 256
#else
#define ATCOMMAND_BUFFER_SIZE 200
#endif

#define UNKNOWN 0

#define CHAR_SEMICOLON		';'
#define CHAR_COMMA			','
#define CHAR_QUOTES		'\"'
#define CHAR_QUERY			'?'
#define CHAR_EQUALS		'='
#define CHAR_BS  			0x08
#define CHAR_CR				0x0D
#define CHAR_LF				0x0A
#define CHAR_AT	      0x2B
#define CHAR_HEADER	  0xAA
#define CHAR_END	    0x8E
#define CTRL_C                      0x0C
#define CHAR_BACKSPACE   0X08

typedef enum
{
    RESULT_CODE_INVALID        = -1,
    RESULT_CODE_OK             =  0,
    RESULT_CODE_ERROR          =  1,
    RESULT_CODE_DENY ,
    RESULT_CODES_NUMBER,
} ResultCode_t;

typedef enum
{
    OPERATION_INVALID,
    OPERATION_ACTION,
    OPERATION_QUERY,
    OPERATION_ASSIGN,
    OPERATION_RANGE,
    OPERATION_NUMBER
} OperationTag_t;


typedef struct
{
    uint8_t position;
    uint16_t length;
    uint8_t  *character;
} CommandLine_t;


typedef struct
{
    char *commandString;
    ResultCode_t (*commandFunc)(CommandLine_t *commandBuffer);
} AtCommand_t;

ResultCode_t ATCommandParser(CommandLine_t  *commandBuffer);
void EngineerMode(uint8_t enable);

void print_result(ResultCode_t res);
OperationTag_t get_operation_tag(CommandLine_t *commandBuffer);
uint8_t get_value_addr(CommandLine_t* commandLine, uint16_t* pos_start, uint16_t *len);
uint8_t get_string_addr(CommandLine_t* commandLine, uint16_t* pos_start, uint16_t* len);
uint8_t get_numerical_value(CommandLine_t* commandLine, uint32_t* value, uint16_t base);
uint8_t get_string_value(CommandLine_t* commandLine, uint8_t* value, uint16_t maxLen);
uint8_t get_value(CommandLine_t * commandLine, uint8_t* value, uint16_t maxLen);

#endif
