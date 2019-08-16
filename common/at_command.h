#ifndef AT_COMMAND_H
#define AT_COMMAND_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "board.h"


#define ATCOMMAND_BUFFER_SIZE 256
#define UNKNOWN 0

#define CHAR_SEMICOLON		';'
#define CHAR_COMMA			','
#define CHAR_QUOTES		'\"'
#define CHAR_QUERY			'?'
#define CHAR_EQUALS		'='
#define CHAR_BS  			0x08
#define CHAR_CR				0x0D
#define CHAR_LF				0x0A
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
    uint16_t position;
    uint16_t length;
    uint8_t  *character;
} CommandLine_t;

typedef struct
{
    char *commandString;
    ResultCode_t (*commandFunc)(CommandLine_t *commandBuffer);
} AtCommand_t;

typedef ResultCode_t(*CommandCallback)(void);
typedef ResultCode_t(*HelpCommandCallback)(AtCommand_t*);
typedef void(*PrintResultCallback)(ResultCode_t);

void SetATCommandCallback(PrintResultCallback printResultCB, HelpCommandCallback helpCB
    , CommandCallback restoreCB, CommandCallback saveConfigCB);
void SetATCommandList(const AtCommand_t* commandList);

ResultCode_t ATCommandParser(CommandLine_t  *commandBuffer);
OperationTag_t get_operation_tag(CommandLine_t *commandBuffer);

uint8_t get_value_addr(CommandLine_t* commandLine, uint16_t* pos_start, uint16_t *len);
uint8_t get_string_addr(CommandLine_t* commandLine, uint16_t* pos_start, uint16_t* len);
uint8_t get_numerical_value(CommandLine_t* commandLine, uint32_t* value, uint16_t base);
uint8_t get_string_value(CommandLine_t* commandLine, uint8_t* value, uint16_t maxLen);
uint8_t get_value(CommandLine_t * commandLine, uint8_t* value, uint16_t maxLen);

#endif
