#include "FreeRTOS.h"
#include "uart-board.h"
#include "at_command.h"

CommandLine_t commandBuffer_previous;
static AtCommand_t* ATCommandList;
static PrintResultCallback commandPrintResultCB= NULL;
static HelpCommandCallback commandHelpCB = NULL;
static CommandCallback commandRestoreConfigCB= NULL;
static CommandCallback commandSaveConifgCB= NULL;

ResultCode_t parseCommand(CommandLine_t *commandBuffer);
void backupPreviousCommand(CommandLine_t * commandBuffer);

void SetATCommandCallback(PrintResultCallback printResultCB
    , HelpCommandCallback helpCB, CommandCallback restoreCB
    , CommandCallback saveConfigCB)
{
    //set call back function
    commandPrintResultCB = printResultCB;
    commandHelpCB = helpCB;
    commandRestoreConfigCB = restoreCB;
    commandSaveConifgCB = saveConfigCB;
}


void SetATCommandList(const AtCommand_t* commandList)
{
    ATCommandList = (AtCommand_t*) commandList;
}

ResultCode_t ATCommandParser(CommandLine_t *commandBuffer)
{
    ResultCode_t   res = RESULT_CODE_OK;
    if(!strncasecmp((char *) commandBuffer->character, "AT+", 3))
    {
        backupPreviousCommand(commandBuffer);
        res = parseCommand(commandBuffer);
    }
    else if(!strcasecmp((char *) commandBuffer->character, "AT&H"))
    {
        backupPreviousCommand(commandBuffer);
        res = commandHelpCB(ATCommandList);
    }
    else if(!strcasecmp((char *) commandBuffer->character, "ATZ"))
    {
        NVIC_SystemReset();
        return res;
    }
    else if(!strcasecmp((char *) commandBuffer->character, "AT&F"))
    {
        res = commandRestoreConfigCB();
    }
    else if(!strcasecmp((char *) commandBuffer->character, "AT&W"))
    {
        res = commandSaveConifgCB();
    }
    else if(!strcasecmp((char *) commandBuffer->character, "AT"))
    {
    }
    else if(!strcasecmp((char *) commandBuffer->character, "A/"))
    {
        if(commandBuffer_previous.length != 0)
        {
            ATCommandParser(&commandBuffer_previous);
            return res;
        }
    }
    else
    {
        res = RESULT_CODE_INVALID;
    }
    commandPrintResultCB(res);
    return res;
}


OperationTag_t get_operation_tag(CommandLine_t *commandBuffer)
{
    OperationTag_t result;
    if(commandBuffer->position < commandBuffer->length)
    {
        switch(commandBuffer->character[commandBuffer->position])
        {
            case CHAR_QUERY:
            {
                commandBuffer->position++;
                result = OPERATION_QUERY;
                break;
            }
            case CHAR_EQUALS:
            {
                commandBuffer->position++;
                if((commandBuffer->position < commandBuffer->length) &&
                        (commandBuffer->character[commandBuffer->position] == CHAR_QUERY))
                {
                    commandBuffer->position++;
                    result = OPERATION_RANGE;
                }
                else
                {
                    result = OPERATION_ASSIGN;
                }
                break;
            }
            default:
            {
                result = OPERATION_INVALID;
                break;
            }
        }
    }
    else
    {
        result = OPERATION_ACTION;
    }
    return result;
}


ResultCode_t parseCommand(CommandLine_t *commandBuffer)
{
    uint16_t index,pos;
    ResultCode_t result = RESULT_CODE_INVALID;
    index = 0;
    while(ATCommandList[index].commandString != NULL)
    {
        pos = strlen(ATCommandList[index].commandString);
        if(!strncasecmp((char *)commandBuffer->character, ATCommandList[index].commandString, pos))
        {
            commandBuffer->position = pos;
            result = ATCommandList[index].commandFunc(commandBuffer);
            break;
        }
        index++;
    }
    return result;
}

void backupPreviousCommand(CommandLine_t * commandBuffer)
{
    if(strcmp((const char*)commandBuffer_previous.character, (const char*)commandBuffer->character) == 0)
    {
        //same command or previous command is "A/"
        return;
    }
    if(commandBuffer_previous.character != NULL)
    {
        vPortFree(commandBuffer_previous.character);
    }
    commandBuffer_previous.character = (uint8_t*) pvPortMalloc(sizeof(uint8_t) * (commandBuffer->length+1));
    memset(commandBuffer_previous.character,'\0',commandBuffer->length +1);
    memcpy(commandBuffer_previous.character,commandBuffer->character,commandBuffer->length);
    commandBuffer_previous.length = commandBuffer->length;
    commandBuffer_previous.position = commandBuffer ->position;
}

/*
    This function is for getting the start address and the length of value which between
    two CHAR_COMMA. Also, it will move the "commandLine->position" to the next
    value start address or the end position of string
        - pos_start: start address of now value
        - len: length of now value
    Ex:
        Input:
            commandLine(character="at+dtx=3,"123","abc"", position=9, length=20)
        Output:
            pos_start=9, len=5, commandLine->position=15
   Note:
        It should not use this function directly,
        It's better use "get_numerical_value" or "get_string_value" or "get_value" first.
*/
uint8_t get_value_addr(CommandLine_t* commandLine, uint16_t* pos_start, uint16_t* len)
{
    *pos_start = commandLine->position;
    *len = 0;
    if(commandLine->position >= commandLine->length)
    {
        return FAIL;
    }
    while(commandLine->position < commandLine->length &&
            commandLine->character[commandLine->position] != CHAR_COMMA)
    {
        commandLine->position++;
        *len = *len + 1;
    }
    if(commandLine->position < commandLine->length)
    {
        commandLine->position++;
    }
    if(*len + *pos_start > commandLine->length)
    {
        *len = *len -1;
    }
    return OK;
}

/*
    This function is for getting the start address and the length of string (using CHAR_QUOTES
    at start and end). Also, it will move the "commandLine->position" to the next value start
    address or the end position of string.
        -pos_start: string start address
        -len: string length
    Ex:
        Input:
            commandLine(character="at+dtx=3,"123","abc"", position=9, length=20)
        Output:
            pos_start=10, len=3, commandLine->position=15
   Note:
        It should not use this function directly,
        It's better use "get_numerical_value" or "get_string_value" or "get_value" first.
*/
uint8_t get_string_addr(CommandLine_t* commandLine, uint16_t* pos_start, uint16_t* len)
{
    if(commandLine->position >= commandLine->length)
    {
        return FAIL;
    }
    if(commandLine->character[commandLine->position] != CHAR_QUOTES)
    {
        //commandLine->position++;
        return FAIL;
    }
    commandLine->position++;
    *pos_start = commandLine->position;
    *len = 1;
    //Find next CHAR_QUOTES
    while(commandLine->position < commandLine->length &&
            commandLine->character[commandLine->position] != CHAR_QUOTES)
    {
        commandLine->position++;
        *len = *len + 1;
    }
    //If CHAR_QUOTES not at the last char, move the position
    if(commandLine->position + 1 < commandLine->length)
    {
        commandLine->position = commandLine->position + 2;
    }
    if(commandLine->character[*pos_start+*len -1] != CHAR_QUOTES || *len <= 1)
    {
        return FAIL;
    }
    *len = *len -1;
    return OK;
}

/*
    This function is for getting the numerical value . It will move the "commandLine->position"
    to the next value start address or the end position of string.
        -value: numerical number
        -base: 2, 8, 10 or 16
    Ex:
        Input:
            commandLine(character="at+dtx=3,"123","abc"", position=7, length=20)
        Output:
            value=3, commandLine->position=9
*/

uint8_t get_numerical_value(CommandLine_t* commandLine, uint32_t* value, uint16_t base)
{
    uint8_t res;
    uint16_t pos_start,len;
    char* ptr = NULL;
    res = get_value_addr(commandLine, &pos_start, &len);
    if(res != OK || len ==0)
    {
        return FAIL;
    }
    *value = strtol((const char*)&(commandLine->character[pos_start]), &ptr, base);
    if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
    {
        *value = 0;
        return FAIL;
    }
    return OK;
}

/*
    This function is for getting the string . It will move the "commandLine->position"
    to the next value start address or the end position of string.
        -value: string
        -maxLen: string length
    Ex:
        Input:
            commandLine(character="at+dtx=3,"123","abc"", position=9, length=20)
        Output:
            value=123, commandLine->position=15
*/
uint8_t get_string_value(CommandLine_t* commandLine, uint8_t* value, uint16_t maxLen)
{
    uint8_t res;
    uint16_t pos_start;
    uint16_t len;
    if(value == NULL)
    {
        return FAIL;
    }
    memset(value,'\0', maxLen);
    res = get_string_addr(commandLine, &pos_start, &len);
    if(res != OK || len > maxLen)
    {
        return FAIL;
    }
    memcpy(value,&(commandLine->character[pos_start]),len);
    return OK;
}


uint8_t get_value(CommandLine_t * commandLine, uint8_t* value, uint16_t maxLen)
{
    uint8_t res;
    uint16_t pos_start,len;
    if(value ==NULL)
    {
        return FAIL;
    }
    memset(value,'\0', maxLen);
    res = get_value_addr(commandLine, &pos_start, &len);
    if(res != OK || len > maxLen)
    {
        return FAIL;
    }
    memcpy(value,&(commandLine->character[pos_start]),len);
    return OK;
}


