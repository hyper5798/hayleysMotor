#include "FreeRTOS.h"
#include "uart_task.h"
#include "at_command_app.h"
#include "at_command_list.h"

CommandLine_t commandBuffer_previous;
static AtCommand_t* ATCommandList;

const AtCommand_t CommandList_User[] =
{
    {"AT+GADM",commandGADM},/* Must at first place for skip in help */
    {"AT+DTX", commandDTX},
    {"AT+DRXI",commandDRXI},
    {"AT+DRX",commandDRX},
    {"AT+DTTX", commandDTTX},
    {"AT+CSYNC",commandCSYNC},
    {"AT+CAPORT",commandCAPORT},
    {"AT+CBAP",commandCBAP},
    {"AT+CSF",commandCSF},
    {"AT+CSID",commandCSID},
    {"AT+CPIN",commandCPIN},
    {"AT+CSQ",commandCSQ},
    {"AT+CRPTM",commandCRPTM},
    /* Add by Eric Date: 2017/02/17   Log: Shift command from admin mode to end user mode */
    {"AT+CTXP", commandCTXP},
    {"AT+CPT",commandGPT},
    {"AT+CKEY",commandCKEY},
    {"AT+CMAC",commandCMAC},
    {"AT+CCHO",commandCCHO},
    {"AT+CCH",commandCCH},
    {"AT+CRXC",commandCRXC},
    {"AT+CCLASS",commandCCLASS},
#ifdef ANT_PA
    {"AT+CPAE",commandCPAE},
#endif
    //End Add
    {"AT+SLMR",commandSLMR},
    {"AT+SGMR", commandSGMR},
    {"AT+SGMI", commandSGMI},
    {"AT+SGMM", commandSGMM},
    {"AT+SGMD",commandSGMD},
    {"AT+STIMER1",commandSTIMER1},
    {"AT+STIMER",commandSTIMER},
    {"AT+SIRQ",commandSIRQ},
    {"AT+SGPIO",commandSGPIO},
    {"AT+ECHO", commandECHO},
    {"AT+IBR",commandIBR},
    {"AT+SPWMOD",commandSPWMOD},
    {"AT+CNWKSKEY",commandCNWKSKEY},
    {"AT+CAPPSKEY",commandCAPPSKEY},
    {"AT+CAPPKEY",commandCAPPKEY},
    {"AT+CDEVEUI",commandCDEVEUI},
    {"AT+CAPPEUI",commandCAPPEUI},
    //CQOS, CSYNC, CBC, SPWMOD
    {"AT+SPROFILE",commandSPROFILE},  //daniel add on 2017.1.12 new command for set senser profile
    {"AT+TESTPOWER",commandTESTPOWER}, //Jason add on 2018.11.15 new command for test power
    {"AT+TESTBAT",commandTESTBAT},     //Jason add on 2018.11.15 new command for test battery
		{"AT+TESTPINC",commandTESTPINC},   //Jason add on 2018.12.19 new command for test pc0~12
		{"AT+UPBACKPINC",commandUPBACKPINC},   //Jason add on 2018.12.19 new command for up feed back pin
		{"AT+DOWNBACKPINC",commandDOWNBACKPINC},   //Jason add on 2018.12.19 new command for down feed back pin
		{"AT+SDEVICETYPE",commandSDEVICETYPE},   //Jason add on 2019.01.29 new command for switch sensor hub and control box
    {NULL,       NULL}
};

const AtCommand_t CommandList_Eng[] =
{
    /*User Mode*/
    {"AT+IBR",commandIBR},
    {"AT+ECHO", commandECHO},
    {"AT+GADM",commandGADM},
    {"AT+DTX", commandDTX},
    {"AT+DRXI",commandDRXI},
    {"AT+DRX",commandDRX},
    {"AT+DTTX", commandDTTX},
    {"AT+CSYNC",commandCSYNC},
    {"AT+CAPORT",commandCAPORT},
    {"AT+CBAP",commandCBAP},
    {"AT+CSF",commandCSF},
    {"AT+CSID",commandCSID},
    {"AT+CPIN",commandCPIN},
    {"AT+CSQ",commandCSQ},
    {"AT+CRPTM",commandCRPTM},
    /* Add by Eric Date: 2017/02/17   Log: Shift command from admin mode to end user mode */
    {"AT+CTXP", commandCTXP},
    {"AT+CPT",commandGPT},
    {"AT+CKEY",commandCKEY},
    {"AT+CMAC",commandCMAC},
    {"AT+CCHO",commandCCHO},
    {"AT+CCH",commandCCH},
    {"AT+CRXC",commandCRXC},
    {"AT+CCLASS",commandCCLASS},
#ifdef ANT_PA
    {"AT+CPAE",commandCPAE},
#endif
    {"AT+CNWKSKEY",commandCNWKSKEY},
    {"AT+CAPPSKEY",commandCAPPSKEY},
    {"AT+CAPPKEY",commandCAPPKEY},
    {"AT+CDEVEUI",commandCDEVEUI},
    {"AT+CAPPEUI",commandCAPPEUI},

    //End Add
    {"AT+SLMR",commandSLMR},
    {"AT+SGMR", commandSGMR},
    {"AT+SGMI", commandSGMI},
    {"AT+SGMM", commandSGMM},
    {"AT+SGMD",commandSGMD},
    {"AT+STIMER1",commandSTIMER1},
    {"AT+STIMER",commandSTIMER},
    {"AT+SGPIO",commandSGPIO},
    {"AT+SIRQ",commandSIRQ},
    {"AT+SPWMOD",commandSPWMOD},
    {"AT+SPROFILE",commandSPROFILE},  //daniel add on 2017.1.12 new command for set senser profile
    {"AT+TESTPOWER",commandTESTPOWER}, //Jason add on 2018.11.15 new command for test power
    {"AT+TESTBAT",commandTESTBAT},     //Jason add on 2018.11.15 new command for test battery
		{"AT+TESTPINC",commandTESTPINC},   //Jason add on 2018.12.19 new command for test pc0~12
		{"AT+UPBACKPINC",commandUPBACKPINC},   //Jason add on 2018.12.19 new command for up feed back pin
		{"AT+DOWNBACKPINC",commandDOWNBACKPINC},   //Jason add on 2018.12.19 new command for down feed back pin
		{"AT+SDEVICETYPE",commandSDEVICETYPE},   //Jason add on 2019.01.29 new command for switch sensor hub and control box
    /*Eng Mode*/
    {"AT+GKEY",commandGKEY},
    {"AT+GPIN",commandGPIN},
    {"AT+GTXD",commandGTXD},
    {"AT+GCHO", commandGCHO},
    {"AT+GCH", commandGCH},
    {"AT+GGMD", commandGGMD},
    {"AT+GRST", commandGRST},
    {"AT+GPT", commandGPT},
    {"AT+GLMR", commandGLMR},
    {"AT+GMTXP", commandGMTXP},
    {"AT+IOTEST",commandIoTest},
    {"AT+CALRSSI",commandCaliRSSI},
    {"AT+GSYSC",commandGSYSC},
    {"AT+GCPW",commandGCPW},
    {"AT+GUUID",commandGUUID},
    {"AT+GRXC",commandGRXC},
    {NULL,       NULL}
};

ResultCode_t parse_command_buffer(CommandLine_t *commandBuffer);
void backup_previous_commandbuffer(CommandLine_t * commandBuffer);
ResultCode_t commandAtHelp(void);

void EngineerMode(uint8_t enable)
{
    if(enable == 1)
    {
        ATCommandList = (AtCommand_t* )CommandList_Eng;
    }
    else
    {
        ATCommandList = (AtCommand_t* )CommandList_User;
    }
}

ResultCode_t ATCommandParser(CommandLine_t *commandBuffer)
{
    ResultCode_t   res = RESULT_CODE_OK;
    if(!strncasecmp((char *) commandBuffer->character, "AT+", 3))
    {
        if(commandBuffer->length < 64)
        {
            backup_previous_commandbuffer(commandBuffer);
        }
        res = parse_command_buffer(commandBuffer);
    }
    else if(!strcasecmp((char *) commandBuffer->character, "AT&H"))
    {
        backup_previous_commandbuffer(commandBuffer);
        res = commandAtHelp();
    }
    else if(!strcasecmp((char *) commandBuffer->character, "ATZ"))
    {
        NVIC_SystemReset();
        return res;
    }
    else if(!strcasecmp((char *) commandBuffer->character, "AT&F"))
    {
        RestoreUserConfigToDefault();
        SaveConfigToEEPROM();
        NVIC_SystemReset();
    }
    else if(!strcasecmp((char *) commandBuffer->character, "AT&W"))
    {
        SaveConfigToEEPROM();
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
    print_result(res);
    return res;
}


void print_result(ResultCode_t res)
{
    switch(res)
    {
        case RESULT_CODE_OK:
        {
            SendMsgToUart("OK\r\n\r\n", UseUartID);
            break;
        }
        case RESULT_CODE_INVALID:
        case RESULT_CODE_ERROR:
        default:
        {
            SendMsgToUart("ERROR\r\n\r\n", UseUartID);
            break;
        }
    }
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


ResultCode_t parse_command_buffer(CommandLine_t *commandBuffer)
{
    uint8_t index,pos;
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


ResultCode_t commandAtHelp()
{
    uint16_t index;

    uint8_t msg[128] ="\r\nAll available AT commands:\r\n";
    strcat((char*)msg,"AT, AT&H, A/, ATZ, AT&W, AT&F\r\n");
    SendToUartImmediately(msg);
    memset(msg,'\0',128);

    for(index = 1; ATCommandList[index].commandString != NULL; index++)
    {
        if(index % 5 == 0)
        {
            strcat((char*)msg,"\r\n+");
            SendToUartImmediately(msg);
            memset(msg,'\0',128);
        }
        else
        {
            if(index == 1)
                strcat((char*)msg,"+");
            else
                strcat((char*)msg,", +");
        }
        strcat((char*)msg,&ATCommandList[index].commandString[3]);
    }

    strcat((char*)msg,"\r\n");
    SendToUartImmediately(msg);
    return RESULT_CODE_OK;
}


void backup_previous_commandbuffer(CommandLine_t * commandBuffer)
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

//daniel add on 2017.1.23
uint8_t get_sensor_value(CommandLine_t* commandLine, SensorConfig_t* sensorConfig)
{
    uint8_t res;
    uint16_t pos_start,len;
    char* ptr = NULL;
    int i;
    SensorConfig_t sConfig;

    memset(&sConfig,0x00,sizeof(SensorConfig_t));
    memcpy(sConfig.ManufactureID,(const char*)SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE-1);

    //1 get sensor type
    res = get_value_addr(commandLine, &pos_start, &len);
    if(res != OK || len == 0)
    {
        return FAIL;
    }
    sConfig.sensorType = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
		if(sConfig.sensorType > 4) //Jason replace 2 to 4 on 2018.11.16
    {
        return FAIL;
    }
    if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
    {
        return FAIL;
    }

    //2 get uart baud rate 
    res = get_value_addr(commandLine, &pos_start, &len);
    if(res != OK || len ==0)
    {
        return FAIL;
    }
    sConfig.uartBR = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
    if(sConfig.uartBR > 7)
    {
        return FAIL;
    }
    if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
    {
        return FAIL;
    }

    //3 get uart data bits
    res = get_value_addr(commandLine, &pos_start, &len);
    if(res != OK || len ==0)
    {
        return FAIL;
    }
    sConfig.uartDB = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
    if(sConfig.uartDB > 1)
    {
        return FAIL;
    }
    if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
    {
        return FAIL;
    }

    //4 get uart parity check
    res = get_value_addr(commandLine, &pos_start, &len);
    if(res != OK || len ==0)
    {
        return FAIL;
    }
    sConfig.uartPC = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
    if(sConfig.uartPC > 2)
    {
        return FAIL;
    }
    if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
    {
        return FAIL;
    }

    //5 get uart stop bits
    res = get_value_addr(commandLine, &pos_start, &len);
    if(res != OK || len ==0)
    {
        return FAIL;
    }
    sConfig.uartSB = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
    if(sConfig.uartSB > 3)
    {
        return FAIL;
    }
    if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
    {
        return FAIL;
    }

    //6 get length of reply data
    res = get_value_addr(commandLine, &pos_start, &len);
    if(res != OK || len == 0)
    {
        return FAIL;
    }
    sConfig.replyLEN = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
    if(sConfig.replyLEN > 64)
    {
        return FAIL;
    }
    if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
    {
        return FAIL;
    }

    //7 get start parsing byte of reply data
    res = get_value_addr(commandLine, &pos_start, &len);
    if(res != OK || len == 0)
    {
        return FAIL;
    }
    sConfig.startBYTE = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
    if(sConfig.startBYTE > 64)
    {
        return FAIL;
    }
    if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
    {
        return FAIL;
    }

    //8 get read length of reply data
    res = get_value_addr(commandLine, &pos_start, &len);
    if(res != OK || len == 0)
    {
        return FAIL;
    }
    sConfig.readLEN = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
    if(sConfig.readLEN > 8)
    {
        return FAIL;
    }
    if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
    {
        return FAIL;
    }

    //9 get modbus commandLEN
    res = get_value_addr(commandLine, &pos_start, &len);
    if(res != OK || len == 0)
    {
        return FAIL;
    }
    sConfig.modbusCMDLEN = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
    if(sConfig.modbusCMDLEN > 16)
    {
        return FAIL;
    }
    if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
    {
        return FAIL;
    }

    //10 get modbus command
    for(i=0; i<sConfig.modbusCMDLEN; i++)
    {
        res = get_value_addr(commandLine, &pos_start, &len);
        if(res != OK || len == 0)
        {
            return FAIL;
        }
        //sConfig.modbusCMD[i] = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
        sConfig.modbusCMD[i] = strtoul((const char*)&(commandLine->character[pos_start]), &ptr, 16); //daniel modify on 2017.9.25 for input HEX string
        if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
        {
            return FAIL;
        }
    }

    //Jason add for second uart-rs485 sensor on 2018.11.16 
		if(sConfig.sensorType >= 3)
    {
        //11 get length of second reply data
				res = get_value_addr(commandLine, &pos_start, &len);
				if(res != OK || len == 0)
				{
						return FAIL;
				}
				sConfig.replyLEN2 = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
				if(sConfig.replyLEN2 > 64)
				{
						return FAIL;
				}
				if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
				{
						return FAIL;
				}
				//12 get start parsing byte of second reply data
				res = get_value_addr(commandLine, &pos_start, &len);
				if(res != OK || len == 0)
				{
						return FAIL;
				}
				sConfig.startBYTE2 = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
				if(sConfig.startBYTE2 > 64)
				{
						return FAIL;
				}
				if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
				{
						return FAIL;
				}

				//13 get read length of second reply data
				res = get_value_addr(commandLine, &pos_start, &len);
				if(res != OK || len == 0)
				{
						return FAIL;
				}
				sConfig.readLEN2 = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
				if(sConfig.readLEN2 > 8)
				{
						return FAIL;
				}
				if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
				{
						return FAIL;
				}

				//14 get second modbus commandLEN
				res = get_value_addr(commandLine, &pos_start, &len);
				if(res != OK || len == 0)
				{
						return FAIL;
				}
				sConfig.modbusCMDLEN2 = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
				if(sConfig.modbusCMDLEN2 > 16)
				{
						return FAIL;
				}
				if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
				{
						return FAIL;
				}

				//15 get second modbus command
				for(i=0; i<sConfig.modbusCMDLEN2; i++)
				{
						res = get_value_addr(commandLine, &pos_start, &len);
						if(res != OK || len == 0)
						{
								return FAIL;
						}
						//sConfig.modbusCMD[i] = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
						sConfig.modbusCMD2[i] = strtoul((const char*)&(commandLine->character[pos_start]), &ptr, 16); //daniel modify on 2017.9.25 for input HEX string
						if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
						{
								return FAIL;
						}
				}
    } else {
			//Jason add for check 2 sensors format error on 2018.11.12
			res = get_value_addr(commandLine, &pos_start, &len);
			if(res != FAIL)
			{
					return FAIL;
			}
		}
		//Jason add for third uart-rs485 sensor on 2018.11.16
		if(sConfig.sensorType == 4)
    {
        //16 get length of second reply data
				res = get_value_addr(commandLine, &pos_start, &len);
				if(res != OK || len == 0)
				{
						return FAIL;
				}
				sConfig.replyLEN3 = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
				if(sConfig.replyLEN3 > 64)
				{
						return FAIL;
				}
				if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
				{
						return FAIL;
				}
				//12 get start parsing byte of second reply data
				res = get_value_addr(commandLine, &pos_start, &len);
				if(res != OK || len == 0)
				{
						return FAIL;
				}
				sConfig.startBYTE3 = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
				if(sConfig.startBYTE3 > 64)
				{
						return FAIL;
				}
				if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
				{
						return FAIL;
				}

				//13 get read length of second reply data
				res = get_value_addr(commandLine, &pos_start, &len);
				if(res != OK || len == 0)
				{
						return FAIL;
				}
				sConfig.readLEN3 = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
				if(sConfig.readLEN3 > 8)
				{
						return FAIL;
				}
				if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
				{
						return FAIL;
				}

				//14 get second modbus commandLEN
				res = get_value_addr(commandLine, &pos_start, &len);
				if(res != OK || len == 0)
				{
						return FAIL;
				}
				sConfig.modbusCMDLEN3 = strtol((const char*)&(commandLine->character[pos_start]), &ptr, 10);
				if(sConfig.modbusCMDLEN3 > 16)
				{
						return FAIL;
				}
				if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
				{
						return FAIL;
				}

				//15 get second modbus command
				for(i=0; i<sConfig.modbusCMDLEN3; i++)
				{
						res = get_value_addr(commandLine, &pos_start, &len);
						if(res != OK || len == 0)
						{
								return FAIL;
						}
						sConfig.modbusCMD3[i] = strtoul((const char*)&(commandLine->character[pos_start]), &ptr, 16); //daniel modify on 2017.9.25 for input HEX string
						if(ptr == NULL || (ptr[0] != CHAR_COMMA && ptr[0] != '\0'))
						{
								return FAIL;
						}
				}
				//Jason add for check 3 sensors format error on 2018.11.12
				res = get_value_addr(commandLine, &pos_start, &len);
				if(res != FAIL)
				{
						return FAIL;
				}
    }
    //
    memcpy(sensorConfig,&sConfig,sizeof(SensorConfig_t));
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

