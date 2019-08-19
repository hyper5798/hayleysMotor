#include "at_command_list.h"
#include "at_command_app.h"
#include "uart_task.h"
#include "board.h"
#include "eeprom-board.h"
#include "radio_task.h"
#include "timer_task.h"
#include "utilities.h"
#include "LoRaMacCrypto.h"

#include "sx1276.h"
#include "RadioChannel.h"
#include "Production.h"
#include "uart_task.h"
#include "adc-board.h"
#include "board.h"
#include "crc16.h"



/*Variable for Radio*/
#ifdef STM32L073xx
#define MAX_RADIO_TX_SIZE 256
#else
#define MAX_RADIO_TX_SIZE 128
#endif
RadioSendData_t RadioSendData;
uint8_t RadioTxDelayFlag = 0;
uint32_t RadioTxDelayTimer;
uint32_t DTTXTimer;

/*DRX*/
#ifdef STM32L073xx
#define MAX_RADIO_RX_DATA_SIZE 256
#else
#define MAX_RADIO_RX_DATA_SIZE 128
#endif
RadioRxData_t RadioRxData;

/*Variable for Node configuration*/
NodeConfig_t NodeConfig;
AtConfig_t AtConfig;
At2Config_t At2Config;//add on 2018.6.7 for new command
EngConfig1_t EngConfig1;
UserConfig1_t UserConfig1;
RxConfig_t RxConfig;
OthersConfig_t OthersConfig;
LoRaWanConfig_t LoRaWanConfig;
SensorConfig_t SensorConfig; //daniel add on 2017.1.17 for get sensor profile

uint8_t EEPROM_KEY[] =
{
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

/*Production test*/
uint8_t TXPPRunningFlag = 0;
uint8_t TXCMRunningFlag = 0;

/*Local Functions*/
void unUsedGpioInit(void);
void updateLoRaMacInitNwkIds(void);
void engineerModeRestoreDefault(void);
void dTTXCallback(uint32_t timer);
//uint8_t powerMappingTable(uint8_t value, uint8_t* index);
uint8_t sFMappingTable(uint8_t value, uint8_t* index);
uint8_t checkVauleFormate(uint8_t* str, uint8_t base, uint8_t length);
uint8_t converterStringToAscii(uint8_t* asciiArray, uint8_t* strArray);
uint32_t baudRateMappingTable(uint8_t ibr);


/*Production test*/
uint8_t GpioTest(uint8_t* pinNameIn, uint8_t* pinNameOut, uint8_t pinOutValue);
NodeTest TxppConvertStringToNumerical(txpp_t *rawdata, bool *masterID, uint16_t *timeout, uint16_t *number);
NodeTest TxcmConvertStringToNumerical(txcm_t *rawdata, RadioTxContinuousMode_t *data);
NodeTest GetProductionTestCommand(uint8_t *string);
NodeTest GetTXCMData(uint8_t* string, RadioTxContinuousMode_t* RadioPTModeData);
NodeTest GetTXPPData(uint8_t* string, RadioTxContinuousMode_t* RadioPTModeData
                     , bool *isMaster, uint16_t* TxNumOrRxTimeIdx, uint8_t* label);
//daniel add on 2017.3.24
extern uint8_t get_sensor_value(CommandLine_t* commandLine, SensorConfig_t* sensorConfig);


/* Initial all configuration at startup
    1. Echo mode
    2. Engineer mode
    3. Mac addr
    4. App key and network key
    5. baudrate

*/
#define ID_FLASH_ADDRESS1		0x1FF8004C
#define TM_ID_GetFlashSize()	(*(uint16_t *) (ID_FLASH_ADDRESS1))

volatile uint32_t FreeHeapSize;

uint8_t InitAtCommandList()
{
    uint8_t sendBuff[120];
    uint32_t baudrate;
    uint8_t sfIndex;
    Rx2ChannelParams_t parameter;

    //Check Flash size
    //TM_ID_GetFlashSize1()
    RadioSendData.Data = pvPortMalloc(MAX_RADIO_TX_SIZE+1);

    unUsedGpioInit();
    BoardGetEEPROMKey(EEPROM_KEY);
    /* Modify by Eric Date: 2017/02/17   Log: Fix Load Configration from EEPROM issue. */
    LoadNodeConfigFromEEPROM();
    LoadAtConfigFromEEPROM();
    LoadEngConfigFromEEPROM();
    LoadUserConfigFromEEPROM();
    LoadRxConfigFromEEPROM();
    LoadOthersConfigFromEEPROM();
    LoadLoRaWanFromEEPROM();

    //daniel add on 2017.3.15 for test //replace default - begin
    #if 0
    memset(NodeConfig.MacAddr,'\0',CONFIG_MAC_SIZE);
    memset(NodeConfig.SystemId,'\0',CONFIG_SYS_ID_SIZE);
    memset(NodeConfig.NwkSessionKey,'\0',CONFIG_NWKSKEY_SIZE);
    memset(NodeConfig.AppSessionKey,'\0',CONFIG_APPSKEY_SIZE);
    memset(NodeConfig.PinCode,'\0',CONFIG_PINCODE_SIZE);
    memset(NodeConfig.Sn,'\0',CONFIG_SN_SIZE);
    //
    memcpy(NodeConfig.MacAddr,MY_DEFAULT_MAC_ADDR,CONFIG_MAC_SIZE);
    memcpy(NodeConfig.SystemId,MY_DEFALUT_SYSTEM_ID,CONFIG_SYS_ID_SIZE);
    memcpy(NodeConfig.NwkSessionKey,MY_DEFALUT_NETWORK_KEY,CONFIG_NWKSKEY_SIZE);
    memcpy(NodeConfig.AppSessionKey,MY_DEFALUT_APPLICATION_KEY,CONFIG_APPSKEY_SIZE);
    memcpy(NodeConfig.PinCode,MY_DEFAULT_PINCODE,CONFIG_PINCODE_SIZE);
    memcpy(NodeConfig.Sn,MY_DEFALUT_SN,CONFIG_SN_SIZE);
    SaveConfigToEEPROM();
    #endif
    //daniel add on 2017.3.15 for test //replace default - end

    /*Radio*/
    updateLoRaMacInitNwkIds();
    SetRadioChannelOnOffset(NodeConfig.FreqA1,NodeConfig.FreqA2
                            ,NodeConfig.FreqB1,NodeConfig.FreqB2
                            ,NodeConfig.Offset1,NodeConfig.Offset2);
    sFMappingTable(AtConfig.SF_TX, &sfIndex);
    SetRadioDatarate(sfIndex);
    //Set Rx config
    sFMappingTable(RxConfig.RxSF, &sfIndex);
    parameter.Datarate = sfIndex;
    parameter.Frequency = RxConfig.RxFreq;
    SetRadioRxChannel(parameter);
    SetRadioRxBandwidth(RxConfig.RxBW);
    if(NodeConfig.ClassMode<3)
    {
        // Prevent default value incorrect
        SetClassMode((DeviceClass_t) NodeConfig.ClassMode);
    }
    else
    {
        /*Modify by Eric Date: 2017/04/17 Log: Init class mode value to prevent invalid value*/
        /*Modify by Gavin Date: 2017/04/14 Log: Prevent not init Radio when define STM32L073xx*/
        NodeConfig.ClassMode = (uint8_t)CLASS_A;
        SetClassMode(CLASS_A);
    }
    //Set Tx Power
    SetRadioDefaultPower(AtConfig.Power_TX);
    SX1276SetRSSICalHF(NodeConfig.CaliRSSI);
    /*Init Engineer Mode*/
    EngineerMode(AtConfig.EngineerModeFlag);
    /* Init Uart Task */
    if(AtConfig.EchoFlag== 0)
    {
        SendQueueToUartTask(UartDisableEcho, UseUartID, NULL);
    }
    else
    {
        SendQueueToUartTask(UartEnableEcho, UseUartID, NULL);
    }
    /* Init PA */
#ifdef ANT_PA
    if(OthersConfig.PAEnable == 0)
    {
        BoardSetPAEnable(0);
    }
    else
    {
        BoardSetPAEnable(1);
    }
#endif
    baudrate = baudRateMappingTable(AtConfig.IBR);
    SendQueueToUartTask(UartEnable, UseUartID, &baudrate);
    sprintf((char*)sendBuff, "\r\nGIOT AT Command for LoRa Module %s %s\r\nMAC : %s\r\n"
            ,SIPMODULE_MODEL_ID,SIPMODULE_FW_VERSION,OthersConfig.CMacAddr);
    SendMsgToUart(sendBuff, UseUartID);
    // Turn on LED
    RadioTxLED(0);
		At2Config.PowerFlag = DEFAULT_TESTPOWER; //add on 2018.6.7
    return OK;
}


/**********************************************
    All At command list
***********************************************/

ResultCode_t commandIBR(CommandLine_t *commandLine)
{
    uint8_t res;
    uint32_t iBR = 0;// baudrate: 0->default 9600, 1~5->9600,19200,38400,57600,115200
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+IBR:%d\r\n",AtConfig.IBR);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+IBR=<0-5>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint32_t baudrate;
            res = get_numerical_value(commandLine,&iBR, 10);
            if(res != OK || iBR >5)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            AtConfig.IBR = (uint32_t)iBR;
            baudrate = baudRateMappingTable(AtConfig.IBR);
            SendQueueToUartTask(UartDisable,UseUartID,NULL);
            SendQueueToUartTask(UartEnable, UseUartID, &baudrate);
            SendToUartImmediately("\r\n");
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandSLMR(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+SLMR:\"%s\"\r\n",NodeConfig.HWVersion);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandSGMR(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_RANGE:
        {
            uint8_t sendBuff[80];
            if(AtConfig.EngineerModeFlag == 0)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            sprintf((char*)sendBuff, "+Release Date: %s, Time: %s\r\n%s\r\n",__DATE__,__TIME__,FW_DEFINE);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+SGMR:\"%s\"\r\n",SIPMODULE_FW_VERSION);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandSGMI(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+SGMI:\"%s\"\r\n",SIPMODULE_MANUFACTUER_ID);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandSGMM(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+SGMM:\"%s\"\r\n",SIPMODULE_MODEL_ID);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandSGMD(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+SGMD:\"%s\",\"%s\"\r\n",OthersConfig.CMacAddr,NodeConfig.Sn);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+SGMD=\"MAC:length is 8\",\"SN:length is 13\"\r\n", UseUartID);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandSTIMER1(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+STIMER1:%d,%d\r\n"
                    ,UserConfig1.GpioReportTime_minutes1,UserConfig1.GpioReportTime_day1);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+STIMER1=<val of minutes: 1-1440>,<val of days:1-365>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            /* If val of day is set, val of minutes should be 0 otherwise val of day will be ignored */
            uint8_t res;
            uint32_t days;
            uint32_t minutes;
            res = get_numerical_value(commandLine, &minutes, 10);
            if(res != OK || minutes > 1440)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &days, 10);
            if(res == OK)
            {
                if(days >365)
                {
                    result = RESULT_CODE_ERROR;
                    break;
                }
                if(minutes == 0)
                {
                    UserConfig1.GpioReportTime_day1 = (uint16_t)days;
                    UserConfig1.GpioReportTime_minutes1 = 0;
                    break;
                }
            }
            UserConfig1.GpioReportTime_day1 = 0;
            UserConfig1.GpioReportTime_minutes1 = (uint16_t)minutes;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandSTIMER(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+STIMER:%d,%d\r\n"
                    ,AtConfig.GpioReportTime_minutes,AtConfig.GpioReportTime_day);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+STIMER=<val of minutes: 1-1440>,<val of days:1-365>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            /* If val of day is set, val of minutes should be 0 otherwise val of day will be ignored. */
            uint8_t res;
            uint32_t days;
            uint32_t minutes;
            res = get_numerical_value(commandLine, &minutes, 10);
            if(res != OK || minutes > 1440)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &days, 10);
            if(res == OK)
            {
                if(days >365)
                {
                    result = RESULT_CODE_ERROR;
                    break;
                }
                if(minutes == 0)
                {
                    AtConfig.GpioReportTime_day = (uint16_t)days;
                    AtConfig.GpioReportTime_minutes = 0;
                    break;
                }
            }
            AtConfig.GpioReportTime_day = 0;
            AtConfig.GpioReportTime_minutes = (uint16_t)minutes;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandSIRQ(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+SIRQ:%d\r\n"
                    ,AtConfig.IrqTriggerType_0);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+SIRQ=<0-1>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t irq0;
            res = get_numerical_value(commandLine, &irq0, 10);
            if(res != OK || irq0 > 1)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            AtConfig.IrqTriggerType_0 = (uint8_t)irq0;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandSGPIO(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            AdcInit();
            sprintf((char*)sendBuff, "+SGPIO:%d,%d,%d,%d,%d,%d,%d\r\n"
                    ,ReadGpioStatus("PB6"),ReadGpioStatus("PB7"),ReadGpioStatus("PB8")
                    ,ReadGpioStatus("PA11"),ReadGpioStatus("PA12")
#if defined( STM32L073xx )
                    ,GetPinVoltage(ADC_CHANNEL_8),GetPinVoltage(ADC_CHANNEL_9));
#else
                    ,GetPinVoltage(ADC_Channel_8),GetPinVoltage(ADC_Channel_9));
#endif
            SendMsgToUart(sendBuff, UseUartID);
            //ReadGpioStatus wii deInit PB7
            InitATInterrupt("PB7",WakeUpLPMCallback);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+SGPIO=\"Display status of PINs:<PB6>,<PB7>,<PB8>,<PA11>,<PA12>,<PB0>,<PB1>\"\r\n", UseUartID);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandCSID(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            uint8_t id[3];
            memset(id,'\0',3);
            memcpy(id,OthersConfig.CMacAddr,2);
            sprintf((char*)sendBuff, "+CSID:\"%s\"\r\n",id);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandCPIN(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CPIN:%s\r\n",NodeConfig.PinCode);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCSQ(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    uint32_t channels[16];
    channels[0] = (NodeConfig.FreqA1- NodeConfig.Offset2);
    channels[1] = (NodeConfig.FreqA1 - NodeConfig.Offset1);
    channels[2] = (NodeConfig.FreqA1 + NodeConfig.Offset1);
    channels[3] = (NodeConfig.FreqA1 + NodeConfig.Offset2);
    channels[4] = (NodeConfig.FreqA2 - NodeConfig.Offset2);
    channels[5] = (NodeConfig.FreqA2 - NodeConfig.Offset1);
    channels[6] = (NodeConfig.FreqA2 + NodeConfig.Offset1);
    channels[7] = (NodeConfig.FreqA2 + NodeConfig.Offset2);
    channels[8] = (NodeConfig.FreqB1 - NodeConfig.Offset2);
    channels[9] = (NodeConfig.FreqB1 - NodeConfig.Offset1);
    channels[10] = (NodeConfig.FreqB1 + NodeConfig.Offset1);
    channels[11] = (NodeConfig.FreqB1 + NodeConfig.Offset2);
    channels[12] = (NodeConfig.FreqB2 - NodeConfig.Offset2);
    channels[13] = (NodeConfig.FreqB2 - NodeConfig.Offset1);
    channels[14] = (NodeConfig.FreqB2 + NodeConfig.Offset1);
    channels[15] = (NodeConfig.FreqB2 + NodeConfig.Offset2);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t msg[192] = "+CSQ:\r\n";
            uint8_t tmpStr[16] = "";
            int16_t rssi = 0;
            int i;
            for(i=0; i < 16; i++)
            {
                SX1276SetModem(MODEM_FSK);
                SX1276SetChannel(channels[i]);
                SX1276Write(REG_RSSICONFIG, ((2)<<3) | 0x02);  // offset 2 dB
                SX1276Write(REG_RXBW, FSK_READ_RSSI_BW);
                SX1276Write(REG_AFCBW, FSK_READ_RSSI_BW);
                SX1276SetOpMode(RF_OPMODE_RECEIVER);
                vTaskDelay(FSK_READ_RSSI_DELAY/portTICK_PERIOD_MS);
                rssi = SX1276ReadRssi(MODEM_FSK);
                SX1276SetSleep();
                sprintf((char*)tmpStr, "%d:%d\r\n",i,rssi);
                strcat((char*)msg,(char*)tmpStr);
            }
            SendMsgToUart(msg, UseUartID);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCSYNC(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CSYNC:%d\r\n",UserConfig1.SyncFlag);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CSYNC=<0-1>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t sync;
            res = get_numerical_value(commandLine, &sync, 10);
            if(res != OK || sync > 1)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            UserConfig1.SyncFlag = (uint8_t)sync;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCAPORT(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CAPORT:%d\r\n",UserConfig1.AppPort);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CAPORT=<1-223>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t port;
            res = get_numerical_value(commandLine, &port, 10);
            if(res != OK || port == 0 || port >223)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            UserConfig1.AppPort = (uint8_t)port;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCBAP(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CBAP:%d\r\n",UserConfig1.RecPort);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CBAP=<-1, 1-223>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            int32_t port;
            res = get_numerical_value(commandLine, (uint32_t*)&port, 10);
            if(res != OK || port == 0 || port >223 || port < -1)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            UserConfig1.RecPort= (int16_t)port;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCSF(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CSF:%d,%d\r\n",AtConfig.SF_TX,RxConfig.RxSF);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            /* Modify by Eric Date: 2017/03/15   Log: Modify SF range messages.  */
            SendMsgToUart("+CSF=<Tx 7-12>,<Rx 7-12>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res1,res2;
            uint32_t sft,sfr;
            uint8_t iSFR,iSFT;
            Rx2ChannelParams_t parameter;
            res1 = get_numerical_value(commandLine, &sft, 10);
            if(res1 !=OK || sFMappingTable((uint8_t)sft,&iSFT) != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res2 = get_numerical_value(commandLine, &sfr, 10);
            if(res2 !=OK || sFMappingTable((uint8_t)sfr,&iSFR) != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            AtConfig.SF_TX = (uint8_t)sft;
            RxConfig.RxSF= (uint8_t)sfr;
            SetRadioDatarate(iSFT);
            parameter.Datarate = iSFR;
            parameter.Frequency = RxConfig.RxFreq;
            SetRadioRxChannel(parameter);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCRPTM(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    if(NodeConfig.ReportModeEnable == 0)
    {
        return RESULT_CODE_ERROR;
    }
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CRPTM:%d\r\n",AtConfig.Enter_Reporter);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CRPTM=<0-1>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t mode;
            res = get_numerical_value(commandLine, &mode, 10);
            if(res != OK || mode > 1)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            AtConfig.Enter_Reporter = (uint8_t)mode;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandSPWMOD(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+SPWMOD:%d\r\n",OthersConfig.EnterLPMTime);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+SPWMOD=<0-255>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t time;

            res = get_numerical_value(commandLine, &time, 10);
            if(res != OK || time > 255)
            {
                result = RESULT_CODE_ERROR;
                break;
            }

            if(time == 0)
            {
                AtConfig.LowPowerModeFlag = 0;
                OthersConfig.EnterLPMTime = 0;
            }
            else
            {
                AtConfig.LowPowerModeFlag = 1;
                OthersConfig.EnterLPMTime = time;
            }
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandGADM(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+GADM:%d\r\n",AtConfig.EngineerModeFlag);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+GADM=<0-1>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t en;
            uint8_t passwd[PASSWORD_SIZE];
            res = get_numerical_value(commandLine, &en, 10);
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            if(en == 0 && AtConfig.EngineerModeFlag == 1)
            {
                AtConfig.EngineerModeFlag= (uint8_t)en;
                EngineerMode(AtConfig.EngineerModeFlag);
            }
            else if(en ==1)
            {
                res = get_value(commandLine, passwd, PASSWORD_SIZE);
                if(res != OK ||
                        strncmp((const char*)EngConfig1.New_ADM_PASSWORD, (const char*)passwd,PASSWORD_SIZE) != 0)
                    //strncmp((const char*)ADM_PASSWORD, (const char*)passwd,PASSWORD_SIZE) != 0)
                {
                    result = RESULT_CODE_ERROR;
                    break;
                }
                AtConfig.EngineerModeFlag= (uint8_t)en;
                EngineerMode(AtConfig.EngineerModeFlag);
            }
            else
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandGCPW(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+GCPW:\"%s\"\r\n",EngConfig1.New_ADM_PASSWORD);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+GCPW=\"Password length must be more than 7 and less than 16\"\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint8_t new_pw[PASSWORD_SIZE];
            //res1 = get_string_value(commandLine, macAddr, PASSWORD_SIZE);
            //if(res1 != OK || checkVauleFormate(macAddr, 16, CONFIG_MAC_SIZE-1) != OK)
            //{
            //    result = RESULT_CODE_ERROR;
            //    break;
            //}
            res = get_string_value(commandLine,new_pw,PASSWORD_SIZE);
            if(res != OK || strlen((const char*)new_pw) > 16 || strlen((const char*)new_pw) <8)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            memset(EngConfig1.New_ADM_PASSWORD,'\0',PASSWORD_SIZE);
            memcpy(EngConfig1.New_ADM_PASSWORD,new_pw,PASSWORD_SIZE);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandDTX(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_RANGE:
        {
            SendMsgToUart("+DTX=length,payload\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint32_t len=0;
            uint8_t res;

            /*Add by Gavin, Date:20170731, Log: According to PR-1003, no limitation for data length*/
            memset(RadioSendData.Data, 0, MAX_RADIO_TX_SIZE+1);

            if(RadioTxDelayFlag != 0)
            {
                result = RESULT_CODE_ERROR;
                break;
            }

            // Get length
            res = get_numerical_value(commandLine, &len, 10);
            if(res != OK || len ==0)
            {
                result = RESULT_CODE_ERROR;
                break;
            }

            res = get_string_value(commandLine, RadioSendData.Data, (uint16_t)MAX_RADIO_TX_SIZE+1);
            if(res == OK)
            {
                if(len != strlen((const char*)RadioSendData.Data))
                {
                    result = RESULT_CODE_ERROR;
                    break;
                }
            }
            else// data is not string, so we try get hex value
            {
                res = get_value(commandLine, RadioSendData.Data, (uint16_t)MAX_RADIO_TX_SIZE+1);
                if(res != OK || strlen((const char*)RadioSendData.Data) != len
                    || converterStringToAscii(RadioSendData.Data, RadioSendData.Data) != OK)
                {
                    result = RESULT_CODE_ERROR;
                    break;
                }
                len = len/2;
            }

            if(len>MAX_RADIO_TX_SIZE)
            {
                result = RESULT_CODE_ERROR;
                break;
            }

            RadioSendData.DataLen = (uint8_t)len;
            RadioSendData.AppPort = UserConfig1.AppPort;
            if(UserConfig1.SyncFlag == 0)
            {
                SendQueueToRadio(SendUnConfirmedDataUp, &RadioSendData);
            }
            else
            {
                RadioSendData.retries = 3;
                SendQueueToRadio(SendConfirmedDataUp, &RadioSendData);
            }
            RadioTxLED(1);
            result = RESULT_CODE_OK;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandDRX(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[16];
            int i;

            if(RadioRxData.Size == 0)
            {
                SendMsgToUart("+DRX:0,0\r\n", UseUartID);
                break;
            }
            /*Add by Gavin, Date:20170623 Log:Fix hang issue when rx data size too large*/
            sprintf((char*)sendBuff, "\r\n+DRX:%d,",RadioRxData.Size);
            SendToUartImmediately(sendBuff);
            memset(sendBuff,0,16);

            for(i=0; i< RadioRxData.Size; i++)
            {
                sprintf((char*)&sendBuff,"%02x",RadioRxData.RxBuffer[i]);
                SendToUartImmediately(sendBuff);
                memset(sendBuff,0,16);
            }
            SendToUartImmediately("\r\n");
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+DRX=<length of Rx data>,<value of Rx Data>\r\n", UseUartID);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandDRXI(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            if(RadioRxData.Size == 0)
                SendMsgToUart("+DRXI:0\r\n",UseUartID);
            else
                SendMsgToUart("+DRXI:1\r\n",UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+DRXI=<Status of PA8>\r\n",UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t value;
            Gpio_t pin;
            res = get_numerical_value(commandLine, &value, 10);
            if(res != OK || value != 0)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            if(RadioRxData.RxBuffer == NULL)
            {
                break;
            }
            else
            {
                vPortFree(RadioRxData.RxBuffer);
                RadioRxData.RxBuffer = NULL;
                RadioRxData.Size = 0;
            }
            GpioInit(&pin, UART1_CK, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
            GpioWrite(&pin,0);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandDTTX(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_RANGE:
        {
            if(AtConfig.EngineerModeFlag == 0)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            SendMsgToUart("AT+DTTX=<0,1>,<repeat time in second>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t value;
            if(AtConfig.EngineerModeFlag == 0)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &value, 10);
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            if(value == 0)
            {
                DeleteTimer(DTTXTimer);
                DTTXTimer = 0x00;
            }
            else
            {
                res = get_numerical_value(commandLine, &value, 10);
                if(res != OK)
                {
                    result = RESULT_CODE_ERROR;
                    break;
                }
                if(DTTXTimer == 0x00)
                {
                    DTTXTimer = AddTimer(DTTXTimer, value*1000, 1, dTTXCallback);
                }
            }
            /* Modify by Gavin Date: 2017/04/10
                Log: Fix bug of dttx send empty data when no data send before this function*/
            //break;
        }
        case OPERATION_ACTION:
        {
            if(RadioTxDelayFlag != 0)
            {
                result = RESULT_CODE_ERROR;
                break;
            }

            memset(RadioSendData.Data, 0, MAX_RADIO_TX_SIZE+1);
            memcpy(RadioSendData.Data, OthersConfig.CMacAddr, CONFIG_MAC_SIZE-1);
            RadioSendData.DataLen = CONFIG_MAC_SIZE -1;
            RadioSendData.AppPort = UserConfig1.AppPort;
            if(UserConfig1.SyncFlag == 0)
            {
                SendQueueToRadio(SendUnConfirmedDataUp, &RadioSendData);
            }
            else
            {
                RadioSendData.retries = 3;
                SendQueueToRadio(SendConfirmedDataUp, &RadioSendData);
            }
            RadioTxLED(1);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandECHO(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+ECHO:%d\r\n",AtConfig.EchoFlag);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+ECHO=<0-1>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t value;
            res = get_numerical_value(commandLine, &value, 10);
            if(res == OK && value < 2)
            {
                AtConfig.EchoFlag = (uint8_t)value;
                if(AtConfig.EchoFlag == 0)
                {
                    SendQueueToUartTask(UartDisableEcho, UseUartID, NULL);
                }
                else
                {
                    SendQueueToUartTask(UartEnableEcho, UseUartID, NULL);
                }
            }
            else
            {
                result = RESULT_CODE_ERROR;
            }
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

//daniel add on 2017.1.12 new command for senser profile
ResultCode_t commandSPROFILE(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY: //query
        {
            uint8_t sendBuff[120];//Jason replace 80 to 120 on 2018.11.16
            uint8_t cmdBuff[50];//Jason replace 80 to 50 on 2018.11.16
            uint8_t tmpStr[8];
            uint8_t i;
            uint8_t cmdLen;
					  //Jason add on 2018.11.16 - start
					  uint8_t cmdBuff2[50];
						uint8_t cmdBuff3[50];
					  uint8_t cmdLen2;
            uint8_t cmdLen3;
					  memset(cmdBuff2, 0x00, 50);
					  memset(cmdBuff3, 0x00, 50);
            memset(sendBuff, 0x00, 50);
            memset(cmdBuff, 0x00, 120);
					  //Jason add on 2018.11.16 - end
					  cmdLen = SensorConfig.modbusCMDLEN;
					  cmdLen2 = SensorConfig.modbusCMDLEN2;
					  cmdLen3 = SensorConfig.modbusCMDLEN2;
					  if( SensorConfig.sensorType > 4)
						{
							  //result = RESULT_CODE_ERROR;
							  sprintf((char*)sendBuff, "+SPROFILE:SET PRPFILE ERROR\r\n");
							  SendMsgToUart(sendBuff, UseUartID);
						} 
						else 
						{
								if (SensorConfig.sensorType >=1)
								{
										
										if(cmdLen > 16) //for avoid exception
										{
												cmdLen = 16;
										}
										for(i=0; i<cmdLen; i++) //daniel modify on 2017.9.25 for display cmd string
										{
												memset(tmpStr, 0x00, 8);
												if(i != (cmdLen-1))
														sprintf((char*)tmpStr, "%02X,", SensorConfig.modbusCMD[i]);//sprintf((char*)tmpStr, "%d,", SensorConfig.modbusCMD[i]);
												else
														sprintf((char*)tmpStr, "%02X", SensorConfig.modbusCMD[i]);//sprintf((char*)tmpStr, "%d", SensorConfig.modbusCMD[i]);
												strcat((char*)cmdBuff, (char*)tmpStr);
										}
								}
								if (SensorConfig.sensorType >=3 && cmdLen2 != 0)
								{
										if(cmdLen2 > 16) //for avoid exception
										{
												cmdLen2 = 16;
										}
										for(i=0; i<cmdLen2; i++) //daniel modify on 2017.9.25 for display cmd string
										{
												memset(tmpStr, 0x00, 8);
												if(i != (cmdLen2-1))
														sprintf((char*)tmpStr, "%02X,", SensorConfig.modbusCMD2[i]);//sprintf((char*)tmpStr, "%d,", SensorConfig.modbusCMD[i]);
												else
														sprintf((char*)tmpStr, "%02X", SensorConfig.modbusCMD2[i]);//sprintf((char*)tmpStr, "%d", SensorConfig.modbusCMD[i]);
												strcat((char*)cmdBuff2, (char*)tmpStr);
										}
								}
								if (SensorConfig.sensorType ==4 && cmdLen3 != 0)
								{
										if(cmdLen3 > 16) //for avoid exception
										{
												cmdLen3 = 16;
										}
										for(i=0; i<cmdLen3; i++) //daniel modify on 2017.9.25 for display cmd string
										{
												memset(tmpStr, 0x00, 8);
												if(i != (cmdLen3-1))
														sprintf((char*)tmpStr, "%02X,", SensorConfig.modbusCMD3[i]);//sprintf((char*)tmpStr, "%d,", SensorConfig.modbusCMD[i]);
												else
														sprintf((char*)tmpStr, "%02X", SensorConfig.modbusCMD3[i]);//sprintf((char*)tmpStr, "%d", SensorConfig.modbusCMD[i]);
												strcat((char*)cmdBuff3, (char*)tmpStr);
										}
								}
								/*sprintf((char*)sendBuff, "+SPROFILE:%d,%d,%d,%d,%d,%d,%d,%d,%d,<%s>\r\n", SensorConfig.sensorType,
										SensorConfig.uartBR,SensorConfig.uartDB,SensorConfig.uartPC,SensorConfig.uartSB,
										SensorConfig.replyLEN,SensorConfig.startBYTE,SensorConfig.readLEN,SensorConfig.modbusCMDLEN,cmdBuff);*/
								if (SensorConfig.sensorType == 0) 
								{
									  sprintf((char*)sendBuff, "+SPROFILE:%d,%d,%d,%d,%d\r\n",
											SensorConfig.sensorType,SensorConfig.uartBR,SensorConfig.uartDB,SensorConfig.uartPC,SensorConfig.uartSB);
								}
								else if (SensorConfig.sensorType < 3) 
								{
									  sprintf((char*)sendBuff, "+SPROFILE:%d,%d,%d,%d,%d,%d,%d,%d,%d,<%s>\r\n",
											SensorConfig.sensorType,SensorConfig.uartBR,SensorConfig.uartDB,SensorConfig.uartPC,SensorConfig.uartSB,
											SensorConfig.replyLEN,SensorConfig.startBYTE,SensorConfig.readLEN,SensorConfig.modbusCMDLEN,cmdBuff);
								}
								else if (SensorConfig.sensorType == 3) 
								{
									  sprintf((char*)sendBuff, "+SPROFILE:%d,%d,%d,%d,%d,%d,%d,%d,%d,<%s>,%d,%d,%d,%d,<%s>",
											SensorConfig.sensorType,SensorConfig.uartBR,SensorConfig.uartDB,SensorConfig.uartPC,SensorConfig.uartSB,
											SensorConfig.replyLEN,SensorConfig.startBYTE,SensorConfig.readLEN,SensorConfig.modbusCMDLEN,cmdBuff,
											SensorConfig.replyLEN2,SensorConfig.startBYTE2,SensorConfig.readLEN2,SensorConfig.modbusCMDLEN2,cmdBuff2);
								}
								else if (SensorConfig.sensorType == 4) 
								{
									  sprintf((char*)sendBuff, "+SPROFILE:%d,%d,%d,%d,%d,%d,%d,%d,%d,<%s>,%d,%d,%d,%d,<%s>,%d,%d,%d,%d,<%s>\r\n",
											SensorConfig.sensorType,SensorConfig.uartBR,SensorConfig.uartDB,SensorConfig.uartPC,SensorConfig.uartSB,
											SensorConfig.replyLEN,SensorConfig.startBYTE,SensorConfig.readLEN,SensorConfig.modbusCMDLEN,cmdBuff,
											SensorConfig.replyLEN2,SensorConfig.startBYTE2,SensorConfig.readLEN2,SensorConfig.modbusCMDLEN2,cmdBuff2,
											SensorConfig.replyLEN3,SensorConfig.startBYTE3,SensorConfig.readLEN3,SensorConfig.modbusCMDLEN3,cmdBuff3);
								}
								SendMsgToUart(sendBuff, UseUartID);
						}
						break;
        }
        case OPERATION_RANGE:  //set range
        {
            //Jason modify on 2018.11.16
            // SendMsgToUart("+SPROFILE=<0-2>,<0-7>,<0-1>,<0-2>,<0-3>,<0-64>,<0-64>,<0-8>,<0-16>,<cmd code>\r\n", UseUartID);
            SendMsgToUart("+SPROFILE=<0-4>,<0-7>,<0-1>,<0-2>,<0-3>,<0-64>,<0-64>,<0-8>,<0-16>,<cmd code>,<0-64>,<0-64>,<0-8>,<0-16>,<cmd code>,<0-64>,<0-64>,<0-8>,<0-16>,<cmd code>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:  //set settings
        {
            uint8_t res;
            res = get_sensor_value(commandLine, &SensorConfig); //note on 2017.1.20 write a new function for sensor config
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
            }
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

//add on 2018.6.7 new command for test power
ResultCode_t commandTESTPOWER(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+TESTPOWER:%d\r\n",At2Config.PowerFlag);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+TESTPOWER=<0-1>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t value;
            res = get_numerical_value(commandLine, &value, 10);
            if(res == OK && value < 2)
            {
                At2Config.PowerFlag = (uint8_t)value;
                if(At2Config.PowerFlag == 0)
                {
                    PWR5V(0);
                }
                else
                {
                    PWR5V(1);
                }
            }
            else
            {
                result = RESULT_CODE_ERROR;
            }
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

//add on 2018.6.11 new command for test battery
ResultCode_t commandTESTBAT(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            //get BAT status
            uint16_t Adc1;
					  uint16_t Adc2;
            uint8_t sendBuff[80];
            BatDetect(1); //set PA_8 to high 
            delay_ms(2000);
            Adc1 = calADC(ADC_CHANNEL_9, 5);
					  Adc2 = calADC(ADC_CHANNEL_8, 5);
            BatDetect(0);
            sprintf((char*)sendBuff, "+TESTBAT BAT value:%d ADC value:%d\r\n",Adc1,Adc2);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        case OPERATION_ASSIGN:
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

//add on 2018.6.7 new command for test power
ResultCode_t commandTESTPINC(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        case OPERATION_RANGE:
        {
            SendMsgToUart("+TESTPINC=<1-12>,<0-1>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t value1;
					  uint32_t value2;
            res = get_numerical_value(commandLine, &value1, 10);
					  if(res != OK || value1 > 12)
            {
                result = RESULT_CODE_ERROR;
                break;
            }

						res = get_numerical_value(commandLine, &value2, 12);
					  if(res != OK || value2 > 1)
					  {
                result = RESULT_CODE_ERROR;
                break;
            }
						PinTest(value1, value2);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

//Jason add on 2019.1.28 new command for up feed back pin setting
ResultCode_t commandUPBACKPINC(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
				{
					  if(AtConfig.UP_FEEDBACK < 1 || AtConfig.UP_FEEDBACK > 12)  
            {
							SendMsgToUart("+UPBACKPINC:NO\r\n", UseUartID);
						}
						else
						{
							uint8_t sendBuff[40];
							sprintf((char*)sendBuff, "+UPBACKPINC:%d\r\n",AtConfig.UP_FEEDBACK);
							SendMsgToUart(sendBuff, UseUartID);
						}
            
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+TESTPINC=<1-12>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t value;
            res = get_numerical_value(commandLine, &value, 10);
					  if(res != OK || value > 12 || value < 1)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            AtConfig.UP_FEEDBACK = (uint32_t)value;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

//Jason add on 2019.1.28 new command for up feed back pin setting
ResultCode_t commandDOWNBACKPINC(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
				{
					  if(AtConfig.UP_FEEDBACK < 1 || AtConfig.DOWN_FEEDBACK > 12)  
            {
							SendMsgToUart("+DOWNBACKPINC:NO\r\n", UseUartID);
						}
						else
						{
							uint8_t sendBuff[40];
							sprintf((char*)sendBuff, "+DOWNBACKPINC:%d\r\n",AtConfig.DOWN_FEEDBACK);
							SendMsgToUart(sendBuff, UseUartID);
						}
            
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+TESTPINC=<1-12>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t value;
            res = get_numerical_value(commandLine, &value, 10);
					  if(res != OK || value > 12 || value < 1)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            AtConfig.DOWN_FEEDBACK = (uint32_t)value;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

//Jason add on 2019.01.29 new command for switch sensor hub and control box
ResultCode_t commandSDEVICETYPE(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
				{
					  if(AtConfig.DEVICE_TYPE > 3)  
            {
							uint8_t sendBuff[40];
							sprintf((char*)sendBuff, "+SDEVICETYPE:%d\r\n",1);
							SendMsgToUart(sendBuff, UseUartID);
						}
						else
						{
							uint8_t sendBuff[40];
							sprintf((char*)sendBuff, "+SDEVICETYPE:%d\r\n",AtConfig.DEVICE_TYPE);
							SendMsgToUart(sendBuff, UseUartID);
						}
            
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+SDEVICETYPE=<0-3>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t value;
            res = get_numerical_value(commandLine, &value, 10);
					  if(res != OK || value > 3 )
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            AtConfig.DEVICE_TYPE = (uint32_t)value;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

#ifdef ANT_PA
ResultCode_t commandCPAE(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CPAE:%d\r\n",OthersConfig.PAEnable);
            SendMsgToUart(sendBuff,UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CPAE=<0-1>\r\n",UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t value;
            //Check command length
            if(commandLine->length!=9)
            {
                result = RESULT_CODE_ERROR;
                return result;
            }
            res = get_numerical_value(commandLine,(uint32_t*) &value, 10);
            if(res == OK && value<2)
            {
                OthersConfig.PAEnable = value;
                if(OthersConfig.PAEnable == 0)
                {
//                    BoardSetPAEnable(0);
                }
                else
                {
//                    BoardSetPAEnable(1);
                }
            }
            else
            {
                result = RESULT_CODE_ERROR;
            }
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}
#endif

ResultCode_t commandCTXP(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CTXP:%d\r\n",AtConfig.Power_TX);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CTXP=<2-%d>\r\n",NodeConfig.Power_TX_MAX);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res,value;
            res = get_numerical_value(commandLine, (uint32_t*)&value, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            if(res == OK && value >= 2 && value <= NodeConfig.Power_TX_MAX)
            {
                AtConfig.Power_TX = value;
            }
            else
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            SetRadioDefaultPower(value);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandGMTXP(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+GMTXP:%d\r\n",NodeConfig.Power_TX_MAX);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+GMTXP=Max Tx Power\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res,value;
            res = get_numerical_value(commandLine, (uint32_t*)&value, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            if(res == OK && value>=2 && value <= 20)
            {
                NodeConfig.Power_TX_MAX= value;
                if(AtConfig.Power_TX > NodeConfig.Power_TX_MAX)
                {
                    AtConfig.Power_TX = NodeConfig.Power_TX_MAX;
                }
            }
            else
            {
                result = RESULT_CODE_ERROR;
            }
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandGSYSC(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+GSYSC:%d,%d\r\n",NodeConfig.LEDEnable,NodeConfig.ReportModeEnable);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+GSYSC=<LED flag 0-1>,<Report Mode Enable>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t flag;
            uint32_t reportEnable;
            res = get_numerical_value(commandLine, &flag, 10);
            if(res !=OK || flag > 1)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &reportEnable, 10);
            if(res !=OK || reportEnable > 1)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            NodeConfig.LEDEnable = (uint8_t)flag;
            NodeConfig.ReportModeEnable = (uint8_t)reportEnable;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCKEY(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            /* Modify by Gavin Date: 2017/04/10 Log: Hidden characters for keys according to PR-1016*/
            uint8_t sendBuff[64];
            uint8_t nwkSKey[16];
            uint8_t appSKey[16];

            TransferKeyFromStrToNum(nwkSKey, NodeConfig.NwkSessionKey ,16);
            TransferKeyFromStrToNum(appSKey, NodeConfig.AppSessionKey ,16);

            sprintf((char*)sendBuff, "+CKEY:%02X,%02X\r\n"
                ,crcStd16(nwkSKey,16),crcStd16(appSKey,16));
            SendMsgToUart(sendBuff,UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CKEY=<NetworkKey:length is 32>,<ApplicationKey: length is 32>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint8_t key1[CONFIG_NWKSKEY_SIZE],key2[CONFIG_NWKSKEY_SIZE];
            res = get_value(commandLine, key1,CONFIG_NWKSKEY_SIZE);
            if(res != OK
                    || checkVauleFormate(key1, 16, CONFIG_NWKSKEY_SIZE-1) == FAIL)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_value(commandLine, key2,CONFIG_APPSKEY_SIZE);
            if((res != OK
                    && commandLine->character[commandLine->position -1] !=CHAR_COMMA)
                    || checkVauleFormate(key2, 16,CONFIG_APPSKEY_SIZE-1) == FAIL)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            if(strlen((const char*)key1) != 0)
            {
                memset(NodeConfig.NwkSessionKey,'\0',CONFIG_NWKSKEY_SIZE);
                memcpy(NodeConfig.NwkSessionKey,key1,CONFIG_NWKSKEY_SIZE);
            }
            if(strlen((const char*)key2) != 0)
            {
                memset(NodeConfig.AppSessionKey,'\0',CONFIG_NWKSKEY_SIZE);
                memcpy(NodeConfig.AppSessionKey,key2,CONFIG_NWKSKEY_SIZE);
            }
            updateLoRaMacInitNwkIds();
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandGKEY(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[64];

            uint8_t nwkSKey[16];
            uint8_t appSKey[16];

            TransferKeyFromStrToNum(nwkSKey, NodeConfig.NwkSessionKey ,16);
            TransferKeyFromStrToNum(appSKey, NodeConfig.AppSessionKey ,16);

            sprintf((char*)sendBuff, "+GKEY:%02X,%02X\r\n"
                ,crcStd16(nwkSKey ,16),crcStd16(appSKey,16));
            SendMsgToUart(sendBuff,UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+GKEY=<NetworkKey:length is 32>,<ApplicationKey: length is 32>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint8_t key1[CONFIG_NWKSKEY_SIZE],key2[CONFIG_NWKSKEY_SIZE];
            res = get_value(commandLine, key1,CONFIG_NWKSKEY_SIZE);
            if(res != OK
                    || checkVauleFormate(key1, 16, CONFIG_NWKSKEY_SIZE-1) == FAIL)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_value(commandLine, key2,CONFIG_APPSKEY_SIZE);
            if((res != OK
                    && commandLine->character[commandLine->position -1] !=CHAR_COMMA)
                    || checkVauleFormate(key2, 16,CONFIG_APPSKEY_SIZE-1) == FAIL)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            if(strlen((const char*)key1) != 0)
            {
                memset(NodeConfig.NwkSessionKey,'\0',CONFIG_NWKSKEY_SIZE);
                memcpy(NodeConfig.NwkSessionKey,key1,CONFIG_NWKSKEY_SIZE);

                memset(LoRaWanConfig.NwkSKey,'\0',CONFIG_NWKSKEY_SIZE);
                memcpy(LoRaWanConfig.NwkSKey,key1,CONFIG_NWKSKEY_SIZE);
            }
            if(strlen((const char*)key2) != 0)
            {
                memset(NodeConfig.AppSessionKey,'\0',CONFIG_NWKSKEY_SIZE);
                memcpy(NodeConfig.AppSessionKey,key2,CONFIG_NWKSKEY_SIZE);

                memset(LoRaWanConfig.AppSKey,'\0',CONFIG_NWKSKEY_SIZE);
                memcpy(LoRaWanConfig.AppSKey,key2,CONFIG_NWKSKEY_SIZE);
            }
            updateLoRaMacInitNwkIds();
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandGPIN(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+GPIN:%s\r\n",NodeConfig.PinCode);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+GPIN=<Pin code length is 4 with decimal>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint8_t pin[CONFIG_PINCODE_SIZE];
            res = get_value(commandLine, pin,CONFIG_PINCODE_SIZE);
            if(res != OK
                    || checkVauleFormate(pin, 10, CONFIG_PINCODE_SIZE-1) == FAIL)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            memset(NodeConfig.PinCode,'\0',CONFIG_PINCODE_SIZE);
            memcpy(NodeConfig.PinCode,pin,CONFIG_PINCODE_SIZE);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandGLMR(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    uint8_t res;
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+GLMR:\"%s\"\r\n",NodeConfig.HWVersion);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+GLMR=\"HW Version\"\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t hWVersion[CONFIG_HW_VER_SIZE];
            res = get_string_value(commandLine, hWVersion, CONFIG_HW_VER_SIZE);
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            memset(NodeConfig.HWVersion,'\0',CONFIG_HW_VER_SIZE);
            memcpy(NodeConfig.HWVersion,hWVersion,CONFIG_HW_VER_SIZE);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandGTXD(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+GTXD:%d (second)\r\n",NodeConfig.RadioTxDelayTime / 1000);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+GTXD= Radio Tx Delay Time (second)\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t time;
            res = get_numerical_value(commandLine, &time, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            if(time > RADIO_TX_DELAY_MAX || time < RADIO_TX_DELAY_MIN)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            NodeConfig.RadioTxDelayTime = time * 1000;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCCH(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    uint8_t res;
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CCH:%d,%d,%d,%d\r\n"
                    ,NodeConfig.FreqA1,NodeConfig.FreqA2,NodeConfig.FreqB1,NodeConfig.FreqB2);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CCH=<Freq A1>,<Freq A2>,<Freq B1>,<Freq B2>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint32_t a1,a2,b1,b2,tmp;
            uint8_t index;
            Rx2ChannelParams_t parameter;
            res = get_numerical_value(commandLine, &a1, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &a2, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &b1, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &b2, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &tmp, 10);
            if(res ==OK)
            {
                // prevent user enter more than 4 channel
                result = RESULT_CODE_ERROR;
                break;
            }
            NodeConfig.FreqA1 = a1;
            NodeConfig.FreqA2 = a2;
            NodeConfig.FreqB1 = b1;
            NodeConfig.FreqB2 = b2;
            SetRadioChannelOnOffset(NodeConfig.FreqA1,NodeConfig.FreqA2
                                    ,NodeConfig.FreqB1,NodeConfig.FreqB2
                                    ,NodeConfig.Offset1,NodeConfig.Offset2);
            /*Add by Gavin, Date:20170606, Log:G78S use default channel*/
#ifndef SIPMODULE_G78S
            RxConfig.RxFreq = b2;
            sFMappingTable(RxConfig.RxSF, &index);
            parameter.Datarate = index;
            parameter.Frequency = RxConfig.RxFreq;
            SetRadioRxChannel(parameter);
#endif
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandGCH(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    uint8_t res;
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+GCH:%d,%d,%d,%d\r\n"
                    ,NodeConfig.FreqA1,NodeConfig.FreqA2,NodeConfig.FreqB1,NodeConfig.FreqB2);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+GCH=<Freq A1>,<Freq A2>,<Freq B1>,<Freq B2>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint32_t a1,a2,b1,b2,tmp;
            uint8_t index;
            Rx2ChannelParams_t parameter;
            res = get_numerical_value(commandLine, &a1, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &a2, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &b1, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &b2, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &tmp, 10);
            if(res ==OK)
            {
                // prevent user enter more than 4 channel
                result = RESULT_CODE_ERROR;
                break;
            }
            NodeConfig.FreqA1 = a1;
            NodeConfig.FreqA2 = a2;
            NodeConfig.FreqB1 = b1;
            NodeConfig.FreqB2 = b2;
            SetRadioChannelOnOffset(NodeConfig.FreqA1,NodeConfig.FreqA2
                                    ,NodeConfig.FreqB1,NodeConfig.FreqB2
                                    ,NodeConfig.Offset1,NodeConfig.Offset2);
#ifndef SIPMODULE_G78S
            RxConfig.RxFreq = b2;
            sFMappingTable(RxConfig.RxSF, &index);
            parameter.Datarate = index;
            parameter.Frequency = RxConfig.RxFreq;
            SetRadioRxChannel(parameter);
#endif
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCCHO(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    uint8_t res;
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CCHO:%d,%d\r\n"
                    ,NodeConfig.Offset1,NodeConfig.Offset2);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CCHO=<Freq Offset 1>,<Freq Offset 2>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint32_t o1,o2,tmp;
            res = get_numerical_value(commandLine, &o1, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &o2, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &tmp, 10);
            if(res ==OK)
            {
                // prevent user enter more than 2 channel offset
                result = RESULT_CODE_ERROR;
                break;
            }
            NodeConfig.Offset1= o1;
            NodeConfig.Offset2 = o2;
            SetRadioChannelOnOffset(NodeConfig.FreqA1,NodeConfig.FreqA2
                                    ,NodeConfig.FreqB1,NodeConfig.FreqB2
                                    ,NodeConfig.Offset1,NodeConfig.Offset2);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandGCHO(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    uint8_t res;
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+GCHO:%d,%d\r\n"
                    ,NodeConfig.Offset1,NodeConfig.Offset2);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+GCHO=<Freq Offset 1>,<Freq Offset 2>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint32_t o1,o2,tmp;
            res = get_numerical_value(commandLine, &o1, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &o2, 10);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &tmp, 10);
            if(res ==OK)
            {
                // prevent user enter more than 2 channel offset
                result = RESULT_CODE_ERROR;
                break;
            }
            NodeConfig.Offset1= o1;
            NodeConfig.Offset2 = o2;
            SetRadioChannelOnOffset(NodeConfig.FreqA1,NodeConfig.FreqA2
                                    ,NodeConfig.FreqB1,NodeConfig.FreqB2
                                    ,NodeConfig.Offset1,NodeConfig.Offset2);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCMAC(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CMAC:\"%s\"\r\n",OthersConfig.CMacAddr);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CMAC=\"MAC:length is 8\"\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res1;
            uint8_t macAddr[CONFIG_MAC_SIZE];
            res1 = get_string_value(commandLine, macAddr, CONFIG_MAC_SIZE);
            if(res1 != OK || checkVauleFormate(macAddr, 16, CONFIG_MAC_SIZE-1) != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            memset(OthersConfig.CMacAddr,'\0',CONFIG_MAC_SIZE);
            memcpy(OthersConfig.CMacAddr,macAddr,CONFIG_MAC_SIZE);
            updateLoRaMacInitNwkIds();
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandCAPPEUI(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CAPPEUI:%s\r\n",LoRaWanConfig.AppEUI);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CAPPEUI=<AppEUI:length is 16>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            /* Modify by Eric Date: 2017/02/21   Log: Add engineer mode flag. */
            if(AtConfig.EngineerModeFlag == 1)
            {
                uint8_t res;
                uint8_t appEUI[CONFIG_APPEUI_SIZE];
                res = get_value(commandLine, appEUI,CONFIG_APPEUI_SIZE);
                if(res != OK
                        || checkVauleFormate(appEUI, 16, CONFIG_APPEUI_SIZE-1) == FAIL)
                {
                    result = RESULT_CODE_ERROR;
                    break;
                }
                memcpy(LoRaWanConfig.AppEUI,appEUI,CONFIG_APPEUI_SIZE);
            }
            else
                result = RESULT_CODE_ERROR;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCDEVEUI(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CDEVEUI:%s\r\n",LoRaWanConfig.DevEUI);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CDEVEUI=<DevEUI:length is 16>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            /* Modify by Eric Date: 2017/02/21   Log: Add engineer mode flag. */
            if(AtConfig.EngineerModeFlag ==1)
            {
                uint8_t res;
                uint8_t DevEUI[CONFIG_DEVEUI_SIZE];
                res = get_value(commandLine, DevEUI,CONFIG_DEVEUI_SIZE);
                if(res != OK
                        || checkVauleFormate(DevEUI, 16, CONFIG_DEVEUI_SIZE-1) == FAIL)
                {
                    result = RESULT_CODE_ERROR;
                    break;
                }
                memcpy(LoRaWanConfig.DevEUI,DevEUI,CONFIG_DEVEUI_SIZE);
            }
            else
                result = RESULT_CODE_ERROR;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCAPPKEY(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CAPPKEY:%s\r\n",LoRaWanConfig.AppKey);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CAPPKEY=<AppKey:length is 32>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint8_t AppKey[CONFIG_APPKEY_SIZE];
            res = get_value(commandLine, AppKey,CONFIG_APPKEY_SIZE);
            if(res != OK
                    || checkVauleFormate(AppKey, 16, CONFIG_APPKEY_SIZE-1) == FAIL)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            memcpy(LoRaWanConfig.AppKey,AppKey,CONFIG_APPKEY_SIZE);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCAPPSKEY(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            /* Modify by Gavin Date: 2017/04/13  Log: Hidden characters for keys according to PR-1016 */
            uint8_t sendBuff[64];
            uint8_t appSKey[16];

            TransferKeyFromStrToNum(appSKey, LoRaWanConfig.AppSKey ,16);
            sprintf((char*)sendBuff, "+CAPPSKEY:%02X\r\n", crcStd16(appSKey,16));
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CAPPSKEY=<App session key:length is 32>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint8_t appSKey[CONFIG_APPSKEY_SIZE];
            res = get_value(commandLine, appSKey,CONFIG_APPSKEY_SIZE);
            if(res != OK
                    || checkVauleFormate(appSKey, 16, CONFIG_APPSKEY_SIZE-1) == FAIL)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            memcpy(LoRaWanConfig.AppSKey,appSKey,CONFIG_APPSKEY_SIZE);
            memcpy(NodeConfig.AppSessionKey,appSKey,CONFIG_NWKSKEY_SIZE);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCNWKSKEY(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            /* Modify by Gavin Date: 2017/04/13  Log: Hidden characters for keys according to PR-1016 */
            uint8_t sendBuff[64];
            uint8_t nwkSKey[16];

            TransferKeyFromStrToNum(nwkSKey, LoRaWanConfig.NwkSKey ,16);
            sprintf((char*)sendBuff, "+CNWKSKEY:%02X\r\n",crcStd16(nwkSKey,16));
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CNWKSKEY=<Network session key:length is 32>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint8_t nwSKey[CONFIG_NWKSKEY_SIZE];
            res = get_value(commandLine, nwSKey,CONFIG_NWKSKEY_SIZE);
            if(res != OK || checkVauleFormate(nwSKey, 16, CONFIG_NWKSKEY_SIZE-1) == FAIL)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            memcpy(LoRaWanConfig.NwkSKey,nwSKey,CONFIG_NWKSKEY_SIZE);
            memcpy(NodeConfig.NwkSessionKey,nwSKey,CONFIG_NWKSKEY_SIZE);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandGGMD(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+GGMD:\"%s\",\"%s\"\r\n",NodeConfig.MacAddr,NodeConfig.Sn);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+GGMD=\"MAC:length is 8\",\"SN:length is 13\"\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res1,res2;
            uint8_t sn[CONFIG_SN_SIZE];
            uint8_t macAddr[CONFIG_MAC_SIZE];
            res1 = get_string_value(commandLine, macAddr, CONFIG_MAC_SIZE);
            if(res1 != OK || checkVauleFormate(macAddr, 16, CONFIG_MAC_SIZE-1) != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res2 = get_string_value(commandLine,sn,CONFIG_SN_SIZE);
            if(res2 != OK || strlen((const char*)sn) != 13)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            memset(NodeConfig.MacAddr,'\0',CONFIG_MAC_SIZE);
            memcpy(NodeConfig.MacAddr,macAddr,CONFIG_MAC_SIZE);
            memset(OthersConfig.CMacAddr,'\0',CONFIG_MAC_SIZE);
            memcpy(OthersConfig.CMacAddr,macAddr,CONFIG_MAC_SIZE);
            memset(NodeConfig.Sn,'\0',CONFIG_SN_SIZE);
            memcpy(NodeConfig.Sn,sn,CONFIG_SN_SIZE);

            memset(LoRaWanConfig.DevAddr,'\0',CONFIG_MAC_SIZE);
            memcpy(LoRaWanConfig.DevAddr,macAddr,CONFIG_MAC_SIZE);
            updateLoRaMacInitNwkIds();
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandGPT(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_ERROR;
    OperationTag_t  operation = get_operation_tag(commandLine);
    uint8_t res;
    switch(operation)
    {
        case OPERATION_ASSIGN:
        {
            uint8_t productTestStr[128];
            NodeTest testCmd;
            res = get_string_value(commandLine, productTestStr, 128);
            if(res != OK)
            {
                break;
            }
            if(strcmp("QUIT",(const char*)productTestStr) ==0)
            {
                if(TXCMRunningFlag == 1)
                {
                    SendMsgToUart("QUIT: Stop TXCM\r\n", UseUartID);
                    SendQueueToRadio(TxContinuousModeStop, NULL);
                    TXCMRunningFlag = 0;
                    result = RESULT_CODE_OK;
                }
                if(TXPPRunningFlag == 1)
                {
                    SendMsgToUart("QUIT: Stop TXPP\r\n", UseUartID);
                    SendQueueToUartTask(UartRadioTestStop,(UartId_t)NULL, NULL);
                    SendQueueToRadio(PingPongStop, NULL);
                    TXPPRunningFlag = 0;
                    result = RESULT_CODE_OK;
                }
                break;
            }
            else if(strncmp("LPM",(const char*)productTestStr,3) ==0)
            {
                SendToUartImmediately("\r\nEnter Power Saving Mode!\r\n\r\n");
                vTaskDelay(1/portTICK_PERIOD_MS);// Wait 1 ms
                SendQueueToUartTask(UartDisable,UseUartID,NULL);
                result = RESULT_CODE_OK;
                break;
            }
            else if(strncmp("LED,0,0",(const char*)productTestStr,7) ==0)
            {
                RadioTxLED(0);
                result = RESULT_CODE_OK;
                break;
            }
            else if(strncmp("LED,0,1",(const char*)productTestStr,7) ==0)
            {
                RadioTxLED(1);
                result = RESULT_CODE_OK;
                break;
            }
            if(TXCMRunningFlag == 1 || TXPPRunningFlag == 1)
            {
                SendMsgToUart("TXCM or TXPP is running, please QUIT it first.\r\n", UseUartID);
                break;
            }
            productTestStr[strlen((const char*)productTestStr)] = (uint8_t)'\r';
            testCmd = GetProductionTestCommand(productTestStr);
            switch(testCmd)
            {
                case PROD_TEST_TXCM_START:
                {
                    RadioTxContinuousMode_t RadioPTModeData;
                    testCmd = GetTXCMData(productTestStr,&RadioPTModeData);
                    if(testCmd == PROD_TEST_TXCM_START)
                    {
                        SendMsgToUart("Start TXCM\r\n", UseUartID);
                        SendQueueToRadio(TxContinuousModeStart, (void *)&RadioPTModeData);
                        TXCMRunningFlag = 1;
                        result = RESULT_CODE_OK;
                        break;
                    }
                    break;
                }
                case PROD_TEST_TXPP_START:
                {
                    bool isMaster;
                    uint8_t PT_Label[5];
                    uint16_t PT_TxNumOrRxTimeIdx = 0;//this should use in uart_task.c
                    RadioTxContinuousMode_t RadioPTModeData;
                    testCmd = GetTXPPData(productTestStr, &RadioPTModeData
                                          , &isMaster, &PT_TxNumOrRxTimeIdx, PT_Label);
                    if(testCmd != PROD_TEST_TXPP_START)
                    {
                        break;
                    }
                    SetPingPongMaxTxNumOrRxTimeIdx(PT_TxNumOrRxTimeIdx);
                    SetTXPPLable(PT_Label);
                    if(isMaster == 0)
                    {
                        SendMsgToUart("Start Slave Ping Pong\r\n", UseUartID);
                        SendQueueToRadio(SlavePingPongStart, (void *)&RadioPTModeData);
                    }
                    else
                    {
                        SendMsgToUart("Start Master Ping Pong\r\n", UseUartID);
                        SendQueueToRadio(MasterPingPongStart, (void *)&RadioPTModeData);
                    }
                    TXPPRunningFlag = 1;
                    result = RESULT_CODE_OK;
                    break;
                }
            }
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandIoTest(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_RANGE:
        {
            SendMsgToUart("AT+IOTEST=Input_Pin_Name,Out_Pin_Name,Output_Value\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint8_t pinNameOut[5],pinNameIn[5];
            uint32_t pinOutValue;
            res = get_value(commandLine, pinNameIn, 5);
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_value(commandLine, pinNameOut, 5);
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine, &pinOutValue, 10);
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = GpioTest(pinNameIn, pinNameOut,(uint8_t) pinOutValue);
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandGRST(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_ACTION:
        {
            engineerModeRestoreDefault();
            SaveConfigToEEPROM();
            NVIC_SystemReset();
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandCRXC(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CRXC=<Freq>,<Bandwidth>\r\n0:125k, 1:250k, 2:500k\r\n", UseUartID);
            break;
        }
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CRXC:%d,%d\r\n",RxConfig.RxFreq,RxConfig.RxBW);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t freq,bw;
            uint8_t index;
            Rx2ChannelParams_t parameter;
            res = get_numerical_value(commandLine,&freq, 10);
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
//            res = get_numerical_value(commandLine,&sf, 10);
//            if(res != OK)
//            {
//                result = RESULT_CODE_ERROR;
//                break;
//            }
            res = get_numerical_value(commandLine,&bw, 10);
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            RxConfig.RxFreq = freq;
//            RxConfig.RxSF = sf;
            RxConfig.RxBW = bw;
            sFMappingTable(RxConfig.RxSF, &index);
            parameter.Datarate = index;
            parameter.Frequency = RxConfig.RxFreq;
            SetRadioRxChannel(parameter);
            SetRadioRxBandwidth(RxConfig.RxBW);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

ResultCode_t commandGRXC(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_RANGE:
        {
            SendMsgToUart("+GRXC=<Freq>,<Bandwidth>\r\n0:125k, 1:250k, 2:500k\r\n", UseUartID);
            break;
        }
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+GRXC:%d,%d\r\n",RxConfig.RxFreq,RxConfig.RxBW);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint32_t freq,bw;
            uint8_t index;
            Rx2ChannelParams_t parameter;
            res = get_numerical_value(commandLine,&freq, 10);
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            res = get_numerical_value(commandLine,&bw, 10);
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            RxConfig.RxFreq = freq;
            RxConfig.RxBW = bw;
            sFMappingTable(RxConfig.RxSF, &index);
            parameter.Datarate = index;
            parameter.Frequency = RxConfig.RxFreq;
            SetRadioRxChannel(parameter);
            SetRadioRxBandwidth(RxConfig.RxBW);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandCaliRSSI(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+CALRSSI:%d\r\n",NodeConfig.CaliRSSI);
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            int32_t calibration;
            res = get_numerical_value(commandLine,(uint32_t*)&calibration, 10);
            if(res != OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            NodeConfig.CaliRSSI = (int16_t)calibration;
            SX1276SetRSSICalHF(NodeConfig.CaliRSSI);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}


ResultCode_t commandGUUID(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            uint8_t sendBuff[80];
            sprintf((char*)sendBuff, "+GUUID:%08X%08X%08X\r\n",(*(uint32_t*)ID3),(*(uint32_t*)ID2),(*(uint32_t*)ID1));
            SendMsgToUart(sendBuff, UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            result = RESULT_CODE_ERROR;
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

/*Modify by Gavin, Date:20170517, Log:Change to customer command*/
ResultCode_t commandCCLASS(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        {
            if((DeviceClass_t) NodeConfig.ClassMode == CLASS_C)
                SendMsgToUart("+CCLASS:C\r\n", UseUartID);
            else if((DeviceClass_t) NodeConfig.ClassMode == CLASS_B)
                SendMsgToUart("+CCLASS:B\r\n", UseUartID);
            else
                SendMsgToUart("+CCLASS:A\r\n", UseUartID);
            break;
        }
        case OPERATION_RANGE:
        {
            SendMsgToUart("+CCLASS=<A,B,C>\r\n", UseUartID);
            break;
        }
        case OPERATION_ASSIGN:
        {
            uint8_t res;
            uint8_t OpClass[2];
            res = get_value(commandLine, OpClass, 2);
            if(res !=OK)
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            if('A' == OpClass[0])
            {
                NodeConfig.ClassMode = (uint8_t) CLASS_A;
            }
            //else if('B' == OpClass[0])
            //{
            //    NodeConfig.ClassMode = (uint8_t) CLASS_B;
            //}
            else if('C' == OpClass[0])
            {
                NodeConfig.ClassMode = (uint8_t) CLASS_C;
            }
            else
            {
                result = RESULT_CODE_ERROR;
                break;
            }
            SetClassMode((DeviceClass_t)NodeConfig.ClassMode);
            break;
        }
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

/*This function is sample*/
ResultCode_t commandDefault(CommandLine_t *commandLine)
{
    ResultCode_t result = RESULT_CODE_OK;
    OperationTag_t  operation = get_operation_tag(commandLine);
    switch(operation)
    {
        case OPERATION_QUERY:
        case OPERATION_RANGE:
        case OPERATION_ASSIGN:
        case OPERATION_ACTION:
            result = RESULT_CODE_OK;
            break;
        default:
            result = RESULT_CODE_ERROR;
            break;
    }
    return result;
}

/*************************
    Other Functions
**************************/

/* Modify by Gavin Date: 2017/04/10
    Log: 1. Optimize time of saving config to eeprom by using InternalEepromWriteBufferWord.
           2. Prevent using too much stack size(declare too much varables at one function).
*/
#define MAX_STRUCT_SIZE 256 // The real max structure size is NodeConfig_t 172 bytes
uint8_t EncryptConfigToEEPROM(uint32_t eepromAddr, uint8_t* config, uint16_t size)
{
    uint8_t encryptConfig[MAX_STRUCT_SIZE];
    /*Encrypt Config*/
    memset(encryptConfig,'\0',MAX_STRUCT_SIZE);
    LoRaMacPayloadEncrypt(config, size, (uint8_t*)&EEPROM_KEY, 0, 0, 0, encryptConfig);
    return InternalEepromWriteBufferWord(eepromAddr ,encryptConfig, size);
}

uint8_t SaveConfigToEEPROM(void)
{
    uint8_t res;
    res = EncryptConfigToEEPROM(EEPROM_NODE_CONFIG_ADDR,(uint8_t *)&NodeConfig,sizeof(NodeConfig_t));
    if(res != OK)
    {
        return FAIL;
    }
    res = EncryptConfigToEEPROM(EEPROM_AT_CONFIG_ADDR,(uint8_t *)&AtConfig,sizeof(AtConfig_t));
    if(res != OK)
    {
        return FAIL;
    }
    res = EncryptConfigToEEPROM(EEPROM_ENG_CONFIG1_ADDR,(uint8_t *)&EngConfig1,sizeof(EngConfig1_t));
    if(res != OK)
    {
        return FAIL;
    }
    res = EncryptConfigToEEPROM(EEPROM_USER_CONFIG1_ADDR,(uint8_t *)&UserConfig1,sizeof(UserConfig1_t));
    if(res != OK)
    {
        return FAIL;
    }
    res = EncryptConfigToEEPROM(EEPROM_RX_CONFIG_ADDR,(uint8_t *)&RxConfig,sizeof(RxConfig_t));
    if(res != OK)
    {
        return FAIL;
    }
    res = EncryptConfigToEEPROM(EEPROM_OTHERS_CONFIG_ADDR,(uint8_t *)&OthersConfig,sizeof(OthersConfig_t));
    if(res != OK)
    {
        return FAIL;
    }
    res = EncryptConfigToEEPROM(EEPROM_LORAWAN_CONFIG_ADDR,(uint8_t *)&LoRaWanConfig,sizeof(LoRaWanConfig_t));
    if(res != OK)
    {
        return FAIL;
    }
    //daniel add on 2017.12.25
    res = EncryptConfigToEEPROM(EEPROM_SENSOR_CONFIG_ADDR,(uint8_t *)&SensorConfig,sizeof(SensorConfig_t));
    if(res != OK)
    {
        return FAIL;
    }
    return OK;
}


/* Modify by Eric Date: 2017/02/17   Log: Modify Load Functions To Fix Load Configration from EEPROM issue. */
uint8_t LoadNodeConfigFromEEPROM(void)
{
    uint8_t decryptNodeConfig[sizeof(NodeConfig_t)];
    uint8_t decryptSensorConfig[sizeof(SensorConfig_t)]; //daniel merge on 2017.7.26 for get SensorConfig from EEPROM
    /*Decrypt NodeConfig*/
    memset(decryptNodeConfig,'\0',sizeof(NodeConfig_t));
    InternalEepromReadBytes(EEPROM_NODE_CONFIG_ADDR, decryptNodeConfig, sizeof(NodeConfig_t));
    LoRaMacPayloadDecrypt(decryptNodeConfig, sizeof(NodeConfig_t), (uint8_t*)&EEPROM_KEY, 0, 0, 0, (uint8_t*)&NodeConfig);
    if(memcmp((const char*)NodeConfig.ManufactureID,(const char*)SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE-1) != 0)
    {
        engineerModeRestoreDefault();
        SaveConfigToEEPROM();
        NVIC_SystemReset();
    }
    //daniel merge on 2017.7.26 for get SensorConfig from EEPROM
    //daniel - begin
    memset(decryptSensorConfig,'\0',sizeof(SensorConfig_t));
    InternalEepromReadBytes(EEPROM_SENSOR_CONFIG_ADDR, decryptSensorConfig, sizeof(SensorConfig_t));
    LoRaMacPayloadDecrypt(decryptSensorConfig, sizeof(SensorConfig_t), (uint8_t*)&EEPROM_KEY, 0, 0, 0, (uint8_t*)&SensorConfig);
    if(memcmp((const char*)SensorConfig.ManufactureID,(const char*)SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE-1) != 0)
    {
        memset(&SensorConfig,0x00,sizeof(SensorConfig_t));
        memcpy(SensorConfig.ManufactureID,(const char*)SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE-1);
        SaveConfigToEEPROM();
    }
    //daniel - end
    return OK;
}

uint8_t LoadAtConfigFromEEPROM(void)
{
    uint8_t decryptAtConfig[sizeof(AtConfig_t)];
    /*Decrypt AtConfig*/
    memset(decryptAtConfig,'\0',sizeof(AtConfig_t));
    InternalEepromReadBytes(EEPROM_AT_CONFIG_ADDR, decryptAtConfig, sizeof(AtConfig_t));
    LoRaMacPayloadDecrypt(decryptAtConfig, sizeof(AtConfig_t), (uint8_t*)&EEPROM_KEY, 0, 0, 0, (uint8_t*)&AtConfig);
    if(memcmp((const char*)AtConfig.ManufactureID,(const char*)SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE-1) != 0)
    {
        engineerModeRestoreDefault();
        SaveConfigToEEPROM();
        NVIC_SystemReset();
    }
    return OK;
}

uint8_t LoadEngConfigFromEEPROM(void)
{
    uint8_t decryptEngConfig1[sizeof(EngConfig1_t)];
    /*Decrypt EngConfig1*/
    memset(decryptEngConfig1,'\0',sizeof(EngConfig1_t));
    InternalEepromReadBytes(EEPROM_ENG_CONFIG1_ADDR, decryptEngConfig1, sizeof(EngConfig1_t));
    LoRaMacPayloadDecrypt(decryptEngConfig1, sizeof(EngConfig1_t), (uint8_t*)&EEPROM_KEY, 0, 0, 0, (uint8_t*)&EngConfig1);
    if(memcmp((const char*)EngConfig1.ManufactureID,(const char*)SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE-1) != 0)
    {
        memset(EngConfig1.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
        memcpy(EngConfig1.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);
        memset(EngConfig1.New_ADM_PASSWORD,'\0',PASSWORD_SIZE);
        memcpy(EngConfig1.New_ADM_PASSWORD,DEFAULT_ADM_PASSWORD,PASSWORD_SIZE);
        SaveConfigToEEPROM();
    }
    return OK;
}

uint8_t LoadUserConfigFromEEPROM(void)
{
    uint8_t decryptUserConfig1[sizeof(UserConfig1_t)];
    /*Decrypt UserConfig1*/
    memset(decryptUserConfig1,'\0',sizeof(UserConfig1_t));
    InternalEepromReadBytes(EEPROM_USER_CONFIG1_ADDR, decryptUserConfig1, sizeof(UserConfig1_t));
    LoRaMacPayloadDecrypt(decryptUserConfig1, sizeof(UserConfig1_t), (uint8_t*)&EEPROM_KEY, 0, 0, 0, (uint8_t*)&UserConfig1);
    if(memcmp((const char*)UserConfig1.ManufactureID,(const char*)SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE-1) != 0)
    {
        memset(UserConfig1.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
        memcpy(UserConfig1.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);
        UserConfig1.AppPort = DEFAULT_APP_PORT;
        UserConfig1.RecPort = DEFAULT_RECPORT;
        UserConfig1.SyncFlag = DEFAULT_SYNCFLAG;
        UserConfig1.GpioReportTime_day1 = DEFAULT_GPIO_REPORT_TIME_DAY_1;
        UserConfig1.GpioReportTime_minutes1 = DEFAULT_GPIO_REPORT_TIME_MINUTES_1;
        SaveConfigToEEPROM();
    }
    return OK;
}

uint8_t LoadRxConfigFromEEPROM(void)
{
    uint8_t decryptRxConfig[sizeof(RxConfig_t)];
    /*Decrypt RxConfig*/
    memset(decryptRxConfig,'\0',sizeof(RxConfig_t));
    InternalEepromReadBytes(EEPROM_RX_CONFIG_ADDR, decryptRxConfig, sizeof(RxConfig_t));
    LoRaMacPayloadDecrypt(decryptRxConfig, sizeof(RxConfig_t), (uint8_t*)&EEPROM_KEY, 0, 0, 0, (uint8_t*)&RxConfig);
    if(memcmp((const char*)RxConfig.ManufactureID,(const char*)SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE-1) != 0)
    {
        //Restore RxConfig
        memset(RxConfig.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
        memcpy(RxConfig.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);
#ifdef SIPMODULE_G78S
        RxConfig.RxFreq = DEFAULT_RX2_FREQ;
        RxConfig.RxSF= DEFAULT_RX2_SF;
        RxConfig.RxBW = DEFAULT_RX2_BANDWIDTH;
#else
        RxConfig.RxFreq = NodeConfig.FreqB2;
        RxConfig.RxSF= DEFAULT_RX_SF;
        RxConfig.RxBW = UNCONFIRMED_MSG_BW;
#endif
        SaveConfigToEEPROM();
    }
    return OK;
}

uint8_t LoadOthersConfigFromEEPROM(void)
{
    uint8_t decryptOthersConfig[sizeof(OthersConfig_t)];
    /*Decrypt OthersConfig*/
    memset(decryptOthersConfig,'\0',sizeof(OthersConfig_t));
    InternalEepromReadBytes(EEPROM_OTHERS_CONFIG_ADDR, decryptOthersConfig, sizeof(OthersConfig_t));
    LoRaMacPayloadDecrypt(decryptOthersConfig, sizeof(OthersConfig_t), (uint8_t*)&EEPROM_KEY, 0, 0, 0, (uint8_t*)&OthersConfig);
    if(memcmp((const char*)OthersConfig.ManufactureID,(const char*)SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE-1) != 0)
    {
        //Restore OthersConfig
        memset(OthersConfig.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
        memcpy(OthersConfig.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);
        OthersConfig.PAEnable = DEFAULT_PA_ENABLE;
        memset(OthersConfig.CMacAddr,'\0',CONFIG_MAC_SIZE);
        /* Modify by Eric Date: 2017/03/23   Log: Prevent load wrong value when upgrade from old fw. */
//        memcpy(OthersConfig.CMacAddr,DEFAULT_MAC_ADDR,CONFIG_MAC_SIZE);
        memcpy(OthersConfig.CMacAddr,NodeConfig.MacAddr,CONFIG_MAC_SIZE);
        if(AtConfig.LowPowerModeFlag == 0)
        {
            OthersConfig.EnterLPMTime = 0;
        }
        else
        {
            OthersConfig.EnterLPMTime = DEFAULT_ENTER_LPM_TIME;
        }
        OthersConfig.u8temp_3 = 0;
        OthersConfig.u8temp_4 = 0;
        OthersConfig.u8temp_5 = 0;
        OthersConfig.u16temp_0 = 0;
        OthersConfig.u16temp_1 = 0;
        OthersConfig.u16temp_2 = 0;
        OthersConfig.u16temp_3 = 0;
        OthersConfig.u16temp_4 = 0;
        OthersConfig.u16temp_5 = 0;
        OthersConfig.u32temp_0 = 0;
        OthersConfig.u32temp_1 = 0;
        OthersConfig.u32temp_2 = 0;
        OthersConfig.u32temp_3 = 0;
        OthersConfig.u32temp_4 = 0;
        OthersConfig.u32temp_5 = 0;
        SaveConfigToEEPROM();
    }
    return OK;
}

uint8_t LoadLoRaWanFromEEPROM(void)
{
    uint8_t decryptConfig[sizeof(LoRaWanConfig_t)];

    memset(decryptConfig,'\0',sizeof(LoRaWanConfig_t));
    InternalEepromReadBytes(EEPROM_LORAWAN_CONFIG_ADDR, decryptConfig, sizeof(LoRaWanConfig_t));
    LoRaMacPayloadDecrypt(decryptConfig, sizeof(LoRaWanConfig_t), (uint8_t*)&EEPROM_KEY, 0, 0, 0, (uint8_t*)&LoRaWanConfig);

    if(memcmp((const char*)LoRaWanConfig.ManufactureID,(const char*)SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE-1) != 0)
    {
        memset(LoRaWanConfig.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
        memcpy(LoRaWanConfig.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);
        memset(LoRaWanConfig.DevAddr,'\0',CONFIG_MAC_SIZE);
        memcpy(LoRaWanConfig.DevAddr,NodeConfig.MacAddr,CONFIG_MAC_SIZE);
        memset(LoRaWanConfig.AppSKey,'\0',CONFIG_APPSKEY_SIZE);
        memcpy(LoRaWanConfig.AppSKey,NodeConfig.AppSessionKey,CONFIG_APPSKEY_SIZE);
        memset(LoRaWanConfig.NwkSKey,'\0',CONFIG_NWKSKEY_SIZE);
        memcpy(LoRaWanConfig.NwkSKey,NodeConfig.NwkSessionKey,CONFIG_NWKSKEY_SIZE);

        memset(LoRaWanConfig.AppEUI,'\0',CONFIG_APPEUI_SIZE);
        memset(LoRaWanConfig.DevEUI,'\0',CONFIG_DEVEUI_SIZE);
        memset(LoRaWanConfig.AppKey,'\0',CONFIG_APPKEY_SIZE);
        memcpy(LoRaWanConfig.AppKey,DEFALUT_OTAA_APP_KEY,CONFIG_APPSKEY_SIZE);

        SaveConfigToEEPROM();
    }

    //Backup ABP config of NodeConfig to LoRaWanConfig
    if(memcmp((const char*)LoRaWanConfig.DevAddr,(const char*)NodeConfig.MacAddr,CONFIG_MAC_SIZE-1) != 0
        || memcmp((const char*)LoRaWanConfig.AppSKey,(const char*)NodeConfig.AppSessionKey,CONFIG_APPSKEY_SIZE-1) != 0
        || memcmp((const char*)LoRaWanConfig.NwkSKey,(const char*)NodeConfig.NwkSessionKey,CONFIG_NWKSKEY_SIZE-1) != 0)
    {
        memset(LoRaWanConfig.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
        memcpy(LoRaWanConfig.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);
        memset(LoRaWanConfig.DevAddr,'\0',CONFIG_MAC_SIZE);
        memcpy(LoRaWanConfig.DevAddr,NodeConfig.MacAddr,CONFIG_MAC_SIZE);
        memset(LoRaWanConfig.AppSKey,'\0',CONFIG_APPSKEY_SIZE);
        memcpy(LoRaWanConfig.AppSKey,NodeConfig.AppSessionKey,CONFIG_APPSKEY_SIZE);
        memset(LoRaWanConfig.NwkSKey,'\0',CONFIG_NWKSKEY_SIZE);
        memcpy(LoRaWanConfig.NwkSKey,NodeConfig.NwkSessionKey,CONFIG_NWKSKEY_SIZE);
        memcpy(LoRaWanConfig.AppKey,DEFALUT_OTAA_APP_KEY,CONFIG_APPSKEY_SIZE);
        SaveConfigToEEPROM();
    }
    return OK;
}


void RestoreUserConfigToDefault(void)
{
    //Restore NodeConfig
    AtConfig.IBR = DEFAULT_IBR;
    AtConfig.EchoFlag= DEFAULT_ECHO;
    AtConfig.EngineerModeFlag = DEFAULT_ENG_MODE_FLAG;
    AtConfig.LowPowerModeFlag = DEFAULT_LOW_POWER_MODE;
    AtConfig.SF_TX =DEFAULT_SF_TX;
    AtConfig.Power_TX = NodeConfig.Power_TX_MAX;
    AtConfig.Enter_Reporter = DEFAULT_ENTER_REPORTER;
    AtConfig.IrqTriggerType_0 = DEFAULT_IRQ_TRIGGER_TYPE_0;
    AtConfig.GpioReportTime_day = DEFAULT_GPIO_REPORT_TIME_DAY;
    AtConfig.GpioReportTime_minutes = DEFAULT_GPIO_REPORT_TIME_MINUTES;
	  AtConfig.DEVICE_TYPE = DEFAULT_DEVICE_TYPE;
    //Restore UserConfig1
    UserConfig1.AppPort = DEFAULT_APP_PORT;
    UserConfig1.RecPort = DEFAULT_RECPORT;
    UserConfig1.SyncFlag = DEFAULT_SYNCFLAG;
    UserConfig1.GpioReportTime_day1 = DEFAULT_GPIO_REPORT_TIME_DAY_1;
    UserConfig1.GpioReportTime_minutes1 = DEFAULT_GPIO_REPORT_TIME_MINUTES_1;

    //Shift commands
    //modify on 2017.8.31 for keep mac & key after reset to default
//    memset(NodeConfig.MacAddr,'\0',CONFIG_MAC_SIZE);
//    memset(NodeConfig.NwkSessionKey,'\0',CONFIG_NWKSKEY_SIZE); //daniel mark on 2017.12.25
//    memset(NodeConfig.AppSessionKey,'\0',CONFIG_APPSKEY_SIZE); //daniel mark on 2017.12.25
//    memcpy(NodeConfig.MacAddr,DEFAULT_MAC_ADDR,CONFIG_MAC_SIZE);
//    memcpy(NodeConfig.NwkSessionKey,DEFALUT_NETWORK_KEY,CONFIG_NWKSKEY_SIZE); //daniel mark on 2017.12.25
//    memcpy(NodeConfig.AppSessionKey,DEFALUT_APPLICATION_KEY,CONFIG_APPSKEY_SIZE); //daniel mark on 2017.12.25
    memset(&SensorConfig,0x00,sizeof(SensorConfig_t)); //daniel add on 2017.10.2 for clear sensor settings
    memcpy(SensorConfig.ManufactureID,(const char*)SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE-1); //daniel add on 2017.10.2 for clear sensor settings
    //
/*  NodeConfig.FreqA1 = MODULE_A_PLL_1;
    NodeConfig.FreqA2= MODULE_A_PLL_2;
    NodeConfig.FreqB1= MODULE_B_PLL_1;
    NodeConfig.FreqB2= MODULE_B_PLL_2;
    NodeConfig.Offset1 = DEFAULT_FREQ_OFFSET_1;
    NodeConfig.Offset2 = DEFAULT_FREQ_OFFSET_2;
    NodeConfig.ClassMode = (uint8_t)CLASS_A;
#ifdef SIPMODULE_G78S
    RxConfig.RxFreq = DEFAULT_RX2_FREQ;
    RxConfig.RxSF= DEFAULT_RX2_SF;
    RxConfig.RxBW = DEFAULT_RX2_BANDWIDTH;
#else
    RxConfig.RxFreq = NodeConfig.FreqB2;
    RxConfig.RxSF= DEFAULT_RX_SF;
    RxConfig.RxBW = UNCONFIRMED_MSG_BW;
#endif */
    //Restore OthersConfig
    OthersConfig.PAEnable = DEFAULT_PA_ENABLE;
    memset(OthersConfig.CMacAddr,'\0',CONFIG_MAC_SIZE);
    memcpy(OthersConfig.CMacAddr,NodeConfig.MacAddr,CONFIG_MAC_SIZE);
    if(AtConfig.LowPowerModeFlag == 0)
        OthersConfig.EnterLPMTime = 0;
    else
        OthersConfig.EnterLPMTime = DEFAULT_ENTER_LPM_TIME;
    OthersConfig.u8temp_3 = 0;
    OthersConfig.u8temp_4 = 0;
    OthersConfig.u8temp_5 = 0;
    OthersConfig.u16temp_0 = 0;
    OthersConfig.u16temp_1 = 0;
    OthersConfig.u16temp_2 = 0;
    OthersConfig.u16temp_3 = 0;
    OthersConfig.u16temp_4 = 0;
    OthersConfig.u16temp_5 = 0;
    OthersConfig.u32temp_0 = 0;
    OthersConfig.u32temp_1 = 0;
    OthersConfig.u32temp_2 = 0;
    OthersConfig.u32temp_3 = 0;
    OthersConfig.u32temp_4 = 0;
    OthersConfig.u32temp_5 = 0;

    memset(LoRaWanConfig.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
    memcpy(LoRaWanConfig.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);
    memset(LoRaWanConfig.DevAddr,'\0',CONFIG_MAC_SIZE);
    memcpy(LoRaWanConfig.DevAddr,NodeConfig.MacAddr,CONFIG_MAC_SIZE);
    memset(LoRaWanConfig.AppSKey,'\0',CONFIG_APPSKEY_SIZE);
    memcpy(LoRaWanConfig.AppSKey,NodeConfig.AppSessionKey,CONFIG_APPSKEY_SIZE);
    memset(LoRaWanConfig.NwkSKey,'\0',CONFIG_NWKSKEY_SIZE);
    memcpy(LoRaWanConfig.NwkSKey,NodeConfig.NwkSessionKey,CONFIG_NWKSKEY_SIZE);

    memset(LoRaWanConfig.AppKey,'\0',CONFIG_APPKEY_SIZE);
    memcpy(LoRaWanConfig.AppKey,DEFALUT_OTAA_APP_KEY,CONFIG_APPSKEY_SIZE);

}


void engineerModeRestoreDefault(void)
{
    //Restore NodeConfig
    memset(NodeConfig.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
    memset(NodeConfig.Sn,'\0',CONFIG_SN_SIZE);
    memset(NodeConfig.MacAddr,'\0',CONFIG_MAC_SIZE);
    memset(NodeConfig.SystemId,'\0',CONFIG_SYS_ID_SIZE);
    memset(NodeConfig.NwkSessionKey,'\0',CONFIG_NWKSKEY_SIZE);
    memset(NodeConfig.AppSessionKey,'\0',CONFIG_APPSKEY_SIZE);
    memset(NodeConfig.PinCode,'\0',CONFIG_PINCODE_SIZE);
    memset(NodeConfig.HWVersion,'\0',CONFIG_HW_VER_SIZE);
    memcpy(NodeConfig.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);
    memcpy(NodeConfig.Sn,DEFALUT_SN,CONFIG_SN_SIZE);
    memcpy(NodeConfig.MacAddr,DEFAULT_MAC_ADDR,CONFIG_MAC_SIZE);
    memcpy(NodeConfig.SystemId,DEFALUT_SYSTEM_ID,CONFIG_SYS_ID_SIZE);
    memcpy(NodeConfig.NwkSessionKey,DEFALUT_NETWORK_KEY,CONFIG_NWKSKEY_SIZE);
    memcpy(NodeConfig.AppSessionKey,DEFALUT_APPLICATION_KEY,CONFIG_APPSKEY_SIZE);
    memcpy(NodeConfig.PinCode,DEFAULT_PINCODE,CONFIG_PINCODE_SIZE);
    memcpy(NodeConfig.HWVersion,DEFAULT_HWVersion,CONFIG_HW_VER_SIZE);
    NodeConfig.RadioTxDelayTime = DEFAULT_RADIO_Delay_TIME;
    NodeConfig.CaliRSSI = DEFAULT_CALI_RSSI;
    NodeConfig.LEDEnable = DEFAULT_LED_FLAG;
    NodeConfig.ReportModeEnable = DEFAULT_REPORT_MODE_ENABLE;
    NodeConfig.Power_TX_MAX= DEFAULT_POWER_TX_MAX;
    NodeConfig.FreqA1 = MODULE_A_PLL_1;
    NodeConfig.FreqA2= MODULE_A_PLL_2;
    NodeConfig.FreqB1= MODULE_B_PLL_1;
    NodeConfig.FreqB2= MODULE_B_PLL_2;
    NodeConfig.Offset1 = DEFAULT_FREQ_OFFSET_1;
    NodeConfig.Offset2 = DEFAULT_FREQ_OFFSET_2;
//    NodeConfig.ClassMode = (uint8_t)CLASS_A;
//    NodeConfig.BW_RX = 0;
    //Restore EngConfig1
    memset(EngConfig1.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
    memcpy(EngConfig1.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);
    memset(EngConfig1.New_ADM_PASSWORD,'\0',PASSWORD_SIZE);
    memcpy(EngConfig1.New_ADM_PASSWORD,DEFAULT_ADM_PASSWORD,PASSWORD_SIZE);
    //Restore AtConfig
    memset(AtConfig.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
    memcpy(AtConfig.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);
    //Restore UserConfig
    memset(UserConfig1.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
    memcpy(UserConfig1.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);
    //Restore RxConfig
    memset(RxConfig.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
    memcpy(RxConfig.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);

    //Restore OthersConfig
    memset(OthersConfig.ManufactureID,'\0',MANUFACTUER_ID_SIZE);
    memcpy(OthersConfig.ManufactureID,SIPMODULE_MANUFACTUER_ID,MANUFACTUER_ID_SIZE);

    memset(LoRaWanConfig.AppEUI,'\0',CONFIG_APPEUI_SIZE);
    memset(LoRaWanConfig.DevEUI,'\0',CONFIG_DEVEUI_SIZE);

    RestoreUserConfigToDefault();
}


uint32_t baudRateMappingTable(uint8_t ibr)
{
    switch(ibr)
    {
        case 1:
            return 9600;
        case 2:
            return 19200;
        case 3:
            return 38400;
        case 4:
            return 57600;
        case 5:
            return 115200;
        case 0:
        default:
            return 9600;
    }
}

/*
uint8_t powerMappingTable(uint8_t value, uint8_t* index)
{
    switch(value)
    {
        case 20:
        {
            *index = 0;
            break;
        }
        case 14:
        {
            *index = 1;
            break;
        }
        case 11:
        {
            *index = 2;
            break;
        }
        case 8:
        {
            *index = 3;
            break;
        }
        case 5:
        {
            *index = 4;
            break;
        }
        case 2:
        {
            *index = 5;
            break;
        }
        default:
            return FAIL;
    }
    return OK;
}
*/

uint8_t sFMappingTable(uint8_t value, uint8_t* index)
{
/* Modify by Eric Date: 2017/03/15   Log: Remove unnecessary flags.  */
    if(value >=7 && value <=12)
    {
        *index = 12-value;
        return OK;
    }
    *index = 0xFF;
    return FAIL;
}

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


void updateLoRaMacInitNwkIds(void)
{
    uint32_t macAddr;
    uint8_t nwkSKey[16];
    uint8_t appSKey[16];
    macAddr = strtoul((const char*)OthersConfig.CMacAddr, NULL, 16);
    TransferKeyFromStrToNum(nwkSKey, NodeConfig.NwkSessionKey ,16);
    TransferKeyFromStrToNum(appSKey, NodeConfig.AppSessionKey ,16);
    LoRaMacInitNwkIds(0, macAddr, nwkSKey,appSKey);
}

void StopRadioTxDelay(uint32_t timerID)
{
#if defined( STM32L073xx )
    /* Modify by Eric Date: 2017/02/22   Log: Add UartStopRadioTxDelay1(gtxd=1) to avoid Rx2 receive issue.  */
    if(NodeConfig.RadioTxDelayTime==1000)
        SendQueueToUartTask(UartStopRadioTxDelay1, (UartId_t)NULL, NULL);
    else
        SendQueueToUartTask(UartStopRadioTxDelay, (UartId_t)NULL, NULL);
#else
    SendQueueToUartTask(UartStopRadioTxDelay, (UartId_t)NULL, NULL);
#endif
}

void StopRadioTxDelayTimer()
{
    if(RadioTxDelayTimer != 0x00)
    {
        DeleteTimer(RadioTxDelayTimer);
        RadioTxDelayTimer =0x00;
    }
    RadioTxDelayFlag = 0;
}

void StartRadioTxDelay()
{
    if(RadioTxDelayFlag == 0)
    {
        RadioTxDelayFlag = 1;
        RadioTxDelayTimer = AddTimer(RadioTxDelayTimerID, NodeConfig.RadioTxDelayTime
                                     , 0, StopRadioTxDelay);
    }
}

uint8_t ReceiveRadioRxData(RadioRxData_t* rxData)
{
    uint8_t sendBuff[64];
    Gpio_t pin;
    if(UserConfig1.RecPort != -1 && rxData->RxPort != UserConfig1.RecPort)
    {
        return FAIL;
    }
    //int i;
    sprintf((char*)sendBuff, "Radio Rx Done Len:%d RSSI:%d SNR:%d\r\n"
            ,rxData->Size,rxData->RssiValue,rxData->SnrValue);
    /*for(i=0;i<  radiostatus.RxData.Size; i++)
    {
        sprintf((char*)&tmp,"%x",radiostatus.RxData.RxBuffer[i]);
        strcat((char*)sendBuff,(char*)tmp);
    }
    strcat((char*)sendBuff,"\r\n");*/
    SendMsgToUart(sendBuff,UseUartID);
    if(rxData->Size == 0 || rxData->Size > MAX_RADIO_RX_DATA_SIZE)
    {
        return FAIL;
    }
    RadioRxData.Size = rxData->Size;
    RadioRxData.RssiValue = rxData->RssiValue;
    RadioRxData.RxPort = rxData->RxPort;
    RadioRxData.SnrValue = rxData->SnrValue;
    if(RadioRxData.RxBuffer != NULL)
    {
        vPortFree(RadioRxData.RxBuffer);
    }
    RadioRxData.RxBuffer = (uint8_t*)pvPortMalloc(rxData->Size +1);
    memset(RadioRxData.RxBuffer, '\0', rxData->Size+1);
    memcpy(RadioRxData.RxBuffer, rxData->RxBuffer ,rxData->Size);
    // Pull high PA8
    GpioInit(&pin, UART1_CK, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 1);
    GpioWrite(&pin,1);
    return OK;
}

uint8_t converterStringToAscii(uint8_t* asciiArray, uint8_t* strArray)
{
    uint8_t length;
    uint8_t tmp[3];
    int i;
    length = strlen((const char*) strArray);
    if((length %2) != 0 || checkVauleFormate(strArray,16,0) != OK)
    {
        return FAIL;
    }
    for(i=0; i<length; i=i+2)
    {
        memset(tmp,'\0',3);
        memcpy(tmp,&strArray[i],2);
        asciiArray[i/2] = strtol((const char*)tmp,NULL,16);
    }
    return OK;
}


const GpioMapping_t GpioMappingTable[]=
{
    {"PB8",LED_FCT}, // For Restore default pin
    //{"PA3",UART_RX},// UART 2 RX //Jason mark at 2019.08.19
    //{"PA2",UART_TX},// UART 2 TX //Jason mark at 2019.08.19
    {"PB6",I2C_SCL}, // for IRQ0
    {"PB7",I2C_SDA},// for IRQ1
    //{"PA4",SPI1_NSS}, //Jason mark at 2019.08.19
    //{"PA5",SPI1_SCK}, //Jason mark at 2019.08.19
    //{"PA6",SPI1_MISO},//Jason mark at 2019.08.19
    //{"PA7",SPI1_MOSI},//Jason mark at 2019.08.19
    {"PB1",INT_2},
    {"PB0",INT_1},
    {"PA12",UART1_RTS},
    {"PA11",UART1_CTS},
    //{"PA9",UART1_TX},//UART1_TX
    //{"PA10",UART1_RX},//UART1_RX
    {"PA8",UART1_CK},
    //{"PA14",SWCLK},//SWCLK
    //{"PA13",SWDIO},//SWDIO
    {"PB5",LED_TX},
};

uint8_t ReadGpioStatus(uint8_t* pinName)
{
    Gpio_t ioIn;
    int inIndex;
    uint8_t value;
    for(inIndex=0; inIndex<sizeof(GpioMappingTable)/sizeof(GpioMapping_t); inIndex++)
    {
        if(strcmp((const char*)GpioMappingTable[inIndex].pinName,(const char*)pinName) ==0)
        {
            break;
        }
    }
    // can't find this gpio pin
    if(inIndex == sizeof(GpioMappingTable)/sizeof(GpioMapping_t)|| GpioMappingTable[inIndex].gpioPin == NULL)
    {
        return 2;
    }
    GpioInit(&ioIn, GpioMappingTable[inIndex].gpioPin, GPIO_PINMODE_READ_INTERRUPT, GPIO_PINCONFIGS_READ_INTERRUPT, GPIO_PINTYPES_READ_INTERRUPT, 0);
    value = GpioReadInput(&ioIn);
    //DeInit pin
    if(strcmp("PB7",(const char*)pinName) ==0)
        GpioInit(&ioIn, GpioMappingTable[inIndex].gpioPin, GPIO_PINMODE_PB7_UNUSED, GPIO_PINCONFIGS_PB7_UNUSED, GPIO_PINTYPES_PB7_UNUSED, 0);
    else
        GpioInit(&ioIn, GpioMappingTable[inIndex].gpioPin, GPIO_PINMODE_UNUSED, GPIO_PINCONFIGS_UNUSED, GPIO_PINTYPES_UNUSED, 0);
    return value;
}


void unUsedGpioInit()
{
    uint8_t gpioTableSize;
    uint8_t gpioIndex;
    Gpio_t gpio;
    gpioTableSize = sizeof(GpioMappingTable)/sizeof(GpioMapping_t);
    for(gpioIndex=0; gpioIndex<gpioTableSize; gpioIndex++)
    {
        if(GpioMappingTable[gpioIndex].gpioPin != NULL)
        {
            if(strcmp("PB7",(const char*)GpioMappingTable[gpioIndex].pinName) ==0)
                GpioInit(&gpio, GpioMappingTable[gpioIndex].gpioPin, GPIO_PINMODE_PB7_UNUSED, GPIO_PINCONFIGS_PB7_UNUSED, GPIO_PINTYPES_PB7_UNUSED, 0);
            else
                GpioInit(&gpio, GpioMappingTable[gpioIndex].gpioPin, GPIO_PINMODE_UNUSED, GPIO_PINCONFIGS_UNUSED, GPIO_PINTYPES_UNUSED, 0);
        }
    }
}

uint8_t GpioTest(uint8_t* pinNameIn, uint8_t* pinNameOut, uint8_t pinOutValue)
{
    uint8_t gpioTableSize;
    Gpio_t ioIn, ioOut;
    int inIndex,outIndex;
    uint8_t pinInValue;
    uint8_t sendBuff[80];
    gpioTableSize = sizeof(GpioMappingTable)/sizeof(GpioMapping_t);
    for(inIndex=0; inIndex<gpioTableSize; inIndex++)
    {
        if(strcmp((const char*)GpioMappingTable[inIndex].pinName,(const char*)pinNameIn) ==0)
        {
            break;
        }
    }
    if(inIndex ==gpioTableSize)
    {
        SendMsgToUart("pinNameIn not found\r\n", UseUartID);
        return FAIL;
    }
    if(GpioMappingTable[inIndex].gpioPin == NULL)
    {
        SendMsgToUart("pinNameIn can't use\r\n", UseUartID);
        return FAIL;
    }
    for(outIndex=0; outIndex<gpioTableSize; outIndex++)
    {
        if(strcmp((const char*)GpioMappingTable[outIndex].pinName,(const char*)pinNameOut) ==0)
        {
            break;
        }
    }
    if(outIndex ==gpioTableSize)
    {
        SendMsgToUart("pinNameOut not found\r\n", UseUartID);
        return FAIL;
    }
    if(GpioMappingTable[outIndex].gpioPin == NULL)
    {
        SendMsgToUart("pinNameOut can't use\r\n", UseUartID);
        return FAIL;
    }
    GpioInit(&ioIn, GpioMappingTable[inIndex].gpioPin, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
    GpioInit(&ioOut, GpioMappingTable[outIndex].gpioPin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, pinOutValue);
    GpioWrite(&ioOut,pinOutValue);
    vTaskDelay(50/portTICK_PERIOD_MS);
    pinInValue = GpioReadInput(&ioIn);
    if(pinInValue == pinOutValue)
    {
        sprintf((char*)sendBuff,"IO [%s,%s] test match value:%d\r\n",GpioMappingTable[inIndex].pinName
                ,GpioMappingTable[outIndex].pinName,pinOutValue);
        SendMsgToUart(sendBuff, UseUartID);
    }
    else
    {
        sprintf((char*)sendBuff,"IO [%s,%s] test not match, in value:%d, out value:%d\r\n",GpioMappingTable[inIndex].pinName
                ,GpioMappingTable[outIndex].pinName,pinInValue,pinOutValue);
        SendMsgToUart(sendBuff, UseUartID);
    }
    GpioInit(&ioOut, GpioMappingTable[outIndex].gpioPin, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
    return OK;
}


void dTTXCallback(uint32_t timer)
{
    SendQueueToRadio(SendUnConfirmedDataUp, &RadioSendData);
}


/*FactoryTest*/
NodeTest TxcmConvertStringToNumerical(txcm_t *rawdata, RadioTxContinuousMode_t *data)
{
    int i;
    // Convert from ASCII to uint8_t values
    for(i = 0 ; i < 8 ; i++)
    {
        rawdata->LoRaFreq[i]						= rawdata->LoRaFreq[i] & 0xF;
    }
    for(i = 0 ; i < 3 ; i++)
    {
        rawdata->LoRaTxPwr[i] 					= rawdata->LoRaTxPwr[i] & 0xF;
    }
    for(i = 0 ; i < 2 ; i++)
    {
        rawdata->LoRaBwd[i] 						= rawdata->LoRaBwd[i] & 0xF;
        rawdata->LoRaSpFactor[i] 				= rawdata->LoRaSpFactor[i] & 0xF;
        rawdata->LoRaCodeRate[i] 				= rawdata->LoRaCodeRate[i] & 0xF;
        rawdata->LoRaPreambleLen[i] 		= rawdata->LoRaPreambleLen[i] & 0xF;
        rawdata->LoRaFixLenPayloadOn[i] = rawdata->LoRaFixLenPayloadOn[i] & 0xF;
        rawdata->LoRaEnableCrc[i] 			= rawdata->LoRaEnableCrc[i] & 0xF;
        rawdata->LoRaIqInversionOn[i] 	= rawdata->LoRaIqInversionOn[i] & 0xF;
    }
    //There are 8 Values to be configured. However, we'll skip Preamble Length first
    /*-------------------------------------------------*/
    data->FreqVal						= rawdata->LoRaFreq[0] * 100.0 + rawdata->LoRaFreq[1] * 10.0 + rawdata->LoRaFreq[2] * 1.0 +
                                          rawdata->LoRaFreq[4] / 10.0	 + rawdata->LoRaFreq[5] / 100.0 + rawdata->LoRaFreq[6] / 1000.0;
    if((BAND_434MHZ_MIN < data->FreqVal) && (data->FreqVal < BAND_434MHZ_MAX))
    {
        data->FreqIdx						= 0;
    }
    else if((BAND_470MHZ_MIN < data->FreqVal) && (data->FreqVal < BAND_470MHZ_MAX))
    {
        data->FreqIdx						= 1;
    }
    else if((BAND_868MHZ_MIN < data->FreqVal) && (data->FreqVal < BAND_868MHZ_MAX))
    {
        data->FreqIdx						= 2;
    }
    else if((BAND_915MHZ_MIN <= data->FreqVal) && (data->FreqVal <= BAND_915MHZ_MAX))
    {
        data->FreqIdx						= 3;
    }
    else
        return	PROD_TEST_FAIL;
    /*-------------------------------------------------*/
    data->TxPwrVal 					= rawdata->LoRaTxPwr[0] * 10.0 + rawdata->LoRaTxPwr[1] * 1.0;
    if(data->TxPwrVal < 0 || data->TxPwrVal > TXPWR_MAX_VAL)
        return	PROD_TEST_FAIL;
    /*-------------------------------------------------*/
    data->BwdIdx 						= rawdata->LoRaBwd[0];
    if(data->BwdIdx > BWD_MAX_VAL)
        return	PROD_TEST_FAIL;
    /*-------------------------------------------------*/
    data->SpFactorIdx  			= rawdata->LoRaSpFactor[0];
    if(data->SpFactorIdx > SP_FACTOR_MAX_VAL)
        return	PROD_TEST_FAIL;
    /*-------------------------------------------------*/
    data->CodeRateIdx  			= rawdata->LoRaCodeRate[0];
    if(data->CodeRateIdx < 1 || data->CodeRateIdx > CR_MAX_VAL)
        return	PROD_TEST_FAIL;
    /*-------------------------------------------------*/
//		PreLenIdx 				= UartData.LoRaPreambleLen[0];
//
//		if(PreLenIdx < 0 || PreLenIdx > PRE_LEN_MAX_VAL)
//				return	PROD_TEST_FAIL;
    /*-------------------------------------------------*/
    data->FixLenPayloadOn 	= rawdata->LoRaFixLenPayloadOn[0];
    if(data->FixLenPayloadOn > FIX_LEN_PAYLOAD_MAX_VAL)
        return	PROD_TEST_FAIL;
    /*-------------------------------------------------*/
    data->LoRaEnableCrcOn		= rawdata->LoRaEnableCrc[0];
    if(data->LoRaEnableCrcOn > EN_CRC_MAX_VAL)
        return	PROD_TEST_FAIL;
    /*-------------------------------------------------*/
    data->IqInversionOn 		= rawdata->LoRaIqInversionOn[0];
    if(data->IqInversionOn > IQ_INVER_MAX_VAL)
        return	PROD_TEST_FAIL;
    /*-------------------------------------------------*/
    return PROD_TEST_TXCM_START;
}

NodeTest TxppConvertStringToNumerical(txpp_t *rawdata, bool *masterID, uint16_t *timeout, uint16_t *number)
{
    int i;
    // Convert from ASCII to uint8_t values
    for(i = 0 ; i < 2 ; i++)
    {
        rawdata->PingPongMaster[i]			= rawdata->PingPongMaster[i] & 0xF;
    }
    for(i = 0 ; i < 5 ; i++)
    {
        rawdata->PingPongMaxTxNumOrRxTime[i]		= rawdata->PingPongMaxTxNumOrRxTime[i] & 0xF;
        rawdata->PingPongTxOrRxTimeout[i]		= rawdata->PingPongTxOrRxTimeout[i] & 0xF;
    }
    *masterID		= rawdata->PingPongMaster[0];
    if(*masterID > 1)
        return	PROD_TEST_FAIL;
    /*-------------------------------------------------*/
    *timeout		= rawdata->PingPongTxOrRxTimeout[0] * 1000.0 + rawdata->PingPongTxOrRxTimeout[1] * 100.0 +
                      rawdata->PingPongTxOrRxTimeout[2] * 10.0 + rawdata->PingPongTxOrRxTimeout[3];
    if(*timeout < MIN_PING_VAL || *timeout > MAX_PING_TX_RX_VAL)
        return	PROD_TEST_FAIL;
    /*-------------------------------------------------*/
    *number		= rawdata->PingPongMaxTxNumOrRxTime[0] * 1000.0 + rawdata->PingPongMaxTxNumOrRxTime[1] * 100.0 +
                  rawdata->PingPongMaxTxNumOrRxTime[2] * 10.0 + rawdata->PingPongMaxTxNumOrRxTime[3];
    if(*number < MIN_PING_VAL || *number > MAX_PING_TX_RX_VAL)
        return	PROD_TEST_FAIL;
    return PROD_TEST_TXPP_START;
}


NodeTest GetProductionTestCommand(uint8_t *string)
{
    if(strncmp((const char*)string, (const char*)"TXCM", strlen("TXCM")) == 0)
        return PROD_TEST_TXCM_START;
    else if(strncmp((const char*)string, (const char*)"TXPP", strlen("TXPP")) == 0)
        return PROD_TEST_TXPP_START;
    return PROD_TEST_FAIL;
}


NodeTest GetTXCMData(uint8_t* string, RadioTxContinuousMode_t* RadioPTModeData)
{
    NodeTest testCmd;
    txcm_t txcmdata;
    testCmd = parseTXCMdata((int8_t*)string, &txcmdata);
    if(testCmd !=PROD_TEST_TXCM_START)
    {
        return PROD_TEST_FAIL;
    }
    testCmd = TxcmConvertStringToNumerical(&txcmdata, RadioPTModeData);
    if(testCmd !=PROD_TEST_TXCM_START)
    {
        return PROD_TEST_FAIL;
    }
    return PROD_TEST_TXCM_START;
}

NodeTest GetTXPPData(uint8_t* string, RadioTxContinuousMode_t* RadioPTModeData
                     , bool *isMaster, uint16_t* TxNumOrRxTimeIdx, uint8_t* label)
{
    NodeTest testCmd;
    txcm_t txcmdata;
    txpp_t txppdata;
    uint16_t TxOrRxTimeoutIdx;
    //Get txpp data and txcmdata from string
    testCmd = parseTXPPdata((int8_t *)string, &txppdata, &txcmdata);
    if(testCmd !=PROD_TEST_TXPP_START)
    {
        return PROD_TEST_FAIL;
    }
    //Get isMaster, TxOrRxTimeout and TXNumOrRxTimeIdex from txppdata
    testCmd = TxppConvertStringToNumerical(&txppdata, isMaster
                                           , &TxOrRxTimeoutIdx, TxNumOrRxTimeIdx);
    if(testCmd !=PROD_TEST_TXPP_START)
    {
        return PROD_TEST_FAIL;
    }
    memcpy(label, txppdata.Label,5);
    RadioPTModeData->PingPongTxOrRxTimeoutIdx = TxOrRxTimeoutIdx;
    testCmd = TxcmConvertStringToNumerical(&txcmdata, RadioPTModeData);
    if(testCmd !=PROD_TEST_TXCM_START)
    {
        return PROD_TEST_FAIL;
    }
    return PROD_TEST_TXPP_START;
}

void SetTXPPFlag(uint8_t flag)
{
    TXPPRunningFlag = flag;
}

//add on 2018.11.15
uint8_t GetEEPROMConfig(AtConfig_t* atConfig, UserConfig1_t* userConfig1)
{
    atConfig->GpioReportTime_day = AtConfig.GpioReportTime_day;
    atConfig->GpioReportTime_minutes= AtConfig.GpioReportTime_minutes;
    atConfig->IrqTriggerType_0= AtConfig.IrqTriggerType_0;
    userConfig1->AppPort = UserConfig1.AppPort;
    userConfig1->SyncFlag= UserConfig1.SyncFlag;
    userConfig1->RecPort= UserConfig1.RecPort;
    userConfig1->GpioReportTime_day1= UserConfig1.GpioReportTime_day1;
    userConfig1->GpioReportTime_minutes1= UserConfig1.GpioReportTime_minutes1;
//    AtConfig->IrqTriggerType_1= AtConfig.IrqTriggerType_1;
    return OK;
}

uint32_t GetTxDelayTime()
{
    return NodeConfig.RadioTxDelayTime;
}

uint8_t GetReporterMode()
{
    if(NodeConfig.ReportModeEnable == 1 && AtConfig.Enter_Reporter ==1 && AtConfig.EngineerModeFlag ==0)
        return 1;
    else
        return 0;
}

uint8_t GetLowPowerModeFlag(uint8_t* enterLPMTime)
{
    if(AtConfig.EngineerModeFlag ==0  && AtConfig.LowPowerModeFlag ==1)
    {
        if(OthersConfig.EnterLPMTime == 0)
        {
            //When upgrade from FW v1.15.09, EnterLPMTime will be 0 and it should use default vaule
            OthersConfig.EnterLPMTime = DEFAULT_ENTER_LPM_TIME;
        }
        *enterLPMTime = OthersConfig.EnterLPMTime;
        return 1;
    }
    else
    {
        return 0;
    }
}

//daniel add on 2017.2.2 SensorConfig data copy to senConfig
uint8_t GetSensorConfig(SensorConfig_t* senConfig)
{
    memcpy(senConfig, &SensorConfig, sizeof(SensorConfig_t));
    return OK;
}

//daniel add on 2017.3.28 for return AtConfig
AtConfig_t GetAtConfig(void)
{
    return AtConfig;
}

void RadioTxLED(uint8_t onTx)
{
    // PA2: UART_TX, PA3:UART_RX
    if(NodeConfig.LEDEnable == 1)
    {
        Gpio_t pin;
        if(onTx == 0)
        {
            GpioInit(&pin, UART_TX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
            GpioWrite(&pin,0);
						/* Remove by Eric Date: 2017/03/9   Log: Remove flag definition and fix led issue. */
            GpioInit(&pin, UART_RX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
            GpioWrite(&pin,1);
        }
        else if(onTx == 1)
        {
            GpioInit(&pin, UART_TX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
            GpioWrite(&pin,1);
            GpioInit(&pin, UART_RX, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
            GpioWrite(&pin,0);
        }
    }
}

void SendRadioTxRetryData()
{
    RadioTxLED(1);
    SendQueueToRadio(SendConfirmedDataUp, &RadioSendData);
}

uint8_t GetPinVoltage(uint32_t channel)
{
#if defined( STM32L073xx )
    float pmSensor_V_bat;
    uint16_t vdiv = 0, vref = 0;
    AdcInit();
    vdiv = AdcRead(channel);
    vref = AdcRead(ADC_CHANNEL_17);
    pmSensor_V_bat = 1.198*vdiv/vref;
    return pmSensor_V_bat*10;
#else
    int i;
    float pmSensor_V_bat,pmSensor_V_bat_sum=0;
    uint16_t vdiv = 0, vref = 0;
    AdcInit();
    for(i=0; i<5; i++)
    {
        vdiv = AdcRead(channel);
        vref = AdcRead(ADC_Channel_17);
        pmSensor_V_bat = 1.224*vdiv/vref;
        pmSensor_V_bat_sum = pmSensor_V_bat_sum + pmSensor_V_bat;
    }
    return (pmSensor_V_bat_sum/5)*10;
#endif
}

void InitATInterrupt(uint8_t *pinName, GpioIrqHandler *irqHandler)
{
    Gpio_t gpio;
    if(strcmp("PB6",(const char*)pinName) ==0)
    {
        GpioInit(&gpio, I2C_SCL, GPIO_PINMODE_READ_INTERRUPT, GPIO_PINCONFIGS_READ_INTERRUPT, GPIO_PINTYPES_READ_INTERRUPT, 0);
        GpioSetInterrupt(&gpio, IRQ_RISING_EDGE, IRQ_MEDIUM_PRIORITY, irqHandler);
    }
    if(strcmp("PB7",(const char*)pinName) ==0 && GetLowPowerModeFlag(NULL) == 1 && GetReporterMode() == 0)
    {
        GpioInit(&gpio, I2C_SDA, GPIO_PINMODE_READ_INTERRUPT, GPIO_PINCONFIGS_READ_INTERRUPT, GPIO_PINTYPES_READ_INTERRUPT, 0);
        GpioSetInterrupt(&gpio, IRQ_RISING_EDGE, IRQ_MEDIUM_PRIORITY, irqHandler);
    }
}

