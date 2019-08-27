#ifndef COMMAND_LIST_H
#define COMMAND_LIST_H
#include "at_command_app.h"
#include "Gpio-board.h"
#include "radio_task.h"

#ifdef LOW_POWER_MODE
#define FW_DEFINE "LOW_POWER_MODE"
#else
#define FW_DEFINE "SIPMODULE"
#endif

/*GPIO status define*/
#define GPIO_PINMODE_UNUSED                        PIN_ANALOGIC
#define GPIO_PINCONFIGS_UNUSED                   PIN_PUSH_PULL
#define GPIO_PINTYPES_UNUSED                       PIN_PULL_DOWN

#define GPIO_PINMODE_READ_INTERRUPT         PIN_INPUT
#define GPIO_PINCONFIGS_READ_INTERRUPT    PIN_PUSH_PULL
#define GPIO_PINTYPES_READ_INTERRUPT        PIN_PULL_DOWN

#define GPIO_PINMODE_PB7_UNUSED         PIN_INPUT
#define GPIO_PINCONFIGS_PB7_UNUSED    PIN_PUSH_PULL
#define GPIO_PINTYPES_PB7_UNUSED        PIN_PULL_DOWN

#define EEPROM_CONFIG_START_ADDR      0x08080000
#define EEPROM_NODE_CONFIG_ADDR  ( EEPROM_CONFIG_START_ADDR+0x30)
#define EEPROM_ENG_CONFIG1_ADDR  ( EEPROM_CONFIG_START_ADDR+0xF0)
#define EEPROM_USER_CONFIG1_ADDR  ( EEPROM_CONFIG_START_ADDR+0x120)
#define EEPROM_AT_CONFIG_ADDR  ( EEPROM_CONFIG_START_ADDR+0x150)
#define EEPROM_RX_CONFIG_ADDR  ( EEPROM_CONFIG_START_ADDR+0x170)
#define EEPROM_OTHERS_CONFIG_ADDR  ( EEPROM_CONFIG_START_ADDR+0x190)
#define EEPROM_LORAWAN_CONFIG_ADDR  ( EEPROM_CONFIG_START_ADDR+0x1D0)
#define EEPROM_SENSOR_CONFIG_ADDR  ( EEPROM_CONFIG_START_ADDR+0x270) //daniel modify on 2017.12.26 //? must check real size

#define RADIO_DATA_SIZE_SF10 11
#define RADIO_DATA_SIZE 50
#define RADIO_TX_DELAY_MAX 2678400 // 31 days
#define RADIO_TX_DELAY_MIN 1// 1 second
#define MAX_RADIO_RX_SIZE  32

#define CONFIG_SN_SIZE              16
#define CONFIG_MAC_SIZE            9
#define CONFIG_SYS_ID_SIZE	     15
#define CONFIG_NWKSKEY_SIZE   33
#define CONFIG_APPSKEY_SIZE    33
#define CONFIG_PINCODE_SIZE    5
#define CONFIG_DEVEUI_SIZE            17
#define CONFIG_APPEUI_SIZE            17
#define CONFIG_APPKEY_SIZE    33

#define PASSWORD_SIZE              16
#define MANUFACTUER_ID_SIZE 7
#define CONFIG_HW_VER_SIZE 12

#define ENTER_RESTORE_TIME 10 // 10s, unit is seconds
#define ENTER_LOW_POWER_MODE_TIME 60 // 60s, unit is seconds
#define CONTROL_PIN_SIZE   12 // Jason add for pin c control on 2019.01.28
#define IO_PINC_SIZE       8  // Jason add for pin c control on 2019.01.28
/*Constant Value*/
#if defined( STM32L073xx )
static const uint8_t SIPMODULE_FW_VERSION[] = "v1.15.10.5_modify9";//for SGMR //Jason modify on 2019.08.27
#ifdef SIPMODULE_G78S
static const uint8_t SIPMODULE_MODEL_ID[] = "G78S";//for SGMM
#else
static const uint8_t SIPMODULE_MODEL_ID[] = "G76S";//for SGMM
#endif
#else
static const uint8_t SIPMODULE_FW_VERSION[] = "v1.15.10_modify2";//for SGMR
static const uint8_t SIPMODULE_MODEL_ID[] = "WMDS-203";//for SGMM
#endif
static const uint8_t SIPMODULE_MANUFACTUER_ID[] = "Gemtek";//for SGMI

//daniel add on 2017.3.15
#if 0 //for test
static const uint8_t MY_DEFALUT_SN[] = "GLN016140074F";
static const uint8_t MY_DEFAULT_MAC_ADDR[] = "040008b2"; //modify on 2016.10.4 for demo //replace default
static const uint8_t MY_DEFALUT_SYSTEM_ID [] = "04"; //modify on 2016.10.4 for demo //replace default
static const uint8_t MY_DEFALUT_NETWORK_KEY[] = "43610A1F04719BB807A8073F8AECB131"; //modify on 2016.10.4 for demo //replace default
static const uint8_t MY_DEFALUT_APPLICATION_KEY[] = "43610A1F04719BB807A8073F8AECB131"; //modify on 2016.10.4 for demo //replace default
static const uint8_t MY_DEFAULT_PINCODE[] = "7424";
#endif //for test

/*Default Node Configurations Value*/
static const uint8_t DEFALUT_SN[] = "GLN0165100000";
static const uint8_t DEFAULT_MAC_ADDR[] = "00000000";
static const uint8_t DEFALUT_SYSTEM_ID [] = "00";

static const uint8_t DEFALUT_OTAA_APP_KEY[] = "CD28A875B06982921E7F946F2A546A61";
#if defined( STM32L073xx )
static const uint8_t DEFALUT_NETWORK_KEY[] = "92222D13B016A5D599BE3E827214CA89";
static const uint8_t DEFALUT_APPLICATION_KEY[] = "F7740AF4B0FA5B7FBF23B95369D9BA89";
#else
static const uint8_t DEFALUT_NETWORK_KEY[] = "63D83C7F054A18D423BDFB712D8F4371";
static const uint8_t DEFALUT_APPLICATION_KEY[] = "63D83C7F054A18D423BDFB712D8F4371";
#endif
static const uint8_t DEFAULT_PINCODE[] = "0000";
static const uint8_t DEFAULT_HWVersion[] = "HW_Version0";
static const uint8_t DEFAULT_IBR = 0;
static const int16_t DEFAULT_CALI_RSSI = -147;
static const uint8_t DEFAULT_LOW_POWER_MODE = 0;
static const uint8_t DEFAULT_ECHO = 0;
static const uint8_t DEFAULT_ENG_MODE_FLAG = 0;
static const uint8_t DEFAULT_LED_FLAG = 0;
static const uint8_t DEFAULT_REPORT_MODE_ENABLE = 1;
static const uint32_t DEFAULT_RADIO_Delay_TIME = 5000;//5 second, according to PR-1006 //daniel note on 2017.3.16 -> minimum is 5000
static const uint8_t DEFAULT_TESTPOWER = 0; //add for new command TESTPOWER on 2018.6.7
static const uint8_t DEFAULT_DEVICE_TYPE = 3;//Jason on 2019.01.29

/*Default CapConfig Value*/
static const uint8_t DEFAULT_SF_TX = 10;//SF10
static const uint8_t DEFAULT_POWER_TX_MAX = 20;
#if defined( STM32L073xx )
#ifdef SIPMODULE_G78S
static const uint32_t DEFAULT_FREQ_OFFSET_1 = 100000;
static const uint32_t DEFAULT_FREQ_OFFSET_2 = 300000;
static const uint32_t DEFAULT_RX2_FREQ = 505300000;
static const uint8_t   DEFAULT_RX2_BANDWIDTH = 0;
static const uint8_t   DEFAULT_RX2_SF = 12;
#else
static const uint32_t DEFAULT_FREQ_OFFSET_1 = 100000; //125000;
static const uint32_t DEFAULT_FREQ_OFFSET_2 = 300000; //375000;
#endif
#else
static const uint32_t DEFAULT_FREQ_OFFSET_1 = 100000; //125000;
static const uint32_t DEFAULT_FREQ_OFFSET_2 = 300000; //375000;
#endif
/*Default AtConfig*/
static const uint8_t DEFAULT_ENTER_REPORTER = 0;
static const uint8_t DEFAULT_GPIO_REPORT_TIME_MINUTES = 60;
static const uint8_t DEFAULT_GPIO_REPORT_TIME_DAY = 0;
static const uint8_t DEFAULT_IRQ_TRIGGER_TYPE_0 = 1;
static const uint8_t DEFAULT_PA_ENABLE = 0;

/*Default EngConfig1*/
#if defined( STM32L073xx )
static const uint8_t DEFAULT_ADM_PASSWORD[] = "$id@uh#e9"; //for GADM
#else
static const uint8_t DEFAULT_ADM_PASSWORD[] = "Gemtek888"; //for GADM
#endif

/*Default UserConfig1*/
static const int16_t DEFAULT_RECPORT = -1;
static const uint8_t DEFAULT_SYNCFLAG = 0;
static const uint8_t DEFAULT_APP_PORT = 1;
static const uint8_t DEFAULT_GPIO_REPORT_TIME_MINUTES_1 = 0;
static const uint8_t DEFAULT_GPIO_REPORT_TIME_DAY_1 = 0;

/*Defalut RxConfig*/
static const uint8_t DEFAULT_RX_BW = 0;
static const uint8_t DEFAULT_RX_SF = 10;

/*Default OthersConfig*/
static const uint8_t DEFAULT_ENTER_LPM_TIME = 60;


/*NodeConfig: 172 bytes*/
typedef struct NodeConfig
{
    uint8_t ManufactureID[MANUFACTUER_ID_SIZE];
    uint8_t Sn[CONFIG_SN_SIZE];
    uint8_t MacAddr[CONFIG_MAC_SIZE];
    uint8_t SystemId[CONFIG_SYS_ID_SIZE];
    uint8_t NwkSessionKey[CONFIG_NWKSKEY_SIZE];
    uint8_t AppSessionKey[CONFIG_APPSKEY_SIZE];
    uint8_t PinCode[CONFIG_PINCODE_SIZE];
    uint8_t HWVersion[CONFIG_HW_VER_SIZE];
    uint8_t ReportModeEnable;
    uint8_t LEDEnable;
    int16_t CaliRSSI;
    uint32_t RadioTxDelayTime;
    /*Capacity Configuration*/
    uint8_t Power_TX_MAX;
    uint32_t FreqA1;
    uint32_t FreqA2;
    uint32_t FreqB1;
    uint32_t FreqB2;
    uint32_t Offset1;
    uint32_t Offset2;
    uint8_t ClassMode;
    uint8_t temp;// this vairable no longer used
} NodeConfig_t;

/*AtConfig: 20 bytes*/
typedef struct AtConfig
{
    uint8_t ManufactureID[MANUFACTUER_ID_SIZE];
    uint8_t IBR;
    uint8_t EchoFlag;// enable:1, disable:0
    uint8_t EngineerModeFlag;
    uint8_t LowPowerModeFlag;
    uint8_t SF_TX;
    uint8_t SF_RX;
    uint8_t Power_TX;
    uint8_t Enter_Reporter;
    uint8_t IrqTriggerType_0;
    uint16_t GpioReportTime_day;
    uint16_t GpioReportTime_minutes;
	  uint32_t UP_FEEDBACK;
	  uint32_t DOWN_FEEDBACK;
	  uint8_t DEVICE_TYPE;
	  uint8_t PINC[IO_PINC_SIZE];
}AtConfig_t;

typedef struct At2Config
{
    uint8_t PowerFlag;// on:1, off:0 add on 2018.6.7 for power test
}At2Config_t;

/*EngConfig1: 23 bytes*/
typedef struct EngConfig1
{
    uint8_t ManufactureID[MANUFACTUER_ID_SIZE];
    uint8_t New_ADM_PASSWORD[PASSWORD_SIZE];
}EngConfig1_t;

/*UserConfig1: 16 bytes*/
typedef struct UserConfig1
{
    uint8_t ManufactureID[MANUFACTUER_ID_SIZE];
    int16_t RecPort;
    uint8_t SyncFlag;
    uint8_t AppPort;
    uint16_t GpioReportTime_day1;
    uint16_t GpioReportTime_minutes1;

}UserConfig1_t;

/*RxConfig: 16 bytes*/
typedef struct RxConfig
{
    uint8_t ManufactureID[MANUFACTUER_ID_SIZE];
    uint32_t RxFreq;
    uint8_t RxBW; // 0:125 1:250 2:500
    uint8_t RxSF;
}RxConfig_t;

typedef struct
{
    uint8_t pinName[5];
    PinNames gpioPin;
} GpioMapping_t;

typedef struct LoRaWanConfig
{
    uint8_t ManufactureID[MANUFACTUER_ID_SIZE];
    uint8_t DevEUI[CONFIG_DEVEUI_SIZE];
    uint8_t AppKey[CONFIG_APPKEY_SIZE];
    uint8_t AppEUI[CONFIG_APPEUI_SIZE];
    uint8_t DevAddr[CONFIG_MAC_SIZE];
    uint8_t NwkSKey[CONFIG_NWKSKEY_SIZE];
    uint8_t AppSKey[CONFIG_APPSKEY_SIZE];
}LoRaWanConfig_t;

/* Add by Eric Date: 2017/02/17   Log: Add PA enable value and reserved some parameters for future */
/*RxConfig: 60 bytes*/
typedef struct OthersConfig
{
    uint8_t ManufactureID[MANUFACTUER_ID_SIZE];
    uint8_t CMacAddr[CONFIG_MAC_SIZE];
    uint8_t PAEnable;
    uint8_t EnterLPMTime;
    uint8_t u8temp_3;
    uint8_t u8temp_4;
    uint8_t u8temp_5;
    uint16_t u16temp_0;
    uint16_t u16temp_1;
    uint16_t u16temp_2;
    uint16_t u16temp_3;
    uint16_t u16temp_4;
    uint16_t u16temp_5;
    uint32_t u32temp_0;
    uint32_t u32temp_1;
    uint32_t u32temp_2;
    uint32_t u32temp_3;
    uint32_t u32temp_4;
    uint32_t u32temp_5;
} OthersConfig_t;

//daniel add on 2017.1.23
typedef struct SensorConfig
{
    uint8_t ManufactureID[MANUFACTUER_ID_SIZE];
    uint8_t sensorType; //0:none uart sensor 1:uart-ttl sensor 2:uart-rs485 sensor //add on 2017.3.6
    uint8_t uartBR;    //baud rate: 1200, 2400, 4800, 19200, 38400, 57600, 115200 default:9600
    uint8_t uartDB;    //data bits: UART_9_BIT, default:UART_8_BIT
    uint8_t uartPC;    //parity check: EVEN_PARITY, ODD_PARITY, default:NO_PARITY
    uint8_t uartSB;    //stop bits: UART_0_5_STOP_BIT, UART_2_STOP_BIT, UART_1_5_STOP_BIT, default:UART_1_STOP_BIT
    uint8_t replyLEN;  //length of reply data: this value must <= 64
    uint8_t startBYTE; //start byte pos for parsing reply data: this value must <= 64
    uint8_t readLEN;   //read bytes for parsing reply data: this value must <= 8
    uint8_t modbusCMDLEN; //query command length for modbus sensor, this value must <= 16 //add on 2017.3.6
    uint8_t modbusCMD[16]; //query command for modbus sensor //modify on 2017.3.6
	  //Jason add for second uart-rs485 sensor on 2018.11.15
	  uint8_t replyLEN2;  //length of reply data: this value must <= 64
    uint8_t startBYTE2; //start byte pos for parsing reply data: this value must <= 64
    uint8_t readLEN2;   //read bytes for parsing reply data: this value must <= 8
    uint8_t modbusCMDLEN2; //query command length for modbus sensor, this value must <= 16
    uint8_t modbusCMD2[16]; //query command for modbus sensor
	  //Jason add for third uart-rs485 sensor on 2018.10.24
	  uint8_t replyLEN3;  //length of reply data: this value must <= 64
    uint8_t startBYTE3; //start byte pos for parsing reply data: this value must <= 64
    uint8_t readLEN3;   //read bytes for parsing reply data: this value must <= 8
    uint8_t modbusCMDLEN3; //query command length for modbus sensor, this value must <= 16
    uint8_t modbusCMD3[16]; //query command for modbus sensor
} SensorConfig_t;

/*User Mode*/
ResultCode_t commandDTX(CommandLine_t *commandLine);
ResultCode_t commandDRX(CommandLine_t *commandLine);
ResultCode_t commandDTTX(CommandLine_t *commandLine);
ResultCode_t commandDRXI(CommandLine_t *commandLine);
ResultCode_t commandCSYNC(CommandLine_t *commandLine);
ResultCode_t commandCAPORT(CommandLine_t *commandLine);
ResultCode_t commandCBAP(CommandLine_t *commandLine);
ResultCode_t commandCSF(CommandLine_t *commandLine);
ResultCode_t commandCSID(CommandLine_t *commandLine);
ResultCode_t commandCPIN(CommandLine_t *commandLine);
ResultCode_t commandCSQ(CommandLine_t *commandLine);
ResultCode_t commandCRPTM(CommandLine_t *commandLine);
ResultCode_t commandSLMR(CommandLine_t *commandLine);
ResultCode_t commandSGMR(CommandLine_t *commandLine);
ResultCode_t commandSGMI(CommandLine_t *commandLine);
ResultCode_t commandSGMM(CommandLine_t *commandLine);
ResultCode_t commandSGMD(CommandLine_t *commandLine);
ResultCode_t commandSTIMER(CommandLine_t *commandLine);
ResultCode_t commandSTIMER1(CommandLine_t *commandLine);
ResultCode_t commandSIRQ(CommandLine_t *commandLine);
ResultCode_t commandSGPIO(CommandLine_t *commandLine);
ResultCode_t commandIBR(CommandLine_t *commandLine);
ResultCode_t commandECHO(CommandLine_t *commandLine);
//Shift command from admin mode to end user mode
ResultCode_t commandCTXP(CommandLine_t *commandLine);
ResultCode_t commandCKEY(CommandLine_t *commandLine);
ResultCode_t commandCMAC(CommandLine_t *commandLine);
ResultCode_t commandCCH(CommandLine_t *commandLine);
ResultCode_t commandCCHO(CommandLine_t *commandLine);
ResultCode_t commandCRXC(CommandLine_t *commandLine);
#ifdef ANT_PA
ResultCode_t commandCPAE(CommandLine_t *commandLine);
#endif
//End Shift

ResultCode_t commandSPROFILE(CommandLine_t *commandLine);  //daniel add on 2017.1.12 new command for set sensor profile
ResultCode_t commandTESTPOWER(CommandLine_t *commandLine); //Jaadd on 2018.6.7 new command for test power
ResultCode_t commandTESTBAT(CommandLine_t *commandLine);   //Jason add on 2018.6.11 new command for test battery
ResultCode_t commandTESTPINC(CommandLine_t *commandLine);   //Jason add on 2018.6.11 new command for test battery
ResultCode_t commandUPBACKPINC(CommandLine_t *commandLine);   //Jason add on 2018.6.11 new command for up feed back pin setting
ResultCode_t commandDOWNBACKPINC(CommandLine_t *commandLine);   //Jason add on 2018.6.11 new command for down feed back pin setting
ResultCode_t commandSDEVICETYPE(CommandLine_t *commandLine); //Jason add on 2019.01.29 new command for switch sensor hub and control box

/*Eng Mode*/
ResultCode_t commandGADM(CommandLine_t *commandLine);
ResultCode_t commandGSID(CommandLine_t *commandLine);
ResultCode_t commandGKEY(CommandLine_t *commandLine);
ResultCode_t commandGPIN(CommandLine_t *commandLine);
ResultCode_t commandGTXD(CommandLine_t *commandLine);
ResultCode_t commandGRST(CommandLine_t *commandLine);
ResultCode_t commandGCH(CommandLine_t *commandLine);
ResultCode_t commandGCHO(CommandLine_t *commandLine);
ResultCode_t commandGGMD(CommandLine_t *commandLine);
ResultCode_t commandGPT(CommandLine_t *commandLine);
ResultCode_t commandGLMR(CommandLine_t *commandLine);
ResultCode_t commandIoTest(CommandLine_t *commandLine);
ResultCode_t commandCaliRSSI(CommandLine_t *commandLine);
ResultCode_t commandGMTXP(CommandLine_t *commandLine);
ResultCode_t commandGSYSC(CommandLine_t *commandLine);
ResultCode_t commandSPWMOD(CommandLine_t *commandLine);
ResultCode_t commandGCPW(CommandLine_t *commandLine);
ResultCode_t commandGUUID(CommandLine_t *commandLine);
ResultCode_t commandGRXC(CommandLine_t *commandLine);
ResultCode_t commandCCLASS(CommandLine_t *commandLine);


ResultCode_t commandCNWKSKEY(CommandLine_t *commandLine);
ResultCode_t commandCAPPSKEY(CommandLine_t *commandLine);
ResultCode_t commandCAPPKEY(CommandLine_t *commandLine);
ResultCode_t commandCDEVEUI(CommandLine_t *commandLine);
ResultCode_t commandCAPPEUI(CommandLine_t *commandLine);

uint8_t InitAtCommandList(void);
void RestoreUserConfigToDefault(void);
uint8_t SaveConfigToEEPROM(void);
/* Modify by Eric Date: 2017/02/17   Log: Fix Load Configration from EEPROM issue. */
uint8_t LoadNodeConfigFromEEPROM(void);
uint8_t LoadAtConfigFromEEPROM(void);
uint8_t LoadEngConfigFromEEPROM(void);
uint8_t LoadUserConfigFromEEPROM(void);
uint8_t LoadRxConfigFromEEPROM(void);
uint8_t LoadOthersConfigFromEEPROM(void);
uint8_t LoadConfigFromEEPROM(void);
uint8_t LoadLoRaWanFromEEPROM(void);

void StartRadioTxDelay(void);
void StopRadioTxDelayTimer(void);
uint8_t ReceiveRadioRxData(RadioRxData_t* rxData);
uint8_t ReadGpioStatus(uint8_t* pinName);
uint8_t GetEEPROMConfig(AtConfig_t* atConfig, UserConfig1_t* userConfig1);//Add on 2018.11.15
uint32_t GetTxDelayTime(void);
uint8_t GetReporterMode(void);
uint8_t GetLowPowerModeFlag(uint8_t* enterLPMTime);
void RadioTxLED(uint8_t onTx);
uint8_t GetPinVoltage( uint32_t channel );
void InitATInterrupt(uint8_t *pinName, GpioIrqHandler *irqHandler);
uint8_t GetSensorConfig(SensorConfig_t* senConfig); //daniel add on 2017.2.2
AtConfig_t GetAtConfig(void); //daniel add on 2017.3.28 for return AtConfig

/*For Production Test*/
void SetTXPPFlag(uint8_t flag);

extern NodeConfig_t NodeConfig;
extern AtConfig_t AtConfig;
extern EngConfig1_t EngConfig1;
extern UserConfig1_t UserConfig1;
extern RxConfig_t RxConfig;
extern OthersConfig_t OthersConfig;
extern LoRaWanConfig_t LoRaWanConfig;
extern SensorConfig_t SensorConfig; //daniel add on 2017.12.25 for get sensor profile

#endif

