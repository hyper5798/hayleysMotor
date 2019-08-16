#ifndef __TRACKER_COMMON_H__
#define __TRACKER_COMMON_H__

#ifdef __cplusplus
extern "C" {
#endif

#ifndef OK
    #define OK                      1
#endif

#ifndef FAIL
    #define FAIL                    0
#endif

// 512-Byte for both Config and Profile
#define DATA_EEPROM_START_ADDR      0x08080000
#define DATA_EEPROM_END_ADDR        0x080803FF
#define CONFIG_ADD_OFFSET           0X30
#define PROFILE_ADD_OFFSET          0X230
#define TEMP_CAL_ADD_OFFSET         0X300
#define CHANNEL_INFO_OFFSET         0X100
#define EEPROM_CONFIG_ADDR        ( DATA_EEPROM_START_ADDR + CONFIG_ADD_OFFSET )
#define EEPROM_PROFILE_ADDR       ( DATA_EEPROM_START_ADDR + PROFILE_ADD_OFFSET )
#define EEPROM_TEMP_CAL_ADDR      ( DATA_EEPROM_START_ADDR + TEMP_CAL_ADD_OFFSET )
#define EEPROM_CHANNEL_INFO_ADDR  ( DATA_EEPROM_START_ADDR + CHANNEL_INFO_OFFSET )
#define EEPROM_ERROR_CODE_ADDR    ( DATA_EEPROM_START_ADDR + 0x20 )

#define UART1_TX_BUFF_LEN           256
#define OTA_STATUS_STRING_TRUE      "true"
#define OTA_STATUS_STRING_FALSE     "false"
#define STATUS_STRING_SUCCESS       "success"
#define STATUS_STRING_FAIL          "fail"
#define OTA_STATUS_STRING_SUCCESS   STATUS_STRING_SUCCESS
#define OTA_STATUS_STRING_FAIL      STATUS_STRING_FAIL

#define LORA_F_PORT             3
#define LORA_RETTRIES           0

enum {
    OTA_STATUS_NORMAL  = 0,   //normal boot
    OTA_STATUS_EEPROM     ,   //start to recieve fw
    OTA_STATUS_FLASH      ,   //start to program flash
    OTA_STATUS_ENTER_BLE  ,   //enter BLE state directly after boot (skip normal state)
    
    OTA_RET_CODE_100   = 100, //ota state ready
    
    OTA_RET_CODE_101   = 101, //The model_id is different, stop ota
    OTA_RET_CODE_102        , //The fw version of Node >= the version in JSON string, stop ota
    OTA_RET_CODE_103        , //JSON parse error, resource not ready or too busy
    OTA_RET_CODE_104        , //unknow json string
    
    OTA_RET_CODE_110   = 110, //start receive fw
    
    OTA_RET_CODE_201   = 201, //Fw sending timeout, retry
    OTA_RET_CODE_202        , //CRC check error, retry
    OTA_RET_CODE_203        , //EEPROM write error, stop ota
    OTA_RET_CODE_204        , //Retry too many times, stop ota
    
    OTA_RET_CODE_210   = 210, //Get next piece of fw
    
    OTA_RET_CODE_300   = 300, //Start programming flash
    OTA_RET_CODE_301        , //Program flash error, stop ota
};

enum {
    BLE_CMD_TYPE_UPDATE_BEACON = 1, //update BT beacon info
};

enum ConfigItemLength
{
    UPGRADE_VERSION_LEN   = 10,
    VERSION_LEN           = 10,
    SERIAL_NUM_LEN        = 16,
    MODEL_ID_LEN          = 20,
    MAC_ADDR_LEN          = 12,
    SYSTEM_ID_LEN         = 20,
    FW_STATUS_FLAG_LEN    =  1,
    DEBUG_MODE_LEN        =  1,
    BIND_STATUS_LEN       =  1,
    NWK_SESSION_KEY_LEN   = 33,
    APP_SESSION_KEY_LEN   = 33,
    BATT_CAL_LEN          =  1,
};

typedef struct NodeConfig
{
    uint8_t UpgradeVersion[UPGRADE_VERSION_LEN];
    uint8_t Version[VERSION_LEN];
    uint8_t Sn[SERIAL_NUM_LEN];
    uint8_t ModelId[MODEL_ID_LEN];
    uint8_t MacAddr[MAC_ADDR_LEN];
    uint8_t SystemId[SYSTEM_ID_LEN];
    uint8_t FwStatusFlag;
    uint8_t DebugMode;          // enable level: 1 or 2, disable:0
    uint8_t BindStatus;         // fail:0, success:1
    uint8_t NwkSessionKey[NWK_SESSION_KEY_LEN];
    uint8_t AppSessionKey[APP_SESSION_KEY_LEN];
    uint16_t BatteryCal;
    uint8_t LedDebugMode; /* Modified by Crux, Date: 2017/08/24, Log: Add LED debug flag. */
}NodeConfig_t;

typedef struct NodeProfile
{
    uint8_t EnableGps;                  // Enable : 1, Disable : 0
    uint8_t EnableGSensor;              // Enable : 1, Disable : 0
    int8_t  TemperatureToAlarm;         // Temperature ( in Celsius ) that will trigger the alarm
    uint8_t EnableTemperatureAlarm;     // Enable : 1, Disable : 0
    uint8_t ReportPeriod;               // Report Period in minute
    uint8_t ProfileModeNum;             // Profile Mode
    float   GSenSensity;                // G-Sensor Sensitivity (range: 0.0625~1)
    
}NodeProfile_t;

typedef struct ChannelInfo
{
    uint32_t A1;
    uint32_t A2;
    uint32_t B1;
    uint32_t B2;
    uint32_t Offset1;
    uint32_t Offset2;
    uint32_t Conutry;
    
    uint8_t  TxPower;
    uint8_t  Datarate;
    uint16_t chMask;
    uint8_t  enDutyCycle;
}ChannelInfo_t;

#ifdef __cplusplus
}
#endif

#endif /*__TRACKER_COMMON_H__*/
