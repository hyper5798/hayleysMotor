#ifndef __PRODUCTION_H__
#define __PRODUCTION_H__

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#ifdef TRACKER_BOARD
#include <tracker_common.h>
#endif
/*these value should never be changed for Tx continuous mode*/
#define BAND_434MHZ_MIN             421
#define BAND_434MHZ_MAX             447
#define BAND_470MHZ_MIN             457
#define BAND_470MHZ_MAX             510
#define BAND_868MHZ_MIN             855
#define BAND_868MHZ_MAX             881
#define BAND_915MHZ_MIN             902
#define BAND_915MHZ_MAX             928
#define FREQ_MAX_VAL                3
#define TXPWR_MAX_VAL               20
#define	BWD_MAX_VAL                 2
#define	SP_FACTOR_MAX_VAL           6
#define CR_MAX_VAL                  4
#define PRE_LEN_MAX_VAL             255 //TBD
#define FIX_LEN_PAYLOAD_MAX_VAL     1
#define IQ_INVER_MAX_VAL            1
#define EN_CRC_MAX_VAL              1
#define MAX_PING_TX_RX_VAL          9999
#define MAX_PING_SEC                999
#define MIN_PING_VAL                1

#define TXPP_MASTER     1
#define TXPP_SLAVE      0


typedef enum {
    PROD_TEST_UART_START = 0,
    PROD_TEST_TXCM_START,
    PROD_TEST_TXPP_START,
    PROD_TEST_TXNM_START,
    PROD_TEST_RX,
    PROD_TEST_LNA_READ,
    PROD_TEST_LNA_BOOST,
    PROD_TEST_VER,
    PROD_TEST_MAC_READ,
    PROD_TEST_MAC_WRITE,
    PROD_TEST_SNUM_READ,
    PROD_TEST_SNUM_WRITE,
    PROD_TEST_SX1276_READ,
    PROD_TEST_SX1276_WRITE,
    PROD_TEST_RSSI_START,
    PROD_TEST_LED,
    PROD_TEST_HELP,
    PROD_TEST_QUIT,
    PROD_TEST_ENTER,
    PROD_TEST_EXIT,
    PROD_TEST_CONFIG_READ,
    PROD_TEST_CONFIG_WRITE,
    PROD_TEST_PROFILE_READ,
    PROD_TEST_PROFILE_WRITE,
    PROD_TEST_MODID_READ,
    PROD_TEST_MODID_WRITE,
    PROD_TEST_SYSID_READ,
    PROD_TEST_SYSID_WRITE,
    PROD_TEST_NWKSKEY_READ,
    PROD_TEST_NWKSKEY_WRITE,
    PROD_TEST_APPSKEY_READ,
    PROD_TEST_APPSKEY_WRITE,
    PROD_TEST_GSENSOR_START,
    PROD_TEST_GSENSOR_READ,
    PROD_TEST_TEMPERATURE_START,
    PROD_TEST_TEMPERATURE_READ,
    PROD_TEST_TEMPERATURE_CAL_READ,
    PROD_TEST_TEMPERATURE_CAL_WRITE,
    PROD_TEST_ICTEMPERATURE,
    PROD_TEST_GPS_START,
    PROD_TEST_FAC_READ,
    PROD_TEST_FAC_WRITE,
    //PROD_TEST_UPDATE_CONFIG,
    PROD_TEST_CAP_TEST_READ,
    PROD_TEST_CAP_TEST_WRITE,
    PROD_TEST_BAT_READ,
    PROD_TEST_BAT_CAL_START,
    PROD_TEST_BAT_CAL_READ,
    PROD_TEST_BAT_CAL_WRITE,
    PROD_TEST_BUZZ,
    PROD_TEST_DEBUG_READ,
    PROD_TEST_DEBUG_WRITE,
    PROD_TEST_FREQ_READ,
    PROD_TEST_FREQ_WRITE,
    PROD_TEST_EXT_EEPROM,
    PROD_TEST_LOW_POWER,
    PROD_TEST_HARDFAULT_READ,
    PROD_TEST_HARDFAULT_WRITE,
    PROD_TEST_FW_COUNTRY,
    PROD_TEST_LORA_RESET,
    PROD_TEST_APP_EUI_READ,
    PROD_TEST_APP_EUI_WRITE,
    PROD_TEST_DEV_EUI_READ,
    PROD_TEST_DEV_EUI_WRITE,
    PROD_TEST_APP_KEY_READ,
    PROD_TEST_APP_KEY_WRITE,
    PROD_TEST_LORA_CONFIGURATION,
    PROD_TEST_PROV_READ,
    PROD_TEST_PROV_WRITE,
    PROD_TEST_CAD_INFO_READ,
    PROD_TEST_CAD_INFO_WRITE,

    PROD_TEST_FAIL,
    PROD_TEST_SUCCESS,
}NodeTest;

/*tx continuous mode*/
typedef struct
{
    char LoRaFreq[8];
    char LoRaTxPwr[3];                  // Valid Power value in dBm for Tx = 02~20 and Rx should always fill with 00
    char LoRaBwd[2];                    // 0: 125kHz, 1: 250kHz, 2: 500kHz
    char LoRaSpFactor[2];               // 0: SF6 1: SF7, 2: SF8, 3: SF9, 4: SF10, 5: SF11, 6: SF12
    char LoRaCodeRate[2];               // 1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8
    char LoRaPreambleLen[2];            // Default
    char LoRaFixLenPayloadOn[2];        // 0: false, 1: true
    char LoRaIqInversionOn[2];          // 0: false, 1: true
    char LoRaEnableCrc[2];              // 0: false, 1: true
}txcm_t;

/*tx ping pong mode*/
typedef struct
{
    char PingPongMaster[2];             // 0: Slave, 1: Master
    char PingPongMaxTxNumOrRxTime[5];   // Num from 0000 ~ 9999	times or in seconds
    char PingPongTxOrRxTimeout[5];      // Num from 0000 ~ 9999	in milliseconds
    char Label[5];
}txpp_t;
//typedef struct
//{
//    // Frequencies from 902.000 to 928.000, 855.000 to 881.000, 457.000 to 483.000, 421.000 to 447.000
//    char LoRaFreq[8];													
//    char LoRaTxPwr[3];												// Valid Power value in dBm for Tx = 02~20 and Rx should always fill with 00
//    char LoRaBwd[2];													// 0: 125kHz, 1: 250kHz, 2: 500kHz
//    char LoRaSpFactor[2];											// 0: SF6 1: SF7, 2: SF8, 3: SF9, 4: SF10, 5: SF11, 6: SF12
//    char LoRaCodeRate[2];											// 1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8
//    char LoRaPreambleLen[2];									// Default
//    char LoRaFixLenPayloadOn[2];							// 0: false, 1: true
//    char LoRaIqInversionOn[2];								// 0: false, 1: true
//    char LoRaEnableCrc[2];										// 0: false, 1: true
//}txpp_t;

NodeTest checkCmd(uint8_t *string);
NodeTest parseTXCMdata(int8_t *string, txcm_t *data);
NodeTest parseTXPPdata(int8_t *string, txpp_t *data,txcm_t *txcmdata);

NodeTest isLoRaMacValid(uint8_t *mac_str);   //check LoRa MAC format
NodeTest isSnValid(uint8_t *sn_str);         //check SN format
NodeTest isAesKeyValid(uint8_t *sn_str);     //check AES key

typedef struct
{
    uint8_t argc;
    uint8_t argv_ptr[16];
    uint8_t cmd[256];
}NodeTestCmdParseBuff_t;

/*!
 * MFT command parser.
 * Author: Crux
 *
 * example:
 *
 * uint8_t cmd[] = "NSKEY,W,aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
 * NodeTestCmdParseBuff_t parse_buff;
 * uint8_t ret;
 * 
 * ret = parseTestCmd(cmd, &parse_buff);
 * 
 * //result:
 * // cmd = NSKEY,W,aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
 * // parse_buff.argc = 3
 * // (parse_buff.cmd + parse_buff.argv_ptr[0]): NSKEY
 * // (parse_buff.cmd + parse_buff.argv_ptr[1]): W
 * // (parse_buff.cmd + parse_buff.argv_ptr[2]): aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
 *
 * \param [IN] cmd      input command
 * \param [OUT] buff    parse result
 * \param [return]      PROD_TEST_FAIL or PROD_TEST_SUCCESS
 */
NodeTest parseTestCmd(uint8_t *cmd, NodeTestCmdParseBuff_t *buff);

#ifdef __cplusplus
}
#endif
#endif /*__PRODUCTION_H__*/
