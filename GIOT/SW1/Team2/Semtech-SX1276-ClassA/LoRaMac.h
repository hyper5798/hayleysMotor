#ifndef __LORAMAC_H__
#define __LORAMAC_H__

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>

/*!
 * Beacon interval in ms
 */
#define BEACON_INTERVAL                             128000000

/*!
 * Maximum allowed gap for the FCNT field
 */
#define MAX_FCNT_GAP                                16384

/*!
 * ADR acknowledgement counter limit
 */
#define ADR_ACK_LIMIT                               64

/*!
 * Number of ADR acknowledgement requests before returning to default datarate
 */
#define ADR_ACK_DELAY


/*!
 * RSSI free threshold
 */
//#define RSSI_FREE_TH                                ( int8_t )( -97 ) // [dBm]
#define RSSI_FREE_TH                                ( int8_t )( -90 ) // [dBm]


/*!
 * Frame direction definition
 */
#define UP_LINK                                     0
#define DOWN_LINK                                   1

/*!
 * Sets the length of the LoRaMAC footer field.
 * Mainly indicates the MIC field length
 */
#define LORAMAC_MFR_LEN                             4

#define AES_128_KEY_SIZE							16


/*!
 * Radio wakeup time from SLEEP mode
 */
#define RADIO_WAKEUP_TIME                           1000 // [us]
#define MAX_RX_WINDOW1													1000000			// 1 [s]
/*!
 * Maximum PHY layer payload size
 */
#ifdef SIPMODULE_BOARD

#ifdef STM32L073xx
#define LORAMAC_PHY_MAXPAYLOAD                      256
#else
#define LORAMAC_PHY_MAXPAYLOAD                      160
#endif

#else
#define LORAMAC_PHY_MAXPAYLOAD                      64
#endif

#define LORA_PREAMBLE_LENGTH                        	8         // Same for Tx and Rx

#define LORA_SYMBOL_TIMEOUT                        		5         // Symbols

/*!
 * Minimal datarate that can be used by the node
 */
#define LORAMAC_MIN_DATARATE                        DR_SF10

/*!
 * Minimal datarate that can be used by the node
 */
#define LORAMAC_MAX_DATARATE                        DR_FSK

/*!
 * Minimal Tx output power that can be used by the node
 */
#define LORAMAC_MIN_TX_POWER                        TX_POWER_02_DBM

/*!
 * Minimal Tx output power that can be used by the node
 */
#define LORAMAC_MAX_TX_POWER                        TX_POWER_20_DBM

/*!
 * Returns individual channel mask
 *
 * \param[IN] channelIndex Channel index 1 based
 * \retval channelMask
 */
#define LC( channelIndex )            ( uint16_t )( 1 << ( channelIndex - 1 ) )

/*!
 * LoRaMac TxPower definition
 */
#define TX_POWER_20_DBM                             0
#define TX_POWER_14_DBM                             1
#define TX_POWER_11_DBM                             2
#define TX_POWER_08_DBM                             3
#define TX_POWER_05_DBM                             4
#define TX_POWER_02_DBM                             5

/*!
 * LoRaMac datarates definition
 */
#define DR_SF12                                     0
#define DR_SF11                                     1
#define DR_SF10                                     2
#define DR_SF9                                      3
#define DR_SF8                                      4
#define DR_SF7                                      5
#define DR_SF7H                                     6
#define DR_FSK                                      7

/*!
 * LoRaMac default channels definition
 */

#define LORA_BW_125													0
#define LORA_BW_250													1
#define LORA_BW_500													2


extern const uint8_t Datarates[];
/*!
 * LoRaMAC channels parameters definition
 */
typedef union
{
    int8_t Value;
    struct
    {
        int8_t Min : 4;
        int8_t Max : 4;
    }__attribute__((__packed__)) Fields;
}__attribute__((__packed__)) DrRange_t;

typedef struct
{
    uint32_t Frequency; // Hz
    DrRange_t DrRange;  // Max datarate [0: SF12, 1: SF11, 2: SF10, 3: SF9, 4: SF8, 5: SF7, 6: SF7, 7: FSK]
                        // Min datarate [0: SF12, 1: SF11, 2: SF10, 3: SF9, 4: SF8, 5: SF7, 6: SF7, 7: FSK]
    int8_t DutyCycle;   // 0 = 100% .. 15 = 0.003%
}__attribute__((__packed__)) ChannelParams_t;

typedef struct
{
    uint32_t Frequency; // Hz
    uint8_t  Datarate; // [0: SF12, 1: SF11, 2: SF10, 3: SF9, 4: SF8, 5: SF7, 6: SF7, 7: FSK]
}__attribute__((__packed__)) Rx2ChannelParams_t;

/*!
 * LoRaMAC frame types
 */
typedef enum
{
    FRAME_TYPE_JOIN_REQ              = 0x00,
    FRAME_TYPE_JOIN_ACCEPT           = 0x01,
    FRAME_TYPE_DATA_UNCONFIRMED_UP   = 0x02,
    FRAME_TYPE_DATA_UNCONFIRMED_DOWN = 0x03,
    FRAME_TYPE_DATA_CONFIRMED_UP     = 0x04,
    FRAME_TYPE_DATA_CONFIRMED_DOWN   = 0x05,
    FRAME_TYPE_RFU                   = 0x06,
    FRAME_TYPE_PROPRIETARY           = 0x07,
}__attribute__((__packed__)) LoRaMacFrameType_t;

/*!
 * LoRaMAC mote MAC commands
 */
typedef enum
{
    MOTE_MAC_LINK_CHECK_REQ          = 0x02,
    MOTE_MAC_LINK_ADR_ANS            = 0x03,
    MOTE_MAC_DUTY_CYCLE_ANS          = 0x04,
    MOTE_MAC_RX2_SETUP_ANS           = 0x05,
    MOTE_MAC_DEV_STATUS_ANS          = 0x06,
    MOTE_MAC_NEW_CHANNEL_ANS         = 0x07,
}__attribute__((__packed__)) LoRaMacMoteCmd_t;

/*!
 * LoRaMAC server MAC commands
 */
typedef enum
{
    SRV_MAC_LINK_CHECK_ANS           = 0x02,
    SRV_MAC_LINK_ADR_REQ             = 0x03,
    SRV_MAC_DUTY_CYCLE_REQ           = 0x04,
    SRV_MAC_RX2_SETUP_REQ            = 0x05,
    SRV_MAC_DEV_STATUS_REQ           = 0x06,
    SRV_MAC_NEW_CHANNEL_REQ          = 0x07,
}__attribute__((__packed__)) LoRaMacSrvCmd_t;

/*!
 * LoRaMAC Battery level indicator
 */
typedef enum
{
    BAT_LEVEL_EXT_SRC                = 0x00,
    BAT_LEVEL_EMPTY                  = 0x01,
    BAT_LEVEL_FULL                   = 0xFE,
    BAT_LEVEL_NO_MEASURE             = 0xFF,
}__attribute__((__packed__)) LoRaMacBatteryLevel_t;

/*!
 * LoRaMAC header field definition
 */
typedef union
{
    uint8_t Value;
    struct
    {
        uint8_t Major           : 2;
        uint8_t RFU             : 3;
        uint8_t MType           : 3;
    }__attribute__((__packed__)) Bits;
}__attribute__((__packed__)) LoRaMacHeader_t;

/*!
 * LoRaMAC frame header field definition
 */
typedef union
{
    uint8_t Value;
    struct
    {
        uint8_t OptionsLength   : 4;
        uint8_t FPending        : 1;
        uint8_t Ack             : 1;
        uint8_t AdrAckReq       : 1;
        uint8_t Adr             : 1;
    }__attribute__((__packed__)) Bits;
}__attribute__((__packed__)) LoRaMacFrameCtrl_t;

/*!
 * LoRaMAC event flags
 */
typedef union
{
    uint8_t Value;
    struct
    {
        uint8_t Tx              : 1;
        uint8_t Rx              : 1;
        uint8_t LinkCheck       : 1;
        uint8_t                 : 4;
        uint8_t JoinAccept      : 1;
    }__attribute__((__packed__)) Bits;
}__attribute__((__packed__)) LoRaMacEventFlags_t;

typedef enum
{
    LORAMAC_EVENT_INFO_STATUS_OK = 0,
    LORAMAC_EVENT_INFO_STATUS_ERROR,
}__attribute__((__packed__)) LoRaMacEventInfoStatus_t;

/*!
 * LoRaMAC event information
 */
typedef struct
{
    LoRaMacEventInfoStatus_t Status;
    bool TxAckReceived;
    uint8_t TxNbRetries;
    uint8_t TxDatarate;
    uint8_t RxPort;
    uint8_t *RxBuffer;
    uint8_t RxBufferSize;
    int16_t RxRssi;
    uint8_t RxSnr;
    uint16_t Energy;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}__attribute__((__packed__)) LoRaMacEventInfo_t;

#if defined(SIPMODULE_BOARD )

typedef struct
{
    int32_t RxWindow1Offset;
    int32_t RxWindow2Offset;
    uint32_t RxWindow1Timeout;
    uint32_t RxWindow2Timeout;
}RxWindowParams_t;

uint8_t ComputeRxWindowParameters(RxWindowParams_t* rxWindowParams);

#endif

void SetRadioDefaultPower(int8_t power);
int8_t GetRadioDefaultPower(void);
void SetRadioDatarate(int8_t datarate);
int8_t GetRadioDatarate(void);
void SetRadioRxChannel(Rx2ChannelParams_t parameter);
void SetRadioRxBandwidth(uint8_t bandwidth);
void SetRadioChannel(uint32_t FreqA1, uint32_t FreqA2, uint32_t FreqB1, uint32_t FreqB2);
void SetRadioChannelOnOffset(uint32_t FreqA1, uint32_t FreqA2, uint32_t FreqB1, uint32_t FreqB2, uint32_t offset1, uint32_t offset2);
void LoRaMacInitNwkIds( uint32_t netID, uint32_t devAddr, uint8_t *nwkSKey, uint8_t *appSKey );
uint8_t LoRaMacSetNextChannel( ChannelParams_t *NowChannel);
//void LoRaMacReceiveFrameOnChannel( ChannelParams_t channel );
uint8_t LoRaMacSendFrameOnChannel( ChannelParams_t channel );
uint8_t LoRaMacPrepareFrame( LoRaMacHeader_t *macHdr, LoRaMacFrameCtrl_t *fCtrl, uint8_t *fOpts, uint8_t fPort, void *fBuffer, uint16_t fBufferSize );
uint8_t LoRaMacParseFrame( uint8_t *payload, uint16_t size, uint8_t *data, uint16_t *datasize);
void LoRaMacRxWindowSetup( uint32_t freq, int8_t datarate, uint32_t bandwidth, uint16_t timeout, bool rxContinuous );
void LoRaMacRxWindown2Setup(bool rxContinuous);
uint32_t LoRaGetUpLinkCounter(void);
void LoRaSetUpLinkCounter(uint32_t upLinkCounter);
void RadioCadInit(uint32_t freq, uint8_t bandwidth, uint8_t dataRate, uint16_t preambleLen,uint16_t symbolTimeout, uint32_t rxTimeout);
uint8_t  GetCadInit (void);
uint32_t GetCadFrequency (void);
uint8_t  GetCadBandwidth (void);
uint8_t  GetCadDatarate (void);
uint16_t GetCadPreambleLen (void);
uint16_t GetCadSymbolTimeout (void);
uint16_t GetCadCadRxTimeout (void);

#ifdef __cplusplus
}
#endif

#endif // _LORAMAC_H__
