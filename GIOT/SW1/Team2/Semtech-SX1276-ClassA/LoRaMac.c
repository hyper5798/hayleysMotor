#include <string.h>
#include "LoRaMac.h"
#include "RadioChannel.h"
#include "sx1276.h"
#include "LoRaMacCrypto.h"
#include "utilities.h"
#include "math.h"

#define MAX_RX_WINDOW                               3000000

#ifdef SIPMODULE_G78S
#define CN470_FIRST_RX1_CHANNEL                     ( (uint32_t) 500.3e6 )
#define CN470_STEPWIDTH_RX1_CHANNEL                 ( (uint32_t) 200e3 )
#endif

#ifdef BOPI_CN470
#define CN470_FIRST_TX_CHANNEL                      ( (uint32_t) 470.3e6 )
#define CN470_FIRST_RX1_CHANNEL                     ( (uint32_t) 500.3e6 )
#define CN470_STEPWIDTH_RX1_CHANNEL                 ( (uint32_t) 200e3 )
#endif


/*!
 * Device IEEE EUI
 */
static uint8_t *LoRaMacDevEui;

/*!
 * Application IEEE EUI
 */
static uint8_t *LoRaMacAppEui;

/*!
 * AES encryption/decryption cipher application key
 */
static uint8_t *LoRaMacAppKey;

/*!
 * AES encryption/decryption cipher network session key
 */
static uint8_t LoRaMacNwkSKey[] =
{
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

/*!
 * AES encryption/decryption cipher application session key
 */
static uint8_t LoRaMacAppSKey[] =
{
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

/*!
 * Device nonce is a random value extracted by issuing a sequence of RSSI
 * measurements
 */
static uint16_t LoRaMacDevNonce;

/*!
 * Network ID ( 3 bytes )
 */
//static uint32_t LoRaMacNetID;

/*!
 * Mote Address
 */
static uint32_t LoRaMacDevAddr;

/*!
 * Buffer containing the data to be sent or received.
 */
static uint8_t LoRaMacBuffer[LORAMAC_PHY_MAXPAYLOAD];

/*!
 * Length of packet in LoRaMacBuffer
 */
static uint16_t LoRaMacBufferPktLen = 0;

/*!
 * Buffer containing the upper layer data.
 */
static uint8_t LoRaMacPayload[LORAMAC_PHY_MAXPAYLOAD];

/*!
 * Data rates table definition
 */
const uint8_t Datarates[]  = { 12, 11, 10,  9,  8,  7,  7, 50 };

/*!
 * Tx output powers table definition
 */
const int8_t TxPowers[]    = { 20, 14, 11,  8,  5,  2 };

/*------------------------------------------------------------------------------*/
/*!
 * Current channel index
 */
static uint8_t Channel;

/*!
 * Indicates if the MAC layer has already joined a network.
 */
//static bool IsLoRaMacNetworkJoined = false;

/*!
 * Indicates if the MAC layer is already transmitting a frame.
 */
//static bool IsLoRaMacTransmitting = false;

/*!
 * LoRaMac
 */
//LoRaMacEventFlags_t LoRaMacEventFlags;

/*!
 * Channels Tx output power
 */
#ifndef LORAMAC_DEFAULT_TX_POWER
#define LORAMAC_DEFAULT_TX_POWER                    TX_POWER_14_DBM
#endif
static int8_t ChannelsTxPower = LORAMAC_DEFAULT_TX_POWER;

/*!
 * Channels datarate
 */
#ifndef LORAMAC_DEFAULT_DATARATE
#define LORAMAC_DEFAULT_DATARATE                    DR_SF10
#endif
static int8_t ChannelsDatarate = LORAMAC_DEFAULT_DATARATE;

/*!
 * LoRaMAC 2nd reception window settings
 */
#ifndef RX_WND_2_CHANNEL
#define RX_WND_2_CHANNEL                                  { ( MODULE_B_PLL_2 ), DR_SF10 }
#endif
static Rx2ChannelParams_t Rx2Channel = RX_WND_2_CHANNEL;

#ifdef BOPI_CN470
static uint8_t Rx2Bandwidth = CONFIRMED_MSG_BW;
#else
static uint8_t Rx2Bandwidth = UNCONFIRMED_MSG_BW;
#endif

static uint32_t UpLinkCounter = 1;

/* Modified by Griffith Date: 2017/07/25   Log: CAD related variables*/
static uint8_t  CadInit       = 0;
static uint32_t CadFrequency  = 0;
static uint8_t  CadBandwidth  = 0;
static uint8_t  CadDatarate   = 0;
static uint16_t PreambleLen   = 0;
static uint16_t SymbolTimeout = 0;
static uint32_t CadRxTimeout  = 0;
/*!
 * LoRaMAC frame counter. Each time a packet is received the counter is incremented.
 * Only the 16 LSB bits are received
 */
//static uint32_t DownLinkCounter = 0;


#if defined(SIPMODULE_BOARD )

#define LoRaMacParamsDefaults_SystemMaxRxError 10
#define LoRaMacParamsDefaults_MinRxSymbols  6
uint32_t RxWindow2Timeout = 0;

double RegionCommonComputeSymbolTimeLoRa( uint8_t phyDr, uint32_t bandwidth )
{
    return ( ( double )( 1 << phyDr ) / ( double )bandwidth ) * 1e3;
}

double RegionCommonComputeSymbolTimeFsk( uint8_t phyDr )
{
    return ( 8.0 / ( double )phyDr ); // 1 symbol equals 1 byte
}

void RegionCommonComputeRxWindowParameters( double tSymbol, uint8_t minRxSymbols, uint32_t rxError, uint32_t wakeUpTime, uint32_t* windowTimeout, int32_t* windowOffset )
{
    *windowTimeout = MAX( ( uint32_t )ceil( ( ( 2 * minRxSymbols - 8 ) * tSymbol + 2 * rxError ) / tSymbol ), minRxSymbols ); // Computed number of symbols
    *windowOffset = ( int32_t )ceil( ( 4.0 * tSymbol ) - ( ( *windowTimeout * tSymbol ) / 2.0 ) - wakeUpTime );
}

uint8_t ComputeRxWindowParameters(RxWindowParams_t* rxWindowParams)
{
    double tSymbol = 0.0;
    uint32_t bw = 125000;

    //Calculus Rx1 symbol timout and Rx offset
    if( ChannelsDatarate == DR_FSK )
    { // FSK
        tSymbol = RegionCommonComputeSymbolTimeFsk( Datarates[ChannelsDatarate] );
    }
    else
    { // LoRa
        tSymbol = RegionCommonComputeSymbolTimeLoRa( Datarates[ChannelsDatarate], bw );
    }

    RegionCommonComputeRxWindowParameters( tSymbol, LoRaMacParamsDefaults_MinRxSymbols, LoRaMacParamsDefaults_SystemMaxRxError, RADIO_WAKEUP_TIME/1000
        , &rxWindowParams->RxWindow1Timeout, &rxWindowParams->RxWindow1Offset);


    //Calculus Rx2 symbol timout and Rx offset
    if(Rx2Bandwidth == 0)
    {
        bw = 125000;
    }
    else if(Rx2Bandwidth == 1)
    {
        bw = 250000;
    }
    else
    {
        bw = 500000;
    }

    if( Rx2Channel.Datarate == DR_FSK )
    { // FSK
        tSymbol = RegionCommonComputeSymbolTimeFsk( Datarates[Rx2Channel.Datarate] );
    }
    else
    { // LoRa
        tSymbol = RegionCommonComputeSymbolTimeLoRa( Datarates[Rx2Channel.Datarate], bw);
    }

    RegionCommonComputeRxWindowParameters( tSymbol, LoRaMacParamsDefaults_MinRxSymbols, LoRaMacParamsDefaults_SystemMaxRxError, RADIO_WAKEUP_TIME/1000
        , &rxWindowParams->RxWindow2Timeout, &rxWindowParams->RxWindow2Offset);

    RxWindow2Timeout = rxWindowParams->RxWindow2Timeout;

    return 0;
}

#endif



void SetRadioDefaultPower(int8_t power)
{
    ChannelsTxPower = power;
}

/*!
 * Get now Tx power.
 * Author: Crux
 * \param [IN]      N/A
 * \param [OUT]     N/A
 * \param [return]  Tx power value
 */
int8_t GetRadioDefaultPower(void)
{
    return ChannelsTxPower;
}

void SetRadioDatarate(int8_t datarate)
{
    ChannelsDatarate = datarate;
}

int8_t GetRadioDatarate(void)
{
    return ChannelsDatarate;
}

void SetRadioRxChannel(Rx2ChannelParams_t parameter)
{
    Rx2Channel = parameter;
}

void SetRadioRxBandwidth(uint8_t bandwidth)
{
    Rx2Bandwidth = bandwidth;
}

void SetRadioChannel(uint32_t FreqA1, uint32_t FreqA2, uint32_t FreqB1, uint32_t FreqB2)
{
#if USE_8CH_MODULE_A
Channels[0].Frequency = ( FreqA1 - 375000 );
Channels[1].Frequency = ( FreqA1 - 125000 );
Channels[2].Frequency = ( FreqA1 + 125000 );
Channels[3].Frequency = ( FreqA1 + 375000 );
Channels[4].Frequency = ( FreqA2 - 375000 );
Channels[5].Frequency = ( FreqA2 - 125000 );
Channels[6].Frequency = ( FreqA2 + 125000 );
Channels[7].Frequency =  ( FreqA2 + 375000 );

#elif USE_8CH_MODULE_B
Channels[0].Frequency = ( FreqB1 - 375000 );
Channels[1].Frequency = ( FreqB1 - 125000 );
Channels[2].Frequency = ( FreqB1 + 125000 );
Channels[3].Frequency = ( FreqB1 + 375000 );
Channels[4].Frequency = ( FreqB2 - 375000 );
Channels[5].Frequency = ( FreqB2 - 125000 );
Channels[6].Frequency = ( FreqB2 + 125000 );
Channels[7].Frequency =  ( FreqB2 + 375000 );

#elif USE_16CH
#ifdef USE_JAPAN_CH
Channels[0].Frequency = ( FreqA1 - 300000 );
Channels[1].Frequency = ( FreqA1 - 100000 );
Channels[2].Frequency = ( FreqA1 + 100000 );
Channels[3].Frequency = ( FreqA1 + 300000 );
Channels[4].Frequency = ( FreqA2 - 300000 );
Channels[5].Frequency = ( FreqA2 - 100000 );
Channels[6].Frequency = ( FreqA2 + 100000 );
Channels[7].Frequency =  ( FreqA2 + 300000 );
Channels[8].Frequency = ( FreqB1 - 300000 );
Channels[9].Frequency = ( FreqB1 - 100000 );
Channels[10].Frequency = ( FreqB1 + 100000 );
Channels[11].Frequency = ( FreqB1 + 300000 );
Channels[12].Frequency = ( FreqB2 - 300000 );
Channels[13].Frequency = ( FreqB2 - 100000 );
Channels[14].Frequency = ( FreqB2 + 100000 );
Channels[15].Frequency =  ( FreqB2 + 300000 );
#else
Channels[0].Frequency = ( FreqA1 - 375000 );
Channels[1].Frequency = ( FreqA1 - 125000 );
Channels[2].Frequency = ( FreqA1 + 125000 );
Channels[3].Frequency = ( FreqA1 + 375000 );
Channels[4].Frequency = ( FreqA2 - 375000 );
Channels[5].Frequency = ( FreqA2 - 125000 );
Channels[6].Frequency = ( FreqA2 + 125000 );
Channels[7].Frequency =  ( FreqA2 + 375000 );
Channels[8].Frequency = ( FreqB1 - 375000 );
Channels[9].Frequency = ( FreqB1 - 125000 );
Channels[10].Frequency = ( FreqB1 + 125000 );
Channels[11].Frequency = ( FreqB1 + 375000 );
Channels[12].Frequency = ( FreqB2 - 375000 );
Channels[13].Frequency = ( FreqB2 - 125000 );
Channels[14].Frequency = ( FreqB2 + 125000 );
Channels[15].Frequency =  ( FreqB2 + 375000 );
#endif
#endif
}

void SetRadioChannelOnOffset(uint32_t FreqA1, uint32_t FreqA2, uint32_t FreqB1, uint32_t FreqB2, uint32_t offset1, uint32_t offset2)
{
#if USE_8CH_MODULE_A
Channels[0].Frequency = ( FreqA1 - offset2 );
Channels[1].Frequency = ( FreqA1 - offset1 );
Channels[2].Frequency = ( FreqA1 + offset1 );
Channels[3].Frequency = ( FreqA1 + offset2 );
Channels[4].Frequency = ( FreqA2 - offset2 );
Channels[5].Frequency = ( FreqA2 - offset1 );
Channels[6].Frequency = ( FreqA2 + offset1 );
Channels[7].Frequency = ( FreqA2 + offset2 );

#elif USE_8CH_MODULE_B
Channels[0].Frequency = ( FreqB1 - offset2 );
Channels[1].Frequency = ( FreqB1 - offset1 );
Channels[2].Frequency = ( FreqB1 + offset1 );
Channels[3].Frequency = ( FreqB1 + offset2 );
Channels[4].Frequency = ( FreqB2 - offset2 );
Channels[5].Frequency = ( FreqB2 - offset1 );
Channels[6].Frequency = ( FreqB2 + offset1 );
Channels[7].Frequency = ( FreqB2 + offset2 );

#elif USE_16CH
#ifdef USE_JAPAN_CH
Channels[0].Frequency =  ( FreqA1 - 300000 );
Channels[1].Frequency =  ( FreqA1 - 100000 );
Channels[2].Frequency =  ( FreqA1 + 100000 );
Channels[3].Frequency =  ( FreqA1 + 300000 );
Channels[4].Frequency =  ( FreqA2 - 300000 );
Channels[5].Frequency =  ( FreqA2 - 100000 );
Channels[6].Frequency =  ( FreqA2 + 100000 );
Channels[7].Frequency =  ( FreqA2 + 300000 );
Channels[8].Frequency =  ( FreqB1 - 300000 );
Channels[9].Frequency =  ( FreqB1 - 100000 );
Channels[10].Frequency = ( FreqB1 + 100000 );
Channels[11].Frequency = ( FreqB1 + 300000 );
Channels[12].Frequency = ( FreqB2 - 300000 );
Channels[13].Frequency = ( FreqB2 - 100000 );
Channels[14].Frequency = ( FreqB2 + 100000 );
Channels[15].Frequency = ( FreqB2 + 300000 );
#else
Channels[0].Frequency =  ( FreqA1 - offset2 );
Channels[1].Frequency =  ( FreqA1 - offset1 );
Channels[2].Frequency =  ( FreqA1 + offset1 );
Channels[3].Frequency =  ( FreqA1 + offset2 );
Channels[4].Frequency =  ( FreqA2 - offset2 );
Channels[5].Frequency =  ( FreqA2 - offset1 );
Channels[6].Frequency =  ( FreqA2 + offset1 );
Channels[7].Frequency =  ( FreqA2 + offset2 );
Channels[8].Frequency =  ( FreqB1 - offset2 );
Channels[9].Frequency =  ( FreqB1 - offset1 );
Channels[10].Frequency = ( FreqB1 + offset1 );
Channels[11].Frequency = ( FreqB1 + offset2 );
Channels[12].Frequency = ( FreqB2 - offset2 );
Channels[13].Frequency = ( FreqB2 - offset1 );
Channels[14].Frequency = ( FreqB2 + offset1 );
Channels[15].Frequency = ( FreqB2 + offset2 );
#endif
#endif
}

void LoRaMacInitNwkIds( uint32_t netID, uint32_t devAddr, uint8_t *nwkSKey, uint8_t *appSKey )
{
//    LoRaMacNetID = netID;
    LoRaMacDevAddr = devAddr;
    memcpy(LoRaMacNwkSKey, nwkSKey, 16 );
    memcpy(LoRaMacAppSKey, appSKey, 16 );
}

uint8_t LoRaMacSetNextChannel( ChannelParams_t *NowChannel )
{
    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t channelNext = Channel;
    uint8_t nbEnabledChannels = 0;
    uint8_t enabledChannels[LORA_MAX_NB_CHANNELS];

    // Search how many channels are enabled
    for( i = 0; i < LORA_MAX_NB_CHANNELS; i++ )
    {
        if( ( ( 1 << i ) & ChannelsMask ) != 0 )
        {
            if( ( ( Channels[i].DrRange.Fields.Min <= ChannelsDatarate ) &&
                  ( ChannelsDatarate <= Channels[i].DrRange.Fields.Max ) ) == false )
            { // Check if the current channel selection supports the given datarate
                continue;
            }
            enabledChannels[nbEnabledChannels++] = i;
        }
    }

    /*Merge hanson's code*/
    ShufflePosition( enabledChannels, nbEnabledChannels );
    for( j = 0 ; j < nbEnabledChannels ; j++ )
    {
        channelNext = enabledChannels[j];
        if( SX1276IsChannelFree( MODEM_FSK, Channels[channelNext].Frequency, RSSI_FREE_TH) == true )
        {
            // Free channel found
            Channel = channelNext;
            *NowChannel = Channels[Channel];
            return 0;
         }
    }

/*
    for( i = 0, j = randr( 0, nbEnabledChannels - 1 ); i < LORA_MAX_NB_CHANNELS; i++ )
    {
        channelNext = enabledChannels[j];
        j = ( j + 1 ) % nbEnabledChannels;

        if( SX1276IsChannelFree( MODEM_LORA, Channels[channelNext].Frequency, RSSI_FREE_TH ) == true )
        {
            // Free channel found
            Channel = channelNext;
            *NowChannel = Channels[Channel];
            return 0;
        }
    }
*/
    // No free channel found. Keep previous channel
    return 1;
}

//void LoRaMacReceiveFrameOnChannel( ChannelParams_t channel )
//{
//    uint16_t symbTimeout = 5;
//     SX1276SetChannel( channel.Frequency );

//        if( ChannelsDatarate == DR_FSK )
//        {
//            SX1276SetRxConfig( MODEM_FSK, 50e3, Datarates[Rx2Channel.Datarate] * 1e3, 0, 83.333e3, 5, 0, false, 0, true, 0, 0, false, false );
//        }
//        else if( ChannelsDatarate == DR_SF7H )
//        {
//            symbTimeout = 8;
//            SX1276SetRxConfig( MODEM_LORA, 1, TxPowers[ChannelsTxPower], 1, 0, 8, symbTimeout, false, 0, false, 0, 0, true, false );		//hanson mod, change to rx single mode
//        }
//        else
//        {
//            // For low SF, we increase the number of symbols generating a Rx Timeout
//            if( ChannelsDatarate < DR_SF9 )
//            { // DR_SF10 = 2, DR_SF11 = 1, DR_SF12 = 0
//                symbTimeout = 5;
//            }
//            else
//            { // DR_SF7 = 5, DR_SF8 = 4, DR_SF9 = 3
//                symbTimeout = 8;
//            }
//            SX1276SetRxConfig( MODEM_LORA, LORA_BW_125, Datarates[ChannelsDatarate], 1, 0, 8, symbTimeout, false, 0, false, 0, 0, true, false );		//hanson mod, change to rx single mode and bw 250kHz

//        }
//		SX1276SetRx( MAX_RX_WINDOW1 - RADIO_WAKEUP_TIME );
//}

uint8_t LoRaMacSendFrameOnChannel( ChannelParams_t channel )
{
    //LoRaMacEventInfo.TxDatarate = Datarates[ChannelsDatarate];

    SX1276SetChannel( channel.Frequency );

    if( ChannelsDatarate == DR_FSK )
    { // High Speed FSK channel
#if defined (TRACKER_BOARD_V1)
        /* Modified by Crux, Date: 2017/03/17, Log: Tracker_v1 not use tx power index */
        SX1276SetTxConfig( MODEM_FSK, ChannelsTxPower, 25e3, 0, Datarates[ChannelsDatarate] * 1e3, 0, 5, false, true, 0, 0, false, 3e6 );
#else
        SX1276SetTxConfig( MODEM_FSK, TxPowers[ChannelsTxPower], 25e3, 0, Datarates[ChannelsDatarate] * 1e3, 0, 5, false, true, 0, 0, false, 3e6 );
#endif
    }
    else if( ChannelsDatarate == DR_SF7H )
    { // High speed LoRa channel
#if defined (TRACKER_BOARD_V1)
        /* Modified by Crux, Date: 2017/03/17, Log: Tracker_v1 not use tx power index */
        SX1276SetTxConfig( MODEM_LORA, ChannelsTxPower, 0, 1, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6 );
#else
        SX1276SetTxConfig( MODEM_LORA, TxPowers[ChannelsTxPower], 0, 1, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6 );
#endif
    }
    else
    { // Normal LoRa channel
#if defined (MANHOLE_BOARD)
        SX1276SetTxConfig( MODEM_LORA, TxPowers[ChannelsTxPower], 0, 0, Datarates[ChannelsDatarate], 3, 8, false, true, 0, 0, false, 3e6 );
#elif defined (SIPMODULE_BOARD)// AT command not used index
        SX1276SetTxConfig( MODEM_LORA, ChannelsTxPower, 0, 0, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6 );
#elif defined (TRACKER_BOARD_V1)
        /* Modified by Crux, Date: 2017/03/17, Log: Tracker_v1 not use tx power index */
        SX1276SetTxConfig( MODEM_LORA, ChannelsTxPower, 0, 0, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6 );
#else
        SX1276SetTxConfig( MODEM_LORA, TxPowers[ChannelsTxPower], 0, 0, Datarates[ChannelsDatarate], 1, 8, false, true, 0, 0, false, 3e6 );
#endif
    }

    UpLinkCounter++;
    //IsLoRaMacTransmitting = true;
    SX1276Send( LoRaMacBuffer, LoRaMacBufferPktLen );

    return 0;
}

uint8_t LoRaMacPrepareFrame( LoRaMacHeader_t *macHdr, LoRaMacFrameCtrl_t *fCtrl, uint8_t *fOpts, uint8_t fPort, void *fBuffer, uint16_t fBufferSize )
{
    uint16_t i;
    uint8_t pktHeaderLen = 0;
    uint32_t mic = 0;

    LoRaMacBufferPktLen = 0;

//    NodeAckRequested = false;

    if( fBuffer == NULL )
    {
        fBufferSize = 0;
    }

    LoRaMacBuffer[pktHeaderLen++] = macHdr->Value;

    switch( macHdr->Bits.MType )
    {
        case FRAME_TYPE_JOIN_REQ:

            LoRaMacBufferPktLen = pktHeaderLen;

            memcpy( LoRaMacBuffer + LoRaMacBufferPktLen, LoRaMacAppEui, 8 );
            LoRaMacBufferPktLen += 8;
            memcpy( LoRaMacBuffer + LoRaMacBufferPktLen, LoRaMacDevEui,  8 );
            LoRaMacBufferPktLen += 8;

            LoRaMacDevNonce = SX1276Random( );

            LoRaMacBuffer[LoRaMacBufferPktLen++] = LoRaMacDevNonce & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen++] = ( LoRaMacDevNonce >> 8 ) & 0xFF;

            LoRaMacJoinComputeMic( LoRaMacBuffer, LoRaMacBufferPktLen & 0xFF, LoRaMacAppKey, &mic );

            LoRaMacBuffer[LoRaMacBufferPktLen++] = mic & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen++] = ( mic >> 8 ) & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen++] = ( mic >> 16 ) & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen++] = ( mic >> 24 ) & 0xFF;

            break;
        case FRAME_TYPE_DATA_CONFIRMED_UP:
//            NodeAckRequested = true;
            //Intentional falltrough
        case FRAME_TYPE_DATA_UNCONFIRMED_UP:
//            if( IsLoRaMacNetworkJoined == false )
//            {
//                return 2; // No network has been joined yet
//            }

            if( fOpts == NULL )
            {
                fCtrl->Bits.OptionsLength = 0;
            }


            LoRaMacBuffer[pktHeaderLen++] = ( LoRaMacDevAddr ) & 0xFF;
            LoRaMacBuffer[pktHeaderLen++] = ( LoRaMacDevAddr >> 8 ) & 0xFF;
            LoRaMacBuffer[pktHeaderLen++] = ( LoRaMacDevAddr >> 16 ) & 0xFF;
            LoRaMacBuffer[pktHeaderLen++] = ( LoRaMacDevAddr >> 24 ) & 0xFF;

            LoRaMacBuffer[pktHeaderLen++] = fCtrl->Value;

            LoRaMacBuffer[pktHeaderLen++] = UpLinkCounter & 0xFF;
            LoRaMacBuffer[pktHeaderLen++] = ( UpLinkCounter >> 8 ) & 0xFF;

            if( fOpts != NULL )
            {
                for( i = 0; i < fCtrl->Bits.OptionsLength; i++ )
                {
                    LoRaMacBuffer[pktHeaderLen++] = fOpts[i];
                }
            }

            if( ( pktHeaderLen + fBufferSize ) > LORAMAC_PHY_MAXPAYLOAD )
            {
                return 3;
            }

            if( fBuffer != NULL )
            {
                LoRaMacBuffer[pktHeaderLen] = fPort;

                if( fPort == 0 )
                {
                    LoRaMacPayloadEncrypt( fBuffer, fBufferSize, LoRaMacNwkSKey, LoRaMacDevAddr, UP_LINK, UpLinkCounter, LoRaMacPayload );
                }
                else
                {
                    LoRaMacPayloadEncrypt( fBuffer, fBufferSize, LoRaMacAppSKey, LoRaMacDevAddr, UP_LINK, UpLinkCounter, LoRaMacPayload );
                }
                memcpy( LoRaMacBuffer + pktHeaderLen + 1, LoRaMacPayload, fBufferSize );
            }
            LoRaMacBufferPktLen = pktHeaderLen + 1 + fBufferSize;

            LoRaMacComputeMic( LoRaMacBuffer, LoRaMacBufferPktLen, LoRaMacNwkSKey, LoRaMacDevAddr, UP_LINK, UpLinkCounter, &mic );

            if( ( LoRaMacBufferPktLen + LORAMAC_MFR_LEN ) > LORAMAC_PHY_MAXPAYLOAD )
            {
                return 3;
            }
            LoRaMacBuffer[LoRaMacBufferPktLen + 0] = mic & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen + 1] = ( mic >> 8 ) & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen + 2] = ( mic >> 16 ) & 0xFF;
            LoRaMacBuffer[LoRaMacBufferPktLen + 3] = ( mic >> 24 ) & 0xFF;

            LoRaMacBufferPktLen += LORAMAC_MFR_LEN;
            break;
        default:
            return 4;
    }

    return 0;
}


uint8_t LoRaMacParseFrame( uint8_t *payload, uint16_t size, uint8_t *data, uint16_t *datasize)
{
    uint8_t pktHeaderLen = 0;
    uint32_t address = 0;
    LoRaMacFrameCtrl_t fCtrl;
    uint32_t mic = 0;
    uint32_t micRx = 0;
    uint8_t appPayloadStartIndex = 0;
    uint8_t port = 0xFF;
    uint8_t frameLen = 0;
    uint16_t sequenceCounter = 0;

    uint8_t *nwkSKey = LoRaMacNwkSKey;

    pktHeaderLen++;
    address = payload[pktHeaderLen++];
    address |= ( (uint32_t)payload[pktHeaderLen++] << 8 );
    address |= ( (uint32_t)payload[pktHeaderLen++] << 16 );
    address |= ( (uint32_t)payload[pktHeaderLen++] << 24 );

    fCtrl.Value = payload[pktHeaderLen++];

    appPayloadStartIndex = 8 + fCtrl.Bits.OptionsLength;

    sequenceCounter = ( uint16_t )payload[pktHeaderLen++];
    sequenceCounter |= ( uint16_t )payload[pktHeaderLen++] << 8;

    appPayloadStartIndex++;
    frameLen = ( size - 4 ) - appPayloadStartIndex;

    micRx |= ( uint32_t )payload[size - LORAMAC_MFR_LEN];
    micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 1] << 8 );
    micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 2] << 16 );
    micRx |= ( ( uint32_t )payload[size - LORAMAC_MFR_LEN + 3] << 24 );

    if( address != LoRaMacDevAddr )
        return 1;

    LoRaMacComputeMic( payload, size - LORAMAC_MFR_LEN, nwkSKey, address, DOWN_LINK, sequenceCounter, &mic );

    if(micRx != mic)
        return 2;

    if( ( ( size - 4 ) - appPayloadStartIndex ) > 0 )
    {
        port = payload[appPayloadStartIndex-1];
        if(port != 0)
        {
            /* Modified by Gavin Date: 2017/02/21   Log: Fix issue of using wrong key when decrypt application payload */
            LoRaMacPayloadDecrypt( payload + appPayloadStartIndex,frameLen,LoRaMacAppSKey,address, DOWN_LINK,sequenceCounter,data );
        }
    }
    else
    {
        return 3;// Received ack bit, no payload need to decrypt
    }

    *datasize = frameLen;
    return 0;
}

void LoRaMacRxWindowSetup( uint32_t freq, int8_t datarate, uint32_t bandwidth, uint16_t timeout, bool rxContinuous )
{
    uint8_t downlinkDatarate = Datarates[datarate];
    RadioModems_t modem;

    if( SX1276GetStatus( ) == RF_IDLE )
    {
#ifdef SIPMODULE_G78S
        if(Channels[0].Frequency == Channels[8].Frequency && Channels[7].Frequency == Channels[15].Frequency)
        {
            SX1276SetChannel(CN470_FIRST_RX1_CHANNEL + ( Channel % 8 ) * CN470_STEPWIDTH_RX1_CHANNEL);
        }
        else
        {
            SX1276SetChannel(CN470_FIRST_RX1_CHANNEL + ( Channel % 16 ) * CN470_STEPWIDTH_RX1_CHANNEL);
        }
#elif BOPI_CN470
        SX1276SetChannel(CN470_FIRST_RX1_CHANNEL + ((((freq - CN470_FIRST_TX_CHANNEL)/CN470_STEPWIDTH_RX1_CHANNEL)%48) * CN470_STEPWIDTH_RX1_CHANNEL) );
#else
        SX1276SetChannel( freq );
#endif

        // Store downlink datarate
//        McpsIndication.RxDatarate = ( uint8_t ) datarate;

//#if defined( USE_BAND_433 ) || defined( USE_BAND_780 ) || defined( USE_BAND_868 )
//        if( datarate == DR_7 )
//        {
//            modem = MODEM_FSK;
//            SX1276SetRxConfig( modem, 50e3, downlinkDatarate * 1e3, 0, 83.333e3, 5, 0, false, 0, true, 0, 0, false, rxContinuous );
//        }
//        else
//        {
//            modem = MODEM_LORA;
//            SX1276SetRxConfig( modem, bandwidth, downlinkDatarate, 1, 0, 8, timeout, false, 0, false, 0, 0, true, rxContinuous );
//        }
//#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
        modem = MODEM_LORA;
        SX1276SetRxConfig( modem, bandwidth, downlinkDatarate, 1, 0, 8, timeout, false, 0, false, 0, 0, true, rxContinuous );
//#endif

//        if( RepeaterSupport == true )
//        {
//            Radio.SetMaxPayloadLength( modem, MaxPayloadOfDatarateRepeater[datarate] );
//        }
//        else
//        {
//            Radio.SetMaxPayloadLength( modem, MaxPayloadOfDatarate[datarate] );
//        }

        if( rxContinuous == false )
        {
            SX1276SetRx( MAX_RX_WINDOW );
        }
        else
        {
            SX1276SetRx( 0 ); // Continuous mode
        }
    }
}

void LoRaMacRxWindown2Setup(bool rxContinuous)
{
    uint16_t symbTimeout = 5;

    // RX2 Defaults to SF9/125kHz - 869.525MHz
    SX1276SetChannel( Rx2Channel.Frequency );

    if( Rx2Channel.Datarate == DR_FSK )
    {
        SX1276SetRxConfig( MODEM_FSK, 50e3, Datarates[Rx2Channel.Datarate] * 1e3, 0, 83.333e3, 5, 0, false, 0, true, 0, 0, false, false );
    }
    else if( Rx2Channel.Datarate == DR_SF7H )
    {
        symbTimeout = 8;
        SX1276SetRxConfig( MODEM_LORA, 1, Datarates[Rx2Channel.Datarate], 1, 0, 8, symbTimeout, false, 0, false, 0, 0, true, false );		//hanson mod, change to rx single mode
    }
    else
    {
        // For low SF, we increase the number of symbols generating a Rx Timeout
        if( Rx2Channel.Datarate < DR_SF9 )
        { // DR_SF10 = 2, DR_SF11 = 1, DR_SF12 = 0
            symbTimeout = 5;
        }
        else
        { // DR_SF7 = 5, DR_SF8 = 4, DR_SF9 = 3
            symbTimeout = 8;
        }

#if defined(SIPMODULE_BOARD )
        symbTimeout = RxWindow2Timeout;
#endif

        SX1276SetRxConfig( MODEM_LORA, Rx2Bandwidth, Datarates[Rx2Channel.Datarate], 1, 0, 8, symbTimeout, false, 0, false, 0, 0, true, rxContinuous );		//hanson mod

        /*Add by Gavin, Date:2017/09/14, Log:Fix can't receive 242 bytes issue*/
        SX1276SetMaxPayloadLength( MODEM_LORA, 255);
    }

    /*Add by Gavin, Date:2017/07/13, Log:Flow by LoRaWAN code*/
    if(rxContinuous == false)
    {
        SX1276SetRx( 999);
    }
    else
    {
        SX1276SetRx( 0);
    }
}

uint32_t LoRaGetUpLinkCounter(void)
{
    return UpLinkCounter;
}

void LoRaSetUpLinkCounter(uint32_t upLinkCounter)
{
    UpLinkCounter = upLinkCounter;
}



/*!
*   Radio CAD initi function
* \param [IN]  bandwidth          CAD bandwidth
* \param [IN]  dataRate           CAD data rate
* \param [IN]  preambleLen        CAD preamble length
* \param [IN]  symbolTimeout      CAD symbol timeout
* \param [IN]  rxTimeout          CAD rx timeout
*/
void RadioCadInit(uint32_t freq, uint8_t bandwidth, uint8_t dataRate, uint16_t preambleLen
    ,uint16_t symbolTimeout, uint32_t rxTimeout)
{
    CadInit = 1;
    CadFrequency  = freq;
    CadBandwidth  = bandwidth;
    CadDatarate   = dataRate;
    PreambleLen   = preambleLen;
    SymbolTimeout = symbolTimeout;
    CadRxTimeout  = rxTimeout;
}

/*!
*   Get CAD Init Status
* \param [return]  CadInit          CAD initial status
*/
uint8_t GetCadInit (void) {
    return CadInit;
}

/*!
*   Get CAD Frequency
* \param [return]  CadFrequency     CAD listen frequency
*/
uint32_t GetCadFrequency (void) {
    return CadFrequency;
}

/*!
*   Get CAD Bandwidth
* \param [return]  CadBandwidth     CAD listen bandwidth
*/
uint8_t GetCadBandwidth (void) {
    return CadBandwidth;
}

/*!
*   Get CAD Datarate
* \param [return]  CadDatarate      CAD listen datarate
*/
uint8_t GetCadDatarate (void) {
    return CadDatarate;
}

/*!
*   Get CAD Preamble Length
* \param [return]  PreambleLen      CAD preamble length
*/
uint16_t GetCadPreambleLen (void) {
    return PreambleLen;
}

/*!
*   Get CAD Symbol Timeout
* \param [return]  SymbolTimeout     CAD symbol timeout
*/
uint16_t GetCadSymbolTimeout (void) {
    return SymbolTimeout;
}

/*!
*   Get CAD Rx Timeout
* \param [return]  CadRxTimeout      CAD Rx timeout
*/
uint16_t GetCadCadRxTimeout (void) {
    return CadRxTimeout;
}
