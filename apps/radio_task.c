#include "radio_task.h"
#include "sx1276.h"
#include "board.h"
#include "timer_task.h"
#include <string.h>
#include "LoRaMac.h"
#include "utilities.h"
/* Add by Eric Date: 2017/03/2   Log: Fix node can not receive Rx2 downlink issue.  */
/* Modify by Eric Date: 2017/03/15   Log: Modify RxDelayTime2 to 1993.  */
#ifdef SIPMODULE_BOARD
#define RxDelayTime1 1000
#define RxDelayTime2 2000
#else
#define RxDelayTime1 999
#define RxDelayTime2 2000
#endif

#define TXLBTDelayTime 400 /*0.4s*/
#define LoRaMacLBTRetryTime 3

#define TX_ACK_RERTY_TIME 3000

#ifdef SIPMODULE_BOARD

#ifdef STM32L073xx
#define LORAMAXLEN 256
#else
#define LORAMAXLEN 160
#endif

#else
#define LORAMAXLEN 64
#endif

DeviceClass_t LoRaMacDeviceClass = CLASS_A;

char LoRaMacSendBuffer[LORAMAXLEN] = {0};

bool FactoryMode = false;
bool RadioInitFlag = false;
bool CadDetectFlag = false;


// For Tx Ack Retry
static uint8_t TxAckTimeoutRetries = 0;
static uint8_t TxAckTimeoutRetryCount = 0;

static QueueHandle_t Radio_Task_Queue = NULL;

/*timer array*/
uint32_t RadioSWTimer[SX1276TotalTimer];

static RadioIrqHandler *RadioCallback = NULL;
TaskHandle_t xRadioTaskHandle = NULL;

#if defined(SIPMODULE_BOARD )
static RxWindowParams_t RxWindowParams;
#endif

void SX1276Callback(SX1276DIO_t status)
{
    switch (status)
    {
        case SX1276DIO0:
            SendQueueToRadioFromISR(RADIODIO0);
            break;
        case SX1276DIO1:
            SendQueueToRadioFromISR(RADIODIO1);
            break;
        case SX1276DIO2:
            SendQueueToRadioFromISR(RADIODIO2);
            break;
        case SX1276DIO3:
            SendQueueToRadioFromISR(RADIODIO3);
            break;
        case SX1276DIO4:
            SendQueueToRadioFromISR(RADIODIO4);
            break;
        case SX1276DIO5:
            SendQueueToRadioFromISR(RADIODIO5);
            break;
        default:
            break;
    }
}

void RadioTimerCallback(uint32_t timerID)
{
    SendQueueToRadio(RadioTimeOut, &timerID);
}

void SetRadioCallback(RadioIrqHandler *callback)
{
    RadioCallback = callback;
}

void RadioDeTask(void)
{
    vQueueDelete(Radio_Task_Queue);
    vTaskDelete(xRadioTaskHandle);
}

void RadioTask( void * pvParameters )
{
    RadioMsgBody_t RadioReceived;
    ChannelParams_t ChoiceChannel;
    LoRaMacHeader_t macHdr;
    LoRaMacFrameCtrl_t fCtrl;
    uint16_t LoRaMacBufferPktLen;
    uint8_t LoRaMacLBTRetryCount = 0;
    uint8_t AppPort;
    bool SrvAckRequested = false;
	/* Create the queue as described at the top of this file. */
	Radio_Task_Queue = xQueueCreate( RADIO_TASK_QUEUE_LENGTH, sizeof(RadioMsgBody_t) );
	configASSERT( Radio_Task_Queue );

    /* mac and key*/
//    srand( RAND_SEED );
//    LoRaMacDevAddr = randr( 0, 0x01FFFFFF );
/* Modified by Nick Date: 2017/03/21  Log: add for STM32L073xx double radioInit() in Class C, do not init radio in radio task on STM32L073xx */
/* Moidyf by Gavin, Date:2017/07/13 Log: Let SIPMODULE_BOARD have same behavior*/
#if !defined( SIPMODULE_BOARD )
        radioInit();
#endif
    //    LoRaMacNetID = 0x000000;
    //LoRaMacInitNwkIds( 0x000000, DevAddr, NwkSKey, AppSKey );

    for( ;; )
    {
        xQueueReceive( Radio_Task_Queue, &RadioReceived, portMAX_DELAY );

        switch(RadioReceived.type)
        {
            case RadioDeInit:
            {
                SX1276SetStby();
                SX1276SetSleep();
                SX1276IoDeIrqInit();
                if(RadioCallback != NULL)
                {
                    RadioStatus_t event;
                    event.Status = RadioStdby;
                    RadioCallback(event);
                }
                RadioInitFlag = false;
                break;
            }
            case RadioInit:
            {
                if(RadioInitFlag != true)
                {
                    SX1276SetStby();
                    radioInit();
                    RadioInitFlag = true;
                }
                break;
            }
            case SendUnConfirmedDataUp:
            case SendConfirmedDataUp:
            {
                uint32_t txTimeOnAir;
                //uint8_t AppPort;
                if(LoRaMacDeviceClass == CLASS_C && SX1276GetStatus() != RF_IDLE)
                {
                    SX1276SetStby();
                }

                if(SX1276GetStatus() != RF_IDLE)/*sx1276 be used*/
                {
                    vPortFree(RadioReceived.body.SendData.Data);
                    break;
                }

#if defined(SIPMODULE_BOARD )
                ComputeRxWindowParameters(&RxWindowParams);
#endif

                macHdr.Value = 0;
                fCtrl.Value = 0;
                if(RadioReceived.type == SendUnConfirmedDataUp)
                    macHdr.Bits.MType = FRAME_TYPE_DATA_UNCONFIRMED_UP;
                else
                    macHdr.Bits.MType = FRAME_TYPE_DATA_CONFIRMED_UP;

                if( SrvAckRequested == true )
                {
                    SrvAckRequested = false;
                    fCtrl.Bits.Ack = 1;
                }

                AppPort = RadioReceived.body.SendData.AppPort;
                if(AppPort == 0)
                {
                    AppPort = 2; // port 0 is for Mac Command, so use default value
                }

                /*global buffer only have 64 bytes*/
                LoRaMacBufferPktLen = (uint16_t)RadioReceived.body.SendData.DataLen;
                //LoRaMacSendBuffer = pvPortMalloc(LoRaMacBufferPktLen + 1);
                memset(LoRaMacSendBuffer, 0x00, LoRaMacBufferPktLen + 1);
                memcpy((char *)LoRaMacSendBuffer, (char *)RadioReceived.body.SendData.Data, LoRaMacBufferPktLen);
                vPortFree(RadioReceived.body.SendData.Data);
                LoRaMacLBTRetryCount = 0;
                if( LoRaMacSetNextChannel(&ChoiceChannel ) != 0 )
                {
                    /*no channel can choose*/
                    RadioSWTimer[SX1276LBTDelayTimeout] = AddTimer(SX1276LBTDelayTimeout, TXLBTDelayTime, 0, RadioTimerCallback);
                    LoRaMacLBTRetryCount++;
                    //vPortFree(RadioReceived.body.SendData.Data);
                    break;
                }
                LoRaMacPrepareFrame(&macHdr, &fCtrl, NULL, AppPort, LoRaMacSendBuffer, LoRaMacBufferPktLen);

                //Add by Gavin, Date:20170914, Log:Add on air time for prevent tx timeout when transmit  bytes over 256 on SF12
                txTimeOnAir = SX1276GetTimeOnAir( MODEM_LORA, LoRaMacBufferPktLen );
                LoRaMacSendFrameOnChannel(ChoiceChannel);
                RadioSWTimer[SX1276TxTimeout] = AddTimer(SX1276TxTimeout, txTimeOnAir, 0, RadioTimerCallback);

                //Add for Ack Retry
                if(RadioSWTimer[SX1276RetryTimeout] != NULL)
                {
                    RadioSWTimer[SX1276RetryTimeout]= DeleteTimer(RadioSWTimer[SX1276RetryTimeout]);
                }

                if(RadioReceived.type == SendConfirmedDataUp)
                {
                    TxAckTimeoutRetryCount = 0;
                    TxAckTimeoutRetries = RadioReceived.body.SendData.retries;
                    RadioSWTimer[SX1276RetryTimeout] = AddTimer(SX1276RetryTimeout, TX_ACK_RERTY_TIME, 1, RadioTimerCallback);
                }
                else
                {
                    TxAckTimeoutRetryCount = 0;
                    TxAckTimeoutRetries = 0;
                }
                break;
            }
            case ListenCADSymbol:
            {
                uint32_t CadFrequency;
                uint8_t  CadBandwidth;
                uint8_t  CadDatarate;
                uint16_t PreambleLen;
                uint16_t SymbolTimeout;

                if(SX1276GetStatus() != RF_IDLE || (GetCadInit() == 0) )/*sx1276 be used*/
                {
                    break;
                }

                CadFrequency  = GetCadFrequency();
                CadBandwidth  = GetCadBandwidth();
                CadDatarate   = GetCadDatarate();
                PreambleLen   = GetCadPreambleLen();
                SymbolTimeout = GetCadSymbolTimeout();

                SX1276SetChannel( CadFrequency );
                SX1276SetRxConfig( MODEM_LORA, CadBandwidth, CadDatarate, 1, 0, PreambleLen, SymbolTimeout, false, 0, true, 0, 0, false, 0);
                SX1276Write( REG_LR_INVERTIQ, ( ( SX1276Read( REG_LR_INVERTIQ ) & RFLR_INVERTIQ_TX_MASK & RFLR_INVERTIQ_RX_MASK ) | RFLR_INVERTIQ_RX_OFF | RFLR_INVERTIQ_TX_OFF ) );

                SX1276StartCad();
                break;
            }
            /* radio state timeout handle*/
            case RadioTimeOut:
            {
                switch(RadioReceived.body.TimeOutID)
                {
                    case SX1276TxTimeout:/*sx1276 can't send data*/
                    {
                        RadioSWTimer[SX1276TxTimeout] = 0x00;

                        if(LoRaMacDeviceClass != CLASS_C)
                        {
                            SX1276SetSleep();
                        }

                        if(RadioCallback != NULL)
                        {
                            RadioStatus_t event;
                            event.Status = RadioTxTimeout;
                            RadioCallback(event);
                        }
                        break;
                    }
                    case SX1276RxTimeout:/*sx1276 can't receive data*/
                    {
                        RadioSWTimer[SX1276RxTimeout] = 0x00;

                        if(LoRaMacDeviceClass != CLASS_C)
                        {
                            SX1276SetSleep();
                        }
                        break;
                    }
                    case SX1276RxDelayTimeout1:
                    {
                        uint16_t symbTimeout = 5;
                        RadioSWTimer[SX1276RxDelayTimeout1] = 0x00;
                        if(RadioInitFlag == false) /*radio deinit*/
                        {
                            break;
                        }
                        if(SX1276GetStatus() != RF_IDLE)/*sx1276 be used*/
                            break;

                        if( GetRadioDatarate() == DR_SF7H )
                        {
                            symbTimeout = 8;
                        }
                        else
                        {
                            // For low SF, we increase the number of symbols generating a Rx Timeout
                            if( GetRadioDatarate() < DR_SF9 )
                            { // DR_SF10 = 2, DR_SF11 = 1, DR_SF12 = 0
                                symbTimeout = 5;
                            }
                            else if( GetRadioDatarate() == DR_SF7 )
                            {
                                symbTimeout = 10;
                            }
                            else
                            { // DR_SF7 = 5, DR_SF8 = 4, DR_SF9 = 3
                                symbTimeout = 8;
                            }
                        }
#if defined(SIPMODULE_BOARD )
                        symbTimeout = RxWindowParams.RxWindow1Timeout;
#endif
                        LoRaMacRxWindowSetup(ChoiceChannel.Frequency, GetRadioDatarate(), LORA_BW_125, symbTimeout ,0);
//                        LoRaMacReceiveFrameOnChannel(ChoiceChannel);
                        break;
                    }
                    case SX1276RxDelayTimeout2:
                    {
//                        uint16_t symbTimeout = 5;
                        RadioSWTimer[SX1276RxDelayTimeout2] = 0x00;
                        if(RadioInitFlag == false) /*radio deinit*/
                        {
                            break;
                        }
                        if(SX1276GetStatus() != RF_IDLE)/*sx1276 be used*/
                            break;
                        LoRaMacRxWindown2Setup(false);
//                        if( Rx2Channel. == DR_SF7H )
//                        {
//                            symbTimeout = 8;
//                        }
//                        else
//                        {
//                            // For low SF, we increase the number of symbols generating a Rx Timeout
//                            if( ChoiceChannel.DrRange.Fields.Min < DR_SF9 )
//                            { // DR_SF10 = 2, DR_SF11 = 1, DR_SF12 = 0
//                                symbTimeout = 5;
//                            }
//                            else
//                            { // DR_SF7 = 5, DR_SF8 = 4, DR_SF9 = 3
//                                symbTimeout = 8;
//                            }
//                        }
//                        RxWindowSetup(ChoiceChannel.Frequency, ChoiceChannel.DrRange.Fields.Min, LORA_BW_125, symbTimeout ,0);
//                        LoRaMacReceiveFrameOnChannel(ChoiceChannel);
//                        Rx2Channel
                        break;
                    }
                    case SX1276RxDelayTimeForClassC:
                    {
                        uint16_t symbTimeout = 5;

                        RadioSWTimer[SX1276RxDelayTimeForClassC] = 0x00;

                        SX1276SetStby();
                        if( ChoiceChannel.DrRange.Fields.Min == DR_SF7H )
                        {
                            symbTimeout = 8;
                        }
                        else
                        {
                            if( ChoiceChannel.DrRange.Fields.Min < DR_SF9 )
                            { // DR_SF10 = 2, DR_SF11 = 1, DR_SF12 = 0
                                symbTimeout = 5;
                            }
                            else
                            { // DR_SF7 = 5, DR_SF8 = 4, DR_SF9 = 3
                                symbTimeout = 8;
                            }
                        }
                        LoRaMacRxWindowSetup(ChoiceChannel.Frequency, GetRadioDatarate(), LORA_BW_125, symbTimeout ,0);
                        break;
                    }
                    case SX1276LBTDelayTimeout:
                    {
                        RadioSWTimer[SX1276LBTDelayTimeout] = 0x00;
                        if( LoRaMacSetNextChannel(&ChoiceChannel ) != 0 )
                        {
                            /*no channel can choose*/
                            LoRaMacLBTRetryCount++;
                            if(LoRaMacLBTRetryCount == LoRaMacLBTRetryTime)
                            {
                                if(RadioCallback != NULL)
                                {
                                    RadioStatus_t event;
                                    event.Status = RadioLBTError;
                                    RadioCallback(event);
                                }
                                break;
                            }
                            RadioSWTimer[SX1276LBTDelayTimeout] = AddTimer(SX1276LBTDelayTimeout, TXLBTDelayTime, 0, RadioTimerCallback);
                            break;
                        }
                        LoRaMacPrepareFrame(&macHdr, &fCtrl, NULL, AppPort, LoRaMacSendBuffer, LoRaMacBufferPktLen);
                        LoRaMacSendFrameOnChannel(ChoiceChannel);
                        RadioSWTimer[SX1276TxTimeout] = AddTimer(SX1276TxTimeout, 3000, 0, RadioTimerCallback);

                        //Add for Ack Retry
                        if(RadioSWTimer[SX1276RetryTimeout] != NULL)
                        {
                            RadioSWTimer[SX1276RetryTimeout]= DeleteTimer(RadioSWTimer[SX1276RetryTimeout]);
                        }

                        if(RadioReceived.type == SendConfirmedDataUp)
                        {
                            TxAckTimeoutRetryCount = 0;
                            TxAckTimeoutRetries = RadioReceived.body.SendData.retries;
                            RadioSWTimer[SX1276RetryTimeout] = AddTimer(SX1276RetryTimeout, TX_ACK_RERTY_TIME, 1, RadioTimerCallback);
                        }
                        else
                        {
                            TxAckTimeoutRetryCount = 0;
                            TxAckTimeoutRetries = 0;
                        }
                        break;
                    }
                    case SX1276RetryTimeout:
                    {
                        TxAckTimeoutRetryCount ++;
                        if(TxAckTimeoutRetryCount >= TxAckTimeoutRetries -1)
                        {
                            if(RadioSWTimer[SX1276RetryTimeout] != NULL)
                            {
                                RadioSWTimer[SX1276RetryTimeout]= DeleteTimer(RadioSWTimer[SX1276RetryTimeout]);
                            }
                        }

                        //Keep previous UpLinkCounter
                        LoRaSetUpLinkCounter(LoRaGetUpLinkCounter()-1);
                        LoRaMacPrepareFrame(&macHdr, &fCtrl, NULL, AppPort, LoRaMacSendBuffer, LoRaMacBufferPktLen);
                        LoRaMacSendFrameOnChannel(ChoiceChannel);
                        RadioSWTimer[SX1276TxTimeout] = AddTimer(SX1276TxTimeout, 3000, 0, RadioTimerCallback);
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            /* sx 1276 DIOx irq*/
            case RADIODIO0:
            {
                uint8_t DIO0Data;
                uint16_t RxLen;
                uint8_t RxData[LORAMAC_PHY_MAXPAYLOAD];
                uint8_t checkMACMIC;

                DIO0Data = SX1276Read(REG_DIOMAPPING1)&(~RF_DIOMAPPING1_DIO0_MASK);
                switch(DIO0Data)
                {
                    case RF_DIOMAPPING1_DIO0_00:/*LoRa RxDone*/
                    {
                        uint8_t *RxBuffer;
                        uint16_t dataSize;
                        int8_t SnrValue;
                        int16_t RssiValue;
                        uint8_t FPort;
                        __IO uint8_t irqFlags = 0;

                        if(LoRaMacDeviceClass != CLASS_C)
                        {
                            RadioSWTimer[SX1276RxTimeout] = DeleteTimer(RadioSWTimer[SX1276RxTimeout]);
                        }

                        /*process receive data*/
                        SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXDONE );

                        irqFlags = SX1276Read( REG_LR_IRQFLAGS );
                        if( ( irqFlags & RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK ) == RFLR_IRQFLAGS_PAYLOADCRCERROR )
                        {
                            // Clear Irq
                            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_PAYLOADCRCERROR );

//                            if( SX1276.Settings.LoRa.RxContinuous == false )
//                            {
//                                SX1276.Settings.State = RF_IDLE;
//                            }

#if defined (MANHOLE_BOARD)
                            if( SX1276GetRxContinuousStatus(MODEM_LORA) == false )
                            {
                                SX1276SetStatus( RF_IDLE );
                            }
                            if(LoRaMacDeviceClass != CLASS_C)
                            {
                                SX1276SetSleep();
                            }
#endif
                            /*Add by Gavin, Date:20170921, Log:Fix txpp Rx hang issue*/
                            dataSize = SX1276Read( REG_LR_RXNBBYTES );
                            RxBuffer = pvPortMalloc(dataSize + 1);
                            SX1276ReadFifo( RxBuffer, dataSize);
                            vPortFree(RxBuffer);
//                            TimerStop( &RxTimeoutTimer );

//                            if( ( RadioEvents != NULL ) && ( RadioEvents->RxError != NULL ) )
//                            {
//                                RadioEvents->RxError( );
//                            }
                            break;
                        }

                        SX1276GetPKTRSSIandSNR(&SnrValue, &RssiValue);

                        dataSize = SX1276Read( REG_LR_RXNBBYTES );
                        RxBuffer = pvPortMalloc(dataSize + 1);
                        SX1276ReadFifo( RxBuffer, dataSize);
                        if(FactoryMode == false)
                            checkMACMIC = LoRaMacParseFrame(RxBuffer,dataSize,RxData,&RxLen);
                        else/*for Factory mode don't check MAC and MIC*/
                        {
                            memset(RxData, 0x00, LORAMAC_PHY_MAXPAYLOAD);
                            memcpy(RxData, RxBuffer, dataSize);
                            RxLen = dataSize;
                        }

//                        if( SX1276.Settings.LoRa.RxContinuous == false )
//                        {
//                            SX1276.Settings.State = RF_IDLE;
//                        }
                        if(FactoryMode == false)/*if in factory mode, don't sleep sx1276*/
                        {
                            if(LoRaMacDeviceClass != CLASS_C)
                            {
                                SX1276SetSleep();
                            }

                            if(checkMACMIC == 0 || checkMACMIC == 3)
                            {
                                uint8_t pktHeaderLen = 0;
                                LoRaMacHeader_t macHdr;
//                                LoRaMacFrameCtrl_t fCtrl;

                                macHdr.Value = RxBuffer[pktHeaderLen++];

                                switch( macHdr.Bits.MType )
                                {
                                    case FRAME_TYPE_DATA_CONFIRMED_DOWN:
                                    case FRAME_TYPE_DATA_UNCONFIRMED_DOWN:
                                    {
                                        pktHeaderLen+=4;/*for mac*/
                                        fCtrl.Value = RxBuffer[pktHeaderLen++];
                                        pktHeaderLen+=2;/*for cnt*/
                                        FPort = RxBuffer[pktHeaderLen++];
                                        if( macHdr.Bits.MType == FRAME_TYPE_DATA_CONFIRMED_DOWN )
                                            SrvAckRequested = true;
                                        else
                                            SrvAckRequested = false;

                                        break;
                                    }
                                    default:
                                        break;
                                }
                            }
                            else
                            {
                                vPortFree(RxBuffer);
                                break;// break switch of RF_DIOMAPPING1_DIO0_00
                            }
                        }

                        vPortFree(RxBuffer);
                        if(RadioCallback != NULL)
                        {
                            RadioStatus_t event;

                            if(FactoryMode == false)
                            {
                                /*Modify by Gavin, Date:20170912, Log:Fix no callback ack when receiveing
                                    Rx data and ack at same time*/
                                if(fCtrl.Bits.Ack == 1)
                                {
                                    event.Status = RadioRxAck;
                                    //Received Tx Ack, so delete retry timer
                                    TxAckTimeoutRetryCount = 0;
                                    TxAckTimeoutRetries = 0;
                                    if(RadioSWTimer[SX1276RetryTimeout] != NULL)
                                    {
                                        RadioSWTimer[SX1276RetryTimeout]= DeleteTimer(RadioSWTimer[SX1276RetryTimeout]);
                                    }
                                    event.body.RxData.RssiValue = RssiValue;
                                    event.body.RxData.SnrValue = SnrValue;
                                    event.body.RxData.RxBuffer = RxData;
                                    event.body.RxData.Size = RxLen;
                                    event.body.RxData.RxPort = FPort;
                                    RadioCallback(event);
                                }

                                if(checkMACMIC == 3)
                                {
                                    break;
                                }
                                else
                                {
                                    event.Status = RadioRxDone;
                                }
                            }
                            else
                                event.Status = RadioTestPPRxDone;
                            event.body.RxData.RssiValue = RssiValue;
                            event.body.RxData.SnrValue = SnrValue;
                            event.body.RxData.RxBuffer = RxData;
                            event.body.RxData.Size = RxLen;
                            event.body.RxData.RxPort = FPort;
                            RadioCallback(event);
                        }
                        break;
                    }
                    case RF_DIOMAPPING1_DIO0_01:/*LoRa TxDone*/
                    {
                        RadioSWTimer[SX1276TxTimeout] = DeleteTimer(RadioSWTimer[SX1276TxTimeout]);
                        SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_TXDONE );

                        if(LoRaMacDeviceClass != CLASS_C)
                        {
                            SX1276SetSleep();
                        }

                        if(RadioCallback != NULL)
                        {
                            RadioStatus_t event;
													  //Jason add for fix TXPP unable continue transmit issue at 2019.08.27
													  if(FactoryMode == true)
                            {
                                SX1276SetStatus( RF_IDLE );
							              }
                            if(FactoryMode == false)
                                event.Status = RadioTxDone;
                            else
                                event.Status = RadioTestPPTxDone;
                            RadioCallback(event);
                        }
#if !defined (TRACKER_BOARD) && !defined (TRACKER_BOARD_V1)

                        if(FactoryMode == false)
                        {
                            if(LoRaMacDeviceClass == CLASS_C)
                            {
#if defined(SIPMODULE_BOARD )
                                RadioSWTimer[SX1276RxDelayTimeForClassC] = AddTimer(SX1276RxDelayTimeForClassC, RxDelayTime1 + RxWindowParams.RxWindow1Offset, 0, RadioTimerCallback);
#else
                                RadioSWTimer[SX1276RxDelayTimeForClassC] = AddTimer(SX1276RxDelayTimeForClassC, RxDelayTime1, 0, RadioTimerCallback);
#endif
                                LoRaMacRxWindown2Setup(true);
                            }
                            else
                            {
#if defined(SIPMODULE_BOARD )
                                if(RadioSWTimer[SX1276RxDelayTimeout1] == NULL)
                                {
                                    RadioSWTimer[SX1276RxDelayTimeout1] = AddTimer(SX1276RxDelayTimeout1, RxDelayTime1 + RxWindowParams.RxWindow1Offset, 0, RadioTimerCallback);
                                }
                                if(RadioSWTimer[SX1276RxDelayTimeout2] == NULL)
                                {
                                    RadioSWTimer[SX1276RxDelayTimeout2] = AddTimer(SX1276RxDelayTimeout2, RxDelayTime2 + RxWindowParams.RxWindow2Offset, 0, RadioTimerCallback);
                                }
#else
                                if(RadioSWTimer[SX1276RxDelayTimeout1] == NULL)
                                {
                                    RadioSWTimer[SX1276RxDelayTimeout1] = AddTimer(SX1276RxDelayTimeout1, RxDelayTime1, 0, RadioTimerCallback);
                                }
                                if(RadioSWTimer[SX1276RxDelayTimeout2] == NULL)
                                {
                                    RadioSWTimer[SX1276RxDelayTimeout2] = AddTimer(SX1276RxDelayTimeout2, RxDelayTime2, 0, RadioTimerCallback);
                                }
#endif
                            }
                        }
#endif
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case RADIODIO1:
            {
                uint8_t DIO1Data;
                DIO1Data = SX1276Read(REG_DIOMAPPING1)&(~RF_DIOMAPPING1_DIO1_MASK);
                switch(DIO1Data)
                {
                    case RF_DIOMAPPING1_DIO0_00:/*LoRa Rx Timeout*/
                    {

                        SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_RXTIMEOUT );

                        if(LoRaMacDeviceClass == CLASS_C)
                        {
                            LoRaMacRxWindown2Setup(true);
                        }
                        else
                        {
                            RadioSWTimer[SX1276RxTimeout] = DeleteTimer(RadioSWTimer[SX1276RxTimeout]);
                            SX1276SetSleep();
                        }

                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case RADIODIO2:
            {
                break;
            }
            case RADIODIO3:
            {

#if defined (MANHOLE_BOARD)
                switch (SX1276GetModems()) {
                    case MODEM_FSK:
                        break;
                    case MODEM_LORA:
                        if( ( SX1276Read( REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_CADDETECTED ) == RFLR_IRQFLAGS_CADDETECTED )
                        {
                            uint8_t  CadBandwidth;
                            uint8_t  CadDatarate;
                            uint16_t PreambleLen;
                            uint16_t SymbolTimeout;
                            uint32_t CadRxTimeout;


                            SetCadDetectFlag(true);
                            CadBandwidth  = GetCadBandwidth();
                            CadDatarate   = GetCadDatarate();
                            PreambleLen   = GetCadPreambleLen();
                            SymbolTimeout = GetCadSymbolTimeout();
                            CadRxTimeout  = GetCadCadRxTimeout();
                            // Clear Irq
                            SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDETECTED | RFLR_IRQFLAGS_CADDONE );
                            SX1276SetRxConfig( MODEM_LORA, CadBandwidth, CadDatarate, 1, 0, PreambleLen, SymbolTimeout, false, 0, true, 0, 0, false, 0);

                            SX1276SetRx(CadRxTimeout);
                        }
                        else
                        {
                            // Clear Irq
                            if ((SX1276Read( REG_LR_IRQFLAGS ) & RFLR_IRQFLAGS_CADDONE) ==  RFLR_IRQFLAGS_CADDONE) {
                                SX1276Write( REG_LR_IRQFLAGS, RFLR_IRQFLAGS_CADDONE );
                                SX1276SetSleep();
                            }

                        }
                        break;
                }
#endif
                break;
            }
            case RADIODIO4:
            {
                break;
            }
            case RADIODIO5:
            {
                break;
            }
            case TxContinuousModeStart:
            {
                RadioTxContinuousMode_t txcmpara = RadioReceived.body.TxCM;
                SX1276SetChannel(txcmpara.FreqVal * 1000000);
                if( txcmpara.FreqIdx == 0 || txcmpara.FreqIdx == 1 )
                {
/* Modify by Eric Date: 2017/03/23   Log: Fix tx power issue for 470 band.  */
                        SX1276Write( 0x01, 0x88 );
//                        SX1276Write( 0x44, 0x7B );
                        SX1276Write( 0x3D, 0xA1 );
                        SX1276Write( 0x36, 0x01 );
                        SX1276Write( 0x1e, 0x08 );
//                        SX1276Write( 0x45, 0xDF );
//                        SX1276Write( 0x46, 0x03 );
                        SX1276Write( 0x4D, 0x87 );
                        SX1276Write( 0x52, 0x60 );
                }
                else if( txcmpara.FreqIdx == 2 || txcmpara.FreqIdx == 3 )
                {
                        SX1276Write( 0x01, 0x80 );
                        SX1276Write( 0x3D, 0xA1 );
                        SX1276Write( 0x36, 0x01 );
                        SX1276Write( 0x1e, 0x08 );
                }
                SX1276SetTxConfig( MODEM_LORA, txcmpara.TxPwrVal, 0, txcmpara.BwdIdx,
                                   Datarates[txcmpara.SpFactorIdx], txcmpara.CodeRateIdx,
                                   8, txcmpara.FixLenPayloadOn,
                                   txcmpara.LoRaEnableCrcOn, 0, 0, txcmpara.IqInversionOn, 3e6 );
                FactoryMode = true;
                SX1276Send(NULL, 0);
                break;
            }
            case TxContinuousModeStop:
            {
                SX1276SetStby();
                SX1276Init();
                SX1276SetModem( MODEM_LORA );
                SX1276Write( 0x39, 0x34 );
                SX1276SetStby();
                FactoryMode = false;
                break;
            }
            case MasterPingPongStart:
            {
                RadioStatus_t event;
                RadioTxContinuousMode_t txcmpara = RadioReceived.body.TxCM;
                SX1276SetChannel(txcmpara.FreqVal*1000000);
                SX1276SetTxConfig( MODEM_LORA, txcmpara.TxPwrVal, 0, txcmpara.BwdIdx,
                         Datarates[txcmpara.SpFactorIdx], txcmpara.CodeRateIdx,
                         LORA_PREAMBLE_LENGTH, txcmpara.FixLenPayloadOn,
                         txcmpara.LoRaEnableCrcOn, 0, 0, txcmpara.IqInversionOn, txcmpara.PingPongTxOrRxTimeoutIdx *1000);		// Original PingPongTxTimeout = 3000000
                event.Status = RadioTestPPMasterSettingDone;
                FactoryMode = true;
							  //Jason add for fix TXPP continue transmit issue at 2019.08.27
							  SX1276SetStatus( RF_IDLE );
                if(RadioCallback != NULL)
                            RadioCallback(event);
                break;
            }
            case SlavePingPongStart:
            {
                RadioStatus_t event;
                RadioTxContinuousMode_t txcmpara = RadioReceived.body.TxCM;
                //This will init the sx1276 register RegLna's setting.
                //SX1276Write(REG_LNA, 0x23 );
                SX1276SetChannel(txcmpara.FreqVal * 1000000);

                SX1276SetRxConfig( MODEM_LORA, txcmpara.BwdIdx, Datarates[txcmpara.SpFactorIdx],
                         txcmpara.CodeRateIdx, 0, LORA_PREAMBLE_LENGTH,
                         LORA_SYMBOL_TIMEOUT, txcmpara.FixLenPayloadOn, 0,
                         txcmpara.LoRaEnableCrcOn, 0, 0, txcmpara.IqInversionOn, true );

                SX1276SetRx( 0 );/*use continuous*/

                event.Status = RadioTestPPSlaveSettingDone;
                FactoryMode = true;
							  //Jason add for fix TXPP continue transmit issue at 2019.08.27
							  SX1276SetStatus( RF_IDLE );
                if(RadioCallback != NULL)
                            RadioCallback(event);

                break;
            }
            case MasterPingPongData:
            {
                void *sendBuffer;
                if(SX1276GetStatus() != RF_IDLE)
                {
                    vPortFree(RadioReceived.body.SendData.Data);
                    break;
                }
                sendBuffer = pvPortMalloc(RadioReceived.body.SendData.DataLen + 1);
                memset(sendBuffer, 0x00, RadioReceived.body.SendData.DataLen + 1);
                memcpy((char *)sendBuffer, (char *)RadioReceived.body.SendData.Data, RadioReceived.body.SendData.DataLen);
                SX1276Send((uint8_t *)sendBuffer, RadioReceived.body.SendData.DataLen);
                vPortFree(RadioReceived.body.SendData.Data);
                vPortFree(sendBuffer);
                break;
            }
            case PingPongStop:
            {
                SX1276SetStby();
                FactoryMode = false;
                break;
            }
            default:
                break;
        }
    }
}

void radioInit(void)
{
    SX1276IoInit();
    SX1276Init();
    SX1276SetCallback(SX1276Callback);
    /*check radio chip(sx 1276)*/
    if(readVersionID() != 0x12)
    {
        RadioInitFlag = false;
        return;
    }
    // Change LoRa modem SyncWord
    RadioInitFlag = true;
    SX1276SetModem( MODEM_LORA );
    SX1276Write( 0x39, 0x34 );
    // Random seed initialization
    srand1( SX1276Random( ) );
}

uint8_t SendQueueToRadio(RadioMsgEvent_t type, void *data)
{
    RadioMsgBody_t sendBody;
    sendBody.type = type;

    switch (sendBody.type)
    {
        case RadioTimeOut:
        {
            sendBody.body.TimeOutID = *(uint32_t *)data;
            break;
        }
        case SendUnConfirmedDataUp:
        case SendConfirmedDataUp:
        {
            RadioSendData_t temp = *(RadioSendData_t *)data;
            sendBody.body.SendData.DataLen = temp.DataLen;
            sendBody.body.SendData.AppPort = temp.AppPort;
            sendBody.body.SendData.retries = temp.retries;
            sendBody.body.SendData.Data = pvPortMalloc(temp.DataLen + 1);
            memset(sendBody.body.SendData.Data, 0x00, (temp.DataLen + 1));
            memcpy((char *)sendBody.body.SendData.Data, (char *)temp.Data, temp.DataLen);
            break;
        }
        case TxContinuousModeStart:
        case SlavePingPongStart:
        case MasterPingPongStart:
            sendBody.body.TxCM = *(RadioTxContinuousMode_t *)data;
            break;
        case MasterPingPongData:
        {
            RadioTestSendData_t temp = *(RadioTestSendData_t *)data;
            sendBody.body.SendData.DataLen = temp.DataLen;
            sendBody.body.SendData.Data = pvPortMalloc(temp.DataLen + 1);
            memset(sendBody.body.SendData.Data, 0x00, (temp.DataLen + 1));
            memcpy((char *)sendBody.body.SendData.Data, (char *)temp.Data, temp.DataLen);
            break;
        }
        default:
            break;
    }
    if(xQueueSend( Radio_Task_Queue, ( void * ) &sendBody, ( TickType_t ) 0 ) == pdTRUE)
        return 0;
    else
        return 1;
}

void SendQueueToRadioFromISR(RadioMsgEvent_t type)
{
    BaseType_t xHigherPriorityTaskWoken;
    RadioMsgBody_t sendBody;

    xHigherPriorityTaskWoken = pdFALSE;
    sendBody.type = type;
    xQueueSendFromISR( Radio_Task_Queue, ( void * ) &sendBody, &xHigherPriorityTaskWoken );

    if(xHigherPriorityTaskWoken == pdTRUE)
            portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

bool getRadioFactoryFlag(void)
{
    return FactoryMode;
}

uint8_t GetRadioTxAckTimeoutRetryCount()
{
    return TxAckTimeoutRetryCount;
}

uint8_t SetClassMode(DeviceClass_t classMode)
{
    if(classMode == CLASS_A)
    {
        LoRaMacDeviceClass = classMode;
/* Modified by Nick Date: 2017/03/21  Log: add for STM32L073xx double radioInit() in Class C, init radio in Class A on STM32L073xx */
/* Moidyf by Gavin, Date:2017/07/13 Log: Let SIPMODULE_BOARD have same behavior*/
#if defined( SIPMODULE_BOARD )
	radioInit();
#endif
    }
    else if(classMode == CLASS_B)
    {
        //Not support
        return FAIL;
    }
    else if(classMode == CLASS_C)
    {
        LoRaMacDeviceClass = classMode;
        radioInit();
        LoRaMacRxWindown2Setup(true);
    }
    else
    {
        return FAIL;
    }
    return OK;
}

DeviceClass_t GetClassMode(void)
{
    return LoRaMacDeviceClass;
}

/*!
*   Get CAD detect flag status
* \param [return]  CadDetectFlag        CAD detect flag
*/
bool GetCadDetectFlag (void) {
    return CadDetectFlag;
}

/*!
*   Set CAD detect flag status
* \param [IN]  value          CAD detect flag value
*/
void SetCadDetectFlag (bool value) {
    CadDetectFlag = value;
}
