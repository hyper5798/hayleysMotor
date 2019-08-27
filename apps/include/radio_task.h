#ifndef __RADIO_TASK_H__
#define __RADIO_TASK_H__

#ifdef __cplusplus
 extern "C" {
#endif
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdbool.h>

#define RADIO_TASK_QUEUE_LENGTH					( 5 )


/*------------------------------------------------------------------------------*/


/*!
 * LoRaWAN devices classes definition
 */
typedef enum eDeviceClass
{
    CLASS_A,
    CLASS_B,
    CLASS_C,
}DeviceClass_t;


/*  define All Message event */
typedef enum
{
    RadioTxDone = 0x0,
    RadioRxDone,
    RadioRxAck,
    RadioTxTimeout,
    RadioRxTimeout,
    RadioRxError,
    RadioFhssChangeChannel,
    RadioCadDone,
    RadioStdby,
    RadioLBTError,
    /*factory test*/
    RadioTestTxTimeout = 0x100,
    RadioTestRxTimeout,
    RadioTestPPMasterSettingDone,
    RadioTestPPSlaveSettingDone,
    RadioTestPPTxDone,
    RadioTestPPRxDone,
    RadioTestPPSlaveTimeout,
    RadioTestPPSlaveDone,
    RadioTestPPRxErr
}RadioStatusEvent_t;

typedef struct
{
    int8_t SnrValue;
    int16_t RssiValue;
    uint8_t Size;
    uint8_t *RxBuffer;
    uint8_t RxPort;
}RadioRxData_t;

typedef struct
{
    RadioStatusEvent_t      Status;
    union
    {
        RadioRxData_t           RxData; /*rx done*/
        bool channelActivityDetected ; /*cad done*/
        uint8_t currentChannel; /*FhssChangeChannel*/
    }body;
}RadioStatus_t;

typedef void( RadioIrqHandler )( RadioStatus_t );

typedef enum
{
    /*send to radio task*/
    SX1276TxTimeout = 0x0,
    SX1276RxTimeout,
    SX1276RxDelayTimeout1,
    SX1276RxDelayTimeout2,
    SX1276RxDelayTimeForClassC,
    SX1276LBTDelayTimeout,
    SX1276RetryTimeout,
    SX1276TotalTimer
}RadioTimerID_t;

typedef enum
{
	RadioTimeOut = 0x0,						/* Body: TimeOutID */
    RadioInit,                              /* Body: None*/
    RadioDeInit,                            /* Body: None*/
    SendUnConfirmedDataUp = 0x100,          /* Body: RadioSendData_t*/
    SendConfirmedDataUp,                    /* Body: RadioSendData_t*/
    ListenCADSymbol,                        /* Body: None*/
    RADIODIO0 = 0x200,                      /* Body: None*/
    RADIODIO1,                              /* Body: None*/
    RADIODIO2,                              /* Body: None*/
    RADIODIO3,                              /* Body: None*/
    RADIODIO4,                              /* Body: None*/
    RADIODIO5,                              /* Body: None*/
    /*factory test*/
    RadioTestTimeOut = 0x300,               /* Body: TimeOutID */
    TxContinuousModeStart,                  /* Body: RadioTxContinuousMode_t*/
    TxContinuousModeStop,                   /* Body: None*/
    MasterPingPongStart,                    /* Body: RadioTxContinuousMode_t*/
    MasterPingPongData,                     /* Body: RadioTestSendData_t*/
    SlavePingPongStart,                     /* Body: RadioTxContinuousMode_t*/
    PingPongStop                            /* Body: None*/
}RadioMsgEvent_t;

typedef struct
{
    uint8_t * Data;
    uint8_t DataLen;
    uint8_t AppPort;
    uint8_t retries;
}RadioSendData_t;

typedef struct
{
    double FreqVal;
    uint8_t FreqIdx;
    int8_t TxPwrVal;
    uint32_t BwdIdx;
    uint32_t SpFactorIdx;
    uint8_t CodeRateIdx;
    uint16_t FixLenPayloadOn;
    bool LoRaEnableCrcOn;
    bool IqInversionOn;
    uint16_t PingPongTxOrRxTimeoutIdx;
}RadioTxContinuousMode_t;

typedef struct
{
    uint8_t * Data;
    uint8_t DataLen;
}RadioTestSendData_t;

typedef struct
{
    RadioMsgEvent_t type;
    union
    {
        uint32_t TimeOutID;
        RadioSendData_t SendData;
        RadioTxContinuousMode_t TxCM;
    }body;
}RadioMsgBody_t;

extern TaskHandle_t xRadioTaskHandle;
void RadioDeTask(void);
void RadioTask( void * pvParameters );
void radioInit(void);
void SetRadioCallback(RadioIrqHandler *callback);
uint8_t SendQueueToRadio(RadioMsgEvent_t type, void *data);
void SendQueueToRadioFromISR(RadioMsgEvent_t type);
bool getRadioFactoryFlag(void);
uint8_t GetRadioTxAckTimeoutRetryCount(void);
uint8_t SetClassMode(DeviceClass_t classMode);
DeviceClass_t GetClassMode(void);
bool GetCadDetectFlag(void);
void SetCadDetectFlag (bool value);


#ifdef __cplusplus
}
#endif
#endif /*__RADIO_TASK_H__*/
