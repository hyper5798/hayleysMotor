#ifndef __UART_TASK_H__
#define __UART_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stdint.h"
#include "uart-board.h"
#include "radio_task.h"

#define UART_TASK_QUEUE_LENGTH					( 15 )
/* Size of Trasmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE
extern TaskHandle_t xUartTaskHandle;
/* Jason add size of sensor data on 2018.11.15*/
#define SENSOR_DATA_SIZE                  24
/*  define All Message evnt */
static const UartId_t UseUartID = UART_2;

typedef enum
{
    RadioTxDelayTimerID = 0,
    TXPPSlaveTimerID,
    UartTotalTimer
} UartTimerID_t;


typedef struct
{
    uint8_t * Data;
    uint8_t DataLen;
} UartData_t;

typedef enum
{
    /*from other task*/
    UartEnable = 0x0,                   /* Body: None */
    UartDisable,                        /* Body: None */
    UartSendString,                     /* Body: TaskData */
    UartStopRadioTxDelay,
    UartLowPowerModeCount,
    UartLowPowerModeWakeup,
    UartRadioTxDone,
    UartRadioRxDone,
    UartRadioRxAck,
    UartRadioTxTimeout,
    UartEnableEcho,
    UartDisableEcho,
    UartRadioTestStatus,                   /* Body: UartTestStauts_t*//*For radiotest_task*/
    UartRadioTestStop,
#if defined(STM32L073xx)
    UartStopRadioTxDelay1,    /* Add by Eric Date: 2017/02/22   Log: Add UartStopRadioTxDelay1(gtxd=1) to avoid Rx2 receive issue.  */
#endif
    /*from ISR*/
    UartTxDone = 0x100,				    /* Body: None */
    UartRxDone,                         /* Body: ISRData */
    UartRTCAlarm,  //daniel add on 2017.3.6 for RTC alarm
    UartStopReset, //daniel add on 2017.4.10 for reset to default
		UartCheckReply,
		UartToSleep,
} UartMsgEvent_t;

typedef struct
{
    UartMsgEvent_t	type;
    UartId_t        uartId;
    union
    {
        uint8_t              ISRData;
        uint32_t            TimerID;
        uint32_t            UartBaudrate;
        UartData_t         TaskData;
        RadioStatus_t     RadioStatus;
    } body;
} UartMsgBody_t;

void UartTask(void * pvParameters);
void UartDeTask(void);
uint8_t SendQueueToUartTask(UartMsgEvent_t type, UartId_t uartId, void *data);
uint8_t SendQueueToUartTaskFromISR(UartMsgEvent_t type, UartId_t uartId, void *data);
void SendMsgToUart(const uint8_t* msg, UartId_t uartId);
void SendToUartImmediately(uint8_t* str);
void SetPingPongMaxTxNumOrRxTimeIdx(uint16_t value);
void SetTXPPLable(uint8_t* label);
void WakeUpLPMCallback(void);
//Jason add on 2018.12.19
void delay_ms(uint16_t time);
void PWR5V(uint32_t st);
void BatDetect(uint32_t st);
uint16_t calADC(uint32_t channel, uint8_t sample_num);
//Jason add on 2018.12.20 for pin output test
void PinTest(uint32_t pinNumber, uint32_t pinValue);

#ifdef __cplusplus
}
#endif
#endif /*__UART_TASK_H__*/
