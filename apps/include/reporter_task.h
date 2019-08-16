#ifndef __REPORTER_TASK_H__
#define __REPORTER_TASK_H__

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "radio_task.h"
#include "queue.h"
#include "at_command_list.h"
#include "rtc-board.h"


#define REPORTER_TASK_QUEUE_LENGTH					( 5 )

typedef struct ReporterConfig
{
    uint8_t IrqTriggerType_0;
    uint8_t SyncFlag;
    uint8_t AppPort;
    int16_t RecPort;
    uint16_t GpioReportTime_day;
    uint16_t GpioReportTime_minutes;
    uint16_t GpioReportTime_day1;
    uint16_t GpioReportTime_minutes1;

}ReporterConfig_t;

typedef enum
{
    /*from other task*/
    ReporterEnable,
    ReporterDisable,
    /*from ISR*/
    ReporterGpioIrq0 = 0x100,		          /* Body: None */
    ReporterGpioIrq1,                     /* Body: None */
    ReporterRtcIrq,
    ReporterResetIrq0,
    ReporterStopRestore,
    ReporterRadioRxDone,

}ReporterMsgEvent_t;


typedef struct
{
    ReporterMsgEvent_t	type;

    union
    {
        uint16_t		        cycleTime;
        ReporterConfig_t ReporterConfig;
        RadioStatus_t     RadioStatus;
        /*TBD*/
    }body;

}ReporterMsgBody_t;


void ReporterTask( void * pvParameters );
uint8_t SendQueueToReporterTask( ReporterMsgEvent_t type, void *data );
uint8_t SendQueueToReporterTaskFromISR( ReporterMsgEvent_t type, void *data );

void InitReporterConfig(ReporterConfig_t reporterConfig);

#ifdef __cplusplus
}
#endif
#endif /*__REPORTER_TASK_H__*/
