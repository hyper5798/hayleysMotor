#ifndef __TIMER_TASK_H__
#define __TIMER_TASK_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#if defined (TRACKER_BOARD_V1) && defined(LORAWANV1_0) //tracker with LoRa WAN 1.0
    #define TIMERMAX 10
#else
/* Add by Gavin Date: 2017/03/20   Log:For prevent hang issue when running SYNIN modbus class A flow*/    
#if defined (MODBUS_MASTER_SIPMODULE) || defined (MODBUS_MASTER_WHITEBOX)    
    #define TIMERMAX 6
#elif defined (MANHOLE_BOARD)
/* Add by Griffith Date: 2017/05/08   Log: Enlarge the timer queue, to prevent hang issue in the special case*/
    #define TIMERMAX 10
#else
    #define TIMERMAX 5
#endif    
#endif

#define TIMER_TASK_QUEUE_LENGTH 10

typedef void( SWTimerHandler )( uint32_t );

typedef enum
{
    TimerState_IDLE = 0,
    TimerState_USED,
}TimerState_t;

typedef enum
{
    NoHWRunning = 0,
    HWRunning,
}TimerRunningState_t;

typedef struct
{
    uint32_t swID;
    TimerState_t timerStateFlag;
    uint32_t count;
    uint32_t value;
    uint8_t repeat;
    SWTimerHandler *CallBackHandler;
}TimerTask_t;


void TimerTask( void * pvParameters );
/*unit: one msec*/
uint32_t AddTimer(uint32_t id, uint32_t msec, uint8_t repeat, SWTimerHandler * Callback);
uint32_t DeleteTimer(uint32_t id);
void SwTimerStart(void);
void SwTimerStop(void);
void CreateTimerTaskQueue(void);
uint32_t GetRemainTime(uint32_t id);
uint32_t ReStartTimer(uint32_t id);

#ifdef __cplusplus
}
#endif
#endif /*__TIMER_TASK_H__*/
