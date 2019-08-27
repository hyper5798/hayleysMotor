#include "timer_task.h"
#include "hwtimer-board.h"
#include <string.h>

QueueHandle_t Timer_Task_Queue = NULL;
SemaphoreHandle_t Timer_Task_Sem = NULL;
TimerRunningState_t timerRunningFlag = NoHWRunning;
TimerTask_t TimerArray[TIMERMAX];

void TimerStateCallback(void)
{
    BaseType_t xHigherPriorityTaskWoken;
    TimerTask_t data;
//    data.timerStateFlag = FromISR;
    xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR( Timer_Task_Queue, ( void * ) &data, &xHigherPriorityTaskWoken );
    if(xHigherPriorityTaskWoken == pdTRUE)
        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

void CreateTimerTaskQueue(void)
{
    Timer_Task_Queue = xQueueCreate( TIMER_TASK_QUEUE_LENGTH, sizeof(TimerTask_t) );
    Timer_Task_Sem = xSemaphoreCreateMutex();
    memset(TimerArray, 0x00, sizeof(TimerArray));
    HwTimerInit();
    HwTimerSetCallback(TimerStateCallback);
}

void TimerTask( void * pvParameters )
{
    //StateMsgBody_t data;
    TimerTask_t TimerData;
    
    for( ;; )
    {
        uint8_t i = 0;
        uint8_t sendFlag = 0;
        xQueueReceive( Timer_Task_Queue, &TimerData, portMAX_DELAY);
        sendFlag = 0;
        if( xSemaphoreTake( Timer_Task_Sem, portMAX_DELAY) == pdTRUE )
        {
            for(i = 0; i < TIMERMAX; i++)
            {
                if(TimerArray[i].timerStateFlag == TimerState_USED)
                {
                    TimerArray[i].count--;
                }
                if((TimerArray[i].count == 0) && (TimerArray[i].timerStateFlag == TimerState_USED))
                {
                    uint32_t tempTimerID = TimerArray[i].swID;
                    
                    sendFlag = 1;
                    if(TimerArray[i].CallBackHandler != NULL)
                        TimerArray[i].CallBackHandler(tempTimerID);
                    
                    if(TimerArray[i].repeat == 1)
                    {
                        TimerArray[i].count = TimerArray[i].value;
                    }
                    else
                    {
                        TimerArray[i].count = 0;
                        TimerArray[i].swID = 0;
                        TimerArray[i].value = 0;
                        TimerArray[i].repeat = 0;
                        TimerArray[i].timerStateFlag = TimerState_IDLE;
                        TimerArray[i].CallBackHandler = NULL;
                    }
                }
            }
            xSemaphoreGive( Timer_Task_Sem );
        }
        else
        {
            while(1);
        }
        /*when timeout, check if stop hw timer*/
        if( sendFlag == 1)
        {
            for(i = 0; i < TIMERMAX; i++)
            {
                if(TimerArray[i].timerStateFlag == TimerState_USED)
                {
                    break;
                }
                if(i == (TIMERMAX -1))
                {
                    HwTimerStop();
                    timerRunningFlag = NoHWRunning;
                }
            }
        }
    }
}

uint32_t AddTimer(uint32_t id, uint32_t msec, uint8_t repeat, SWTimerHandler * Callback)
{
    TimerTask_t data;
    uint8_t i = 0;
    data.count = msec;
    data.value = msec;
    data.repeat = repeat;
    data.swID = id;
    data.timerStateFlag = TimerState_USED;
    data.CallBackHandler = Callback;
    if( xSemaphoreTake( Timer_Task_Sem, portMAX_DELAY) == pdTRUE )
    {
        for(i = 0; i < TIMERMAX; i++)
        {
            if(TimerArray[i].timerStateFlag == TimerState_IDLE)
            {
                TimerArray[i] = data;
                if(timerRunningFlag == NoHWRunning)
                {
                    HwTimerStart();
                    timerRunningFlag = HWRunning;
                }
                break;
            }
            if(i == (TIMERMAX -1))
            {
                while(1);
            }
        }
        
        xSemaphoreGive( Timer_Task_Sem );
        //xQueueSend( Timer_Task_Queue, ( void * ) &data, ( TickType_t ) 0 );
    }
    else
    {
        while(1);
    }
    
    return (uint32_t)&TimerArray[i];
}

uint32_t DeleteTimer(uint32_t id)
{
    if(id == 0)
        return 0;
  
    if( xSemaphoreTake( Timer_Task_Sem, portMAX_DELAY) == pdTRUE )
    {
        uint8_t i;
        TimerTask_t* data = (TimerTask_t *)id;
        data->count = 0;
        data->value = 0;
        data->repeat = 0;
        data->swID = 0;
        data->timerStateFlag = TimerState_IDLE;
        data->CallBackHandler = NULL;

        i = 0;
        for(i = 0; i < TIMERMAX; i++)
        {
            if(TimerArray[i].timerStateFlag == TimerState_USED)
            {
                break;
            }
            if(i == (TIMERMAX -1))
            {
                HwTimerStop();
                timerRunningFlag = NoHWRunning;
            }
        }
//        xQueueSend( Timer_Task_Queue, ( void * ) &data, ( TickType_t ) 0 );
        xSemaphoreGive( Timer_Task_Sem );
    }
    else
    {
        while(1);
    }
    
    return 0;
}

uint32_t GetRemainTime(uint32_t id)
{
    uint32_t RemainCount;
    if(id == 0)
        return 0;
  
    if( xSemaphoreTake( Timer_Task_Sem, portMAX_DELAY) == pdTRUE )
    {
        TimerTask_t* data = (TimerTask_t *)id;
        RemainCount = data->count;
        xSemaphoreGive( Timer_Task_Sem );
    }
    else
    {
        while(1);
    }
    
    return RemainCount;
}

uint32_t ReStartTimer(uint32_t id)
{
    if(id == 0)
        return 0;
  
    if( xSemaphoreTake( Timer_Task_Sem, portMAX_DELAY) == pdTRUE )
    {
        TimerTask_t* data = (TimerTask_t *)id;
        data->count = data->value;
        xSemaphoreGive( Timer_Task_Sem );
    }
    else
    {
        while(1);
    }
    
    return 0;
}

void SwTimerStart(void)
{
    HwTimerStart();
}

void SwTimerStop(void)
{
    HwTimerStop();
}

