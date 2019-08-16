#include "reporter_task.h"
#include "gpio-board.h"
#include "adc-board.h"
#include "timer_task.h"

#define APP_DATA_SIZE 7

static QueueHandle_t Reporter_Task_Queue = NULL;
static uint8_t IRQ0Enable = 0;
static uint8_t IRQ0Flag = 0;
static uint32_t IRQ0DelayTime = 1000;// 1 Seconds
static uint32_t IRQ0Timer = NULL;

static uint32_t ReportCycleTime;
static uint32_t ReportCycleTime1; // for second timer

uint8_t RestoreDefaultPinCount;
uint32_t RestoreDefalutTimer;

uint8_t SyncFlag = 0;
uint8_t AppPort = 2;
int16_t RecPort = -1;
uint8_t PA8Status = 0;

uint32_t PowerOnTxTimer;

void ResetIRQ0(uint32_t timer)
{
    SendQueueToReporterTaskFromISR(ReporterResetIrq0, NULL);
}

void IRQ0Callback(void)
{
    if(IRQ0Flag == 0)
    {
        IRQ0Flag = 1;
        SendQueueToReporterTaskFromISR(ReporterGpioIrq0, NULL);
    }
}

void RTCAlarmStateCallback(MCUALARM_e AlarmName)
{
    SendQueueToReporterTaskFromISR(ReporterRtcIrq, NULL);
}

void DetectRestoreDefaultCallback(uint32_t time)
{
    Gpio_t RestoreDefalutPin;
    GpioInit(&RestoreDefalutPin, LED_FCT, GPIO_PINMODE_READ_INTERRUPT, GPIO_PINCONFIGS_READ_INTERRUPT, GPIO_PINTYPES_READ_INTERRUPT, 0);
    if(GpioReadInput(&RestoreDefalutPin) == 1)
    {
        RestoreDefaultPinCount++;
        if(RestoreDefaultPinCount == ENTER_RESTORE_TIME)
        {
            SendQueueToReporterTask(ReporterStopRestore, NULL);
            RestoreUserConfigToDefault();
            SaveConfigToEEPROM();
        }
    }
    else
    {
        SendQueueToReporterTask(ReporterStopRestore, NULL);
    }
    GpioInit(&RestoreDefalutPin, LED_FCT, GPIO_PINMODE_UNUSED, GPIO_PINCONFIGS_UNUSED, GPIO_PINTYPES_UNUSED, 0);
}

void ReportTaskCallback(RadioStatus_t event)
{
    if(event.Status == RadioRxDone)
    {
        SendQueueToReporterTask(ReporterRadioRxDone, &event);
    }
}

void PowerOnTx(uint32_t time)
{
    //Enable IRQ
    if(IRQ0Enable == 1)
    {
        InitATInterrupt("PB6",IRQ0Callback);
    }
    SendQueueToReporterTask(ReporterRtcIrq, NULL);
}

void InitReporterConfig(ReporterConfig_t reporterConfig)
{
    SyncFlag = reporterConfig.SyncFlag;
    AppPort = reporterConfig.AppPort;
    RecPort = reporterConfig.RecPort;

    if(reporterConfig.IrqTriggerType_0 == 1)
    {
        IRQ0Enable = 1;
    }

    if(reporterConfig.GpioReportTime_minutes != 0)
    {
        ReportCycleTime = reporterConfig.GpioReportTime_minutes * SECS_IN_MIN;
    }
    else if(reporterConfig.GpioReportTime_day != 0)
    {
        ReportCycleTime = reporterConfig.GpioReportTime_day * SECS_IN_DAY;
    }
    else
    {
        ReportCycleTime = 0;
    }

    if(reporterConfig.GpioReportTime_minutes1 != 0)
    {
        ReportCycleTime1 = reporterConfig.GpioReportTime_minutes1 * SECS_IN_MIN;
    }
    else if(reporterConfig.GpioReportTime_day1 != 0)
    {
        ReportCycleTime1 = reporterConfig.GpioReportTime_day1 * SECS_IN_DAY;
    }
    else
    {
        ReportCycleTime1 = 0;
    }
}

void ReporterTask(void * pvParameters)
{
    ReporterMsgBody_t ReporterReceived;
    RtcSetAlarmCallback(RTCAlarmStateCallback);
    SetRadioCallback(ReportTaskCallback);
    /* Create the queue as described at the top of this file. */
    Reporter_Task_Queue = xQueueCreate(REPORTER_TASK_QUEUE_LENGTH, sizeof(ReporterMsgBody_t));
    configASSERT(Reporter_Task_Queue);
    /* Detect default pin every 1 seconds. Restore to default if count 10 times. */
    RestoreDefaultPinCount = 0;
    RestoreDefalutTimer = AddTimer(RestoreDefalutTimer, 1000, 1, DetectRestoreDefaultCallback);
    for(;;)
    {
        xQueueReceive(Reporter_Task_Queue, &ReporterReceived, portMAX_DELAY);
        switch(ReporterReceived.type)
        {
            case ReporterStopRestore:
            {
                RestoreDefalutTimer = DeleteTimer(RestoreDefalutTimer);
                break;
            }
            case ReporterEnable:
            {
                RestoreDefalutTimer = AddTimer(PowerOnTxTimer, 50, 0, PowerOnTx);
                break;
            }
            case ReporterDisable:
            {
                RtcDisableAlarm(MCU_ALARM_A);
                break;
            }
            case ReporterGpioIrq0:
            {
                uint8_t PB7Status;
                RadioSendData_t SendData;
                uint8_t AppData[APP_DATA_SIZE] = {0};
                PB7Status = ReadGpioStatus("PB7");
                // Check PB7 for updating RTC timer
                if(PB7Status == 1 && ReportCycleTime1 !=0)
                {
                    RtcStartWakeUpAlarm(ReportCycleTime1,MCU_ALARM_A);
                }
                else
                {
                    if(ReportCycleTime != 0)
                    {
                        RtcStartWakeUpAlarm(ReportCycleTime,MCU_ALARM_A);
                    }
                }
                AppData[0] = 0;
                AppData[1] = 0x00 | ReadGpioStatus("PA12") << 3 | ReadGpioStatus("PA11") <<2
                             | PA8Status << 1 | ReadGpioStatus("PB8");
#if defined( STM32L073xx )
                AppData[2] = 0;
                AppData[3] = GetPinVoltage(ADC_CHANNEL_8);
                AppData[4] = 0;
                AppData[5] = GetPinVoltage(ADC_CHANNEL_9); // PB 1;
#else
                AppData[2] = 0;
                AppData[3] = GetPinVoltage(ADC_Channel_8); //PB0
                AppData[4] = 0;
                AppData[5] = GetPinVoltage(ADC_Channel_9); // PB 1
#endif
                AppData[6] = 0x00 | PB7Status << 1 | 1;//PB6 is interrput that already triggered so PB6 set 1
                SendData.DataLen = sizeof(AppData);
                SendData.Data = AppData;
                SendData.AppPort = AppPort;
                if(SyncFlag == 0)
                {
                    SendQueueToRadio(SendUnConfirmedDataUp, &SendData);
                }
                else
                {
                    SendData.retries = 3;
                    SendQueueToRadio(SendConfirmedDataUp, &SendData);
                }
                //Reset IRQ0
                IRQ0Timer = AddTimer(IRQ0Timer, IRQ0DelayTime, 0, ResetIRQ0);
                if(IRQ0Enable == 1)
                {
                    //ReadGpioStatus will DeInit interrupt settings, so reInit PB6
                    InitATInterrupt("PB6",IRQ0Callback);
                }
                break;
            }
            case ReporterRtcIrq:
            {
                uint8_t PB7Status;
                RadioSendData_t SendData;
                uint8_t AppData[APP_DATA_SIZE] = {0};
                PB7Status = ReadGpioStatus("PB7");
                if(PB7Status == 1 && ReportCycleTime1 !=0)
                {
                    RtcStartWakeUpAlarm(ReportCycleTime1,MCU_ALARM_A);
                }
                else
                {
                    if(ReportCycleTime != 0)
                    {
                        RtcStartWakeUpAlarm(ReportCycleTime,MCU_ALARM_A);
                    }
                }
                AppData[0] = 0;
                AppData[1] = 0x00 | ReadGpioStatus("PA12") << 3 | ReadGpioStatus("PA11") <<2
                             | PA8Status << 1 | ReadGpioStatus("PB8");
#if defined( STM32L073xx )
                AppData[2] = 0;
                AppData[3] = GetPinVoltage(ADC_CHANNEL_8); //PB0;
                AppData[4] = 0;
                AppData[5] = GetPinVoltage(ADC_CHANNEL_9); // PB 1;
#else
                AppData[2] = 0;
                AppData[3] = GetPinVoltage(ADC_Channel_8); //PB0
                AppData[4] = 0;
                AppData[5] = GetPinVoltage(ADC_Channel_9); // PB 1
#endif
                AppData[6] = 0x00 | PB7Status << 1 | ReadGpioStatus("PB6");
                SendData.DataLen = sizeof(AppData);
                SendData.Data = AppData;
                SendData.AppPort = AppPort;
                if(SyncFlag == 0)
                {
                    SendQueueToRadio(SendUnConfirmedDataUp, &SendData);
                }
                else
                {
                    SendData.retries = 3;
                    SendQueueToRadio(SendConfirmedDataUp, &SendData);
                }
                if(IRQ0Enable == 1)
                {
                    //ReadGpioStatus will DeInit interrupt settings, so reInit PB6
                    InitATInterrupt("PB6",IRQ0Callback);
                }
                break;
            }
            case ReporterResetIrq0:
            {
                IRQ0Flag = 0;
                break;
            }
            case ReporterRadioRxDone:
            {
                RadioStatus_t radioStatus = ReporterReceived.body.RadioStatus;
                if(RecPort == -1 || radioStatus.body.RxData.RxPort == RecPort)
                {
                    Gpio_t pin;
                    if(radioStatus.body.RxData.RxBuffer[0] == 0)
                    {
                        PA8Status = 0;
                        GpioInit(&pin, UART1_CK, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
                        GpioWrite(&pin,0);
                    }
                    else if(radioStatus.body.RxData.RxBuffer[0] == 1)
                    {
                        PA8Status = 1;
                        GpioInit(&pin, UART1_CK, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 1);
                        GpioWrite(&pin,1);
                    }
                }
                if(radioStatus.body.RxData.RxBuffer != NULL)
                {
                    vPortFree(radioStatus.body.RxData.RxBuffer);
                    radioStatus.body.RxData.RxBuffer = NULL;
                }
                break;
            }
            default:
                break;
        }
    }
}


uint8_t SendQueueToReporterTask(ReporterMsgEvent_t type, void *data)
{
    ReporterMsgBody_t sendBody;
    sendBody.type = type;
    switch(sendBody.type)
    {
        case ReporterEnable:
        {
            break;
        }
        case ReporterRadioRxDone:
        {
            RadioStatus_t tmp = *(RadioStatus_t *)data;
            sendBody.body.RadioStatus.Status = tmp.Status;
            sendBody.body.RadioStatus.body.RxData.RssiValue = tmp.body.RxData.RssiValue;
            sendBody.body.RadioStatus.body.RxData.SnrValue = tmp.body.RxData.SnrValue;
            sendBody.body.RadioStatus.body.RxData.RxPort = tmp.body.RxData.RxPort;
            sendBody.body.RadioStatus.body.RxData.Size = tmp.body.RxData.Size;
            sendBody.body.RadioStatus.body.RxData.RxBuffer = pvPortMalloc(tmp.body.RxData.Size + 1);
            memset(sendBody.body.RadioStatus.body.RxData.RxBuffer, 0x00, (tmp.body.RxData.Size + 1));
            memcpy((char *)sendBody.body.RadioStatus.body.RxData.RxBuffer, (char *)tmp.body.RxData.RxBuffer, tmp.body.RxData.Size);
            break;
        }
        case ReporterDisable:
        default:
            break;
    }
    if(xQueueSend(Reporter_Task_Queue, (void *) &sendBody, (TickType_t) 0) == pdTRUE)
        return 0;
    else
        return 1;
}

uint8_t SendQueueToReporterTaskFromISR(ReporterMsgEvent_t type, void *data)
{
    ReporterMsgBody_t sendBody;
    BaseType_t xHigherPriorityTaskWoken;
    sendBody.type = type;
    xHigherPriorityTaskWoken = pdFALSE;
    switch(sendBody.type)
    {
        case ReporterGpioIrq0:
        {
            break;
        }
        case ReporterRtcIrq:
        {
            break;
        }
        default:
            break;
    }
    if(xQueueSendFromISR(Reporter_Task_Queue, (void *) &sendBody, (TickType_t) 0) == pdTRUE)
    {
        if(xHigherPriorityTaskWoken == pdTRUE)
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return 0;
    }
    else
        return 1;
}

