#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "uart_task.h"
#include "board.h"
#include "uart-board.h"
#include "gpio-board.h"
#include "radio_task.h"
#include "timer_task.h"
#include "at_command_app.h"
#include "at_command_list.h"
#include "reporter_task.h"
#include "adc-board.h" //daniel add on 2017.5.6

#if defined( STM32L073xx )
#define FIFO_TX_SIZE                                512
#else
#define FIFO_TX_SIZE                                256
#endif
#define FIFO_RX_SIZE                                2

//Jason add for test mode
bool testFlag = true;
uint32_t testTimer; //Jason add for test at 2019.08.16
void firstUartRTCAlarm(uint32_t timer);

uint8_t *UartTxBuffer = NULL;
uint8_t *UartRxBuffer = NULL;

//add on 2019.5.17
uint8_t WirelessModule = 1; //0: Lora 1:CAT-M1
uint8_t *UartTxBuffer2 = NULL;
uint8_t *UartRxBuffer2 = NULL;
uint8_t testBuff[80];
uint8_t myStr[8];
UartData_t UART2senddata;
uint8_t cmdNumber = 0;

Uart_t Uart1;
Uart_t Uart2;

Uart_t UartPort[UART_Total];

static uint8_t myPollingMode; //daniel add on 2017.3.28 for check working mode
static uint32_t myReportCycleTime; //daniel add on 2017.2.17 for RTC timer
static SensorConfig_t mySenConfig; //daniel add on 2017.2.2 for get sensor config
uint8_t ResetDefaultPinCount; //daniel add on 2017.4.10 for count reset to default pin time
uint32_t ResetDefalutTimer; //daniel add on 2017.4.10 for reset to default timer
uint32_t PollingTimer; //daniel add on 2017.4.21 for get UART sensor data timer
uint32_t CheckReplyTimer; //daniel add on 2017.5.15 for check if UART sensor response timer
uint32_t CheckReplyTimer2; //Jason add on 2018.11.15 for check if UART sensor 2 response timer
uint32_t CheckReplyTimer3; //Jason add on 2018.11.15 for check if UART sensor 3 response timer
static uint8_t UartSensorReplyFlag; //daniel add on 2017.5.15 for check if UART sensor response
static uint8_t UartSensorSendFlag = 0; //daniel add on 2018.5.8 for check if send cmd to UART sensor
void GetSensorDataCallback(uint32_t time); //daniel add on 2017.4.21 for timer callback
void GetSensorDataCallback2(uint32_t time); //Jason add on 2018.11.15 for timer callback
void GetSensorDataCallback3(uint32_t time); //Jason add on 2018.11.15 for timer callback
void DeInitMyPin(void); //daniel add on 2018.5.13 for deinit the pin that we use
uint16_t AdcBat = 0; //daniel add on 2018.5.8
uint32_t stopMotorTimer; //Jason add on 2019.3.13 for auto stop if motot motion timer

static QueueHandle_t Uart_Task_Queue = NULL;
TaskHandle_t xUartTaskHandle = NULL;

/*For AT command*/
UartData_t sendData;
uint32_t receivedLength = 0;
static CommandLine_t receivedCommandLine = {0};
static unsigned char receivedCharBuffer[ATCOMMAND_BUFFER_SIZE];
static uint8_t EchoEnable = 0;

/* Jason add  sensor data on 2018.10.31*/
uint32_t mySensorType = 0;
unsigned char sensorData[SENSOR_DATA_SIZE];
uint8_t sensorDataLength; //Jason add on 2018.10.31 for caulate sensor data length
void SaveSensorData(unsigned char *receivedata);
void sendSensorData(uint8_t type, unsigned char *data);
void CheckSensor1ReplyCallback(uint32_t time);//Jason add on 2018.11.02 for sensor 1 time out
void CheckSensor2ReplyCallback(uint32_t time);// Jason add on 2018.11.02 for sensor 2 time out
void CheckSensor3ReplyCallback(uint32_t time);// Jason add on 2018.11.02 for sensor 3 time out
void checkBTCommand(unsigned char *receivedata);// Jason modify on 2019.05.03 for BT Command check
void replyCommand(unsigned char *receivedata);// Jason add on 2019.03.13 for BT Command reply

// Jason add for control on 2019.01.25
// uint8_t up_flag = 0;
// uint8_t down_flag = 0;
uint8_t read_size;
uint8_t read_buff[32];
RadioSendData_t ControlSendData;
uint32_t UpDetectTimer; //Jason add on 2018.12.24 for detect pc_2 value
uint32_t DownDetectTimer; //Jason add on 2018.12.24 for detect pc_3 value
void UpFBRiseCallback(uint32_t st);
void DownFBRiseCallback(uint32_t st);
PinNames upFBPin;
PinNames downFBPin;
bool BTFlag = false;
uint32_t pulseTime = 100;//Jason add on 2019.03.13 for pulse time of relay board  (unit: ms)
uint32_t stop2OnTime = 200;//Jason add on 2019.03.13 for pulse time of relay board  (unit: ms)

/*For radiotest_task*/
#define PRODUCT_TEST_BUFFER_SIZE    9
const uint8_t PP_MESG_PING[] 	= "PING";
const uint8_t PP_MESG_DONE[] 	= "DONE";
uint8_t TXPPLabel[5];
uint16_t PingPongMaxTxNumOrRxTimeIdx;
uint16_t PingPongRxCount;
uint32_t TXPPSlaveSWTimer;
void TXPPTimeout(uint32_t timer);

/*For boot flow*/
uint32_t RoutineTaskTimer;

/*For low power mode*/
Gpio_t Uart1RxInterrupt;
uint8_t EnterLPMTime;
uint8_t EnterLPMCount;
uint32_t BaudRateBackup;
uint32_t EnterLPMCountTimer;

/*for pin c 0~7 */
Gpio_t ioIn0;
Gpio_t ioIn1;
Gpio_t ioIn2;
Gpio_t ioIn3;
Gpio_t ioOut4;
Gpio_t ioOut5;
Gpio_t ioOut6;
Gpio_t ioOut7;

//Eagle 馬達驅動器
#if defined( HAYLEYS_MOTOR )
#define SENSOR_MSG_HEADER               0x01
#define SENSOR_MSG_LENGTH                 65     //定義馬達驅動器資料長度   
#define EAGLE_APP_DATA_SIZE							0x22     //定義馬達驅動器透過LORA傳送的資料長度
int8_t UartCmd[8] = {0x01, 0x03, 0x00, 0x05, 0x00, 0x1E, 0xD5, 0xC3};      //馬達驅動器命令     
uint16_t BatVal0;                                //讀取電壓值定義
unsigned int i;                                  //定義重傳次數變數
uint8_t teg = 0;                                   //定義teg為uint8_t 初始值=0
uint8_t readFlag = false;
uint8_t EagleAppdata[EAGLE_APP_DATA_SIZE];
RadioSendData_t EagleSendData;
uint32_t EagleTimer;
#endif

//daniel add on 2017.4.21 for timer callback
void Max485RW(uint32_t st)
{
    Gpio_t SendGpio;
    GpioInit(&SendGpio, PA_11, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1); //modify on 2017.7.28
    GpioWrite(&SendGpio, st);
}

#if defined( HAYLEYS_MOTOR )
void EagleUartSend(uint32_t timer)
{
    Max485RW(1); //daniel modify on 2017.3.28 for set max485 to write mode
    memset(receivedCharBuffer,'\0',ATCOMMAND_BUFFER_SIZE);
    receivedLength = 0;
    sendData.Data = (uint8_t*)UartCmd;
    sendData.DataLen = 8;
    // Jason modify to UART_1 on 2019.05.20
    //SendQueueToUartTask(UartSendString, UseUartID, &sendData);
    SendQueueToUartTask(UartSendString, UART_1, &sendData);
    sendData.Data = NULL;
    sendData.DataLen = 0;
    teg = 1;
}
#endif

//Eagle 馬達驅動器

// Jason add for control on 2019.01.25
void SendCMDToUart(const uint8_t* msg, uint8_t len, UartId_t uartId)
{
    sendData.Data = (uint8_t*)msg;
    sendData.DataLen = len;
    SendQueueToUartTask(UartSendString, uartId, &sendData);
    sendData.Data = NULL;
    sendData.DataLen = 0;
}

void UartDeTask(void)
{
    vQueueDelete(Uart_Task_Queue);
    vTaskDelete(xUartTaskHandle);
}

void UartStateCallback(UartId_t uartId, UartNotifyId_t notifyId, uint8_t data)
{
    if(notifyId == UART_NOTIFY_TX)
    {
        SendQueueToUartTaskFromISR(UartTxDone, uartId, (void *)&data);
    }
    else if(notifyId == UART_NOTIFY_RX)
    {
        SendQueueToUartTaskFromISR(UartRxDone, uartId, (void *)&data);
    }
}

void RadioTaskCallback(RadioStatus_t event)
{
    if(event.Status == RadioTxDone)
    {
        SendQueueToUartTask(UartRadioTxDone,(UartId_t)NULL,NULL);
    }
    else if(event.Status == RadioTxTimeout)
    {
        SendQueueToUartTask(UartRadioTxTimeout,(UartId_t)NULL,NULL);
    }
    else if(event.Status == RadioRxDone)
    {
        SendQueueToUartTask(UartRadioRxDone,(UartId_t)NULL, &event);
    }
    else if(event.Status == RadioRxAck)
    {
        SendQueueToUartTask(UartRadioRxAck,(UartId_t)NULL, &event);
    }
    else if(event.Status >= RadioTestTxTimeout || event.Status <= RadioTestPPRxErr)
    {
        SendQueueToUartTask(UartRadioTestStatus,(UartId_t)NULL, &event);
    }
}

void SendToUartImmediately(uint8_t* str)
{
    //This function should only used before disable the UART
    int i;
    uint8_t len = strlen((const char*)str);

    for(i=0; i<len; i++)
    {
#if defined( STM32L073xx )
        UartPutChar_NoIRQ(&UartPort[UseUartID], str[i]);
#else
        UartPutChar(&UartPort[UseUartID], str[i]);
#endif
    }
}

void SendBackToUart(uint8_t ch)
{
    if(ch == CHAR_CR|| ch == CTRL_C)//"Enter" or "ctrl + c"
    {
#if defined( STM32L073xx )
        UartPutChar_NoIRQ(&UartPort[UseUartID], '\r');
        UartPutChar_NoIRQ(&UartPort[UseUartID], '\n');
#else
        UartPutChar(&UartPort[UseUartID], '\r');
        UartPutChar(&UartPort[UseUartID], '\n');
#endif
    }
    else if(ch == CHAR_BACKSPACE)
    {
#if defined( STM32L073xx )
        UartPutChar_NoIRQ(&UartPort[UseUartID], '\b');
        UartPutChar_NoIRQ(&UartPort[UseUartID], ' ');
        UartPutChar_NoIRQ(&UartPort[UseUartID], '\b');
#else
        UartPutChar(&UartPort[UseUartID], '\b');
        UartPutChar(&UartPort[UseUartID], ' ');
        UartPutChar(&UartPort[UseUartID], '\b');
#endif
    }
    else
    {
#if defined( STM32L073xx )
        UartPutChar_NoIRQ(&UartPort[UseUartID], ch);
#else
        UartPutChar(&UartPort[UseUartID], ch);
#endif
    }
}

void StartReporterTaskCallback(uint32_t time)
{
    ReporterConfig_t reportConfig;

    /*DeInit no Used Item*/
    SendToUartImmediately("Start Report Mode\r\n\r\n");
    SendQueueToUartTask(UartDisable,UseUartID,NULL);
    SetRadioCallback(NULL);
    UartDeTask();

    /*Start Reporter Task*/
    xTaskCreate(ReporterTask, "ReporterTask", 96, NULL, 3, NULL);

    reportConfig.GpioReportTime_minutes = AtConfig.GpioReportTime_minutes;
    reportConfig.GpioReportTime_day = AtConfig.GpioReportTime_day;
    reportConfig.IrqTriggerType_0 = AtConfig.IrqTriggerType_0;
    reportConfig.GpioReportTime_minutes1 = UserConfig1.GpioReportTime_minutes1;
    reportConfig.GpioReportTime_day1 = UserConfig1.GpioReportTime_day1;
    reportConfig.AppPort = UserConfig1.AppPort;
    reportConfig.RecPort = UserConfig1.RecPort;
    reportConfig.SyncFlag = UserConfig1.SyncFlag;

    InitReporterConfig(reportConfig);
    SendQueueToReporterTask(ReporterEnable, NULL);
}

void WakeUpLPMCallback()
{
    if(EnterLPMCount == EnterLPMTime)
    {
        EnterLPMCount = 0;
        SendQueueToUartTaskFromISR(UartEnable,UseUartID,&BaudRateBackup);
        SendQueueToUartTaskFromISR(UartLowPowerModeWakeup,(UartId_t)NULL,NULL);
    }
}

void CountLPMCallback(uint32_t timer)
{
    SendQueueToUartTask(UartLowPowerModeCount,(UartId_t)NULL,NULL);
}

//2017.1.25 check baud rate for SensorConfig
uint32_t SensorBaudRate(uint8_t uarttaskBR)
{
    switch(uarttaskBR)
    {
        case 1:
            return 1200;
        case 2:
            return 2400;
        case 3:
            return 4800;
        case 4:
            return 19200;
        case 5:
            return 38400;
        case 6:
            return 57600;
        case 7:
            return 115200;
        case 0:
        default:
            return 9600;
    }
}

//2017.2.3 Add UART word length
WordLength_t SensorDB(uint8_t wordlength)
{
    switch(wordlength)
    {
        case 1:
            return UART_9_BIT;
        case 0:
        default:
            return UART_8_BIT;
    }
}

//2017.2.3 Add UART stop bits
StopBits_t SensorSB(uint8_t stopbits)
{
    switch(stopbits)
    {
        case 1:
            return UART_0_5_STOP_BIT;
        case 2:
            return UART_2_STOP_BIT;
        case 3:
            return UART_1_5_STOP_BIT;
        case 0:
        default:
            return UART_1_STOP_BIT;
    }
}

//2017.2.3 Add UART parity
Parity_t SensorParity(uint8_t senparity)
{
    switch(senparity)
    {
        case 1:
            return EVEN_PARITY;  //EVEN_parity
        case 2:
            return ODD_PARITY;  //ODD_parity
        case 0:
        default:
            return NO_PARITY;  //ON_parity
    }
}

//2017.2.16 RTC Alarm Callback for uart task in polling mode
void UartAlarmCallback(MCUALARM_e AlarmName)
{
    SendQueueToUartTaskFromISR(UartRTCAlarm, (UartId_t)NULL, NULL);
}

//daniel add on 2017.3.16 for disable uart
void Uart_Disable(UartId_t UartID)
{
	  DeInitMyPin(); //daniel add on 2018.5.13 for deinit the pin that we use
    //Jason modify for UART_1 on 2019.05.20
		UartDeInit(&UartPort[UART_1]);
		if(UartPort[UART_1].FifoTx.Data != NULL)
		{
				vPortFree(UartPort[UART_1].FifoTx.Data);
				UartPort[UART_1].FifoTx.Data = NULL;
		}
		if(UartPort[UART_1].FifoRx.Data != NULL)
		{
				vPortFree(UartPort[UART_1].FifoRx.Data);
				UartPort[UART_1].FifoRx.Data = NULL;
		}
		
		//Jason modify for UART_1 on 2019.05.20
		UartDeInit(&UartPort[UART_2]);
		if(UartPort[UART_2].FifoTx.Data != NULL)
		{
				vPortFree(UartPort[UART_2].FifoTx.Data);
				UartPort[UART_2].FifoTx.Data = NULL;
		}
		if(UartPort[UART_2].FifoRx.Data != NULL)
		{
				vPortFree(UartPort[UART_2].FifoRx.Data);
				UartPort[UART_2].FifoRx.Data = NULL;
		}

		//EnterLPMTime not zero means that low power mode was enabled.
		if(EnterLPMTime != 0)
		{
				//Jason modify UART1_RX to for interrupt 
				//GpioInit(&Uart1RxInterrupt, UART1_RX, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
				GpioInit(&Uart1RxInterrupt, UART_RX, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
				GpioSetInterrupt(&Uart1RxInterrupt, IRQ_RISING_EDGE, IRQ_MEDIUM_PRIORITY, WakeUpLPMCallback);
		}
}

//daniel add on 2017.3.24 for calculate RTC interval
void SetRtcInterval()
{
    uint16_t poll_min;
    uint16_t poll_day;
    poll_min = GetAtConfig().GpioReportTime_minutes;
    poll_day = GetAtConfig().GpioReportTime_day;
    //daniel modify on 2017.4.26
    if(myPollingMode==1 && poll_min!=0)
    {
        myReportCycleTime = poll_min * SECS_IN_MIN;
    }
    else if(myPollingMode==1 && poll_day!=0)
    {
        myReportCycleTime = poll_day * SECS_IN_DAY;
    }
    else //no polling
    {
#if defined( HAYLEYS_MOTOR )
#else
        myReportCycleTime = 0;
        RtcDisableAlarm(MCU_ALARM_A);
#endif
    }
}

//note on 2019.1.23 for turn on / off output power & 485 IC
void PWR5V(uint32_t st)
{
    Gpio_t SendGpio;
    GpioInit(&SendGpio, PA_12, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0); //modify on 2018.11.15
    GpioWrite(&SendGpio, st);
}

//daniel add on 2018.4.13
void BatDetect(uint32_t st)
{
    Gpio_t SendGpio;
    GpioInit(&SendGpio, PA_8, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0); //modify on 2017.5.13
    GpioWrite(&SendGpio, st);
}

// Jason add for control on 2019.01.25
void addFBCallback(uint32_t st) {
	  //Jason add on 2018.02.18
 	  uint32_t upFeedback = GetAtConfig().UP_FEEDBACK;
	  uint32_t downFeedback = GetAtConfig().DOWN_FEEDBACK;
	  if(upFeedback > 1 && upFeedback < 13) 
		{
			UpDetectTimer = AddTimer(UpDetectTimer, 1000, 1, UpFBRiseCallback);
		}
		if(downFeedback > 1 && downFeedback < 13) 
		{
			DownDetectTimer = AddTimer(DownDetectTimer, 1000, 1, DownFBRiseCallback);
		}
		/*
    Gpio_t ioIn1;
    Gpio_t ioIn2;
	  GpioInit(&ioIn1, PC_2, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
		GpioSetInterrupt(&ioIn1, IRQ_RISING_EDGE, IRQ_MEDIUM_PRIORITY, UpFBRiseCallback);
		GpioInit(&ioIn2, PC_7, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
		GpioSetInterrupt(&ioIn2, IRQ_RISING_EDGE, IRQ_MEDIUM_PRIORITY, DownFBRiseCallback);*/
}

// Jason add for init with pinc 0~7 on 2019.04.11
void initPINC0to7(uint32_t st) {
	 GpioInit(&ioIn0, PC_0, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
	 GpioInit(&ioIn1, PC_1, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
	 GpioInit(&ioIn2, PC_2, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
	 GpioInit(&ioIn3, PC_3, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
	 GpioInit(&ioOut4, PC_4, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
	 GpioInit(&ioOut5, PC_5, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
	 GpioInit(&ioOut6, PC_6, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
	 GpioInit(&ioOut7, PC_7, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
	 GpioWrite(&ioOut4, 0);
	 GpioWrite(&ioOut5, 0);
	 GpioWrite(&ioOut6, 0);
	 GpioWrite(&ioOut7, 0);
}

// Jason add for control on 2019.01.25
void MotorUp(uint32_t st)
{   
	  /* if(st == 1) {
			up_flag = 1;
      // SendMsgToUart("MotorUp:HIGH!\r\n", UseUartID);
		} */
		if(AtConfig.DEVICE_TYPE == 1) {
			Gpio_t UpGpio;
			GpioInit(&UpGpio, PC_0, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0); //modify on 2017.5.13
			GpioWrite(&UpGpio, st);
		} 
		else if(AtConfig.DEVICE_TYPE == 2)
		{
			if(st == 1) 
      {
				Gpio_t onGpio;
				GpioInit(&onGpio, PC_4, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0); //modify on 2017.5.13
				GpioWrite(&onGpio, 1);
				delay_ms(pulseTime);
				GpioWrite(&onGpio, 0);
			}
			else if(st == 0) 
      {
				Gpio_t offGpio;
				GpioInit(&offGpio, PC_5, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0); //modify on 2017.5.13
				GpioWrite(&offGpio, 1);
				delay_ms(pulseTime);
				GpioWrite(&offGpio, 0);
			}
		}
}

// Jason add for control on 2019.01.25
void MotorDown(uint32_t st)
{
	  /*if(st == 1) {
			down_flag = 1;
      // SendMsgToUart("MotorDown:HIGH!\r\n", UseUartID);
		} 
		} */
    
		if(AtConfig.DEVICE_TYPE == 1) {
			Gpio_t DownGpio;
			GpioInit(&DownGpio, PC_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0); //modify on 2017.5.13
			GpioWrite(&DownGpio, st);
		} 
		else if(AtConfig.DEVICE_TYPE == 2)
		{
			if(st == 1) 
      {
				Gpio_t onGpio;
				GpioInit(&onGpio, PC_6, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0); //modify on 2017.5.13
				GpioWrite(&onGpio, 1);
				delay_ms(pulseTime);
				GpioWrite(&onGpio, 0);
			}
			else if(st == 0) 
      {
				Gpio_t offGpio;
				GpioInit(&offGpio, PC_7, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0); //modify on 2017.5.13
				GpioWrite(&offGpio, 1);
				delay_ms(pulseTime);
				GpioWrite(&offGpio, 0);
			}
		}
}

// Jason add for auto control on 2019.03.13
void stopMptorCallback(uint32_t time) {
	MotorDown(0);
	MotorUp(0);
}

// Jason add for control on 2019.01.25
void UpFBRiseCallback(uint32_t st)
{
	uint32_t upFeedback = GetAtConfig().UP_FEEDBACK;
	PinNames testPin;
	switch(upFeedback)
  {
			case 0:
			{
				testPin = PC_0;
				break;
			}
			case 1:
			{
				testPin = PC_1;
				break;
			}
			case 2:
			{
				testPin = PC_2;
				break;
			}
			case 3:
			{
				testPin = PC_3;
				break;
			}
			case 4:
			{
				testPin = PC_4;
				break;
			}
			case 5:
			{
				testPin = PC_5;
				break;
			}
			case 6:
			{
				testPin = PC_6;
				break;
			}
			case 7:
			{
				testPin = PC_7;
				break;
			}
			case 8:
			{
				testPin = PC_8;
				break;
			}
			case 9:
			{
				testPin = PC_9;
				break;
			}
			case 10:
			{
				testPin = PC_10;
				break;
			}
			case 11:
			{
				testPin = PC_11;
				break;
			}
			case 12:
			{
				testPin = PC_12;
				break;
			}
	}
	Gpio_t ioIn;
	// GpioInit(&ioIn, PC_2, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
	GpioInit(&ioIn, testPin, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
	uint8_t UpFBvalue = GpioReadInput(&ioIn);
	if(UpFBvalue == 1 /*&& up_flag == 1*/) 
  {
			MotorUp(0);
		  /*up_flag = 0;*/
	} 
}

// Jason add for control on 2019.01.25
void DownFBRiseCallback(uint32_t st) 
{
	uint8_t downFeedback = GetAtConfig().DOWN_FEEDBACK;
	PinNames testPin;
	switch(downFeedback)
  {
			case 0:
			{
				testPin = PC_0;
				break;
			}
			case 1:
			{
				testPin = PC_1;
				break;
			}
			case 2:
			{
				testPin = PC_2;
				break;
			}
			case 3:
			{
				testPin = PC_3;
				break;
			}
			case 4:
			{
				testPin = PC_4;
				break;
			}
			case 5:
			{
				testPin = PC_5;
				break;
			}
			case 6:
			{
				testPin = PC_6;
				break;
			}
			case 7:
			{
				testPin = PC_7;
				break;
			}
			case 8:
			{
				testPin = PC_8;
				break;
			}
			case 9:
			{
				testPin = PC_9;
				break;
			}
			case 10:
			{
				testPin = PC_10;
				break;
			}
			case 11:
			{
				testPin = PC_11;
				break;
			}
			case 12:
			{
				testPin = PC_12;
				break;
			}
	}
	Gpio_t ioIn;
	//GpioInit(&ioIn, PC_3, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
	GpioInit(&ioIn, testPin, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
	uint8_t DownFBvalue = GpioReadInput(&ioIn);
	if(DownFBvalue == 1 /* down_flag == 1*/) 
  {
			MotorDown(0);
		  //down_flag = 0;
	} 
}

//Jason add on 2018.12.20 for get pin
PinNames getPINName(uint32_t pinNumber) {
	PinNames testPin;
	switch(pinNumber)
  {
			case 0:
			{
				testPin = PC_0;
				break;
			}
			case 1:
			{
				testPin = PC_1;
				break;
			}
			case 2:
			{
				testPin = PC_2;
				break;
			}
			case 3:
			{
				testPin = PC_3;
				break;
			}
			case 4:
			{
				testPin = PC_4;
				break;
			}
			case 5:
			{
				testPin = PC_5;
				break;
			}
			case 6:
			{
				testPin = PC_6;
				break;
			}
			case 7:
			{
				testPin = PC_7;
				break;
			}
			case 8:
			{
				testPin = PC_8;
				break;
			}
			case 9:
			{
				testPin = PC_9;
				break;
			}
			case 10:
			{
				testPin = PC_10;
				break;
			}
			case 11:
			{
				testPin = PC_11;
				break;
			}
			case 12:
			{
				testPin = PC_12;
				break;
			}
		}
	return testPin;
}

//Jason add on 2018.12.20 for pin output test
void PinTest(uint32_t pinNumber, uint32_t pinValue) 
{
	uint8_t sendBuff[32];
	PinNames myPin = getPINName(pinNumber);
	Gpio_t testGpio;
		
	GpioInit(&testGpio, myPin, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
	GpioWrite(&testGpio, pinValue);
	sprintf((char*)sendBuff, "+TESTPINC PIN:%d, VALUE:%d\r\n", pinNumber, pinValue);
	SendMsgToUart(sendBuff, UseUartID);
}
//daniel add on 2018.5.13 for deinit the pin that we use
void DeInitMyPin(void)
{
    /*Gpio_t Gpio_2, Gpio_3, Gpio_8, Gpio_11, Gpio_12;
    GpioInit(&Gpio_2, PA_2, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0);
    GpioInit(&Gpio_3, PA_3, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_PULL_UP, 0);*/
	  //Jason modify on 2019.05.21
	  Gpio_t Gpio_8, Gpio_11, Gpio_12;
    GpioInit(&Gpio_8, PA_8, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&Gpio_11, PA_11, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_PULL_UP, 0);
    GpioInit(&Gpio_12, PA_12, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_PULL_UP, 0);
}

//daniel add on 2018.4.17 for delay
void delay_ms(uint16_t time)
{
   uint16_t i=0;
   while(time--)
   {
      i = 12000;
      while(i--);
   }
}

//daniel add on 2017.9.18 for get average value of ADC
//uint16_t calADC(uint8_t channel, uint8_t sample_num)
uint16_t calADC(uint32_t channel, uint8_t sample_num) //daniel modify on 2017.12.25
{
    uint8_t i;
    uint16_t adc, res = 0;
    AdcInit(); //daniel add on 2018.1.30 for fix issue
    adc = AdcRead(channel);
    for(i=0; i<sample_num; i++)
    {
        adc = AdcRead(channel);
        res = res + adc;
    }
    res = res / sample_num;
    return res;
}

//daniel add on 2017.4.10 for reset to default
void DetectResetDefaultCallback(uint32_t time)
{
    Gpio_t RestoreDefalutPin;
    GpioInit(&RestoreDefalutPin, PB_8, GPIO_PINMODE_READ_INTERRUPT, GPIO_PINCONFIGS_READ_INTERRUPT, GPIO_PINTYPES_READ_INTERRUPT, 0);
    if(GpioReadInput(&RestoreDefalutPin) == 1)
    {
        ResetDefaultPinCount++;
        if(ResetDefaultPinCount == ENTER_RESTORE_TIME)
        {
            SendQueueToUartTask(UartStopReset, (UartId_t)NULL, NULL);
            RestoreUserConfigToDefault();
            SaveConfigToEEPROM();
            myPollingMode = 0;
        }
    }
    else
    {
        SendQueueToUartTask(UartStopReset, (UartId_t)NULL, NULL);
    }
    GpioInit( &RestoreDefalutPin, PB_8, GPIO_PINMODE_UNUSED, GPIO_PINCONFIGS_UNUSED, GPIO_PINTYPES_UNUSED, 0 );
}

//Jason modify on 2018.11.15 for check if UART sensor response timer callback
void CheckSensor1ReplyCallback(uint32_t time)
{
    if(UartSensorReplyFlag != 1 ) //sensor not response -> timeout
    {
			  memset(receivedCharBuffer,'\0',ATCOMMAND_BUFFER_SIZE);  //Reset receivedCharBuffer
				receivedLength = 0;
			  if(mySensorType > 2) //If two sensors
				{
						GetSensorDataCallback(0);
				} else 
				{
						sendSensorData(mySensorType, sensorData);
				}
	  }
}

// Jason add on 2018.11.15 for sensor 2 time out
void CheckSensor2ReplyCallback(uint32_t time) {
	  uint8_t i;
	  if(UartSensorReplyFlag != 2 ) //sensor 2 not response -> timeout
    {
			  memset(receivedCharBuffer,'\0',ATCOMMAND_BUFFER_SIZE);  //Reset receivedCharBuffer
				receivedLength = 0;
			  for(i=0; i<8; i++)
				{
						sensorData[8+i] = 0x00;
				}
			  
			  if(mySensorType > 3) //If three sensors
				{
						GetSensorDataCallback(0);
				} else 
				{
						sendSensorData(mySensorType, sensorData);
				}
	  }
}

// Jason add on 2018.11.15 for sensor 3 time out
void CheckSensor3ReplyCallback(uint32_t time) {
	  uint8_t i;
	  if(UartSensorReplyFlag != 3 ) //sensor 3 not response -> timeout
    {
			  memset(receivedCharBuffer,'\0',ATCOMMAND_BUFFER_SIZE);  //Reset receivedCharBuffer
				receivedLength = 0;
			  for(i=0; i<8; i++)
				{
						sensorData[16+i] = 0x00;
				}
			  sendSensorData(mySensorType, sensorData);
	  }
}

//daniel add on 2017.4.21 for timer callback
void GetSensorDataCallback(uint32_t time)
{
    //daniel add on 2018.5.13 for clear UART TX & RX buffer
    //if(UartPort[UseUartID].FifoTx.Data != NULL)
	  if(UartPort[UART_1].FifoTx.Data != NULL)//Jason modify to UART_1 on 2019.05.20
    {
        FifoFlush( &UartPort[UseUartID].FifoTx );
    }
    //if(UartPort[UseUartID].FifoRx.Data != NULL)
		if(UartPort[UART_1].FifoRx.Data != NULL)//Jason modify to UART_1 on 2019.05.20
    {
        FifoFlush( &UartPort[UseUartID].FifoRx );
    }
    //
    Max485RW(1); //daniel modify on 2017.3.28 for set max485 to write mode
    receivedLength = 0;
    //
		if(UartSensorSendFlag == 0) 
		{
			sendData.Data = (uint8_t*)mySenConfig.modbusCMD;
			sendData.DataLen = mySenConfig.modbusCMDLEN;
		}
		else if(UartSensorSendFlag == 1) 
    {
			sendData.Data = (uint8_t*)mySenConfig.modbusCMD2;
			sendData.DataLen = mySenConfig.modbusCMDLEN2;
		}
		else if(UartSensorSendFlag == 2) 
    {
			sendData.Data = (uint8_t*)mySenConfig.modbusCMD3;
			sendData.DataLen = mySenConfig.modbusCMDLEN3;
		}
		SendQueueToUartTask(UartCheckReply,UseUartID,NULL);
    //SendQueueToUartTask(UartSendString, UseUartID, &sendData);
		SendQueueToUartTask(UartSendString, UART_1, &sendData);//Jason modify to UART_1 on 2019.05.20
    sendData.Data = NULL;
    sendData.DataLen = 0;
		UartSensorSendFlag++; //Jason mofify on 2018.11.5 for check if send cmd to UART sensor
}

void SaveSensorData(unsigned char *receivedata)
{
		uint8_t i, inx, data_len, start, sendLength; //Jason modify on 2018.11.15
		//Jason modify on 2018.11.01
		if(UartSensorSendFlag == 1) {
			start = mySenConfig.startBYTE;
			data_len =  mySenConfig.readLEN;
			sendLength = 0;
		} else if(UartSensorSendFlag == 2) {
			start = mySenConfig.startBYTE;
			data_len =  mySenConfig.readLEN2;
			sendLength = 8;
		} else if(UartSensorSendFlag == 3) {
			start = mySenConfig.startBYTE;
			data_len =  mySenConfig.readLEN3;
			sendLength = 16;
		}
		
		for(i=0; i<8; i++)
		{
			  inx = start + i;
				
			  if(i<data_len) {
					sensorData[sendLength+i] = receivedata[inx]; //daniel modify on 2017.9.15 for adjust data format
				}
				else
				{
					sensorData[sendLength+i] = 0x00;
				}
		}
}

void replyCommand(unsigned char *receivedata)
{
	    uint8_t sendBuff[10];
			sendBuff[0] = 0xAA;
			sendBuff[1] = 0xFB;
			sendBuff[2] = 0x00;
			sendBuff[3] = receivedata[1];//Addr
			sendBuff[4] = receivedata[3];//Serial number
			sendBuff[5] = receivedata[4] + 128; //Reply Command
			sendBuff[6] = receivedata[5];//Code
			sendBuff[7] = 0x00;//Resutl
			sendBuff[8] = sendBuff[4] + sendBuff[5] + sendBuff[6] +sendBuff[7];//Checksum
			sendBuff[9] = 0x8E;
      for(int i=0; i<8; i++) {
			}
}

void checkBTCommand(unsigned char *receivedata)
{
	  // for motor command
    if(receivedata[4] == 1 && receivedata[5] == 2 && receivedata[6] == 1)
		{
			//Jason add for reply command on 2019.03.13
			//command flow
			if(receivedata[7]== 0) { //command is STOP
				MotorUp(0);
				MotorDown(0);
				/* up_flag = 0;
				down_flag = 0; */
				return;
			} else if(receivedata[7] == 1) { //command is Down
				//if(up_flag == 1) //up is high?
				//{
					MotorUp(0);
					delay_ms(stop2OnTime);
				//}
				MotorDown(1);
			} else if(receivedata[7] == 2) { //command is UP
				//if(down_flag == 1) //down is high?
				//{
					MotorDown(0);
					delay_ms(stop2OnTime);
				//}
				MotorUp(1);
			}	
			uint32_t time = receivedata[8];
			if(time ==0) {
				time = 60;
			}	
			stopMotorTimer = DeleteTimer(stopMotorTimer);
			stopMotorTimer = AddTimer(stopMotorTimer, time*1000, 0, stopMptorCallback);
			uint8_t sendBuff[10];
			sendBuff[0] = 0xAA;
			sendBuff[1] = 0xFB;
			sendBuff[2] = 0x00;
			sendBuff[3] = receivedata[1];//Addr
			sendBuff[4] = receivedata[3];//Serial number
			sendBuff[5] = receivedata[4] + 128; //Reply Command
			sendBuff[6] = receivedata[5];//Code
			sendBuff[7] = 0x00;//Resutl
			sendBuff[8] = sendBuff[4] + sendBuff[5] + sendBuff[6] +sendBuff[7];//Checksum
			sendBuff[9] = 0x8E;
      for(int i=0; i<10; i++) {
				UartPutChar(&UartPort[UseUartID], sendBuff[i]);
			}
		}
}

void toSleep(uint32_t time) {
	  //Jason add for init RS485 on 2019.01.22
		PWR5V(0);//for turn off output power & 485 IC
		Max485RW(0); //for set max485 to read mode
		//Uart_Disable(UseUartID); //daniel add on 2017.4.21 for enter low power
		RtcStartWakeUpAlarm(myReportCycleTime,MCU_ALARM_A);
}

void sendSensorData(uint8_t type, unsigned char *data)
{
		uint8_t UARTAppdata[31];
	  uint8_t i, data_len=8; //Jason modify on 2018.11.15
		Gpio_t myGpio; //daniel modify on 2017.9.19
		uint16_t Adc1, Adc2; //daniel add on 2017.9.15 for adjust data format
		GpioInit(&myGpio, PA_12, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_UP, 0);
		if(type == 0) //
		{
			//get BAT status
			BatDetect(1); //daniel add on 2018.4.13 for detect battery
			delay_ms(2000); //daniel add on 2018.5.8
			Adc2 = calADC(ADC_CHANNEL_9, 5); //AdcRead(ADC_Channel_9); //daniel add on 2017.9.18 for get average value of ADC
			BatDetect(0); //daniel add on 2018.4.13 for detect battery
			//
			//get sensor status
			delay_ms(2000); //daniel add on 2018.5.8
			Adc1 = calADC(ADC_CHANNEL_8, 5); //AdcRead(ADC_Channel_8); //daniel add on 2017.9.18 for get average value of ADC
			//build data //daniel modify on 2017.9.19 for adjust data format
			memset(UARTAppdata, 0x00, 6);
			UARTAppdata[0] = 0xfa;
			UARTAppdata[1] = GpioReadInput(&myGpio);
			UARTAppdata[2] = (uint8_t)(Adc1>>8);
			UARTAppdata[3] = (uint8_t)(Adc1&0x00FF);
			UARTAppdata[4] = (uint8_t)(Adc2>>8);
			UARTAppdata[5] = (uint8_t)(Adc2&0x00FF);
		}
		else
		{
			UartSensorReplyFlag = 0; //Jason mark on 2018.11.02 for check if UART sensor response
		  UartSensorSendFlag = 0; //daniel add on 2018.5.8 for check if send cmd to UART sensor
			Adc1 = calADC(ADC_CHANNEL_8, 5); //daniel add on 2017.9.18 for get average value of ADC
			Adc2 = AdcBat; //calADC(ADC_Channel_9, 5); //daniel modify on 2018.5.8 for detect battery
			//build data //daniel modify on 2017.9.19 for adjust data format
			memset(UARTAppdata, 0x00, 31);
			UARTAppdata[0] = 0xfb;
			UARTAppdata[1] = GpioReadInput(&myGpio);
			UARTAppdata[2] = (uint8_t)(Adc1>>8);
			UARTAppdata[3] = (uint8_t)(Adc1&0x00FF);
			UARTAppdata[4] = (uint8_t)(Adc2>>8);
			UARTAppdata[5] = (uint8_t)(Adc2&0x00FF);
			UARTAppdata[6] = data_len;
			if(type == 2 || type == 1) {
				data_len = 8;
			} else if(type == 3) {
				data_len =  16;
			} else if(type == 4) {
				data_len = 24;
			}
			//parsing data
			for(i=0; i<24; i++)
			{
					if(i<data_len) 
					{
						UARTAppdata[7+i] = data[i]; //daniel modify on 2017.9.15 for adjust data format
					}
					else
					{
						UARTAppdata[7+i] = 0x00;
					} 
			}
		}
		
		data_len = 31;
		if(type == 0) data_len = 6;
 		if(WirelessModule == 0) 
		{	
			RadioSendData_t UARTsenddata;
			AtConfig_t atConfig; //daniel add on 2017.9.25 for set fport
			UserConfig1_t userConfig1; //daniel add on 2017.9.25 for set fport
			GetEEPROMConfig(&atConfig, &userConfig1); //daniel add on 2017.9.25 for set fport
			//UARTsenddata.DataLen = sizeof(UARTAppdata);  //LoRa Send
			UARTsenddata.DataLen = data_len;
			UARTsenddata.Data = UARTAppdata;
			UARTsenddata.AppPort = userConfig1.AppPort; //daniel add on 2017.9.25 for set fport
			SendQueueToRadio(SendUnConfirmedDataUp, &UARTsenddata);
		}
		else if(WirelessModule == 1) 
		{
			sprintf((char*)testBuff, "AT+ECHO%s","=");
			
			for(i=0; i < data_len; i++) //daniel modify on 2017.9.25 for display cmd string
			{
				sprintf((char*)myStr, "%02X", UARTAppdata[i]);	
				strcat((char*)testBuff, (char*)myStr);
			}  
			strcat((char*)testBuff, "\r\n");
			UART2senddata.DataLen = 8 + (data_len*2);
			UART2senddata.Data = testBuff;
			SendQueueToUartTask(UartSendString, UART_2, &UART2senddata);
		}
		//Jason modifies the delay to disable the UART for 1 second on 2019.05.23
		/*PWR5V(0);//for turn off output power & 485 IC
		Max485RW(0); //for set max485 to read mode
		Uart_Disable(UseUartID); //daniel add on 2017.4.21 for enter low power
		RtcStartWakeUpAlarm(myReportCycleTime,MCU_ALARM_A);*/
		SendQueueToUartTask(UartToSleep,UseUartID,NULL);
}

void sendBindAP(uint32_t time) 
{
	uint8_t UARTAppdata[5];
	memset(UARTAppdata, 0x00, 5);
	UARTAppdata[0] = 0x61;
	UARTAppdata[1] = 0x62;
	UARTAppdata[2] = 0x63;
	UARTAppdata[3] = 0x64;
	UARTAppdata[4] = 0x65;
	AtConfig_t atConfig; //daniel add on 2017.9.25 for set fport
	UserConfig1_t userConfig1; //daniel add on 2017.9.25 for set fport
	GetEEPROMConfig(&atConfig, &userConfig1); //daniel add on 2017.9.25 for set fport
	RadioSendData_t UARTsenddata;
	UARTsenddata.DataLen = 5;  //LoRa Send
	UARTsenddata.Data = UARTAppdata;
	UARTsenddata.AppPort = userConfig1.AppPort; //daniel add on 2017.9.25 for set fport
	SendQueueToRadio(SendUnConfirmedDataUp, &UARTsenddata);
}

//Jason add on 2019.04.10 for motor command reply
void sendMotorReply(uint8_t number) 
{
	uint8_t ReturnData[7];
	ReturnData[0] = 0xAA;
	ReturnData[1] = number;
	ReturnData[2] = 0x81;
	ReturnData[3] = read_buff[3];
	ReturnData[4] = 0x00;
	ReturnData[5] = 0x83;
	ReturnData[6] = 0x8E;
	RadioSendData_t RadioSendData;
	memset(RadioSendData.Data, 0, 7);
	RadioSendData.Data = ReturnData;
	RadioSendData.DataLen = sizeof(ReturnData);;
	RadioSendData.AppPort = UserConfig1.AppPort;
	SendQueueToRadio(SendUnConfirmedDataUp, &RadioSendData);
	/*if(UserConfig1.SyncFlag == 0)
	{
		SendQueueToRadio(SendUnConfirmedDataUp, &RadioSendData);
	}
	else
	{
		RadioSendData.retries = 3;
		SendQueueToRadio(SendConfirmedDataUp, &RadioSendData);
	}
	RadioTxLED(1);*/
}

//Jason add on 2019.04.10 for query pinc 
void getPincStatus(uint32_t time)
{
	AtConfig.PINC[0] = GpioReadInput(&ioIn0);//Get pinc0 input
	AtConfig.PINC[1] = GpioReadInput(&ioIn1);//Get pinc0 input
	AtConfig.PINC[2] = GpioReadInput(&ioIn2);//Get pinc0 input
	AtConfig.PINC[3] = GpioReadInput(&ioIn3);//Get pinc0 input
}

//Jason add on 2019.04.12 for control pinc command reply
void sendControlPinReply(uint8_t number) 
{
	uint8_t ReturnData[7];
	ReturnData[0] = 0xAA;
	ReturnData[1] = number;
	ReturnData[2] = 0x81;
	ReturnData[3] = 0x03;
	ReturnData[4] = 0x00;
	ReturnData[5] = 0x84;
	ReturnData[6] = 0x8E;
	RadioSendData_t RadioSendData;
	memset(RadioSendData.Data, 0, 7);
	RadioSendData.Data = ReturnData;
	RadioSendData.DataLen = sizeof(ReturnData);;
	RadioSendData.AppPort = UserConfig1.AppPort;
	SendQueueToRadio(SendUnConfirmedDataUp, &RadioSendData);
}

//Jason add on 2019.04.10 for query pinc command reply
void sendQueryPinReply(uint8_t number) 
{
	uint8_t j;
	uint8_t sum = 0;
	for(j=0; j<IO_PINC_SIZE; j++)
	{
			sum = sum + (AtConfig.PINC[j] << j);
	}
	uint8_t checksum = 0x83 + 0x16 + 0x01 + sum;
	uint8_t ReturnData[8];
	ReturnData[0] = 0xAA;
	ReturnData[1] = number;
	ReturnData[2] = 0x83; //Command
	ReturnData[3] = 0x16; //Code
	ReturnData[4] = 0x01; //DL
	ReturnData[5] = sum;
	ReturnData[6] = checksum;
	ReturnData[7] = 0x8E;
	
	RadioSendData_t RadioSendData;
	RadioSendData.Data = ReturnData;
	RadioSendData.DataLen = sizeof(ReturnData);;
	RadioSendData.AppPort = UserConfig1.AppPort;
	SendQueueToRadio(SendUnConfirmedDataUp, &RadioSendData);
}

//Jason add on 2019.5.20 for set UART_1 & UART_2 
void setUART(void)
{
	  if(UartPort[UART_1].FifoTx.Data == NULL)
		{
				UartTxBuffer = pvPortMalloc(FIFO_TX_SIZE);
				FifoInit(&UartPort[UART_1].FifoTx, UartTxBuffer, FIFO_TX_SIZE);
		}
		if(UartPort[UART_1].FifoRx.Data == NULL)
		{
				UartRxBuffer = pvPortMalloc(FIFO_RX_SIZE);
				FifoInit(&UartPort[UART_1].FifoRx, UartRxBuffer, FIFO_RX_SIZE);
		}
		UartInit(&UartPort[UART_1], UART_1, UART1_TX, UART1_RX, (PinNames)NC, (PinNames)NC);
		//UartConfig(&UartPort[UART_1], RX_TX, 9600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL);
		//add on 2017.2.14 for set UART settings of sensor
		if(mySenConfig.uartPC == 0)
				UartConfig(&UartPort[UART_1], RX_TX, SensorBaudRate(mySenConfig.uartBR), SensorDB(mySenConfig.uartDB), SensorSB(mySenConfig.uartSB), SensorParity(mySenConfig.uartPC), NO_FLOW_CTRL);
		else //daniel add on 2017.4.10 for even parity or odd parity
				UartConfig(&UartPort[UART_1], RX_TX, SensorBaudRate(mySenConfig.uartBR), UART_9_BIT, SensorSB(mySenConfig.uartSB), SensorParity(mySenConfig.uartPC), NO_FLOW_CTRL);
		//init UART_2
		
		if(UartPort[UART_2].FifoTx.Data == NULL)
		{
				UartTxBuffer2 = pvPortMalloc(FIFO_TX_SIZE);
				FifoInit(&UartPort[UART_2].FifoTx, UartTxBuffer2, FIFO_TX_SIZE);
		}
		if(UartPort[UART_2].FifoRx.Data == NULL)
		{
				UartRxBuffer2 = pvPortMalloc(FIFO_RX_SIZE);
				FifoInit(&UartPort[UART_2].FifoRx, UartRxBuffer2, FIFO_RX_SIZE);
		}
		UartInit(&UartPort[UART_2], UART_2, UART_TX, UART_RX, (PinNames)NC, (PinNames)NC);
		UartConfig(&UartPort[UART_2], RX_TX, 9600, UART_8_BIT, UART_1_STOP_BIT, NO_PARITY, NO_FLOW_CTRL);
}

void UartTask(void * pvParameters)
{
    UartMsgBody_t ReceivedData;
    uint8_t receivedChar;
    UartPort[UART_1] = Uart1;
    UartPort[UART_2] = Uart2;

    UartSetIrqHandler(UartStateCallback);
    SetRadioCallback(RadioTaskCallback);

    /* Create the queue as described at the top of this file. */
    Uart_Task_Queue = xQueueCreate(UART_TASK_QUEUE_LENGTH, sizeof(UartMsgBody_t));
    configASSERT(Uart_Task_Queue);

    /*Start AT command server*/
    InitAtCommandList();

    GetSensorConfig(&mySenConfig); //add on 2017.2.2 for get sensor config
	  //Jason add on 2018.10.31
	  mySensorType = mySenConfig.sensorType;
    myPollingMode = GetAtConfig().Enter_Reporter; //daniel add on 2017.3.28
	  
#if 1 //daniel add on 2017.5.6 for test
    {
    // uint8_t meterCMD[8] = {0xa5, 0x04, 0x00, 0x00, 0x00, 0x07, 0xa8, 0xec};
		AtConfig.DEVICE_TYPE = 0;
    myPollingMode = 1; //0 AT_COMMAND, 1:Report
    // mySenConfig.sensorType = 1; mySenConfig.replyLEN = 19; mySenConfig.startBYTE = 13; mySenConfig.readLEN = 6;
    // mySenConfig.uartPC = EVEN_PARITY; mySenConfig.modbusCMDLEN = 8;
    // memcpy(mySenConfig.modbusCMD, meterCMD, sizeof(meterCMD));
    }
#endif
    //daniel mark on 2017.4.24 for not create reporter task
    /*Start Routine Task Timer*/
    // if(GetReporterMode() == 1)
		// RoutineTaskTimer = AddTimer(RoutineTaskTimer, 300, 0, StartReporterTaskCallback);
		if(AtConfig.DEVICE_TYPE > 0) // 1: control box 2:control box for relay board, 3:pinc0~7: I/O
    {
			  myPollingMode = 0; //0 AT_COMMAND
			  PWR5V(1); //daniel add on 2018.5.13 for turn on 485 IC
			  // Jason add on 2018.02.18
			  if(AtConfig.DEVICE_TYPE == 1) 
				{
			      addFBCallback(0);
				} else if(AtConfig.DEVICE_TYPE == 3)
				{
					initPINC0to7(0);//Init pinc 0~7
					memset(AtConfig.PINC,'\0', IO_PINC_SIZE);
				}
    }
    else
    {// 0: sensor hub
        if(GetLowPowerModeFlag(&EnterLPMTime)==1)
        {
            Gpio_t gpio;
            EnterLPMCount = 0;
            EnterLPMCountTimer = AddTimer(EnterLPMCountTimer, 1000, 1, CountLPMCallback);
            GpioInit(&gpio, I2C_SDA, PIN_INPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0);
            GpioSetInterrupt(&gpio, IRQ_RISING_EDGE, IRQ_MEDIUM_PRIORITY, WakeUpLPMCallback);
        }
				//daniel add on 2017.2.16 for set rtc alarm
				if(myPollingMode == 1)
				{
						RtcSetAlarmCallback(UartAlarmCallback);
				}
    }
		//daniel add on 2017.4.10 for detect if reset to default
		ResetDefaultPinCount = 0;
		ResetDefalutTimer = AddTimer(ResetDefalutTimer, 1000, 1, DetectResetDefaultCallback);
		if(AtConfig.DEVICE_TYPE > 0  && WirelessModule == 0)
		{
			sendBindAP(0);
			CheckReplyTimer = AddTimer(CheckReplyTimer, 43200000, 1, sendBindAP);
		}

    for(;;)
    {
        xQueueReceive(Uart_Task_Queue, &ReceivedData, portMAX_DELAY);
        switch(ReceivedData.type)
        {
            //daniel add on 2017.4.10 for reset to default
            case UartStopReset:
            {
                ResetDefalutTimer = DeleteTimer(ResetDefalutTimer);
                SetRtcInterval();
                myReportCycleTime = 20; //daniel add on 2017.5.6 for test
                if(myPollingMode == 1)
                {
                    //myReportCycleTime = 200; //daniel add on 2017.5.6 for test
										if(AtConfig.DEVICE_TYPE == 0 && myPollingMode == 1)
										{
												//Jason modify for test mode at 2019.08.16
												//SendQueueToUartTask(UartRTCAlarm, (UartId_t)NULL, NULL); //first enter polling cycle
												 testTimer = AddTimer(testTimer, 5000, 0, firstUartRTCAlarm);
										}
                }
                break;
            }

            case UartEnable:
            {
				        PWR5V(1); //daniel add on 2018.5.13 for turn on 485 IC
                //Disable Interrupt
                if(EnterLPMTime != 0)
                {
                    GpioDisableInterrupt(&Uart1RxInterrupt );
                    Uart1RxInterrupt.pinIndex = 0;
                }
								
                BaudRateBackup = ReceivedData.body.UartBaudrate;
								setUART();
                break;
            }
            case UartDisable:
            {
                Uart_Disable(UseUartID);
                break;
            }
            case UartEnableEcho:
            {
                if(ReceivedData.uartId == UseUartID)
                {
                    EchoEnable = 1;
                }
                break;
            }
						
            case UartDisableEcho:
            {
                if(ReceivedData.uartId == UseUartID)
                {
                    EchoEnable = 0;
                }
                break;
            }
						//Jason add on 2018.05.28 for fix timer hang up issue
						case UartCheckReply:
            {                
							  if(UartSensorSendFlag == 0) 
								{
									CheckReplyTimer = AddTimer(CheckReplyTimer, 3000, 0, CheckSensor1ReplyCallback);
								}
								else if(UartSensorSendFlag == 1) 
								{
									CheckReplyTimer = AddTimer(CheckReplyTimer, 3000, 0, CheckSensor2ReplyCallback);
								}
								else if(UartSensorSendFlag == 2) 
								{
									CheckReplyTimer = AddTimer(CheckReplyTimer, 3000, 0, CheckSensor3ReplyCallback);
								}
                break;
            }
						//Jason add on 2018.05.28 for fix timer hang up issue
						case UartToSleep:
            {                
							  toSleep(0);
                break;
            }
            //add on 2017.3.6 for get sensor data in polling mode
            case UartRTCAlarm:
            {
							  PWR5V(1); //daniel move here on 2018.5.8 for turn on output power & 485 IC
							  setUART();//Jason move on 2019.05.20
							  
                if(mySenConfig.sensorType == 0) //none uart sensor
                {
									  sendSensorData(mySenConfig.sensorType, NULL);
                } 
                else //uart sensor
                {
									  //uint8_t testBuff[80];
		                //sprintf((char*)testBuff, "AT+ECHO=FFFFFFFFFFFFFFFF%d\r\n",0);
                    //enable uart
                    BaudRateBackup = ReceivedData.body.UartBaudrate;
                    //get BAT status
                    BatDetect(1); //daniel add on 2018.4.13 for detect battery
                    delay_ms(2000); //daniel add on 2018.5.8
                    AdcBat = calADC(ADC_CHANNEL_9, 5); //AdcRead(ADC_Channel_9); //daniel add on 2017.9.18 for get average value of ADC
                    BatDetect(0); //daniel add on 2018.4.13 for detect battery
                    //
                    //get uart sensor data
									  memset(sensorData,'\0',SENSOR_DATA_SIZE);//Reset sensorData
                    PollingTimer = AddTimer(PollingTimer, 3000, 0, GetSensorDataCallback); //daniel modify on 2017.9.25
                }
                break;
            }

            case UartRxDone:
            {
                if(ReceivedData.uartId != UseUartID)//Jason modify for UART_1 (sensor) on 2019.05.20
                {
										EnterLPMCount = 0; //LowPowerModeCount = 0; //daniel modify on 2017.12.25
										receivedChar = ReceivedData.body.ISRData;
										bool isOK = false;
										//daniel add on 2018.5.13 for avoid getting garbage data - begin
										if(receivedLength==0 && receivedChar==0x00)
										{
										} 
										else//daniel add on 2018.5.13 for avoid getting garbage data - end
										{
												receivedCharBuffer[receivedLength++] = receivedChar;
										}
										//Jason modify on 2018.11.7
										if(UartSensorSendFlag == 1 && receivedLength == mySenConfig.replyLEN) 
										{
												UartSensorReplyFlag = 1; //Jason modify on 2018.11.02 for check if UART sensor1 response
												memset(sensorData,'\0',SENSOR_DATA_SIZE);//Reset sensorData
												isOK = true;
										} 
										else if(UartSensorSendFlag == 2 && receivedLength == mySenConfig.replyLEN2) 
										{
												UartSensorReplyFlag = 2;
												isOK = true;
										}
										else if(UartSensorSendFlag == 3 && receivedLength == mySenConfig.replyLEN3) 
										{
												UartSensorReplyFlag = 3; //Jason modify on 2018.11.02 for check if UART sensor 3 response
												isOK = true;
										}
										if(isOK) 
										{
												CheckReplyTimer = DeleteTimer(CheckReplyTimer);
												SaveSensorData(receivedCharBuffer);
												if( (mySenConfig.sensorType < 3 && UartSensorReplyFlag == 1) || (mySenConfig.sensorType == 3 && UartSensorReplyFlag == 2)
																	|| (mySenConfig.sensorType == 4 && UartSensorReplyFlag == 3) )
												{
													 sendSensorData(mySensorType, sensorData);
												} 
												else
												{
													 GetSensorDataCallback(0);
												}
												memset(receivedCharBuffer,'\0',ATCOMMAND_BUFFER_SIZE);  //Reset receivedCharBuffer
												receivedLength = 0;
										}
                }
								else
								{ //Jason modify for UART_2 (at command & wireless module command) on 2019.05,20
									  EnterLPMCount = 0;
										receivedChar = ReceivedData.body.ISRData;
									  if(myPollingMode == 1) //for polling mode //daniel modify on 2017.2.16
                    { //For NBIOT pre-setting
											  //Jason add for test mode
											  if(testFlag)
												{
													  if((receivedChar == CHAR_LF)||(receivedChar == CHAR_CR))
														{
																if(receivedCharBuffer[0] == 0x54 && receivedCharBuffer[1] == 0x45 && receivedCharBuffer[1] == 0x45)
																{
																	  //Jason modify for test mode at 2019.08.16 
																		myPollingMode = 0;
																		if(receivedLength >= 11){
																				/*After TESTMODE first char polling mode,
																				second char device type,
																				third char wireless module
																				last char myReportCycleTime (*10 second)
																				Example: TESTMODE0101 polling mode:AT_COMMAND,Type:Control box, wireless: Lora, 
																				*/
																		    myPollingMode = receivedCharBuffer[8] - 0X30;
																				AtConfig.DEVICE_TYPE = receivedCharBuffer[9] - 0X30;
																				WirelessModule = receivedCharBuffer[10] - 0X30;
																				if(receivedLength == 12)
																				    myReportCycleTime = (receivedCharBuffer[11] - 0X30)*10;
																				uint8_t sendBuff[40];
							                          sprintf((char*)sendBuff, "myPollingMode:%d, DEVICE_TYPE:%d, WirelessModule :%d, myReportCycleTime:%d\r\n", myPollingMode, AtConfig.DEVICE_TYPE, WirelessModule, myReportCycleTime);
							                          SendMsgToUart(sendBuff, UseUartID);
																		} 
																	  else 
																	  {
																			  SendMsgToUart("Start TEST Mode! (AT_COMMAND) \r\n", UseUartID);
																		}
																		if(myPollingMode == 0)
																		{
																		    testTimer = DeleteTimer(testTimer);
																		}
																	  testFlag = false;
																}
																memset(receivedCharBuffer,'\0',ATCOMMAND_BUFFER_SIZE);
																receivedLength = 0;
																// replyCommand(receivedCharBuffer);
																break;
														} 
														else
														{
																receivedCharBuffer[receivedLength++] = receivedChar;
														}
												}
                    } 
										else
									  {//For both bluetooth and at command
											if(EchoEnable)
											{
													SendBackToUart(receivedChar);
											}
											if(receivedLength > (ATCOMMAND_BUFFER_SIZE-1))
											{
													receivedLength = 0;
													memset(receivedCharBuffer,'\0',ATCOMMAND_BUFFER_SIZE);
													break;
											}
											
											
											// Jason add for bluetooth module RX data on 2019.03.12
											if(receivedChar == CHAR_HEADER )
											{
												 BTFlag = true;
												 memset(receivedCharBuffer,'\0',ATCOMMAND_BUFFER_SIZE);
												 receivedLength = 0;
											}
											if(BTFlag == true)
											{//For Bluetooth
												if(receivedChar == CHAR_END)
												{
														if(receivedLength == 0)
														{
																break;
														}
														receivedCharBuffer[receivedLength++] = receivedChar;
														checkBTCommand(receivedCharBuffer);
														memset(receivedCharBuffer,'\0',ATCOMMAND_BUFFER_SIZE);
														receivedLength = 0;
														// replyCommand(receivedCharBuffer);
														break;
												} 
												else
												{
														receivedCharBuffer[receivedLength++] = receivedChar;
												}
											} 
											else
											{
												//For AT_COmmand
												if((receivedChar == CHAR_LF)||(receivedChar == CHAR_CR))
												{
														receivedCharBuffer[receivedLength] = (uint8_t) 0;
														receivedCommandLine.character = receivedCharBuffer;
														receivedCommandLine.length   = receivedLength;
														receivedCommandLine.position  = 0;
														ATCommandParser(&receivedCommandLine);
														memset(receivedCharBuffer,'\0',ATCOMMAND_BUFFER_SIZE);
														receivedLength = 0;
												}
												else if(receivedChar == CHAR_BS)
												{
														if(receivedLength > 0)
														{
																receivedCharBuffer[receivedLength] = '\0';
																receivedLength--;
														}
												}
												else
												{
														receivedCharBuffer[receivedLength++] = receivedChar;
												}
											}
										}
								} //for at command mode
                break;
            }
            case UartTxDone:
            {
                if(IsFifoEmpty(&UartPort[ReceivedData.uartId].FifoTx) == false)
                {
									uint8_t data;
                  data = FifoPop(&UartPort[ReceivedData.uartId].FifoTx);
                  if(BTFlag == true) 
									{
											if(data != CHAR_HEADER) {
												return;
											}
											BTFlag = false;
									} else {
                    UartPutChar(&UartPort[ReceivedData.uartId], data);
									}
                }
                else //daniel add on 2017.3.28 for set max485 to read mode
                {
                    if(myPollingMode == 1)
                    {
                        Max485RW(0); //for set max485 to read mode
                    }
                }
                break;
            }
            case UartSendString:
            {
                uint8_t count;
                uint8_t fifo_empty;
                if(UartPort[ReceivedData.uartId].FifoTx.Data == NULL
                    || UartPort[ReceivedData.uartId].FifoRx.Data == NULL)
                {
                    // Uart no initial
                    vPortFree(ReceivedData.body.TaskData.Data);
                    break;
                }

                fifo_empty = IsFifoEmpty(&UartPort[ReceivedData.uartId].FifoTx);
                for(count = 0; count < ReceivedData.body.TaskData.DataLen; count++)
                {
                    FifoPush(&UartPort[ReceivedData.uartId].FifoTx, ReceivedData.body.TaskData.Data[count]);
                }
                vPortFree(ReceivedData.body.TaskData.Data); //free memory after push data into fifo
                if(fifo_empty == true)
                {
                    uint8_t data;
                    data = FifoPop(&UartPort[ReceivedData.uartId].FifoTx);
                    UartPutChar(&UartPort[ReceivedData.uartId], data);
                }
                break;
            }
            case UartRadioTxDone:
            {
                if(GetRadioTxAckTimeoutRetryCount() == 0)
                {
                    //First Tx, not Ack timeout retry
                    StartRadioTxDelay();
                }
                RadioTxLED(0);
                SendMsgToUart("Radio Tx Done\r\n", UseUartID);
                break;
            }
            case UartRadioTxTimeout:
            {
                RadioTxLED(0);
                SendMsgToUart("Radio Tx Timeout\r\n", UseUartID);
                break;
            }
            case UartStopRadioTxDelay:
            {
                SendMsgToUart("RadioTxDelayDone\r\n", UseUartID);
                StopRadioTxDelayTimer();
                break;
            }
#if defined(STM32L073xx)
            /* Add by Eric Date: 2017/02/22   Log: Add UartStopRadioTxDelay1(gtxd=1) to avoid Rx2 receive issue.  */
            case UartStopRadioTxDelay1:
            {
                SendMsgToUart("DelayDone\r\n", UseUartID);
                StopRadioTxDelayTimer();
                break;
            }
#endif
            case UartRadioRxDone:
            {
                RadioStatus_t radiostatus = ReceivedData.body.RadioStatus;
                ReceiveRadioRxData(&(radiostatus.body.RxData));
								memset(read_buff, 0x00, 32);
								read_size = radiostatus.body.RxData.Size;
								int i;
								for(i=0; i<read_size; i++)
								{
										read_buff[i] = radiostatus.body.RxData.RxBuffer[i];
								}
								if(read_buff[2] == 1 && read_buff[3] == 2 && read_buff[5])
								{
									// motor command
									if(read_buff[6]== 0) { //command is STOP
										MotorUp(0);
										MotorDown(0);
										// up_flag = 0;
										// down_flag = 0;
									} else if(read_buff[6] == 1) { //command is Down
										{
											MotorUp(0);
											delay_ms(stop2OnTime);
										}
										MotorDown(1);
									} else if(read_buff[6] == 2) { //command is UP
										{
											MotorDown(0);
											delay_ms(stop2OnTime);
										}
										MotorUp(1);
									}
									uint8_t number = read_buff[1];
									sendMotorReply(number);
								} 
								else if(read_buff[2] == 3 && read_buff[3] == 22 && read_buff[5] == 142)
								{ //Query pinc
									getPincStatus(0);
									sendQueryPinReply(0);
								}  
								else if(read_buff[2] == 1 && read_buff[3] == 3 && read_buff[8] == 142)
								{ //Control pinc
									uint8_t pin = read_buff[5];
									uint8_t out = read_buff[6];
									if(pin == 4) 
									{
										GpioWrite(&ioOut4, out);
										AtConfig.PINC[4] = out;
								  } 
									else if(pin == 5)
									{
										GpioWrite(&ioOut5, out);
										AtConfig.PINC[5] = out;
									}
									else if(pin == 6)
									{
										GpioWrite(&ioOut6, out);
										AtConfig.PINC[6] = out;
									}
									else if(pin == 7)
									{
										GpioWrite(&ioOut7, out);
										AtConfig.PINC[7] = out;
									}
									uint8_t number = read_buff[1];
									sendControlPinReply(number);
								}
								if(radiostatus.body.RxData.RxBuffer != NULL)
								{
										vPortFree(radiostatus.body.RxData.RxBuffer);
										radiostatus.body.RxData.RxBuffer = NULL;
								}
								
                break;
            }
            case UartRadioRxAck:
            {
                SendMsgToUart("Radio Tx Received Ack\r\n", UseUartID);
                break;
            }
            case UartLowPowerModeCount:
            {
                EnterLPMCount++;
                if(EnterLPMCount == EnterLPMTime)
                {
                    //Disable Uart and timer for entering low power mode
                    SendToUartImmediately("\r\nEnter Power Saving Mode!\r\n");
                    EnterLPMCountTimer = DeleteTimer(EnterLPMCountTimer);
                    vTaskDelay(10/portTICK_PERIOD_MS);// Wait 1 ms
                    SendQueueToUartTask(UartDisable,UseUartID,NULL);
                }
                break;
            }
            case UartLowPowerModeWakeup:
            {
                radioInit();
                EnterLPMCountTimer = AddTimer(EnterLPMCountTimer, 1000, 1, CountLPMCallback);
                SendMsgToUart("Exit Power Saving Mode!\r\n", UseUartID);
                break;
            }
            case UartRadioTestStatus:/*For radiotest_task*/
            {
                switch(ReceivedData.body.RadioStatus.Status)
                {
                    case RadioTestPPTxDone:
                    case RadioTestPPMasterSettingDone:
                    {
                        uint8_t Buffer[PRODUCT_TEST_BUFFER_SIZE];
                        RadioTestSendData_t data;
                        memset(Buffer,'\0',PRODUCT_TEST_BUFFER_SIZE);
                        if(PingPongMaxTxNumOrRxTimeIdx == 0)
                        {
                            SendMsgToUart("Packets already transmitted, now leaving TXPP mode.\n\r", UseUartID);
                            SetTXPPFlag(0);
                            break;
                        }
                        PingPongMaxTxNumOrRxTimeIdx-=1;
                        if(PingPongMaxTxNumOrRxTimeIdx == 0)
                        {
                            memcpy(Buffer,PP_MESG_DONE,4);
                            memcpy(&Buffer[4],TXPPLabel,4);
                        }
                        else
                        {
                            memcpy(Buffer,PP_MESG_PING,4);
                            memcpy(&Buffer[4],TXPPLabel,4);
                        }
                        data.DataLen = PRODUCT_TEST_BUFFER_SIZE;
                        data.Data = Buffer;
                        SendQueueToRadio(MasterPingPongData, &data);
                        break;
                    }
                    case RadioTestPPSlaveSettingDone:
                    {
                        PingPongRxCount = 0;
                        if(TXPPSlaveSWTimer != NULL)
                        {
                            TXPPSlaveSWTimer = DeleteTimer(TXPPSlaveSWTimer);
                        }
                        TXPPSlaveSWTimer = AddTimer(TXPPSlaveTimerID, PingPongMaxTxNumOrRxTimeIdx *1000, 0, TXPPTimeout);
                        break;
                    }
                    case RadioTestPPRxDone:
                    {
                        uint8_t* buffer = ReceivedData.body.RadioStatus.body.RxData.RxBuffer;
                        uint8_t sendBuff[64];
                        if(strncmp((const char*)&buffer[4], (const char*)TXPPLabel, 4) != 0)
                        {
                            //Label not match
                            vPortFree(buffer);
                            break;
                        }
                        if(strncmp((const char*)buffer, (const char*)PP_MESG_PING, 4) == 0)
                        {
                            PingPongRxCount++;
                        }
                        else if(strncmp((const char*)buffer, (const char*)PP_MESG_DONE, 4) == 0)
                        {
                            RadioStatus_t event;
                            PingPongRxCount++;
                            event.Status =RadioTestPPSlaveDone;
                            SendQueueToUartTask(UartRadioTestStatus,(UartId_t)NULL, &event);
                        }
                        sprintf((char*)sendBuff,"RSSI = %4d, SNR= %4d\n\r"
                                ,ReceivedData.body.RadioStatus.body.RxData.RssiValue
                                ,ReceivedData.body.RadioStatus.body.RxData.SnrValue);
#if defined( STM32L073xx )
                        if(PingPongRxCount %10 == 0)
#endif
                        {
                            SendMsgToUart(sendBuff, UseUartID);
                        }

                        vPortFree(buffer);
                        break;
                    }
                    case RadioTestPPSlaveDone:
                    case RadioTestPPSlaveTimeout:
                    {
                        uint8_t sendBuff[128];
                        if(ReceivedData.body.RadioStatus.Status == RadioTestPPSlaveTimeout)
                        {
                            sprintf((char*)sendBuff, "%d packets received in %d seconds, now leaving TXPP mode.\n\r",
                                    PingPongRxCount,PingPongMaxTxNumOrRxTimeIdx);
                        }
                        else
                        {
                            sprintf((char*)sendBuff, "%d packets including DONE message received in %d seconds, now leaving TXPP mode.\n\r",
                                    PingPongRxCount, PingPongMaxTxNumOrRxTimeIdx - GetRemainTime(TXPPSlaveSWTimer)/1000);
                        }
                        TXPPSlaveSWTimer = DeleteTimer(TXPPSlaveSWTimer);
                        SendMsgToUart(sendBuff, UseUartID);
                        SendQueueToRadio(PingPongStop, NULL);
                        SetTXPPFlag(0);
                        break;
                    }
                    default:
                        break;
                }
                break;
            }
            case UartRadioTestStop:
            {
                if(TXPPSlaveSWTimer != NULL)
                {
                    TXPPSlaveSWTimer = DeleteTimer(TXPPSlaveSWTimer);
                }
                break;
            }
            default:
                break;
        }
    }
}

uint8_t SendQueueToUartTask(UartMsgEvent_t type, UartId_t uartId, void *data)
{
    UartMsgBody_t sendBody;
    sendBody.type = type;
    sendBody.uartId = uartId;
    switch(sendBody.type)
    {
        case UartEnable:
        case UartDisable:
        {
            sendBody.body.UartBaudrate = *(uint32_t *)data;
            break;
        }
        case UartSendString:
        {
            UartData_t temp = *(UartData_t *)data;
            sendBody.body.TaskData.DataLen = temp.DataLen;
            if(sendBody.body.TaskData.DataLen==0)
                return 1;
            sendBody.body.TaskData.Data = pvPortMalloc(temp.DataLen + 1);
            memset(sendBody.body.TaskData.Data, 0x00, (temp.DataLen + 1));
            memcpy((char *)sendBody.body.TaskData.Data, (char *)temp.Data, temp.DataLen);
            break;
        }
        case UartRadioRxDone:
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
        case UartRadioTestStatus:
        {
            RadioStatus_t tmp = *(RadioStatus_t *)data;
            if(tmp.Status == RadioTestPPRxDone)
            {
                sendBody.body.RadioStatus.Status = tmp.Status;
                sendBody.body.RadioStatus.body.RxData.RssiValue = tmp.body.RxData.RssiValue;
                sendBody.body.RadioStatus.body.RxData.SnrValue = tmp.body.RxData.SnrValue;
                sendBody.body.RadioStatus.body.RxData.Size = tmp.body.RxData.Size;
                sendBody.body.RadioStatus.body.RxData.RxBuffer = pvPortMalloc(tmp.body.RxData.Size + 1);
                memset(sendBody.body.RadioStatus.body.RxData.RxBuffer, 0x00, (tmp.body.RxData.Size + 1));
                memcpy((char *)sendBody.body.RadioStatus.body.RxData.RxBuffer, (char *)tmp.body.RxData.RxBuffer, tmp.body.RxData.Size);
            }
            else
            {
                sendBody.body.RadioStatus = tmp;
            }
            break;
        }
        default:
            break;
    }
    if(xQueueSend(Uart_Task_Queue, (void *) &sendBody, (TickType_t) 0) == pdTRUE)
        return 0;
    else
        return 1;
}


uint8_t SendQueueToUartTaskFromISR(UartMsgEvent_t type, UartId_t uartId, void *data)
{
    UartMsgBody_t sendBody;
    BaseType_t xHigherPriorityTaskWoken;
    sendBody.type = type;
    sendBody.uartId = uartId;
    xHigherPriorityTaskWoken = pdFALSE;
    switch(sendBody.type)
    {
        case UartRxDone:
        {
            sendBody.body.ISRData = *(uint8_t *)data;
            break;
        }
        case UartEnable:
        {
            sendBody.body.UartBaudrate = *(uint32_t *)data;
            break;
        }
        default:
            break;
    }
    if(xQueueSendFromISR(Uart_Task_Queue, (void *) &sendBody, (TickType_t) 0) == pdTRUE)
    {
        if(xHigherPriorityTaskWoken == pdTRUE)
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return 0;
    }
    else
        return 1;
}

void SendMsgToUart(const uint8_t* msg, UartId_t uartId)
{
    sendData.Data = (uint8_t*)msg;
    sendData.DataLen = strlen((const char*)msg);
    SendQueueToUartTask(UartSendString, uartId, &sendData);
    sendData.Data = NULL;
    sendData.DataLen = 0;
}


/*For radiotest_task*/

void TXPPTimeout(uint32_t timer)
{
    RadioStatus_t event;
    event.Status =RadioTestPPSlaveTimeout;
    SendQueueToUartTask(UartRadioTestStatus,(UartId_t)NULL, &event);
}

void SetPingPongMaxTxNumOrRxTimeIdx(uint16_t value)
{
    PingPongMaxTxNumOrRxTimeIdx = value;
}

void SetTXPPLable(uint8_t* label)
{
    memset(TXPPLabel,'\0',5);
    memcpy(TXPPLabel,label,4);
}

void firstUartRTCAlarm(uint32_t timer)
{
	  testFlag = false;
    SendQueueToUartTask(UartRTCAlarm, (UartId_t)NULL, NULL); //first enter polling cycle
}
