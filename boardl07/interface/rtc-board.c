#include "stm32l0xx.h"
#include "rtc-board.h"
#include <stddef.h>
#include "board.h"
/* Defines related to Clock configuration */
/* Uncomment to enable the adaquate Clock Source */
/*#define RTC_CLOCK_SOURCE_LSI*/
#define RTC_CLOCK_SOURCE_LSE

#ifdef RTC_CLOCK_SOURCE_LSI
#define RTC_ASYNCH_PREDIV    0x7F
#define RTC_SYNCH_PREDIV     0x0120
#endif

#ifdef RTC_CLOCK_SOURCE_LSE
#define RTC_ASYNCH_PREDIV  0x7F
#define RTC_SYNCH_PREDIV   0x00FF
#endif

static RtcAlarmIrqHandler *AlarmIrqCallback=NULL;
uint8_t RtcInitFlag = 0;

/*!
 * Number of days in each month on a normal year
 */
static const uint8_t DaysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * Number of days in each month on a leap year
 */
static const uint8_t DaysInMonthLeapYear[] = { 31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

/*!
 * \brief RTC Handler
 */
RTC_HandleTypeDef RtcHandle = { 0 };

bool RtcInit(void)
{
    if(RtcInitFlag == 0)
    {
        if(RtcSetConfig( ) != OK)
            return FAIL;
        //RtcSetAlarmConfig( );
        RtcInitFlag = 1;
    }
    return OK;
}

void RtcSetAlarmCallback( RtcAlarmIrqHandler *irqHandler )
{
    AlarmIrqCallback = irqHandler;
}

bool RtcSetConfig( void )
{

    RTC_DateTypeDef CalendarDate; //! Reference time in calendar format
    RTC_TimeTypeDef CalendarTime; //! Reference date in calendar format

    //__HAL_RCC_RTC_ENABLE( );

    /*##-1- Configure the RTC peripheral #######################################*/
    RtcHandle.Instance = RTC;

    /* Configure RTC prescaler and RTC data registers */
    /* RTC configured as follows:
      - Hour Format    = Format 24
      - Asynch Prediv  = Value according to source clock
      - Synch Prediv   = Value according to source clock
      - OutPut         = Output Disable
      - OutPutPolarity = High Polarity
      - OutPutType     = Open Drain */
    RtcHandle.Init.HourFormat     = RTC_HOURFORMAT_24;
    RtcHandle.Init.AsynchPrediv   = RTC_ASYNCH_PREDIV;
    RtcHandle.Init.SynchPrediv    = RTC_SYNCH_PREDIV;
    RtcHandle.Init.OutPut         = RTC_OUTPUT_DISABLE;
    RtcHandle.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    RtcHandle.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;

    if (HAL_RTC_Init(&RtcHandle) != HAL_OK)
    {
        /* Initialization Error */
        while(1)
        {
        }
    }

    // Set Date: Friday 1st of January 2000
    CalendarDate.Year = 0;
    CalendarDate.Month = RTC_MONTH_JANUARY;
    CalendarDate.Date = 1;
    CalendarDate.WeekDay = RTC_WEEKDAY_SATURDAY;
    if(HAL_RTC_SetDate( &RtcHandle, &CalendarDate, RTC_FORMAT_BIN ) != HAL_OK)
        return FAIL;

    // Set Time: 00:00:00
    CalendarTime.Hours = 0;
    CalendarTime.Minutes = 0;
    CalendarTime.Seconds = 0;
    CalendarTime.TimeFormat = RTC_HOURFORMAT12_AM;
    CalendarTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    CalendarTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if(HAL_RTC_SetTime( &RtcHandle, &CalendarTime, RTC_FORMAT_BIN ) != HAL_OK)
        return FAIL;

    return OK;
}

//static void RtcSetAlarmConfig( void )
//{
//    EXTI_InitTypeDef EXTI_InitStructure;
//    RTC_AlarmTypeDef RTC_AlarmStructure;
//    NVIC_InitTypeDef NVIC_InitStructure;

//    /* EXTI configuration */
//    EXTI_ClearITPendingBit( EXTI_Line17 );
//    EXTI_InitStructure.EXTI_Line = EXTI_Line17;
//    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
//    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
//    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
//    EXTI_Init( &EXTI_InitStructure );

//    /* Enable the RTC Alarm Interrupt */
//    NVIC_InitStructure.NVIC_IRQChannel = RTC_Alarm_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init( &NVIC_InitStructure );

//    /* Set the alarmA Masks */
//    RTC_AlarmStructure.RTC_AlarmMask = RTC_AlarmMask_DateWeekDay;
//    RTC_SetAlarm( RTC_Format_BIN, RTC_Alarm_A, &RTC_AlarmStructure );

//    /* Disable AlarmA/B interrupt */
//    RTC_ITConfig( RTC_IT_ALRA, DISABLE );
//    RTC_ITConfig( RTC_IT_ALRB, DISABLE );

//    /* Disable the alarmA/B */
//    RTC_AlarmCmd( RTC_Alarm_A, DISABLE );
//    RTC_AlarmCmd( RTC_Alarm_B, DISABLE );
//}

void RtcGetTime(uint8_t *h, uint8_t *m, uint8_t *s)
{
    RTC_TimeTypeDef stimestructure;
    RTC_DateTypeDef sdatestructure;
    HAL_RTC_GetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &sdatestructure, RTC_FORMAT_BIN);
    *h = stimestructure.Hours;
    *m = stimestructure.Minutes;
    *s = stimestructure.Seconds;
}

void RtcTimeCalibration(uint8_t *h, uint8_t *m, uint8_t *s)
{
    while(*s > 59)
    {
        *s -= 60;
        *m +=  1;
    }

    while(*m > 59)
    {
        *m -= 60;
        *h +=  1;
    }

    if(*h > 23)
        *h = 0;

    return;
}

bool RtcSetTime(uint8_t h, uint8_t m, uint8_t s)
{
    RTC_TimeTypeDef  stimestructure;

    if(h > 23)
        return FAIL;
    if(m > 59)
        return FAIL;
    if(s > 59)
        return FAIL;
    stimestructure.Hours = h;
    stimestructure.Minutes = m;
    stimestructure.Seconds = s;
    stimestructure.TimeFormat = RTC_HOURFORMAT12_AM;
    stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE ;
    stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

    if(HAL_RTC_SetTime(&RtcHandle,&stimestructure,RTC_FORMAT_BIN) != HAL_OK)
    {
        /* Initialization Error */
        return FAIL;
    }

    return OK;
}

void RtcGetDate(uint8_t *y, uint8_t *m, uint8_t *d)
{
    RTC_TimeTypeDef stimestructure;
    RTC_DateTypeDef sdatestructure;
    HAL_RTC_GetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &sdatestructure, RTC_FORMAT_BIN);
    *y = sdatestructure.Year;
    *m = sdatestructure.Month;
    *d = sdatestructure.Date;
}

bool RtcSetDate(uint8_t y, uint8_t m, uint8_t d)
{
    RTC_DateTypeDef  sdatestructure;
    uint8_t leap;

    leap = y%4;

    if(y > 100)
        return FAIL;
    if(m > 12)
        return FAIL;
    if( (m == 1) || (m == 3) || (m == 5) || (m == 7) || (m == 8) || (m == 10) || (m == 12))
    {
        if(d > 31)
            return FAIL;
    }
    else if( (m == 4) || (m == 6) || (m == 9) || (m == 11) )
    {
        if(d > 30)
            return FAIL;
    }
    else
    {
        if(leap == 0)
        {
            if(d > 29)
                return FAIL;
        }
        else
        {
            if(d > 28)
                return FAIL;
        }
    }

    sdatestructure.Year = y;
    sdatestructure.Month = m;
    sdatestructure.Date = d;
    sdatestructure.WeekDay = 0;

    if(HAL_RTC_SetDate(&RtcHandle,&sdatestructure,RTC_FORMAT_BIN) != HAL_OK)
    {
        /* Initialization Error */
        return FAIL;
    }

    return OK;
}

void RtcSetAlarm(uint8_t h, uint8_t m, uint8_t s, MCUALARM_e AlarmName)
{
    RTC_AlarmTypeDef salarmstructure;
    if(AlarmName == MCU_ALARM_A)
    {
        salarmstructure.Alarm = RTC_ALARM_A;
    }
    else if(AlarmName == MCU_ALARM_B)
    {
        salarmstructure.Alarm = RTC_ALARM_B;
    }
    salarmstructure.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
    salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    salarmstructure.AlarmMask = RTC_ALARMMASK_NONE | RTC_ALARMMASK_DATEWEEKDAY;
    //salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
    //salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
    salarmstructure.AlarmTime.Hours = h;
    salarmstructure.AlarmTime.Minutes = m;
    salarmstructure.AlarmTime.Seconds = s;
    //salarmstructure.AlarmTime.SubSeconds = 0x56;

    HAL_RTC_SetAlarm_IT(&RtcHandle,&salarmstructure,RTC_FORMAT_BIN);
//  if(HAL_RTC_SetAlarm_IT(&RtcHandle,&salarmstructure,RTC_FORMAT_BIN) != HAL_OK)
//  {
//    /* Initialization Error */
//    Error_Handler();
//  }
}

void RtcSetDateAlarm(uint8_t d, uint8_t h, uint8_t m, uint8_t s, MCUALARM_e AlarmName)
{
    RTC_AlarmTypeDef salarmstructure;
    if(AlarmName == MCU_ALARM_A)
    {
        salarmstructure.Alarm = RTC_ALARM_A;
    }
    else if(AlarmName == MCU_ALARM_B)
    {
        salarmstructure.Alarm = RTC_ALARM_B;
    }
    salarmstructure.AlarmDateWeekDay = d;
    salarmstructure.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    salarmstructure.AlarmMask = RTC_ALARMMASK_NONE;
    //salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
    //salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
    salarmstructure.AlarmTime.Hours = d;
    salarmstructure.AlarmTime.Minutes = m;
    salarmstructure.AlarmTime.Seconds = s;
    //salarmstructure.AlarmTime.SubSeconds = 0x56;

    HAL_RTC_SetAlarm_IT(&RtcHandle,&salarmstructure,RTC_FORMAT_BIN);
//  if(HAL_RTC_SetAlarm_IT(&RtcHandle,&salarmstructure,RTC_FORMAT_BIN) != HAL_OK)
//  {
//    /* Initialization Error */
//    Error_Handler();
//  }
}

void RtcClearStatus( MCUALARM_e AlarmName )
{
    if(AlarmName == MCU_ALARM_A)
    {
        HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_A);
    }
    else if(AlarmName == MCU_ALARM_B)
    {
        HAL_RTC_DeactivateAlarm(&RtcHandle, RTC_ALARM_B);
    }
}

void RtcDisableAlarm( MCUALARM_e AlarmName )
{
    RtcClearStatus( AlarmName);
}

void RtcStartWakeUpAlarm( uint32_t timeoutValue, MCUALARM_e AlarmName )
{
    uint16_t rtcSeconds = 0;
    uint16_t rtcMinutes = 0;
    uint16_t rtcHours = 0;
    uint16_t rtcDays = 0;

    uint8_t rtcAlarmSeconds = 0;
    uint8_t rtcAlarmMinutes = 0;
    uint8_t rtcAlarmHours = 0;
    uint16_t rtcAlarmDays = 0;

    RTC_DateTypeDef  sdatestructure;
    RTC_TimeTypeDef  stimestructure;
    RTC_AlarmTypeDef salarmstructure;

    if(AlarmName == MCU_ALARM_A)
    {
        salarmstructure.Alarm = RTC_ALARM_A;
    }
    else if(AlarmName == MCU_ALARM_B)
    {
        salarmstructure.Alarm = RTC_ALARM_B;
    }

    RtcClearStatus(AlarmName);

    HAL_RTC_GetTime(&RtcHandle, &stimestructure, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&RtcHandle, &sdatestructure, RTC_FORMAT_BIN);
    HAL_RTC_WaitForSynchro(&RtcHandle);
    rtcSeconds = ( timeoutValue % SECS_IN_MIN ) + stimestructure.Seconds;
    rtcMinutes = ( ( timeoutValue / SECS_IN_MIN ) % SECS_IN_MIN ) + stimestructure.Minutes;
    rtcHours = ( ( timeoutValue / SECS_IN_HR ) % HRS_IN_DAY ) + stimestructure.Hours;
    rtcDays = ( timeoutValue / SECS_IN_DAY ) + sdatestructure.Date;

    rtcAlarmSeconds = ( rtcSeconds ) % 60;
    rtcAlarmMinutes = ( ( rtcSeconds / 60 ) + rtcMinutes ) % 60;
    rtcAlarmHours   = ( ( ( ( rtcSeconds / 60 ) + rtcMinutes ) / 60 ) + rtcHours ) % 24;
    rtcAlarmDays    = ( ( ( ( ( rtcSeconds / 60 ) + rtcMinutes ) / 60 ) + rtcHours ) / 24 ) + rtcDays;

    if( ( sdatestructure.Year == 0 ) || ( sdatestructure.Year % 4 == 0 ) )
    {
        if( rtcAlarmDays > DaysInMonthLeapYear[ sdatestructure.Month - 1 ] )
        {
            rtcAlarmDays = rtcAlarmDays % DaysInMonthLeapYear[ sdatestructure.Month - 1 ];
        }
    }
    else
    {
        if( rtcAlarmDays > DaysInMonth[ sdatestructure.Month - 1 ] )
        {
            rtcAlarmDays = rtcAlarmDays % DaysInMonth[ sdatestructure.Month - 1 ];
        }
    }

    salarmstructure.AlarmTime.Seconds = rtcAlarmSeconds;
    salarmstructure.AlarmTime.Minutes = rtcAlarmMinutes;
    salarmstructure.AlarmTime.Hours  = rtcAlarmHours;
    salarmstructure.AlarmTime.SubSeconds = 0x00;
    salarmstructure.AlarmDateWeekDay      = ( uint8_t )rtcAlarmDays;
    salarmstructure.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_NONE;
    salarmstructure.AlarmTime.TimeFormat = RTC_HOURFORMAT12_AM;
    salarmstructure.AlarmDateWeekDaySel   = RTC_ALARMDATEWEEKDAYSEL_DATE;
    salarmstructure.AlarmMask             = RTC_ALARMMASK_NONE;

    HAL_RTC_SetAlarm_IT(&RtcHandle,&salarmstructure,RTC_FORMAT_BIN);

}


void RTC_IRQHandler( void )
{
#ifdef LOW_POWER_MODE
        //RecoverMcuStatus();
     __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
#endif
    HAL_RTC_AlarmIRQHandler(&RtcHandle);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
//    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
//    SystemClockReConfig();
    if( AlarmIrqCallback != NULL )
    {
        AlarmIrqCallback(MCU_ALARM_A);
    }
}

void HAL_RTC_AlarmBEventCallback(RTC_HandleTypeDef *hrtc)
{
    if( AlarmIrqCallback != NULL )
    {
        AlarmIrqCallback(MCU_ALARM_B);
    }
}

// Convert RTC date/time to epoch time
// input:
//    YY - year, range 0-99
//    MM - month, range 1-12
//    DD - date, range 1-31
//    hh - hour, range 0-23
//    mm - minute, range 0-59
//    ss - second, range 0-59
uint32_t RtcGetEpochFromUTC( uint8_t YY, uint8_t MM, uint8_t DD, uint8_t hh, uint8_t mm, uint8_t ss )
{
    uint8_t  a;
    uint16_t y;
    uint8_t  m;
    uint32_t JDN;

    // Calculate some coefficients
    a = (14 - MM) / 12;
    y = YY + 6800 - a; // years since 1 March, 4801 BC
    m = MM + (12 * a) - 3;

    // Compute Julian day number (from Gregorian calendar date)
    JDN  = DD;
    JDN += ((153 * m) + 2) / 5; // Number of days since 1 march
    JDN += 365 * y;
    JDN += y / 4;
    JDN -= y / 100;
    JDN += y / 400;
    JDN -= 32045;

    // Subtract number of days passed before base date from Julian day number
    JDN -= JULIAN_DATE_BASE;

    // Convert days to seconds
    JDN *= 86400;

    // Add to epoch specified time in seconds
    JDN += hh * 3600;
    JDN += mm * 60;
    JDN += ss;

    // Number of seconds passed since the base date
    return JDN;
}

// Convert epoch time to RTC date/time
// input:
//   epoch - 32-bit epoch value (seconds)
//    YY - year, range 0-99
//    MM - month, range 1-12
//    DD - date, range 1-31
//    hh - hour, range 0-23
//    mm - minute, range 0-59
//    ss - second, range 0-59
void RtcGetUTCFromEpoch( uint32_t epoch, uint8_t *YY, uint8_t *MM, uint8_t *DD, uint8_t *hh, uint8_t *mm, uint8_t *ss )
{
    uint32_t a,b,c,d;

    // Calculate JDN (Julian day number) from a specified epoch value
    a = (epoch / 86400) + JULIAN_DATE_BASE;

    // Day of week
    //date->RTC_WeekDay = (a % 7) + 1;

    // Calculate intermediate values
    a += 32044;
    b  = ((4 * a) + 3) / 146097;
    a -= (146097 * b) / 4;
    c  = ((4 * a) + 3) / 1461;
    a -= (1461 * c) / 4;
    d  = ((5 * a) + 2) / 153;

    // Date
    *DD = a - (((153 * d) + 2) / 5) + 1;
    *MM = d + 3 - (12 * (d / 10));
    *YY = (100 * b) + c - 6800 + (d / 10);

    // Time
    *hh = (epoch / 3600) % 24;
    *mm = (epoch / 60) % 60;
    *ss =  epoch % 60;
}

void HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc)
{
  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

  /*##-1- Enables the PWR Clock and Enables access to the backup domain ###################################*/
  /* To change the source clock of the RTC feature (LSE, LSI), You have to:
     - Enable the power clock using __HAL_RCC_PWR_CLK_ENABLE()
     - Enable write access using HAL_PWR_EnableBkUpAccess() function before to
       configure the RTC clock source (to be done once after reset).
     - Reset the Back up Domain using __HAL_RCC_BACKUPRESET_FORCE() and
       __HAL_RCC_BACKUPRESET_RELEASE().
     - Configure the needed RTC clock source */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  /*##-2- Configue LSE/LSI as RTC clock soucre ###############################*/
#ifdef RTC_CLOCK_SOURCE_LSE
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //Error_Handler();
      while(1)
      {
      }
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    while(1)
      {
      }
  }
#elif defined (RTC_CLOCK_SOURCE_LSI)
  RCC_OscInitStruct.OscillatorType =  RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_OFF;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1)
    {
    }
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    while(1)
    {
    }
  }
#else
#error Please select the RTC Clock source inside the main.h file
#endif /*RTC_CLOCK_SOURCE_LSE*/

  /*##-2- Enable RTC peripheral Clocks #######################################*/
  /* Enable RTC Clock */
  __HAL_RCC_RTC_ENABLE();

  /*##-4- Configure the NVIC for RTC Alarm ###################################*/
  HAL_NVIC_SetPriority(RTC_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(RTC_IRQn);
}

/**
  * @brief RTC MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hrtc: RTC handle pointer
  * @retval None
  */
void HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc)
{
  /*##-1- Reset peripherals ##################################################*/
   __HAL_RCC_RTC_DISABLE();
}

uint32_t RtcGetEpochTimeStamp()
{
    uint8_t YY,MM,DD,hh,mm,ss;
    RtcGetTime(&hh,&mm,&ss);
    RtcGetDate(&YY,&MM,&DD);
    return RtcGetEpochFromUTC(YY, MM, DD, hh, mm, ss );
}

