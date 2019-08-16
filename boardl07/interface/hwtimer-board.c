#include "hwtimer-board.h"
#include <stddef.h>
#include "board.h"
#ifdef STM32L151xBA
#include "stm32l151xba.h"
#endif
#include "rtc-board.h"


static HwTimerIrqHandler *TimerIrqCallback = NULL;

#ifdef LORAWANV1_0
/*!
 * Hardware Time base in us
 */
#define HW_TIMER_TIME_BASE                              100 //us

/*!
 * Hardware Timer tick counter
 */
volatile TimerTime_t TimerTickCounter = 1;

/*!
 * Saved value of the Tick counter at the start of the next event
 */
//static TimerTime_t TimerTickCounterContext = 0;

/*!
 * Value trigging the IRQ
 */
//volatile TimerTime_t TimeoutCntValue = 0;

/*!
 * Increment the Hardware Timer tick counter
 */
void TimerIncrementTickCounter( void );

/*!
 * Counter used for the Delay operations
 */
//volatile uint32_t TimerDelayCounter = 0;

/*!
 * Return the value of the counter used for a Delay
 */
//uint32_t TimerHwGetDelayValue( void );

/*!
 * Increment the value of TimerDelayCounter
 */
void TimerIncrementDelayCounter( void );
TIM_HandleTypeDef    Tim2Handle;
#endif
TIM_HandleTypeDef    Tim3Handle;
void HwTimerSetCallback(HwTimerIrqHandler * irqHandler)
{
    TimerIrqCallback = irqHandler;
}

void HwTimerInit()
{
#ifdef LORAWANV1_0
    /* TIM2 clock enable */
//    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );

//    /* --------------------------NVIC Configuration -------------------------------*/
//    /* Enable the TIM2 gloabal Interrupt */
//    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

//    NVIC_Init( &NVIC_InitStructure );

////    TimeoutCntValue = 0;

//    /* Time base configuration */
//    TIM_TimeBaseStructure.TIM_Period = 3199;
//    TIM_TimeBaseStructure.TIM_Prescaler = 0;
//    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
//    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//    TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure );

//    TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE );

    /* TIM2 disable counter */
//    TIM_Cmd( TIM2, ENABLE );
  /* Set TIMx instance */
  Tim2Handle.Instance = TIM2;

  /* Initialize TIMx peripheral as follows:
       + Period = 10 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
#ifdef TIMER_BASE_MS
  Tim2Handle.Init.Period = 9;
  Tim2Handle.Init.Prescaler = (SystemCoreClock/10000) - 1;/*systemr clock/3200000 - 1*/
#else
  Tim2Handle.Init.Period            = 3200 - 1;
  Tim2Handle.Init.Prescaler         = 0 ;/*systemr clock - 1*/;
#endif
  Tim2Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  Tim2Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;

  if (HAL_TIM_Base_Init(&Tim2Handle) != HAL_OK)
  {
    /* Initialization Error */
    while(1)
    {
    }
  }
	HAL_TIM_Base_Start_IT(&Tim2Handle);
#endif

  /* Set TIMx instance */
  Tim3Handle.Instance = TIM3;

  /* Initialize TIMx peripheral as follows:
       + Period = 10 - 1
       + Prescaler = (SystemCoreClock/10000) - 1
       + ClockDivision = 0
       + Counter direction = Up
  */
  Tim3Handle.Init.Period            = 10 - 1;
  Tim3Handle.Init.Prescaler         = (SystemCoreClock/10000) - 1;/*systemr clock/3200000 - 1*/;
  Tim3Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  Tim3Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;

  if (HAL_TIM_Base_Init(&Tim3Handle) != HAL_OK)
  {
    /* Initialization Error */
    while(1)
    {
    }
  }
}
void HwTimerDeInit(void)
{
    HAL_TIM_Base_DeInit(&Tim3Handle);
}

void HwTimerStart(void)
{
    HAL_TIM_Base_Start_IT(&Tim3Handle);
}

void HwTimerStop(void)
{
    HAL_TIM_Base_Stop_IT(&Tim3Handle);
}

bool HwTimerBusy(void)
{
    if((Tim3Handle.Instance->CR1&& TIM_CR1_CEN) == 1)
        return OK;
    else
        return FAIL;
}

#ifdef LORAWANV1_0

TimerTime_t TimerHwGetTimerValue( void )
{
    TimerTime_t val = 0;

    __disable_irq( );

    val = TimerTickCounter;

    __enable_irq( );

    return( val );
}

TimerTime_t TimerHwGetTime( void )
{
#ifdef TIMER_BASE_MS
    return TimerHwGetTimerValue( );
#else
    return TimerHwGetTimerValue( ) * HW_TIMER_TIME_BASE;
#endif
}

#ifdef TIMER_BASE_MS
TimerTime_t TimerHwGetElapsedTime( TimerTime_t savedTime )
{
    TimerTime_t elapsedTime = 0;

    // Needed at boot, cannot compute with 0 or elapsed time will be equal to current time
    if( savedTime == 0 )
    {
        return 0;
    }

    elapsedTime = TimerHwGetTimerValue();

    if( elapsedTime < savedTime )
    { // roll over of the counter
        return( elapsedTime + ( 0xFFFFFFFF - savedTime ) );
    }
    else
    {
        return( elapsedTime - savedTime );
    }
}
#endif

void TimerIncrementTickCounter( void )
{
    __disable_irq( );

    TimerTickCounter++;

    __enable_irq( );
}

void TIM2_IRQHandler( void )
{
	    HAL_TIM_IRQHandler(&Tim2Handle);
//    if( TIM_GetITStatus( TIM2, TIM_IT_Update ) != RESET )
//    {
//        TimerIncrementTickCounter( );
//        TIM_ClearITPendingBit( TIM2, TIM_IT_Update );

//        if( TimerTickCounter == TimeoutCntValue )
//        {
//            TimerIrqHandler( );
//        }
//    }
}
#endif

/*!
 * Timer IRQ handler
 */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&Tim3Handle);
//    __disable_irq( );
//    if( TIM_GetITStatus( TIM4, TIM_IT_Update ) != RESET )
//    {
//		if( TimerIrqCallback != NULL )
//        {
//            TimerIrqCallback( );
//        }
//        TIM_ClearITPendingBit( TIM4, TIM_IT_Update );
//    }
//     __enable_irq( );
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM3)
    {
        if( TimerIrqCallback != NULL )
        {
            TimerIrqCallback( );
        }
				//TimerIncrementTickCounter( );
    }
#ifdef LORAWANV1_0
		else if(htim->Instance == TIM2)
    {
        TimerIncrementTickCounter( );
    }
#endif
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2)
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* TIMx Peripheral clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();
        /*##-2- Configure the NVIC for TIMx ########################################*/
        /* Set the TIMx priority */
        HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);

        /* Enable the TIMx global Interrupt */
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
    else if(htim->Instance == TIM3)
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* TIMx Peripheral clock enable */
        __HAL_RCC_TIM3_CLK_ENABLE();
        /*##-2- Configure the NVIC for TIMx ########################################*/
        /* Set the TIMx priority */
        HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);

        /* Enable the TIMx global Interrupt */
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
}

#ifdef LORAWANV1_0
volatile uint32_t BackupEpochTimeStamp = 0;
volatile uint64_t BackupTimerTickCounter = 0;

void HwTimerBackupTickCounter()
{
    __disable_irq( );
    BackupEpochTimeStamp = RtcGetEpochTimeStamp();
    BackupTimerTickCounter = TimerHwGetTime();
    __enable_irq( );
}

void HwTimerResume()
{
    __disable_irq( );
    if(BackupEpochTimeStamp != 0)
    {
        int32_t diff = RtcGetEpochTimeStamp() - BackupEpochTimeStamp;
        TimerTickCounter = BackupTimerTickCounter + diff*1000;
    }
    __enable_irq( );
}
#endif
