#include "FreeRTOS.h"
#include "task.h"
#include "timer_task.h"
#include "uart_task.h"
#include "board.h"
#include "radio_task.h"
#include "i2c-board.h"
#include "rtc-board.h"
#include "spi-board.h"
#include "sx1276.h"
#include "hwtimer-board.h"
#include "uart-board.h"

#ifdef STM32L073xx
#include "stm32l0xx_hal.h"
#elif STM32L151xBA
#include "stm32l1xx_hal.h"
#endif

#if configUSE_TICKLESS_IDLE == 1

/* Constants required to manipulate the core.  Registers first... */
#define portNVIC_SYSTICK_CTRL_REG			( * ( ( volatile uint32_t * ) 0xe000e010 ) )
#define portNVIC_SYSTICK_LOAD_REG			( * ( ( volatile uint32_t * ) 0xe000e014 ) )
#define portNVIC_SYSTICK_CURRENT_VALUE_REG	( * ( ( volatile uint32_t * ) 0xe000e018 ) )
#define portNVIC_SYSPRI2_REG				( * ( ( volatile uint32_t * ) 0xe000ed20 ) )
/* ...then bits in the registers. */
#define portNVIC_SYSTICK_INT_BIT			( 1UL << 1UL )
#define portNVIC_SYSTICK_ENABLE_BIT			( 1UL << 0UL )
#define portNVIC_SYSTICK_COUNT_FLAG_BIT		( 1UL << 16UL )
#define portNVIC_PENDSVCLEAR_BIT 			( 1UL << 27UL )
#define portNVIC_PEND_SYSTICK_CLEAR_BIT		( 1UL << 25UL )

/* Constants used with memory barrier intrinsics. */
#define portSY_FULL_READ_WRITE		( 15 )

extern uint32_t xMaximumPossibleSuppressedTicks;
extern uint32_t ulStoppedTimerCompensation;
extern uint32_t ulTimerCountsForOneTick;
#endif

int main()
 {
//    BoardInit();
    BoardInitPeriph();
//	STMPE1801Init();
//    TMP112Init();
//    MMA8451Init();
    CreateTimerTaskQueue();
    xTaskCreate(TimerTask, "TimerTask" ,96,NULL,1,NULL);
#if defined( STM32L073xx )
    xTaskCreate(RadioTask, "RadioTask" ,196,NULL,4,&xRadioTaskHandle);
#else
    xTaskCreate(RadioTask, "RadioTask" ,128,NULL,4,&xRadioTaskHandle);
#endif
    //xTaskCreate(UartTask, "UartTask" ,256,NULL,2,&xUartTaskHandle);
    xTaskCreate(UartTask, "UartTask" ,320,NULL,2,&xUartTaskHandle); //daniel note on 2017.3.15 -> modify from 256 to 320
    vTaskStartScheduler();

}

xTaskHandle *bad_task_handle;
signed char *bad_task_name;

#if defined( STM32L073xx )

#if( configCHECK_FOR_STACK_OVERFLOW == 1 )
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    bad_task_handle = pxTask;     // this seems to give me the crashed task handle
    bad_task_name = pcTaskName;     // this seems to give me a pointer to the name of the crashed task
//mLED_3_On();   // a LED is lit when the task has crashed

    for(;;);
}
#endif
#if( configUSE_MALLOC_FAILED_HOOK == 1 )
void vApplicationMallocFailedHook(void)
{
    for(;;);
}
#endif

#if configUSE_TICKLESS_IDLE == 1
/**
  * @brief  Pre Sleep Processing
  * @param  ulExpectedIdleTime: Expected time in idle state
  * @retval None
  */
void PreSleepProcessing(uint32_t * ulExpectedIdleTime)
{
    /* Called by the kernel before it places the MCU into a sleep mode because
    configPRE_SLEEP_PROCESSING() is #defined to PreSleepProcessing().

    NOTE:  Additional actions can be taken here to get the power consumption
    even lower.  For example, peripherals can be turned off here, and then back
    on again in the post sleep processing function.  For maximum power saving
    ensure all unused pins are in their lowest power state. */

    /*
      (*ulExpectedIdleTime) is set to 0 to indicate that PreSleepProcessing contains
      its own wait for interrupt or wait for event instruction and so the kernel vPortSuppressTicksAndSleep
      function does not need to execute the wfi instruction
    */
    *ulExpectedIdleTime = 0;
    SX1276IoDeInit();
    BoardDeInitPeriph();
    // Disable the Power Voltage Detector
    HAL_PWR_DisablePVD();

    // Enable Ultra low power mode
    HAL_PWREx_EnableUltraLowPower();

    // Enable the fast wake up from Ultra low power mode
    HAL_PWREx_EnableFastWakeUp();
    /*Enter to sleep Mode using the HAL function HAL_PWR_EnterSLEEPMode with WFI instruction*/
    //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
}

/**
  * @brief  Post Sleep Processing
  * @param  ulExpectedIdleTime: Not used
  * @retval None
  */
void PostSleepProcessing(uint32_t * ulExpectedIdleTime)
{
    /* Called by the kernel when the MCU exits a sleep mode because
    configPOST_SLEEP_PROCESSING is #defined to PostSleepProcessing(). */

    /* Avoid compiler warnings about the unused parameter. */
    (void) ulExpectedIdleTime;
    BoardInitPeriph();
    SX1276IoInit();
}

/**
  * @brief  Configure all GPIO's to AN to reduce the power consumption
  * @param  None
  * @retval None
//  */
//static void GPIO_ConfigAN(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct;

//  /* Configure all GPIO as analog to reduce current consumption on non used IOs */
//  /* Enable GPIOs clock */
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//  __HAL_RCC_GPIOC_CLK_ENABLE();
////  __HAL_RCC_GPIOD_CLK_ENABLE();
////  __HAL_RCC_GPIOE_CLK_ENABLE();
////  __HAL_RCC_GPIOF_CLK_ENABLE();
////  __HAL_RCC_GPIOG_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();

//  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Pin = GPIO_PIN_All;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
////  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
////  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
////  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
////  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);
//  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

//  /* Disable GPIOs clock */
//  __HAL_RCC_GPIOA_CLK_DISABLE();
//  __HAL_RCC_GPIOB_CLK_DISABLE();
//  __HAL_RCC_GPIOC_CLK_DISABLE();
////  __HAL_RCC_GPIOD_CLK_DISABLE();
////  __HAL_RCC_GPIOE_CLK_DISABLE();
////  __HAL_RCC_GPIOF_CLK_DISABLE();
////  __HAL_RCC_GPIOG_CLK_DISABLE();
//  __HAL_RCC_GPIOH_CLK_DISABLE();
//}

//static void GPIO_ConfigTest(void)
//{
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//      GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull  = GPIO_PULLUP;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

//  GPIO_InitStruct.Pin = GPIO_PIN_2;
//  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//}
void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements, ulSysTickCTRL;
    TickType_t xModifiableIdleTime;

    /*check Timer enable or disable*/
    if(HwTimerBusy() == OK)
        return;

    /*check Uart enable or disable*/
    if(CheckUartBusy() == OK)
        return;

    /* Make sure the SysTick reload value does not overflow the counter. */
    if(xExpectedIdleTime > xMaximumPossibleSuppressedTicks)
    {
        xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
    }

    /* Stop the SysTick momentarily.  The time the SysTick is stopped for
    is accounted for as best it can be, but using the tickless mode will
    inevitably result in some tiny drift of the time maintained by the
    kernel with respect to calendar time. */
    portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;

    /* Calculate the reload value required to wait xExpectedIdleTime
    tick periods.  -1 is used because this code will execute part way
    through one of the tick periods. */
    ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG + (ulTimerCountsForOneTick * (xExpectedIdleTime - 1UL));
    if(ulReloadValue > ulStoppedTimerCompensation)
    {
        ulReloadValue -= ulStoppedTimerCompensation;
    }

    /* Enter a critical section but don't use the taskENTER_CRITICAL()
    method as that will mask interrupts that should exit sleep mode. */
    __disable_irq();

    /* If a context switch is pending or a task is waiting for the scheduler
    to be unsuspended then abandon the low power entry. */
    if(eTaskConfirmSleepModeStatus() == eAbortSleep)
    {
        /* Restart from whatever is left in the count register to complete
        this tick period. */
        portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;

        /* Restart SysTick. */
        portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

        /* Reset the reload register to the value required for normal tick
        periods. */
        portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;

        /* Re-enable interrupts - see comments above __disable_irq() call
        above. */
        __enable_irq();
    }
    else
    {
        /* Set the new reload value. */
        portNVIC_SYSTICK_LOAD_REG = ulReloadValue;

        /* Clear the SysTick count flag and set the count value back to
        zero. */
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;

        /* Restart SysTick. */
        portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;

        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
        set its parameter to 0 to indicate that its implementation contains
        its own wait for interrupt or wait for event instruction, and so wfi
        should not be executed again.  However, the original expected idle
        time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;
        configPRE_SLEEP_PROCESSING(&xModifiableIdleTime);
        if(xModifiableIdleTime > 0)
        {
            __dsb(portSY_FULL_READ_WRITE);
            __wfi();
            __isb(portSY_FULL_READ_WRITE);
        }
        // Disable the Power Voltage Detector
//        HAL_PWR_DisablePVD( );

//        // Enable Ultra low power mode
//        HAL_PWREx_EnableUltraLowPower( );

//        // Enable the fast wake up from Ultra low power mode
//        HAL_PWREx_EnableFastWakeUp( );
//  /*Enter to sleep Mode using the HAL function HAL_PWR_EnterSLEEPMode with WFI instruction*/
//  //HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
//    HAL_PWR_EnterSTOPMode( PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI );
        configPOST_SLEEP_PROCESSING(&xExpectedIdleTime);

        /* Stop SysTick.  Again, the time the SysTick is stopped for is
        accounted for as best it can be, but using the tickless mode will
        inevitably result in some tiny drift of the time maintained by the
        kernel with respect to calendar time. */
        ulSysTickCTRL = portNVIC_SYSTICK_CTRL_REG;
        portNVIC_SYSTICK_CTRL_REG = (ulSysTickCTRL & ~portNVIC_SYSTICK_ENABLE_BIT);

        /* Re-enable interrupts - see comments above __disable_irq() call
        above. */
        __enable_irq();

        if((ulSysTickCTRL & portNVIC_SYSTICK_COUNT_FLAG_BIT) != 0)
        {
            uint32_t ulCalculatedLoadValue;

            /* The tick interrupt has already executed, and the SysTick
            count reloaded with ulReloadValue.  Reset the
            portNVIC_SYSTICK_LOAD_REG with whatever remains of this tick
            period. */
            ulCalculatedLoadValue = (ulTimerCountsForOneTick - 1UL) - (ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG);

            /* Don't allow a tiny value, or values that have somehow
            underflowed because the post sleep hook did something
            that took too long. */
            if((ulCalculatedLoadValue < ulStoppedTimerCompensation) || (ulCalculatedLoadValue > ulTimerCountsForOneTick))
            {
                ulCalculatedLoadValue = (ulTimerCountsForOneTick - 1UL);
            }

            portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;

            /* The tick interrupt handler will already have pended the tick
            processing in the kernel.  As the pending tick will be
            processed as soon as this function exits, the tick value
            maintained by the tick is stepped forward by one less than the
            time spent waiting. */
            ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
        }
        else
        {
            /* Something other than the tick interrupt ended the sleep.
            Work out how long the sleep lasted rounded to complete tick
            periods (not the ulReload value which accounted for part
            ticks). */
            ulCompletedSysTickDecrements = (xExpectedIdleTime * ulTimerCountsForOneTick) - portNVIC_SYSTICK_CURRENT_VALUE_REG;

            /* How many complete tick periods passed while the processor
            was waiting? */
            ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;

            /* The reload value is set to whatever fraction of a single tick
            period remains. */
            portNVIC_SYSTICK_LOAD_REG = ((ulCompleteTickPeriods + 1) * ulTimerCountsForOneTick) - ulCompletedSysTickDecrements;
        }

        /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
        again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
        value.  The critical section is used to ensure the tick interrupt
        can only execute once in the case that the reload register is near
        zero. */
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
        portENTER_CRITICAL();
        {
            portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
            vTaskStepTick(ulCompleteTickPeriods);
            portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;
        }
        portEXIT_CRITICAL();
    }
}
#endif

#else

#if( configCHECK_FOR_STACK_OVERFLOW == 1 )
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    bad_task_handle = pxTask;     // this seems to give me the crashed task handle
    bad_task_name = pcTaskName;     // this seems to give me a pointer to the name of the crashed task
//mLED_3_On();   // a LED is lit when the task has crashed
    for(;;);
}
#endif
#if( configUSE_MALLOC_FAILED_HOOK == 1 )
void vApplicationMallocFailedHook(void)
{
    for(;;);
}
#endif

#if configUSE_TICKLESS_IDLE == 1
void vPreSleepProcessing(unsigned long xExpectedIdleTime)
{
    __disable_irq();
    SX1276IoDeInit();
    BoardDeInitPeriph();
    __enable_irq();
    /* Disable the Power Voltage Detector */
    PWR_PVDCmd(DISABLE);
    /* Set MCU in ULP (Ultra Low Power) */
    PWR_UltraLowPowerCmd(ENABLE);
    /*Disable fast wakeUp*/
    PWR_FastWakeUpCmd(DISABLE);
    PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}

void vPostSleepProcessing(unsigned long xExpectedIdleTime)
{
    BoardInitPeriph();
    SX1276IoInit();
}

void vPortSuppressTicksAndSleep(TickType_t xExpectedIdleTime)
{
    uint32_t ulReloadValue, ulCompleteTickPeriods, ulCompletedSysTickDecrements, ulSysTickCTRL;
    TickType_t xModifiableIdleTime;
    /*check Timer enable or disable*/
    if((TIM4->CR1&&TIM_CR1_CEN) == 1 || (USART2->CR1 != 0) || (USART1->CR1 != 0))
        return;
    /* Make sure the SysTick reload value does not overflow the counter. */
    if(xExpectedIdleTime > xMaximumPossibleSuppressedTicks)
    {
        xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
    }
    /* Stop the SysTick momentarily.  The time the SysTick is stopped for
    is accounted for as best it can be, but using the tickless mode will
    inevitably result in some tiny drift of the time maintained by the
    kernel with respect to calendar time. */
    portNVIC_SYSTICK_CTRL_REG &= ~portNVIC_SYSTICK_ENABLE_BIT;
    /* Calculate the reload value required to wait xExpectedIdleTime
    tick periods.  -1 is used because this code will execute part way
    through one of the tick periods. */
    ulReloadValue = portNVIC_SYSTICK_CURRENT_VALUE_REG + (ulTimerCountsForOneTick * (xExpectedIdleTime - 1UL));
    if(ulReloadValue > ulStoppedTimerCompensation)
    {
        ulReloadValue -= ulStoppedTimerCompensation;
    }
    /* Enter a critical section but don't use the taskENTER_CRITICAL()
    method as that will mask interrupts that should exit sleep mode. */
    __disable_irq();

    /* If a context switch is pending or a task is waiting for the scheduler
    to be unsuspended then abandon the low power entry. */
    if(eTaskConfirmSleepModeStatus() == eAbortSleep)
    {
        /* Restart from whatever is left in the count register to complete
        this tick period. */
        portNVIC_SYSTICK_LOAD_REG = portNVIC_SYSTICK_CURRENT_VALUE_REG;
        /* Restart SysTick. */
        portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
        /* Reset the reload register to the value required for normal tick
        periods. */
        portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;
        /* Re-enable interrupts - see comments above __disable_irq() call
        above. */
        __enable_irq();
    }
    else
    {
        /* Set the new reload value. */
        portNVIC_SYSTICK_LOAD_REG = ulReloadValue;
        /* Clear the SysTick count flag and set the count value back to
        zero. */
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
        /* Restart SysTick. */
        portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
        /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
        set its parameter to 0 to indicate that its implementation contains
        its own wait for interrupt or wait for event instruction, and so wfi
        should not be executed again.  However, the original expected idle
        time variable must remain unmodified, so a copy is taken. */
        xModifiableIdleTime = xExpectedIdleTime;

        configPRE_SLEEP_PROCESSING(xModifiableIdleTime);
        if(xModifiableIdleTime > 0)
        {
            __dsb(portSY_FULL_READ_WRITE);
            __wfi();
            __isb(portSY_FULL_READ_WRITE);
        }

        /* Disable the Power Voltage Detector */
        //PWR_PVDCmd( DISABLE );
        /* Set MCU in ULP (Ultra Low Power) */
        //PWR_UltraLowPowerCmd( ENABLE );
        /*Disable fast wakeUp*/
        //PWR_FastWakeUpCmd( DISABLE );
        //PWR_EnterSTOPMode( PWR_Regulator_LowPower, PWR_STOPEntry_WFI );
        configPOST_SLEEP_PROCESSING(xExpectedIdleTime);
        /* Stop SysTick.  Again, the time the SysTick is stopped for is
        accounted for as best it can be, but using the tickless mode will
        inevitably result in some tiny drift of the time maintained by the
        kernel with respect to calendar time. */
        ulSysTickCTRL = portNVIC_SYSTICK_CTRL_REG;
        portNVIC_SYSTICK_CTRL_REG = (ulSysTickCTRL & ~portNVIC_SYSTICK_ENABLE_BIT);

        /* Re-enable interrupts - see comments above __disable_irq() call
        above. */
        __enable_irq();
        if((ulSysTickCTRL & portNVIC_SYSTICK_COUNT_FLAG_BIT) != 0)
        {
            uint32_t ulCalculatedLoadValue;
            /* The tick interrupt has already executed, and the SysTick
            count reloaded with ulReloadValue.  Reset the
            portNVIC_SYSTICK_LOAD_REG with whatever remains of this tick
            period. */
            ulCalculatedLoadValue = (ulTimerCountsForOneTick - 1UL) - (ulReloadValue - portNVIC_SYSTICK_CURRENT_VALUE_REG);
            /* Don't allow a tiny value, or values that have somehow
            underflowed because the post sleep hook did something
            that took too long. */
            if((ulCalculatedLoadValue < ulStoppedTimerCompensation) || (ulCalculatedLoadValue > ulTimerCountsForOneTick))
            {
                ulCalculatedLoadValue = (ulTimerCountsForOneTick - 1UL);
            }
            portNVIC_SYSTICK_LOAD_REG = ulCalculatedLoadValue;
            /* The tick interrupt handler will already have pended the tick
            processing in the kernel.  As the pending tick will be
            processed as soon as this function exits, the tick value
            maintained by the tick is stepped forward by one less than the
            time spent waiting. */
            ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
        }
        else
        {
            /* Something other than the tick interrupt ended the sleep.
            Work out how long the sleep lasted rounded to complete tick
            periods (not the ulReload value which accounted for part
            ticks). */
            ulCompletedSysTickDecrements = (xExpectedIdleTime * ulTimerCountsForOneTick) - portNVIC_SYSTICK_CURRENT_VALUE_REG;
            /* How many complete tick periods passed while the processor
            was waiting? */
            ulCompleteTickPeriods = ulCompletedSysTickDecrements / ulTimerCountsForOneTick;
            /* The reload value is set to whatever fraction of a single tick
            period remains. */
            portNVIC_SYSTICK_LOAD_REG = ((ulCompleteTickPeriods + 1) * ulTimerCountsForOneTick) - ulCompletedSysTickDecrements;
        }

        /* Restart SysTick so it runs from portNVIC_SYSTICK_LOAD_REG
        again, then set portNVIC_SYSTICK_LOAD_REG back to its standard
        value.  The critical section is used to ensure the tick interrupt
        can only execute once in the case that the reload register is near
        zero. */
        portNVIC_SYSTICK_CURRENT_VALUE_REG = 0UL;
        portENTER_CRITICAL();
        {
            portNVIC_SYSTICK_CTRL_REG |= portNVIC_SYSTICK_ENABLE_BIT;
            vTaskStepTick(ulCompleteTickPeriods);
            portNVIC_SYSTICK_LOAD_REG = ulTimerCountsForOneTick - 1UL;
        }
        portEXIT_CRITICAL();
    }
}
#endif /* #if configUSE_TICKLESS_IDLE */

#endif
