#include "gpio-board.h"
#include "board.h"
#include <stddef.h>
#include <stdbool.h>

static GpioIrqHandler *GpioIrq[16];
#define TOGGLEPERIOD 1000

void GpioInit( Gpio_t *obj, PinNames pin, PinModes mode,  PinConfigs config, PinTypes type, uint32_t value )
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if( pin == NC )
    {
        return;
    }
    obj->pin = pin;
    obj->pinIndex = ( 0x01 << ( obj->pin & 0x0F ) );

    if( ( obj->pin & 0xF0 ) == 0x00 )
    {
        obj->port = GPIOA;
        __HAL_RCC_GPIOA_CLK_ENABLE( );
    }
    else if( ( obj->pin & 0xF0 ) == 0x10 )
    {
        obj->port = GPIOB;
        __HAL_RCC_GPIOB_CLK_ENABLE( );
    }
    else if( ( obj->pin & 0xF0 ) == 0x20 )
    {
        obj->port = GPIOC;
        __HAL_RCC_GPIOC_CLK_ENABLE( );
    }
    else if( ( obj->pin & 0xF0 ) == 0x30 )
    {
        obj->port = GPIOD;
        __HAL_RCC_GPIOD_CLK_ENABLE( );
    }
    else
    {
        obj->port = GPIOH;
        __HAL_RCC_GPIOH_CLK_ENABLE( );
    }

    GPIO_InitStructure.Pin =  obj->pinIndex ;
    GPIO_InitStructure.Pull = type;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

    if( mode == PIN_INPUT )
    {
        GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    }
    else if( mode == PIN_ANALOGIC )
    {
        GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    }
    else if( mode == PIN_ALTERNATE_FCT )
    {
        if( config == PIN_OPEN_DRAIN )
        {
            GPIO_InitStructure.Mode = GPIO_MODE_AF_OD;
        }
        else
        {
            GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
        }
        GPIO_InitStructure.Alternate = value;
    }
    else // mode ouptut
    {
        if( config == PIN_OPEN_DRAIN )
        {
            GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
        }
        else
        {
            GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
        }
    }

    HAL_GPIO_Init( obj->port, &GPIO_InitStructure );

    // Sets initial output value
    if( mode == PIN_OUTPUT )
    {
        GpioWrite( obj, value );
    }
}

void GpioSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler )
{
    uint32_t priority = 0;

    IRQn_Type IRQnb = EXTI0_1_IRQn;
    GPIO_InitTypeDef   GPIO_InitStructure;

    if( irqHandler == NULL )
    {
        return;
    }

    GPIO_InitStructure.Pin =  obj->pinIndex;

    if( irqMode == IRQ_RISING_EDGE )
    {
        GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING;
    }
    else if( irqMode == IRQ_FALLING_EDGE )
    {
        GPIO_InitStructure.Mode = GPIO_MODE_IT_FALLING;
    }
    else
    {
        GPIO_InitStructure.Mode = GPIO_MODE_IT_RISING_FALLING;
    }

#ifdef EARTAG_BOARD
    GPIO_InitStructure.Pull = GPIO_PULLDOWN;
#else
		GPIO_InitStructure.Pull = GPIO_NOPULL;
#endif
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init( obj->port, &GPIO_InitStructure );

    switch( irqPriority )
    {
    case IRQ_VERY_LOW_PRIORITY:
    case IRQ_LOW_PRIORITY:
        priority = 3;
        break;
    case IRQ_MEDIUM_PRIORITY:
        priority = 2;
        break;
    case IRQ_HIGH_PRIORITY:
        priority = 1;
        break;
    case IRQ_VERY_HIGH_PRIORITY:
    default:
        priority = 0;
        break;
    }

    switch( obj->pinIndex )
    {
    case GPIO_PIN_0:
    case GPIO_PIN_1:
        IRQnb = EXTI0_1_IRQn;
        break;
    case GPIO_PIN_2:
    case GPIO_PIN_3:
        IRQnb = EXTI2_3_IRQn;
        break;
    case GPIO_PIN_4:
    case GPIO_PIN_5:
    case GPIO_PIN_6:
    case GPIO_PIN_7:
    case GPIO_PIN_8:
    case GPIO_PIN_9:
    case GPIO_PIN_10:
    case GPIO_PIN_11:
    case GPIO_PIN_12:
    case GPIO_PIN_13:
    case GPIO_PIN_14:
    case GPIO_PIN_15:
        IRQnb = EXTI4_15_IRQn;
        break;
    default:
        break;
    }

    GpioIrq[(obj->pin ) & 0x0F] = irqHandler;

    HAL_NVIC_SetPriority( IRQnb , priority, 0 );
    HAL_NVIC_EnableIRQ( IRQnb );

}

void GpioDisableInterrupt( Gpio_t *obj )
{
    GPIO_InitTypeDef   GPIO_InitStructure;
	
    GpioIrq[obj->pin & 0x0F] = NULL;
    
    GPIO_InitStructure.Pin =  obj->pinIndex ;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init( obj->port, &GPIO_InitStructure );
}

void GpioWrite( Gpio_t *obj, uint32_t value )
{
    
    if( ( obj == NULL ) || ( obj->port == NULL ) )
    {
        assert_param( FAIL );
    }
    // Check if pin is not connected
    if( obj->pin == NC )
    {
        return;
    }
    HAL_GPIO_WritePin( obj->port, obj->pinIndex , ( GPIO_PinState )value );
}

int8_t GpioReadInput( Gpio_t *obj )
{
    if( obj == NULL )
    {
        assert_param( FAIL );
    }
    // Check if pin is not connected
    if( obj->pin == NC )
    {
        return 0;
    }
    return HAL_GPIO_ReadPin( obj->port, obj->pinIndex );
}

void CreateToggle(PinNames pin, uint32_t speed)
{
//    Gpio_t ioPin;
//    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//    TIM_OCInitTypeDef  TIM_OCInitStructure;
//    uint16_t PrescalerValue = 0;
//    GpioInit( &ioPin, pin, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_UP, 0 );
//    
//    GPIO_PinAFConfig(ioPin.port, (ioPin.pin & 0x0f), GPIO_AF_TIM10);

//  /* TIM10 clock enable */
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);

//  /* Compute the prescaler value */
//  PrescalerValue = (uint16_t) (SystemCoreClock / (speed*TOGGLEPERIOD)) - 1;
//  /* Time base configuration */
//  TIM_TimeBaseStructure.TIM_Period = TOGGLEPERIOD;
//  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
//  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

//  TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);

//  /* PWM1 Mode configuration: Channel1 */
//  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//  TIM_OCInitStructure.TIM_Pulse = TOGGLEPERIOD/2;
//  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

//  TIM_OC1Init(TIM10, &TIM_OCInitStructure);

//  TIM_OC1PreloadConfig(TIM10, TIM_OCPreload_Enable);

//  TIM_ARRPreloadConfig(TIM10, ENABLE);

//  /* TIM10 enable counter */
//  TIM_Cmd(TIM10, ENABLE);

}

void StopToggle(void)
{
//    TIM_Cmd(TIM10, DISABLE);
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, DISABLE);
}

void EXTI0_1_IRQHandler( void )
{
#ifdef LOW_POWER_MODE
        //RecoverMcuStatus();
     __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
#endif
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_0 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_1 );
}

//void EXTI1_IRQHandler( void )
//{
//#ifdef LOW_POWER_MODE
//        //RecoverMcuStatus();
//     __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
//#endif
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_1 );
//}

void EXTI2_3_IRQHandler( void )
{
#ifdef LOW_POWER_MODE
        //RecoverMcuStatus();
     __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
#endif
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_2 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_3 );
}

//void EXTI3_IRQHandler( void )
//{
//#ifdef LOW_POWER_MODE
//        //RecoverMcuStatus();
//     __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
//#endif
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_3 );
//}

void EXTI4_15_IRQHandler( void )
{
#ifdef LOW_POWER_MODE
        //RecoverMcuStatus();
     __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
#endif
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_4 );
        HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_5 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_6 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_7 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_8 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_9 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_10 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_11 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_12 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_13 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_14 );
    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_15 );
}

//void EXTI9_5_IRQHandler( void )
//{
//#ifdef LOW_POWER_MODE
//        //RecoverMcuStatus();
//     __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
//#endif
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_5 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_6 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_7 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_8 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_9 );
//}

//void EXTI15_10_IRQHandler( void )
//{
//#ifdef LOW_POWER_MODE
//        //RecoverMcuStatus();
//     __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
//#endif
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_10 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_11 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_12 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_13 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_14 );
//    HAL_GPIO_EXTI_IRQHandler( GPIO_PIN_15 );
//}

void HAL_GPIO_EXTI_Callback( uint16_t gpioPin )
{
    uint8_t callbackIndex = 0;

    if( gpioPin > 0 )
    {
        while( gpioPin != 0x01 )
        {
            gpioPin = gpioPin >> 1;
            callbackIndex++;
        }
    }

    if( GpioIrq[callbackIndex] != NULL )
    {
        GpioIrq[callbackIndex]( );
    }
}
