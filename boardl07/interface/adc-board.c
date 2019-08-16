#include "board.h"
#include "adc-board.h"
#include "gpio-board.h"

ADC_HandleTypeDef Adc;

void AdcInit(void)
{
    Gpio_t ioPin;

    //Adc.Instance = ( ADC_TypeDef *)ADC1_BASE;

    GpioInit( &ioPin, BAT_LEVEL, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    AdcFormat(ADC_12_BIT, SINGLE_CONVERSION, CONVERT_MANUAL_TRIG, DATA_RIGHT_ALIGNED );
}

void AdcFormat( AdcResolution AdcRes, AdcNumConversion AdcNumConv, AdcTriggerConv AdcTrig, AdcDataAlignement AdcDataAlig )
{
//    if( AdcRes == ADC_12_BIT )
//    {
//        Adc.Init.Resolution = ADC_RESOLUTION_12B;
//    }
//    else if( AdcRes == ADC_10_BIT )
//    {
//        Adc.Init.Resolution = ADC_RESOLUTION_10B;
//    }
//    else if( AdcRes == ADC_8_BIT )
//    {
//        Adc.Init.Resolution = ADC_RESOLUTION_8B;
//    }
//    else if( AdcRes == ADC_6_BIT )
//    {
//        Adc.Init.Resolution = ADC_RESOLUTION_6B;
//    }

//    if( AdcNumConv == SINGLE_CONVERSION )
//    {
//        Adc.Init.ContinuousConvMode = DISABLE;
//    }
//    else
//    {
//        Adc.Init.ContinuousConvMode = ENABLE;
//    }

//    if( AdcTrig == CONVERT_MANUAL_TRIG )
//    {
//        Adc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONVEDGE_NONE;
//    }
//    else if( AdcTrig == CONVERT_RISING_EDGE )
//    {
//        Adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
//    }
//    else if( AdcTrig == CONVERT_FALLING_EDGE )
//    {
//        Adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_FALLING;
//    }
//    else
//    {
//        Adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
//    }

//    if( AdcDataAlig == DATA_RIGHT_ALIGNED )
//    {
//        Adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//    }
//    else
//    {
//        Adc.Init.DataAlign = ADC_DATAALIGN_LEFT;
//    }

    Adc.Instance = ( ADC_TypeDef *)ADC1_BASE;
    // Adc.Init.NbrOfConversion = 1;
    Adc.Init.OversamplingMode      = DISABLE;

    Adc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1;
    Adc.Init.LowPowerAutoPowerOff  = DISABLE;
    Adc.Init.LowPowerFrequencyMode = ENABLE;
    Adc.Init.LowPowerAutoWait      = DISABLE;

    Adc.Init.Resolution            = ADC_RESOLUTION_12B;
#ifdef EARTAG_BOARD
    Adc.Init.SamplingTime          = ADC_SAMPLETIME_239CYCLES_5;
#else
    Adc.Init.SamplingTime          = ADC_SAMPLETIME_7CYCLES_5;
#endif
    Adc.Init.ScanConvMode          = ADC_SCAN_DIRECTION_FORWARD;
    Adc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    Adc.Init.ContinuousConvMode    = DISABLE;
    Adc.Init.DiscontinuousConvMode = DISABLE;
    Adc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    Adc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    Adc.Init.DMAContinuousRequests = DISABLE;
    HAL_ADC_Init( &Adc );

}

uint16_t AdcRead(uint32_t channel )
{
    ADC_HandleTypeDef *hadc;
    ADC_ChannelConfTypeDef adcConf;
    uint16_t adcData = 0;

    hadc = &Adc;

        /* Enable HSI */
    __HAL_RCC_HSI_ENABLE();

    /* Wait till HSI is ready */
    while(__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY) == RESET)
    {
    }

    __HAL_RCC_ADC1_CLK_ENABLE( );
    /* ### Start calibration ###*/
    HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);

		/* Clean CHSEL flag old value */
    Clean_CHSEL_Flags(hadc);

    adcConf.Channel = channel;
    adcConf.Rank = ADC_RANK_CHANNEL_NUMBER;
    HAL_ADC_ConfigChannel( hadc, &adcConf);

    /* Enable ADC1 */
    //__HAL_ADC_ENABLE( hadc) ;

    /* Start ADC1 Software Conversion */
    HAL_ADC_Start( hadc);

    HAL_ADC_PollForConversion( hadc, 10/*HAL_MAX_DELAY*/ );
    if ((HAL_ADC_GetState(hadc) & HAL_ADC_STATE_REG_EOC) == HAL_ADC_STATE_REG_EOC)
    {
        adcData = HAL_ADC_GetValue ( hadc);
		}
    //__HAL_ADC_DISABLE( hadc) ;

    if( ( adcConf.Channel == ADC_CHANNEL_TEMPSENSOR ) || ( adcConf.Channel == ADC_CHANNEL_VREFINT ) )
    {
        HAL_ADC_DeInit( hadc );

    }else
		    HAL_ADC_Stop(hadc);
    __HAL_RCC_ADC1_CLK_DISABLE( );

    /* Disable HSI */
    __HAL_RCC_HSI_DISABLE();

    return adcData;
}

void Clean_CHSEL_Flags(ADC_HandleTypeDef* hadc)
{
    ADC_ChannelConfTypeDef adcConf;
#ifdef STM32L071xx     
    adcConf.Channel = ADC_CHANNEL_0|ADC_CHANNEL_1|ADC_CHANNEL_2|ADC_CHANNEL_3|ADC_CHANNEL_4|ADC_CHANNEL_5
                    |ADC_CHANNEL_6|ADC_CHANNEL_7|ADC_CHANNEL_8|ADC_CHANNEL_9|ADC_CHANNEL_10|ADC_CHANNEL_11
                    |ADC_CHANNEL_12|ADC_CHANNEL_13|ADC_CHANNEL_14|ADC_CHANNEL_15|ADC_CHANNEL_17|ADC_CHANNEL_18;
#else
    adcConf.Channel = ADC_CHANNEL_0|ADC_CHANNEL_1|ADC_CHANNEL_2|ADC_CHANNEL_3|ADC_CHANNEL_4|ADC_CHANNEL_5
                    |ADC_CHANNEL_6|ADC_CHANNEL_7|ADC_CHANNEL_8|ADC_CHANNEL_9|ADC_CHANNEL_10|ADC_CHANNEL_11
                    |ADC_CHANNEL_12|ADC_CHANNEL_13|ADC_CHANNEL_14|ADC_CHANNEL_15|ADC_CHANNEL_16|ADC_CHANNEL_17|ADC_CHANNEL_18;
#endif
    adcConf.Rank = ADC_RANK_NONE;
    HAL_ADC_ConfigChannel( hadc, &adcConf);
}
