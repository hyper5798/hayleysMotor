#ifndef __ADC_BOARD_H__
#define __ADC_BOARD_H__

#ifdef __cplusplus
 extern "C" {
#endif 
#include "stm32l0xx.h"
/*!
 * Register the old AdcMcuReadChannel function to the new function 
 * which makes an additional parameter available to select the read out channel
 */
#define AdcMcuReadChannel( obj ) AdcMcuRead( obj, ADC_Channel_8 )

/*!
 * ADC maximum value
 */
#define ADC_MAX_VALUE  4096

/*!
 * ADC resolution
 */
typedef enum
{
    ADC_12_BIT = 0,
    ADC_10_BIT,
    ADC_8_BIT,
    ADC_6_BIT
}AdcResolution;

/*!
 * ADC conversion trigger
 */
typedef enum
{
    CONVERT_MANUAL_TRIG = 0,
    CONVERT_RISING_EDGE,
    CONVERT_FALLING_EDGE,
    CONVERT_RISING_FALLING_EDGE
}AdcTriggerConv;

/*!
 * ADC data alignment 
 */
typedef enum
{
    DATA_RIGHT_ALIGNED = 0,
    DATA_LEFT_ALIGNED
}AdcDataAlignement;


/*!
 * ADC conversion mode
 */
typedef enum
{
    SINGLE_CONVERSION = 0,
    CONTIMUOUS_CONVERSION
}AdcNumConversion;

/*!
 * \brief Initializes the ADC object and MCU peripheral
 *
 * \param [IN] obj  ADC object
 * \param [IN] scl  ADC input pin
 */
void AdcInit(void);

///*!
// * \brief DeInitializes the ADC object and MCU peripheral
// *
// * \param [IN] obj  ADC object
// */
//void AdcDeInit( Adc_t *obj );

/*!
 * \brief Initializes the ADC internal parameters
 *
 * \param [IN] obj          ADC object
 * \param [IN] AdcRes       ADC resolution 
 * \param [IN] AdcNumConv   ADC number of conversion
 * \param [IN] AdcTrig      ADC conversion trigger
 * \param [IN] AdcDataAlig  ADC data output alignement
 */
void AdcFormat(AdcResolution AdcRes, AdcNumConversion AdcNumConv, AdcTriggerConv AdcTrig, AdcDataAlignement AdcDataAlig );

uint16_t AdcRead(uint32_t channel );
void Clean_CHSEL_Flags(ADC_HandleTypeDef* hadc);
#ifdef __cplusplus
}
#endif
#endif /*__ADC_BOARD_H__*/
