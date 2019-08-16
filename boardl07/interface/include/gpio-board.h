#ifndef __GPIO_BOARD_H__
#define __GPIO_BOARD_H__

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32l0xx.h"
//#include "stm32l1xx_hal.h"
#include "mcu_pin_number.h"

#define RADIO_RESET                                 PB_10
    
//#define RADIO_DIO_0                                 PB_11
//#define RADIO_DIO_1                                 PC_13
//#define RADIO_DIO_2                                 PB_9
//#define RADIO_DIO_3                                 PB_4
//#define RADIO_DIO_4                                 PB_3
//#define RADIO_DIO_5                                 PA_15
/*!
 * Board GPIO pin names
 */
typedef enum 
{
    MCU_PINS,
   // IOE_PINS,
    
    // Not connected
    NC = (int)0xFFFFFFFF
}PinNames;

/*!
 * Operation Mode for the GPIO
 */
typedef enum
{
    PIN_INPUT = 0,
    PIN_OUTPUT,
    PIN_ALTERNATE_FCT,
    PIN_ANALOGIC
}PinModes;

//hanson mod
/*!
 * Register's mode for different GPIO pin
 */
typedef enum
{
		REG_NULL = 0,
		REG_INPUT,
		REG_OUTPUT
}PinRegModes;

/*!
 * Add a pull-up, a pull-down or nothing on the GPIO line
 */
typedef enum
{
    PIN_NO_PULL = 0,
    PIN_PULL_UP,
    PIN_PULL_DOWN
}PinTypes;

/*!
 * Define the GPIO as Push-pull type or Open Drain
 */
typedef enum
{
    PIN_PUSH_PULL = 0,
    PIN_OPEN_DRAIN
}PinConfigs;

/*!
 * Define the GPIO IRQ on a rising, falling or both edges
 */
typedef enum
{
    NO_IRQ = 0,
    IRQ_RISING_EDGE,
    IRQ_FALLING_EDGE,  
    IRQ_RISING_FALLING_EDGE,
		IRQ_NO_SETTINGS
}IrqModes;

/*!
 * Define the IRQ priority on the GPIO
 */
typedef enum
{
    IRQ_VERY_LOW_PRIORITY = 0,
    IRQ_LOW_PRIORITY,
    IRQ_MEDIUM_PRIORITY,  
    IRQ_HIGH_PRIORITY,
    IRQ_VERY_HIGH_PRIORITY,
		//hanson mod, for IOE's settings
		IRQ_NO_PRIORITY
}IrqPriorities;

typedef enum
{
    IRQ_ACTIVE_HIGH = 0,
		IRQ_ACTIVE_LOW
}IrqActiveState;

/*!
 * Structure for the GPIO
 */
typedef struct 
{
    PinNames  pin;
    uint16_t pinIndex;
    void *port;
    uint16_t portIndex;
    //hanson mod
		IrqModes IrqSignalEdge;
    IrqActiveState IrqActiveState;
}Gpio_t;


typedef void( GpioIrqHandler )( void );

void GpioInit( Gpio_t *obj, PinNames pin, PinModes mode,  PinConfigs config, PinTypes type, uint32_t value );
void GpioSetInterrupt( Gpio_t *obj, IrqModes irqMode, IrqPriorities irqPriority, GpioIrqHandler *irqHandler );
void GpioDisableInterrupt( Gpio_t *obj );
void GpioWrite( Gpio_t *obj, uint32_t value );
int8_t GpioReadInput( Gpio_t *obj );
void CreateToggle(PinNames pin, uint32_t speed);
void StopToggle(void);

#ifdef __cplusplus
}
#endif
#endif /*__GPIO_BOARD_H__*/
