#include "board.h"
#include "spi-board.h"
#include "i2c-board.h"
#include "hwtimer-board.h"
#include "rtc-board.h"
#include "adc-board.h"
#include "gpio-board.h"
#include "stm32l0xx_hal.h"
#include "hwtimer-board.h"

#if defined (MANHOLE_BOARD) || defined (EARTAG_BOARD)
#include <math.h>
#endif

#ifndef NULL
    #define NULL                                    ( ( void * )0 )
#endif

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

#if defined( LOW_POWER_MODE )

void BoardUnusedIoInit( void )
{
    Gpio_t ioPin;
#ifdef EARTAG_BOARD

//    GpioInit( &ioPin, BAT_LEVEL, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UART_TX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//
//    GpioInit( &ioPin, SWDIO, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, SWCLK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//
//    GpioInit( &ioPin, RF_ONOFF, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );


//    GpioInit( &ioPin, LED_FCT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UART_TX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UART_RX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, LED_TX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

//    GpioInit( &ioPin, I2C_SCL, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, I2C_SDA, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

//    GpioInit( &ioPin, SPI1_NSS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, SPI1_SCK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, SPI1_MISO, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, SPI1_MOSI, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

//    GpioInit( &ioPin, INT_2, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, INT_1, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, UART1_TX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UART1_RX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UART1_CTS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UART1_RTS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UART1_CK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN1, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN2, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN3, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN4, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN5, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN6, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN7, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN8, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN9, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN10, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN11, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN12, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN13, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );



/*
//    GpioInit( &ioPin, UNUSED_PIN_A0, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_A1, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A2, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A3, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A4, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A5, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A6, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A7, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A8, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A9, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A10, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A11, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A12, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A13, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_A14, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_A15, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, UNUSED_PIN_B0, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_B1, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_B2, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_B3, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_B4, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_B5, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_B6, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_B7, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UNUSED_PIN_B8, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_B9, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_B10, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_B11, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_B12, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_B13, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_B14, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_B15, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

//    GpioInit( &ioPin, UNUSED_PIN_C13, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_C14, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_C15, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//
//    GpioInit( &ioPin, UNUSED_PIN_H0, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UNUSED_PIN_H1, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
*/
#endif

#ifdef MANHOLE_BOARD
    GpioInit( &ioPin, SPI1_NSS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, SPI1_SCK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, SPI1_MISO, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, SPI1_MOSI, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

//    GpioInit( &ioPin, UART1_TX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
//    GpioInit( &ioPin, UART1_RX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UART1_CTS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UART1_RTS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, UART_TX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UART_RX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );


    GpioInit( &ioPin, LED_TX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

#endif

#ifdef SIPMODULE_BOARD_LOWPOWER
    GpioInit( &ioPin, LED_FCT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UART_TX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UART_RX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, LED_TX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, I2C_SCL, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, I2C_SDA, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, SPI1_NSS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, SPI1_SCK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, SPI1_MISO, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, SPI1_MOSI, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, INT_2, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, INT_1, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, UART1_TX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UART1_RX, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UART1_CTS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UART1_RTS, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, UART1_CK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, NO_USE_PD00, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD01, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD02, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD03, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD04, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD05, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD06, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD07, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD08, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD09, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD10, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD11, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD12, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD13, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD14, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD15, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif

#if defined( USE_DEBUGGER )
    HAL_DBGMCU_EnableDBGStopMode( );
    HAL_DBGMCU_EnableDBGSleepMode( );
    HAL_DBGMCU_EnableDBGStandbyMode( );
#else
    HAL_DBGMCU_DisableDBGSleepMode( );
    HAL_DBGMCU_DisableDBGStopMode( );
    HAL_DBGMCU_DisableDBGStandbyMode( );

    GpioInit( &ioPin, SWDIO, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, SWCLK, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif

    GpioInit( &ioPin, BOOT_1, PIN_ANALOGIC, PIN_OPEN_DRAIN, PIN_NO_PULL, 0 );

#ifdef SIPMODULE_BOARD
    GpioInit( &ioPin, NO_USE_PC00, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PC01, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PC02, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PC03, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PC04, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PC05, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PC06, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PC07, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#ifndef ANT_PA
    GpioInit( &ioPin, NO_USE_PC08, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
    GpioInit( &ioPin, NO_USE_PC09, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PC10, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PC11, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PC12, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, NO_USE_PD00, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD01, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD02, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD03, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD04, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD05, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD06, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD07, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD08, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD09, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD10, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD11, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD12, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD13, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD14, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PD15, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, NO_USE_PE00, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE01, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE02, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE03, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE04, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE05, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE06, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE07, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE08, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE09, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE10, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE11, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE12, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE13, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE14, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PE15, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, NO_USE_PH09, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, NO_USE_PH10, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif
}


#endif

bool SystemClockConfig_STOP(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Get the Oscillators configuration according to the internal RCC registers */
  HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

  /* After wake-up from STOP reconfigure the system clock: Enable HSI and PLL */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState            = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
#ifdef EARTAG_BOARD
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_8;
#else
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_4;
#endif
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLLDIV_3;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
      return FAIL;
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
    RCC_ClkInitStruct.ClockType = ( RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 );
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
      return FAIL;
  }

  return OK;
}

void SystemClockReConfig( void )
{
    __HAL_RCC_PWR_CLK_ENABLE( );
    __HAL_PWR_VOLTAGESCALING_CONFIG( PWR_REGULATOR_VOLTAGE_SCALE1 );

    /* Enable HSE */
    __HAL_RCC_HSE_CONFIG( RCC_HSE_ON );

    /* Wait till HSE is ready */
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_HSERDY ) == RESET )
    {
    }

    /* Enable PLL */
    __HAL_RCC_PLL_ENABLE( );

    /* Wait till PLL is ready */
    while( __HAL_RCC_GET_FLAG( RCC_FLAG_PLLRDY ) == RESET )
    {
    }

    /* Select PLL as system clock source */
    __HAL_RCC_SYSCLK_CONFIG ( RCC_SYSCLKSOURCE_PLLCLK );

    /* Wait till PLL is used as system clock source */
    while( __HAL_RCC_GET_SYSCLK_SOURCE( ) != RCC_SYSCLKSOURCE_STATUS_PLLCLK )
    {
    }
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 32000000
  *            HCLK(Hz)                       = 32000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 12000000
  *            HSI Frequency(Hz)              = 0
  *            PLLMUL                         = 8
  *            PLLDIV                         = 3
  *            Flash Latency(WS)              = 1
  * @retval None
  */

bool SystemClockConfig( void )
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState            = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
#ifdef EARTAG_BOARD
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_8;
#else
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLLMUL_4;
#endif
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLLDIV_3;
    /* Enable MSI Oscillator */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
//  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
//  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
//  RCC_OscInitStruct.MSICalibrationValue=0x00;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    return FAIL;
  }

//  /* Set Voltage scale1 as MCU will run at 32MHz */
//  __HAL_RCC_PWR_CLK_ENABLE();
//  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  //while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  //RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  //if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    /* Initialization Error */
    return FAIL;
  }

    /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  //__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /* Disable Power Control clock */
  __HAL_RCC_PWR_CLK_DISABLE();
  return OK;
}

void BoardInitPeriph( void )
{
    //NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
    if(McuInitialized == false)
    {
        HAL_Init( );
        SystemClockConfig();
        McuInitialized = true;
    }
    else
    {
        SystemClockConfig_STOP();
    }
//    GPIO_ConfigAN();
#if defined (MODBUS_BOARD) || defined (NODE_BOARD)
    SpiInit(SPIInterface1);
#elif defined (TRACKER_BOARD) || defined (SIPMODULE_BOARD) || defined (MANHOLE_BOARD) || defined (EARTAG_BOARD)
    SpiInit(SPIInterface2);
#endif

#if defined (TRACKER_BOARD) || defined (MANHOLE_BOARD) || defined (NODE_BOARD) || defined (RPMA_ST_NODE)
    I2C1Init();
#endif

#if defined (TRACKER_BOARD) || defined (SIPMODULE_BOARD) || defined (MANHOLE_BOARD) || defined (NODE_BOARD) || defined (EARTAG_BOARD) || \
        defined (RPMA_ST_NODE)
    RtcInit();
#endif

#ifdef LOW_POWER_MODE
    BoardUnusedIoInit( );
#endif

#ifdef LORAWANV1_0
    HwTimerResume();
#endif

    //AdcInit();
}

void BoardDeInitPeriph( void )
{
#if defined (MANHOLE_BOARD) || defined (SIPMODULE_BOARD_LOWPOWER) || defined (TRACKER_BOARD) || defined (TRACKER_BOARD_V1) || \
        defined (SIPMODULE_BOARD) || defined (EARTAG_BOARD) || defined (RPMA_ST_NODE)
    Gpio_t ioPin;

    GpioInit( &ioPin, OSC_HSE_IN, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, OSC_HSE_OUT, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    GpioInit( &ioPin, OSC_LSE_IN, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, OSC_LSE_OUT, PIN_INPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

//    GpioInit( &ioPin, USB_ON, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
#endif

    SpiDeInit();

#if defined (MANHOLE_BOARD) || defined (RPMA_ST_NODE)
    I2C1DeInit();
#endif

#ifdef LORAWANV1_0
    HwTimerBackupTickCounter();
#endif
}



void RecoverMcuStatus( void )
{
    // Disable IRQ while the MCU is not running on HSE
    __disable_irq( );

//    SystemClockReConfig();

//    /* After wake-up from STOP reconfigure the system clock */
//    /* Enable HSE */
//    RCC_HSEConfig( RCC_HSE_ON );
//
//    /* Wait till HSE is ready */
//    while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET )
//    {}
//
//    /* Enable PLL */
//    RCC_PLLCmd( ENABLE );
//
//    /* Wait till PLL is ready */
//    while( RCC_GetFlagStatus( RCC_FLAG_PLLRDY ) == RESET )
//    {}
//
//    /* Select PLL as system clock source */
//    RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );
//
//    /* Wait till PLL is used as system clock source */
//    while( RCC_GetSYSCLKSource( ) != 0x0C )
//    {}

//    /* Set MCU in ULP (Ultra Low Power) */
//    PWR_UltraLowPowerCmd( DISABLE ); // add up to 3ms wakeup time
//
//    /* Enable the Power Voltage Detector */
//    PWR_PVDCmd( ENABLE );

    BoardInitPeriph( );

    __enable_irq( );
}

/*void BoardGetUniqueId( uint8_t *id )
{
    id[0] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[1] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[2] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[3] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[4] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[5] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[6] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[7] = ( ( *( uint32_t* )ID2 ) );
}*/

/*!
 * Factory power supply
 */
#ifdef TRACKER_BOARD
#define FACTORY_POWER_SUPPLY                        3.9L
#else
#define FACTORY_POWER_SUPPLY                        3.0L
#endif
/*!
 * VREF calibration value
 */
#define VREFINT_CAL                                 ( *( uint16_t* )0x1FF80078 )

/*Temperature calibration*/

#define FACTORY_TSCALIB1                            ( *( uint16_t* )0x1FF8007A )
#define FACTORY_TSCALIB2                            ( *( uint16_t* )0x1FF8007E )
#define HOT_CAL_TEMP 		110
#define COLD_CAL_TEMP  	30

/*!
 * Battery thresholds
 */
#ifdef TRACKER_BOARD
#define BATTERY_MAX_LEVEL                           4050 // mV
#define BATTERY_MIN_LEVEL                           3650 // mV
#define BATTERY_SHUTDOWN_LEVEL                      3550 // mV
#elif defined EARTAG_BOARD
#define BATTERY_MAX_LEVEL                           3700 // mV
#define BATTERY_MIN_LEVEL                           3300 // mV
#define BATTERY_SHUTDOWN_LEVEL                      3200 // mV
#else
#define BATTERY_MAX_LEVEL                           4150 // mV
#define BATTERY_MIN_LEVEL                           3200 // mV
#define BATTERY_SHUTDOWN_LEVEL                      3100 // mV
#endif
#ifdef TRACKER_BOARD
uint16_t batteryCal=1202;
void BoardSetBatteryCal(uint16_t value)
{
    batteryCal = value;
}
uint16_t BoardGetPowerSupply( void )
{
//    uint16_t vref_test = 0;
//    uint16_t vdiv_test = 0;
//    uint16_t vsen_test = 0;
    float vref = 0;
    float vdiv = 0;

    float batteryVoltage = 0;

    AdcInit();

    vref = AdcRead(ADC_Channel_17 );
    vdiv = AdcRead(ADC_Channel_8 );

//    vref_test = vref;
//    vdiv_test = vdiv;
//    vsen_test = AdcRead(ADC_Channel_TempSensor );
//#ifdef TRACKER_BOARD
    batteryVoltage = ( FACTORY_POWER_SUPPLY * batteryCal * vdiv ) / ( vref * ADC_MAX_VALUE );
//#else
//    batteryVoltage = ( FACTORY_POWER_SUPPLY * VREFINT_CAL * vdiv ) / ( vref * ADC_MAX_VALUE );
//#endif

    //                                vDiv
    // Divider bridge  VBAT <-> 1M -<--|-->- 1M <-> GND => vBat = 2 * vDiv
    batteryVoltage = 2 * batteryVoltage;

    return ( uint16_t )( batteryVoltage * 1000 );
}


uint8_t BoardMeasureBatterieLevel( void )
{
    uint16_t batteryVoltage = 0;
    __IO uint8_t batteryLevel = 0;

    batteryVoltage = BoardGetPowerSupply( );

    if( batteryVoltage >= BATTERY_MAX_LEVEL )
    {
        batteryLevel = 254;
    }
    else if( ( batteryVoltage > BATTERY_MIN_LEVEL ) && ( batteryVoltage < BATTERY_MAX_LEVEL ) )
    {
        batteryLevel = ( ( 253 * ( batteryVoltage - BATTERY_MIN_LEVEL ) ) / ( BATTERY_MAX_LEVEL - BATTERY_MIN_LEVEL ) ) + 1;
    }
    else if( batteryVoltage <= BATTERY_SHUTDOWN_LEVEL )
    {
        batteryLevel = 255;
            //GpioInit( &DcDcEnable, DC_DC_EN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
            //GpioInit( &BoardPowerDown, BOARD_POWER_DOWN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    }
    else // BATTERY_MIN_LEVEL
    {
        batteryLevel = 1;
    }

    return batteryLevel;
}

int16_t BoardGetTemperature( void )
{
    int16_t temperature_C;
    uint16_t temp;
//    uint16_t vref;
//    uint32_t vdd_ref;
//    vref = AdcRead(ADC_Channel_17 );
    temp = AdcRead(ADC_Channel_16 );

//    vdd_ref = VREFINT_CAL * 3000 / vref;
//	/* correction factor if VDD <> 3V */
//	temp = temp * vdd_ref / 3000;

    temperature_C = ( (int16_t) temp - (int16_t) FACTORY_TSCALIB1 ) ;
    temperature_C = temperature_C * (int16_t)(HOT_CAL_TEMP - COLD_CAL_TEMP);
    temperature_C = temperature_C / (int16_t)(FACTORY_TSCALIB2 - FACTORY_TSCALIB1);
    temperature_C = temperature_C + COLD_CAL_TEMP;

    return temperature_C;
}

float GetCalTemperature(float temp)
{
    return (temp - TEMPCALMIN) * (TEMPREALMAX - TEMPREALMIN) /(TEMPCALMAX - TEMPCALMIN);
}
#endif

#ifdef EARTAG_BOARD
float EarTag_V_bat;
#define ManHole_V_LDO 2.43

uint8_t BoardGetPowerSupply( void )
{
    uint16_t vdiv = 0, vref = 0;
    uint16_t vdiv1 = 0;
    uint16_t vdiv2 = 0;
    uint16_t vdiv3 = 0;
    uint16_t vdiv4 = 0;
    uint16_t vdiv5 = 0;
    AdcInit();

    vdiv1 = AdcRead(ADC_CHANNEL_8);
    vdiv2 = AdcRead(ADC_CHANNEL_8);
    vdiv3 = AdcRead(ADC_CHANNEL_8);
    vdiv4 = AdcRead(ADC_CHANNEL_8);
    vdiv5 = AdcRead(ADC_CHANNEL_8);
    vdiv = (vdiv1+vdiv2+vdiv3+vdiv4+vdiv5)/5;
    vref = AdcRead(ADC_CHANNEL_17);
    EarTag_V_bat = 2.07*1.224*vdiv/vref;

    return EarTag_V_bat*10;
}

float EarTagGetTemperature( void )
{
    float M_value;
    uint16_t vdiv = 0;
    float vdivFloat = 0;
    float divlog;
    float para;
    float finaltemp;
    float vLDO;

    if (EarTag_V_bat <= ManHole_V_LDO) {
        vLDO = EarTag_V_bat;
    } else {
        vLDO = ManHole_V_LDO;
    }

    M_value = (4096*EarTag_V_bat)/vLDO;
    AdcInit();
    vdiv = AdcRead(ADC_CHANNEL_2 );
    vdivFloat = (float)(M_value - vdiv)/(float)vdiv;
    divlog = log(vdivFloat);
    para = (float)divlog/(float)4250 + (float)1/(float)298 - 0.0005417;
    finaltemp = (float)1/(float)para;
    finaltemp = finaltemp - 273;

    return finaltemp;
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  __HAL_RCC_ADC1_CLK_ENABLE();

}

void Delay_time(uint32_t value){
    HAL_Delay(value);
}

#ifdef LORAWANV1_0
/*!
 * Measure the board's battery level for LoRaWan callback function
 * Author: Griffith
 * \param [return]              battery level from BATTERY_MIN_LEVEL(1) to BATTERY_MAX_LEVEL(254)
 */
uint8_t BoardMeasureBatterieLevel( void )
{
// need to check the battery level is reasonable
    uint16_t batteryVoltage = 0;
    __IO uint8_t batteryLevel = 0;

    batteryVoltage = BoardGetPowerSupply( )*100;

    if( batteryVoltage >= BATTERY_MAX_LEVEL )
    {
        batteryLevel = 254;
    }
    else if( ( batteryVoltage > BATTERY_MIN_LEVEL ) && ( batteryVoltage < BATTERY_MAX_LEVEL ) )
    {
        batteryLevel = ( ( 253 * ( batteryVoltage - BATTERY_MIN_LEVEL ) ) / ( BATTERY_MAX_LEVEL - BATTERY_MIN_LEVEL ) ) + 1;
    }
    else if( batteryVoltage <= BATTERY_SHUTDOWN_LEVEL )
    {
        batteryLevel = 255;
    }
    else // BATTERY_MIN_LEVEL
    {
        batteryLevel = 1;
    }

    return batteryLevel;
}
#endif //LORAWANV1_0

#endif

#ifdef MANHOLE_BOARD
float batteryCal=0.15;
void BoardSetBatteryCal(float value)
{
    batteryCal = value;
}
uint16_t BoardGetPowerSupply( void )
{
    uint16_t vdiv = 0;
    float V_adc;
    float V_bat;

    AdcInit();

    vdiv = AdcRead(ADC_Channel_8 );

    V_adc = vdiv * 2.4 / ADC_MAX_VALUE;

    V_bat = (V_adc*207/100) + batteryCal;

    return V_bat*100;
}
#endif

#ifdef SIPMODULE_BOARD

uint16_t batteryCal=1202;   // Value TBD
#ifdef ANT_PA
uint8_t PA_enable = 0;
#endif
//void BoardSetBatteryCal(uint16_t value)
//{
//    batteryCal = value;
//}
uint16_t BoardGetPowerSupply( void )
{
    float vref = 0;
    float vdiv = 0;

    float batteryVoltage = 0;

    AdcInit();

    vref = AdcRead(ADC_CHANNEL_VREFINT);
    vdiv = AdcRead(ADC_CHANNEL_8 );

    batteryVoltage = ( FACTORY_POWER_SUPPLY * batteryCal * vdiv ) / ( vref * ADC_MAX_VALUE );

    //                                vDiv
    // Divider bridge  VBAT <-> 1M -<--|-->- 1M <-> GND => vBat = 2 * vDiv
    batteryVoltage = 2 * batteryVoltage;

    return ( uint16_t )( batteryVoltage * 1000 );
}

uint8_t BoardMeasureBatterieLevel( void )
{
    uint16_t batteryVoltage = 0;
    __IO uint8_t batteryLevel = 0;

    batteryVoltage = BoardGetPowerSupply( );

    if( batteryVoltage >= BATTERY_MAX_LEVEL )
    {
        batteryLevel = 254;
    }
    else if( ( batteryVoltage > BATTERY_MIN_LEVEL ) && ( batteryVoltage < BATTERY_MAX_LEVEL ) )
    {
        batteryLevel = ( ( 253 * ( batteryVoltage - BATTERY_MIN_LEVEL ) ) / ( BATTERY_MAX_LEVEL - BATTERY_MIN_LEVEL ) ) + 1;
    }
    else if( batteryVoltage <= BATTERY_SHUTDOWN_LEVEL )
    {
        batteryLevel = 255;
            //GpioInit( &DcDcEnable, DC_DC_EN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
            //GpioInit( &BoardPowerDown, BOARD_POWER_DOWN, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    }
    else // BATTERY_MIN_LEVEL
    {
        batteryLevel = 1;
    }

    return batteryLevel;
}

#ifdef ANT_PA
uint8_t BoardGetPAEnable( void )
{
    return PA_enable;
}

void BoardSetPAEnable( uint8_t value )
{
	  PA_enable = value;
}
#endif
#endif

#ifdef LORAWANV1_0
uint32_t BoardGetRandomSeed( void )
{
    return ( ( *( uint32_t* )ID1 ) ^ ( *( uint32_t* )ID2 ) ^ ( *( uint32_t* )ID3 ) );
}

void BoardGetUniqueId( uint8_t *id )
{
    id[7] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 24;
    id[6] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 16;
    id[5] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) ) >> 8;
    id[4] = ( ( *( uint32_t* )ID1 )+ ( *( uint32_t* )ID3 ) );
    id[3] = ( ( *( uint32_t* )ID2 ) ) >> 24;
    id[2] = ( ( *( uint32_t* )ID2 ) ) >> 16;
    id[1] = ( ( *( uint32_t* )ID2 ) ) >> 8;
    id[0] = ( ( *( uint32_t* )ID2 ) );
}
#endif


void BoardGetEEPROMKey( uint8_t *id )
{
    // ID1: 0~31
    // ID2: 32~63
    // ID3: 64~95
    id[0] = ( *( uint32_t* )ID1 )  >> 24;// 31~16
    id[1] = ( *( uint32_t* )ID1 )  >> 16;
    id[2] = ( *( uint32_t* )ID2 )  >> 16;//48~63
    id[3] = ( *( uint32_t* )ID2 )  >> 24;
    id[4] = ( *( uint32_t* )ID3 )  ;// 64~79
    id[5] = ( *( uint32_t* )ID3 )  >>8;
    id[6] = ( *( uint32_t* )ID3 )  >>16;//80~95
    id[7] = ( *( uint32_t* )ID3 )  >>24;
    id[8] = ( *( uint32_t* )ID3 )  >>8;// 79~64
    id[9] = ( *( uint32_t* )ID3 )  ;
    id[10] = ( *( uint32_t* )ID1 ) ;// 0~15
    id[11] = ( *( uint32_t* )ID1 ) >>8;
    id[12] = ( *( uint32_t* )ID1 ) >>16;// 16~31
    id[13] = ( *( uint32_t* )ID1 ) >>24;
    id[14] = ( *( uint32_t* )ID2 ) ;// 32~47
    id[15] = ( *( uint32_t* )ID2 ) >>8;
}
