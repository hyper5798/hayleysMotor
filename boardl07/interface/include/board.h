#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
 extern "C" {
#endif 
#include <stdint.h>

#if defined (MODBUS_BOARD) || defined (NODE_BOARD)
#include "pca9539.h"
#elif defined (TRACKER_BOARD)
#include "stmpe1801.h"
#endif

#ifndef OK
#define OK                                      1
#endif

#ifndef FAIL
#define FAIL                                    0
#endif
/*!
 * Unique Devices IDs register set ( STM32L1xxx )
 */
#define         ID1                                 ( 0x1FF80050 )
#define         ID2                                 ( 0x1FF80054 )
#define         ID3                                 ( 0x1FF80064 )

     /*!
 * Random seed generated using the MCU Unique ID
 */
#define RAND_SEED                                   ( ( *( uint32_t* )ID1 ) ^ \
                                                      ( *( uint32_t* )ID2 ) ^ \
                                                      ( *( uint32_t* )ID3 ) )
     
#ifdef MODBUS_BOARD

#define IRQ_BQ24250                       ISG0
#define IRQ_TMP112                        ISG1
#define GPS_POWER_ON                      ISG2
#define GPS_TIMEPULSE                     ISG3
#define GPS_INT                           ISG4
#define IRQ_1_MMA8451                     ISG5
#define IRQ_2_MMA8451                     ISG6
#define LED4                              ISG15
#define LED3                              ISG14
#define LED2                              ISG13
#define LED1                              ISG12
#define DA14580_INT                       ISG14
#define WKUP1                             PA_8

#define RADIO_RESET                       PB_10
#define SPI_MOSI                          PA_7
#define SPI_MISO                          PA_6
#define SPI_SCLK                          PA_5
#define SPI_CS                            PA_4

#define RADIO_DIO_0                       PB_11
#define RADIO_DIO_1                       PC_13
#define RADIO_DIO_2                       PB_9
#define RADIO_DIO_3                       PB_4
#define RADIO_DIO_4                       PB_3
#define RADIO_DIO_5                       PA_15
#define RADIO_ANT_SWITCH_HF               PA_0
#define RADIO_ANT_SWITCH_LF               PA_1

#define I2C_SCL                           PB_6
#define I2C_SDA                           PB_7

#define UART_TX                           PA_2
#define UART_RX                           PA_3 

#define UART1_BAUDRATE                    9600
#define UART1_TX                          PA_9
#define UART1_RX                          PA_10
#define UART1_CTS                         PA_11
#define UART1_RTS                         PA_12

#define BAT_LEVEL                         PB_0


#elif TRACKER_BOARD

/*!
 * Board IO Extender pins definitions
 */
#define IRQ_BQ24250                                 ISG0
#define IRQ_TMP112                                  ISG1
#define GPS_POWER_ON                                ISG2
#define GPS_TIMEPULSE                               ISG3
#define GPS_INT                                     ISG4
#define IRQ_1_MMA8451                               ISG5
#define IRQ_2_MMA8451                               ISG6
#define LED2                                        ISG8
#define LED1                                        ISG9
#define DA14580_INT                                 ISG14
#define DA14580_INT_DISCONNECT                      ISG15
#define DA14580_CTRL_BT                             ISG17

#define LED_RED                                     LED1
#define LED_BLUE                                    LED2
#define LED_TURN_ON                                 STMPE1801IOLOW
#define LED_TURN_OFF                                STMPE1801IOHIGH
#define GPS_TURN_ON                                 STMPE1801IOLOW
#define GPS_TURN_OFF                                STMPE1801IOHIGH
#define BT_BROADCAST_TURN_OFF                       STMPE1801IOLOW
#define BT_BROADCAST_TURN_ON                        STMPE1801IOHIGH

/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 PB_10

//#define SPI_BASE                                    SPI2
#define SPI_MOSI                                    PB_15
#define SPI_MISO                                    PB_14
#define SPI_SCLK                                    PB_13
#define SPI_CS                                      PB_12

#define RADIO_DIO_0                                 PB_11
#define RADIO_DIO_1                                 PC_13
#define RADIO_DIO_2                                 PB_9
#define RADIO_DIO_3                                 PB_4
#define RADIO_DIO_4                                 PB_3
#define RADIO_DIO_5                                 PA_15

#define RADIO_ANT_SWITCH_HF                         PA_0
#define RADIO_ANT_SWITCH_LF                         PA_1

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

//#define I2C_BASE                                    I2C1
#define I2C_SCL                                     PB_6
#define I2C_SDA                                     PB_7

#define BOOT_1                                      PB_2
    
#define GPS_PPS                                     PB_1
#define UART_TX                                     PA_2
#define UART_RX                                     PA_3

#define UART1_BAUDRATE                              9600
#define UART1_TX                                    PA_9
#define UART1_RX                                    PA_10
#define UART1_CTS                                   PA_11
#define UART1_RTS                                   PA_12

//#define DC_DC_EN                                    PB_8
#define BAT_LEVEL                                   PB_0
#define WKUP1                                       PA_8

#define RF_RXTX                                     PA_5
#define BUZZER                                      PA_6
#define BUZZER_SPEED                                4000

#define SWDIO                                       PA_13
#define SWCLK                                       PA_14

#define PIN_NC                                      PB_5

//#define TEMPCALMIN                                  11.5
//#define TEMPCALMAX                                  64.5
//#define TEMPCALMIN                                  -2
//#define TEMPCALMAX                                  78
#define TEMPCALMIN                                  0.75
#define TEMPCALMAX                                  77.73
#define TEMPREALMIN                                 -5
#define TEMPREALMAX                                 60
void BoardSetBatteryCal(uint16_t value);

#elif MANHOLE_BOARD

#define RADIO_RESET                       PB_10
#define SPI_MOSI                          PB_15
#define SPI_MISO                          PB_14
#define SPI_SCLK                          PB_13
#define SPI_CS                            PB_12

#define RADIO_DIO_0                       PB_11
#define RADIO_DIO_1                       PC_13
#define RADIO_DIO_2                       PB_9
#define RADIO_DIO_3                       PB_4
#define RADIO_DIO_4                       PB_3
#define RADIO_DIO_5                       PA_15
#define RADIO_ANT_SWITCH_HF               PA_0
#define RADIO_ANT_SWITCH_LF               PA_1

#define I2C_SCL                           PB_6
#define I2C_SDA                           PB_7

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define SWDIO                                       PA_13
#define SWCLK                                       PA_14

#define SPI1_NSS                     PA_4
#define SPI1_SCK                     PA_5
#define SPI1_MISO                   PA_6
#define SPI1_MOSI                   PA_7

#define UART1_TX                          PA_9
#define UART1_RX                          PA_10
#define UART1_CTS                        PA_11
#define UART1_RTS                        PA_12
#define RF_ONOFF                          PA_8

#define UART_TX                            PA_2 // uart 2
#define UART_RX                            PA_3// uart 2

#define LED_TX      PB_5
#define LED_FCT     PB_8
#define BAT_LEVEL   PB_0
#define INT_2       PB_1
#define BOOT_1      PB_2

///* Communication boards USART Interface */
//#define USARTx_DR_ADDRESS                0x40013804
//#define USARTx_TX_DMA_CHANNEL            DMA1_Channel4
//#define USARTx_TX_DMA_FLAG_TC            DMA1_FLAG_TC4
//#define USARTx_TX_DMA_FLAG_GL            DMA1_FLAG_GL4
//#define USARTx_RX_DMA_CHANNEL            DMA1_Channel5
//#define USARTx_RX_DMA_FLAG_TC            DMA1_FLAG_TC5
//#define USARTx_RX_DMA_FLAG_GL            DMA1_FLAG_GL5

//#define DMAx_CLK                         RCC_AHBPeriph_DMA1

///* USART Communication boards Interface */
//#define USARTx                           USART1
//#define USARTx_CLK                       RCC_APB2Periph_USART1
//#define USARTx_APBPERIPHCLOCK            RCC_APB2PeriphClockCmd
//#define USARTx_IRQn                      USART1_IRQn
//#define USARTx_IRQHandler                USART1_IRQHandler


//#define TXBUFFERSIZE                     (countof(TxBuffer) - 1)
//#define RXBUFFERSIZE                     TXBUFFERSIZE

#define UART1_BAUDRATE  9600

#elif EARTAG_BOARD

#define RADIO_RESET                       PB_10
#define SPI_MOSI                          PB_15
#define SPI_MISO                          PB_14
#define SPI_SCLK                          PB_13
#define SPI_CS                            PB_12

#define RADIO_DIO_0                       PB_11
#define RADIO_DIO_1                       PC_13
#define RADIO_DIO_2                       PB_9
#define RADIO_DIO_3                       PB_4
#define RADIO_DIO_4                       PB_3
#define RADIO_DIO_5                       PA_15
#define RADIO_ANT_SWITCH_HF               PA_0
#define RADIO_ANT_SWITCH_LF               PA_1

//#define I2C_SCL                           PB_6
//#define I2C_SDA                           PB_7

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define SWDIO                                       PA_13
#define SWCLK                                       PA_14

//#define SPI1_NSS                     PA_4
//#define SPI1_SCK                     PA_5
//#define SPI1_MISO                   PA_6
//#define SPI1_MOSI                   PA_7

#define UART1_TX                          PA_9
#define UART1_RX                          PA_10

//#define UART1_CTS                        PA_11
//#define UART1_RTS                        PA_12
#define RF_ONOFF                          PA_8

#define PIN_TEMPERATURE                            PA_2 // uart 2
//#define UART_RX                            PA_3// uart 2

//#define LED_TX      PB_5
//#define LED_FCT     PB_8
#define BAT_LEVEL   PB_0
//#define INT_2       PB_1
#define BOOT_1      PB_2
#define INT_REBOOT PB_8

#define UNUSED_PIN1 PA_3
#define UNUSED_PIN2 PA_4
#define UNUSED_PIN3 PA_5
#define UNUSED_PIN4 PA_6
#define UNUSED_PIN5 PA_7
#define UNUSED_PIN6 PA_11
#define UNUSED_PIN7 PA_12

#define UNUSED_PIN8 PB_1
//#define UNUSED_PIN9 PB_2
#define UNUSED_PIN10 PB_5
#define UNUSED_PIN11 PB_6
#define UNUSED_PIN12 PB_7
//#define UNUSED_PIN13 PB_8

#define UNUSED_PIN_A0 PA_0
#define UNUSED_PIN_A1 PA_1
#define UNUSED_PIN_A2 PA_2
#define UNUSED_PIN_A3 PA_3
#define UNUSED_PIN_A4 PA_4
#define UNUSED_PIN_A5 PA_5
#define UNUSED_PIN_A6 PA_6
#define UNUSED_PIN_A7 PA_7

#define UNUSED_PIN_A8 PA_8
#define UNUSED_PIN_A9 PA_9
#define UNUSED_PIN_A10 PA_10
#define UNUSED_PIN_A11 PA_11
#define UNUSED_PIN_A12 PA_12
#define UNUSED_PIN_A13 PA_13
#define UNUSED_PIN_A14 PA_14
#define UNUSED_PIN_A15 PA_15

#define UNUSED_PIN_B0 PB_0
#define UNUSED_PIN_B1 PB_1
#define UNUSED_PIN_B2 PB_2
#define UNUSED_PIN_B3 PB_3
#define UNUSED_PIN_B4 PB_4
#define UNUSED_PIN_B5 PB_5
#define UNUSED_PIN_B6 PB_6
#define UNUSED_PIN_B7 PB_7

#define UNUSED_PIN_B8 PB_8
#define UNUSED_PIN_B9 PB_9
#define UNUSED_PIN_B10 PB_10
#define UNUSED_PIN_B11 PB_11
#define UNUSED_PIN_B12 PB_12
#define UNUSED_PIN_B13 PB_13
#define UNUSED_PIN_B14 PB_14
#define UNUSED_PIN_B15 PB_15

#define UNUSED_PIN_C13 PC_13
#define UNUSED_PIN_C14 PC_14
#define UNUSED_PIN_C15 PC_15

#define UNUSED_PIN_H0 PH_0
#define UNUSED_PIN_H1 PH_1

///* Communication boards USART Interface */
//#define USARTx_DR_ADDRESS                0x40013804
//#define USARTx_TX_DMA_CHANNEL            DMA1_Channel4
//#define USARTx_TX_DMA_FLAG_TC            DMA1_FLAG_TC4
//#define USARTx_TX_DMA_FLAG_GL            DMA1_FLAG_GL4
//#define USARTx_RX_DMA_CHANNEL            DMA1_Channel5
//#define USARTx_RX_DMA_FLAG_TC            DMA1_FLAG_TC5
//#define USARTx_RX_DMA_FLAG_GL            DMA1_FLAG_GL5

//#define DMAx_CLK                         RCC_AHBPeriph_DMA1

///* USART Communication boards Interface */
//#define USARTx                           USART1
//#define USARTx_CLK                       RCC_APB2Periph_USART1
//#define USARTx_APBPERIPHCLOCK            RCC_APB2PeriphClockCmd
//#define USARTx_IRQn                      USART1_IRQn
//#define USARTx_IRQHandler                USART1_IRQHandler


//#define TXBUFFERSIZE                     (countof(TxBuffer) - 1)
//#define RXBUFFERSIZE                     TXBUFFERSIZE

#define UART1_BAUDRATE  9600

#elif SIPMODULE_BOARD

#define LED_TX      PB_5
#define LED_FCT    PB_8
#define INT_1        PB_0
#define INT_2        PB_1

#define WKUP1                             PA_0

#define SPI1_NSS                     PA_4
#define SPI1_SCK                     PA_5
#define SPI1_MISO                   PA_6
#define SPI1_MOSI                   PA_7

#define RADIO_RESET                    PB_10
#define SPI_MOSI                          PB_15
#define SPI_MISO                          PB_14
#define SPI_SCLK                          PB_13
#define SPI_CS                              PB_12

#define RADIO_DIO_0                       PB_11
#define RADIO_DIO_1                       PC_13
#define RADIO_DIO_2                       PB_9
#define RADIO_DIO_3                       PB_4
#define RADIO_DIO_4                       PB_3
#define RADIO_DIO_5                       PA_15
#define RADIO_ANT_SWITCH_HF      PA_0
#define RADIO_ANT_SWITCH_LF       PA_1
#ifdef ANT_PA
#define RADIO_ANT_PA              PC_8
#endif
#define I2C_SCL                           PB_6
#define I2C_SDA                          PB_7

#define UART_TX                            PA_2 // uart 2
#define UART_RX                            PA_3// uart 2
#define UART_RTS                          PA_1

#define UART1_BAUDRATE             9600
#define UART1_TX                          PA_9
#define UART1_RX                          PA_10
#define UART1_CTS                         PA_11
#define UART1_RTS                         PA_12
#define UART1_CK                          PA_8
#define BAT_LEVEL                         GM_GPIO_13

#define SWDIO                                       PA_13
#define SWCLK                                       PA_14
#define BOOT_1      PB_2

#define OSC_LSE_IN                                  PC_14
#define OSC_LSE_OUT                                 PC_15

#define OSC_HSE_IN                                  PH_0
#define OSC_HSE_OUT                                 PH_1

#define GM_GPIO_1                       PB_8
#define GM_GPIO_2                       PA_3
#define GM_GPIO_3                       PA_2
#define GM_GPIO_4                       PB_6
#define GM_GPIO_5                       PB_7
#define GM_GPIO_6                       PA_4
#define GM_GPIO_7                       PA_5
#define GM_GPIO_8                       PA_6
#define GM_GPIO_9                       PA_7
#define GM_GPIO_10                       PA_8
#define GM_GPIO_11                       PA_11
#define GM_GPIO_12                       PA_12
#define GM_GPIO_13                       PB_0
#define GM_GPIO_14                       PB_1

/*for STML073xx*/
#define NO_USE_PC00                      PC_0
#define NO_USE_PC01                      PC_1
#define NO_USE_PC02                      PC_2
#define NO_USE_PC03                      PC_3
#define NO_USE_PC04                      PC_4
#define NO_USE_PC05                      PC_5
#define NO_USE_PC06                      PC_6
#define NO_USE_PC07                      PC_7
#ifndef ANT_PA
#define NO_USE_PC08                      PC_8
#endif
#define NO_USE_PC09                      PC_9
#define NO_USE_PC10                      PC_10
#define NO_USE_PC11                      PC_11
#define NO_USE_PC12                      PC_12
//#define NO_USE_PC13                      PC_13
//#define NO_USE_PC14                      PC_14
//#define NO_USE_PC15                      PC_15

#define NO_USE_PD00                      PD_0
#define NO_USE_PD01                      PD_1
#define NO_USE_PD02                      PD_2
#define NO_USE_PD03                      PD_3
#define NO_USE_PD04                      PD_4
#define NO_USE_PD05                      PD_5
#define NO_USE_PD06                      PD_6
#define NO_USE_PD07                      PD_7
#define NO_USE_PD08                      PD_8
#define NO_USE_PD09                      PD_9
#define NO_USE_PD10                      PD_10
#define NO_USE_PD11                      PD_11
#define NO_USE_PD12                      PD_12
#define NO_USE_PD13                      PD_13
#define NO_USE_PD14                      PD_14
#define NO_USE_PD15                      PD_15

#define NO_USE_PE00                      PE_0
#define NO_USE_PE01                      PE_1
#define NO_USE_PE02                      PE_2
#define NO_USE_PE03                      PE_3
#define NO_USE_PE04                      PE_4
#define NO_USE_PE05                      PE_5
#define NO_USE_PE06                      PE_6
#define NO_USE_PE07                      PE_7
#define NO_USE_PE08                      PE_8
#define NO_USE_PE09                      PE_9
#define NO_USE_PE10                      PE_10
#define NO_USE_PE11                      PE_11
#define NO_USE_PE12                      PE_12
#define NO_USE_PE13                      PE_13
#define NO_USE_PE14                      PE_14
#define NO_USE_PE15                      PE_15

#define NO_USE_PH09                      PH_9
#define NO_USE_PH10                      PH_10

#elif NODE_BOARD

#define IRQ_BQ24250                       ISG0
#define IRQ_TMP112                        ISG1
#define GPS_POWER_ON                      ISG2
#define GPS_TIMEPULSE                     ISG3
#define GPS_INT                           ISG4
#define IRQ_1_MMA8451                     ISG5
#define IRQ_2_MMA8451                     ISG6
#define LED3                              ISG15
#define LED2                              ISG14
#define LED1                              ISG13
#define WKUP1                             PA_8

#define RADIO_RESET                       PB_10
#define SPI_MOSI                          PA_7
#define SPI_MISO                          PA_6
#define SPI_SCLK                          PA_5
#define SPI_CS                            PA_4

#define RADIO_DIO_0                       PB_11
#define RADIO_DIO_1                       PC_13
#define RADIO_DIO_2                       PB_9
#define RADIO_DIO_3                       PB_4
#define RADIO_DIO_4                       PB_3
#define RADIO_DIO_5                       PA_15
#define RADIO_ANT_SWITCH_HF               PA_0
#define RADIO_ANT_SWITCH_LF               PA_1

#define I2C_SCL                           PB_6
#define I2C_SDA                           PB_7

#define UART_TX                           PA_2
#define UART_RX                           PA_3 

#define UART1_BAUDRATE                    9600
#define UART1_TX                          PA_9
#define UART1_RX                          PA_10
#define UART1_CTS                         PA_11
#define UART1_RTS                         PA_12

#define BAT_LEVEL                         PB_0


#elif RPMA_ST_NODE
/*!
 * Board IO Extender pins definitions
 */
#define GPS_LNA_ENABLE                                 ISG0
#define VBATT_DISCHARGE_ENABLE                         ISG1
#define BUZZER_ENABLE                                  ISG2
#define VCC_3V3_IO_OUT_ENABLE                          ISG3
#define VCC_5V_IO_OUT_ENABLE                           ISG4
#define CHARGE_ENABLE                                  ISG5
#define CHARGE_SYSOFF_FUNCTION                         ISG6
#define BLE_RESET                                      ISG7
#define ST_MCU_BOOT0                                   ISG8
#define BMA253_INT_1                                   ISG9
#define BMA253_INT_2                                   ISG10
#define INT_BUTTON2                                    ISG11
#define VCC_485_ENABLE                                 ISG12
#define G_SENSOR_LED                                   ISG14
#define RPMA_LED                                       ISG15
#define GPS_LED                                        ISG16
#define BLE_LED                                        ISG17

#define LED_TURN_ON                                 STMPE1801IOHIGH
#define LED_TURN_OFF                                STMPE1801IOLOW
#define GPS_TURN_ON                                 STMPE1801IOHIGH
#define GPS_TURN_OFF                                STMPE1801IOLOW

/*!
 * Board MCU pins definitions
 */
#define RPMA_PICO_NODE_TIME_QUAL            PB_10
#define RPMA_PICO_NODE_TOUT                 PB_11
#define RPMA_PICO_NODE_SRDY                 PA_15
#define RPMA_PICO_NODE_SRQ                  PB_3	
#define RPMA_PICO_NODE_RF_TXENA             PB_4
#define RPMA_PICO_NODE_ON_OFF               PB_9
#define RPMA_PICO_NODE_MRQ                  PB_2
#define RPMA_PICO_NODE_SHDN                 PB_8

#define GPS_ONOFF                           PC_13
#define BLE_ONOFF                           PA_4
#define BLE_BEACON_ONOFF                    PA_7

#define BLE_INT_CONNECT                     PA_5
#define BLE_INT_DISCONNECT                  PA_6

#define VCC_3V3_ENABLE                      PB_5

#define I2C_SCL                             PB_6
#define I2C_SDA                             PB_7

#define UART1_TX                            PA_9
#define UART1_RX                            PA_10
#define UART1_CTS                           PA_11
#define UART1_RTS                           PA_12

#define UART2_TX                            PA_2
#define UART2_RX                            PA_3

//#define SPI_CS                              PA_4
//#define SPI_SCLK                            PA_5
//#define SPI_MISO                            PA_6
//#define SPI_MOSI                            PA_7

#define SPI_CS                              PB_12
#define SPI_SCLK                            PB_13
#define SPI_MISO                            PB_14
#define SPI_MOSI                            PB_15

#define SWDIO                               PA_13
#define SWCLK                               PA_14

#define OSC_LSE_IN                          PC_14
#define OSC_LSE_OUT                         PC_15
#define OSC_HSE_IN                          PH_0
#define OSC_HSE_OUT                         PH_1

#define BAT_LEVEL                           NC
#define BOOT_1                              NC

#define UART1_BAUDRATE                      9600
#define UART2_BAUDRATE                      9600
//end RPMA_ST_NODE
#else
    #error "Please define the board in the compiler options."
#endif

void BoardInitPeriph( void );
void BoardDeInitPeriph( void );
void RecoverMcuStatus( void );
uint8_t BoardMeasureBatterieLevel( void );
int16_t BoardGetTemperature( void ) ;

#ifdef TRACKER_BOARD
float GetCalTemperature(float temp);
#endif //TRACKER_BOARD

#ifdef MANHOLE_BOARD
uint16_t BoardGetPowerSupply( void );
void BoardSetBatteryCal(float value);
#endif //MANHOLE_BOARD

#ifdef SIPMODULE_BOARD
uint16_t BoardGetPowerSupply( void );
uint8_t BoardMeasureBatterieLevel( void ) ;
#ifdef ANT_PA
uint8_t BoardGetPAEnable( void );
void BoardSetPAEnable( uint8_t value );
#endif //ANT_PA
#endif //SIPMODULE_BOARD

#ifdef EARTAG_BOARD
uint8_t BoardGetPowerSupply( void );
float EarTagGetTemperature( void );
void BoardUnusedIoInit(void);
void Delay_time(uint32_t value);
#ifdef LORAWANV1_0
uint8_t BoardMeasureBatterieLevel( void );
#endif //LORAWANV1_0
#endif //EARTAG_BOARD

#ifdef LORAWANV1_0
/*!
 * Returns a pseudo random seed generated using the MCU Unique ID
 *
 * \retval seed Generated pseudo random seed
 */
uint32_t BoardGetRandomSeed( void );

/*!
 * \brief Gets the board 64 bits unique ID 
 *
 * \param [IN] id Pointer to an array that will contain the Unique ID
 */
void BoardGetUniqueId( uint8_t *id );
#endif //LORAWANV1_0

void BoardGetEEPROMKey( uint8_t *id );

#ifdef __cplusplus
}
#endif
#endif /*__BOARD_H__*/
