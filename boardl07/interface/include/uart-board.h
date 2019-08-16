#ifndef __UART_BOARD_H__
#define __UART_BOARD_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "gpio-board.h"
#include "fifo.h"

typedef enum
{
    UART_1 = 0,
    UART_2,
    UART_Total,
    UART_USB_CDC = 255,
}UartId_t;
     
/*!
 * UART notification identifier
 */
typedef enum
{
    UART_NOTIFY_TX,
    UART_NOTIFY_RX
}UartNotifyId_t;

/*!
 * UART object type definition
 */
typedef struct
{
    uint8_t UartId;
    bool IsInitialized;
    Gpio_t Tx;
    Gpio_t Rx;
    Gpio_t Cts;
    Gpio_t Rts;
    Fifo_t FifoTx;
    Fifo_t FifoRx;
    void * UartBase;
    bool needFeedback;
    bool IsBusy;
    /*!
     * IRQ user notification callback prototype.
     */
//    void ( *IrqNotify )( UartNotifyId_t id );
}Uart_t;

/*!
 * Operation Mode for the UART
 */
typedef enum
{
    TX_ONLY = 0,
    RX_ONLY,
    RX_TX
}UartMode_t;

/*!
 * UART word length
 */
typedef enum
{
    UART_8_BIT = 0,
    UART_9_BIT
}WordLength_t;

/*!
 * UART stop bits
 */
typedef enum
{
    UART_1_STOP_BIT = 0,
    UART_0_5_STOP_BIT,
    UART_2_STOP_BIT,
    UART_1_5_STOP_BIT
}StopBits_t;

/*!
 * UART parity
 */
typedef enum
{
    NO_PARITY = 0,
    EVEN_PARITY,
    ODD_PARITY
}Parity_t;

/*!
 * UART flow control
 */
typedef enum
{
    NO_FLOW_CTRL = 0,
    RTS_FLOW_CTRL,
    CTS_FLOW_CTRL,
    RTS_CTS_FLOW_CTRL
}FlowCtrl_t;

typedef void( UARTIrqHandler )(UartId_t uartId, UartNotifyId_t notifyId, uint8_t data);

/*!
 * \brief Initializes the UART object and MCU peripheral
 *
 * \param [IN] obj  UART object
 * \param [IN] tx   UART Tx pin name to be used
 * \param [IN] rx   UART Rx pin name to be used
 */
void UartInit( Uart_t *obj, uint8_t uartId, PinNames tx, PinNames rx, PinNames cts, PinNames rts);

/*!
 * \brief Configures the UART object and MCU peripheral
 *
 * \remark UartInit function must be called first.
 *
 * \param [IN] obj          UART object
 * \param [IN] mode         Mode of operation for the UART
 * \param [IN] baudrate     UART baudrate
 * \param [IN] wordLength   packet length
 * \param [IN] stopBits     stop bits setup
 * \param [IN] parity       packet parity
 * \param [IN] flowCtrl     UART flow control
 */
void UartConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl );

/*!
 * \brief DeInitializes the UART object and MCU pin
 *
 * \param [IN] obj  UART object
 */
void UartDeInit( Uart_t *obj );

///*!
// * \brief Sends a character to the UART
// *
// * \param [IN] obj   UART object
// * \param [IN] data  Character to be sent
// * \retval status    [0: OK, 1: Busy]
// */
//uint8_t UartPutChar( Uart_t *obj, uint8_t data );

///*!
// * \brief Gets a character from the UART
// *
// * \param [IN] obj   UART object
// * \param [IN] data  Received character
// * \retval status    [0: OK, 1: Busy]
// */
//uint8_t UartGetChar( Uart_t *obj, uint8_t *data );

///*!
// * \brief Sends a buffer to the UART
// *
// * \param [IN] obj    UART object
// * \param [IN] buffer Buffer to be sent
// * \param [IN] size   Buffer size
// * \retval status     [0: OK, 1: Busy]
// */
//uint8_t UartPutBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size );

///*!
// * \brief Gets a character from the UART
// *
// * \param [IN] obj          UART object
// * \param [IN] buffer       Buffer to be sent
// * \param [IN] size         Buffer size
// * \param [OUT] nbReadBytes Number of bytes really read
// * \retval status           [0: OK, 1: Busy]
// */
//uint8_t UartGetBuffer( Uart_t *obj, uint8_t *buffer, uint16_t size, uint16_t *nbReadBytes );

uint8_t UartPutChar( Uart_t *obj, uint8_t data );
uint8_t UartPutChar_NoIRQ( Uart_t *obj, uint8_t data );
void UartSetIrqHandler(UARTIrqHandler * irqHandler);
/* cts and rts are both output pin*/
void UartInitForModbus( Uart_t *obj, uint8_t uartId, PinNames tx, PinNames rx, PinNames cts, PinNames rts );
bool CheckUartBusy(void);
#ifdef __cplusplus
}
#endif
#endif /*__UART_BOARD_H__*/
