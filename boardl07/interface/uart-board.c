#include "board.h"
#include "uart-board.h"
#include <string.h>

/*!
 * Number of times the UartPutBuffer will try to send the buffer before
 * returning ERROR
 */
//#define TX_BUFFER_RETRY_COUNT                       10

static UARTIrqHandler *UartNotifyCallback = NULL;

/* UART1 handler declaration */
UART_HandleTypeDef Uart1Handle;
/* UART2 handler declaration */
UART_HandleTypeDef Uart2Handle;

uint8_t UART1RxBuffer;
uint8_t UART2RxBuffer;
__IO ITStatus UartReady = RESET;
bool UartBusy = FAIL;
void UartInitForModbus( Uart_t *obj, uint8_t uartId, PinNames tx, PinNames rx, PinNames cts, PinNames rts )
{
    obj->UartId = uartId;

    obj->Tx.portIndex = ( uint32_t ) tx >> 4;
    obj->Tx.pin = tx;
    obj->Tx.pinIndex = ( 0x01 << ( obj->Tx.pin & 0x0F ) );

    obj->Rx.portIndex = ( uint32_t ) rx >> 4;
    obj->Rx.pin = rx;
    obj->Rx.pinIndex = ( 0x01 << ( obj->Rx.pin & 0x0F ) );

    if( uartId == UART_1 ) //init uart 1
    {
        obj->UartBase = USART1;
    }
    else if( uartId == UART_2 ) //init uart 2
    {
        obj->UartBase = USART2;
    }
    if(cts != NC)
        GpioInit( &obj->Cts, cts, PIN_INPUT,         PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    if(rts != NC)
        GpioInit( &obj->Rts, rts, PIN_OUTPUT,        PIN_PUSH_PULL, PIN_PULL_UP, 0 );

    obj->Cts.pin = cts;
    obj->Rts.pin = rts;
}


void UartInit( Uart_t *obj, uint8_t uartId, PinNames tx, PinNames rx, PinNames cts, PinNames rts )
{
    obj->UartId = uartId;

    obj->Tx.portIndex = ( uint32_t ) tx >> 4;
    obj->Tx.pin = tx;
    obj->Tx.pinIndex = ( 0x01 << ( obj->Tx.pin & 0x0F ) );

    obj->Rx.portIndex = ( uint32_t ) rx >> 4;
    obj->Rx.pin = rx;
    obj->Rx.pinIndex = ( 0x01 << ( obj->Rx.pin & 0x0F ) );

    if( uartId == UART_1 ) //init uart 1
    {
        obj->UartBase = USART1;
    }
    else if( uartId == UART_2 ) //init uart 2
    {
        obj->UartBase = USART2;
    }
    if(cts != NC)
        GpioInit( &obj->Cts, cts, PIN_INPUT,         PIN_PUSH_PULL, PIN_PULL_UP, 0 );
    if(rts != NC)
        GpioInit( &obj->Rts, rts, PIN_OUTPUT,        PIN_PUSH_PULL, PIN_PULL_UP, 0 );

    obj->Cts.pin = cts;
    obj->Rts.pin = rts;
}

void UartConfig( Uart_t *obj, UartMode_t mode, uint32_t baudrate, WordLength_t wordLength, StopBits_t stopBits, Parity_t parity, FlowCtrl_t flowCtrl )
{
    UART_HandleTypeDef USART_InitStructure;

    USART_InitStructure.Instance = obj->UartBase;

    if( mode == TX_ONLY )
    {
        if( obj->FifoTx.Data == NULL )
        {
            while( 1 );
        }

        USART_InitStructure.Init.Mode = UART_MODE_TX;
    }
    else if( mode == RX_ONLY )
    {
        if( obj->FifoRx.Data == NULL )
        {
            while( 1 );
        }

        USART_InitStructure.Init.Mode = UART_MODE_RX;
    }
    else
    {
        if( ( obj->FifoTx.Data == NULL ) || ( obj->FifoRx.Data == NULL ) )
        {
            while( 1 );
        }

        USART_InitStructure.Init.Mode = UART_MODE_TX_RX;
    }

    USART_InitStructure.Init.BaudRate = baudrate;

    if( wordLength == UART_8_BIT )
    {
        USART_InitStructure.Init.WordLength = 0x0000;
    }
    else 
    {
        USART_InitStructure.Init.WordLength = 0x1000;
    }

    if( stopBits == UART_1_STOP_BIT )
    {
        USART_InitStructure.Init.StopBits = UART_STOPBITS_1;
    }
    else if( stopBits == UART_0_5_STOP_BIT )
    {
        USART_InitStructure.Init.StopBits = USART_CR2_STOP_0;
    }
    else if( stopBits == UART_2_STOP_BIT )
    {
        USART_InitStructure.Init.StopBits = UART_STOPBITS_2;
    }
    else if( stopBits == UART_1_5_STOP_BIT )
    {
        USART_InitStructure.Init.StopBits = USART_CR2_STOP;
    }

    if( parity == NO_PARITY )
    {
        USART_InitStructure.Init.Parity = UART_PARITY_NONE;
    }
    else if( parity == EVEN_PARITY )
    {
        USART_InitStructure.Init.Parity = UART_PARITY_EVEN;
    }
    else
    {
        USART_InitStructure.Init.Parity = UART_PARITY_ODD;
    }

    if( flowCtrl == NO_FLOW_CTRL )
    {
        USART_InitStructure.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    }
    else if( flowCtrl == RTS_FLOW_CTRL )
    {
        USART_InitStructure.Init.HwFlowCtl = UART_HWCONTROL_RTS;
    }
    else if( flowCtrl == CTS_FLOW_CTRL )
    {
        USART_InitStructure.Init.HwFlowCtl = UART_HWCONTROL_CTS;
    }
    else if( flowCtrl == RTS_CTS_FLOW_CTRL )
    {
        USART_InitStructure.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
    }
    /* The OverSampling value must to be set 0, otherwise the init will fail. */
    USART_InitStructure.Init.OverSampling = 0x0000;
    /* The AdvFeatureInit value must to be set UART_ADVFEATURE_NO_INIT, otherwise the init will fail. */
    USART_InitStructure.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if(HAL_UART_DeInit(&USART_InitStructure) != HAL_OK)
    {
        while(1){}
    }
    
    if(HAL_UART_Init(&USART_InitStructure) == HAL_OK)
    {
        UartBusy = 1;
        if( obj->UartId == UART_1 )
        {
            Uart1Handle = USART_InitStructure;
            HAL_UART_Receive_IT( &Uart1Handle, &UART1RxBuffer, 1 );
        }
        else if( obj->UartId == UART_2 )
        {
            Uart2Handle = USART_InitStructure;
            HAL_UART_Receive_IT( &Uart2Handle, &UART2RxBuffer, 1 );
        }
    }
}

void UartDeInit( Uart_t *obj )
{
    if(obj->UartId == UART_1)
        HAL_UART_DeInit(&Uart1Handle);
    else if(obj->UartId == UART_2)
        HAL_UART_DeInit(&Uart2Handle);
    GpioInit( &obj->Tx, obj->Tx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &obj->Rx, obj->Rx.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );

    if(obj->Cts.pin != NC)
        GpioInit( &obj->Cts, obj->Cts.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    if(obj->Rts.pin != NC)
        GpioInit( &obj->Rts, obj->Rts.pin, PIN_ANALOGIC, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    UartBusy = 0;
}

void UartSetIrqHandler(UARTIrqHandler * irqHandler)
{
    UartNotifyCallback = irqHandler;
}

uint8_t UartPutChar( Uart_t *obj, uint8_t data )
{
    if(obj->UartId == UART_1)
        HAL_UART_Transmit_IT( &Uart1Handle, &data, 1 );
    else if(obj->UartId == UART_2)
        HAL_UART_Transmit_IT( &Uart2Handle, &data, 1 );

    /* Wait the transmit complete to avoid UART messages error */
    while ( UartReady == RESET ) {}
    UartReady = RESET;

    return 0;
}

uint8_t UartPutChar_NoIRQ( Uart_t *obj, uint8_t data )
{
    if(obj->UartId == UART_1)
        HAL_UART_Transmit( &Uart1Handle, &data, 1 ,5000);
    else if(obj->UartId == UART_2)
        HAL_UART_Transmit( &Uart2Handle, &data, 1 ,5000);

    return 0;
}

void HAL_UART_TxCpltCallback( UART_HandleTypeDef *UartHandle )
{
    uint8_t data;
    UartId_t uartid = UART_Total;
    UartReady = SET;
    if(UartHandle->Instance == USART1)
    {
        uartid = UART_1;
    }
    else if(UartHandle->Instance == USART2)
    {
        uartid = UART_2;
    }
    if(UartNotifyCallback != NULL)
            UartNotifyCallback(uartid, UART_NOTIFY_TX, data);
}

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *UartHandle )
{
    uint8_t data;
    UartId_t uartid = UART_Total;
    uint8_t *RxAddress;
    if(UartHandle->Instance == USART1)
    {
        uartid = UART_1;
        data = UART1RxBuffer;
        RxAddress = &UART1RxBuffer;
    }
    else if(UartHandle->Instance == USART2)
    {
        uartid = UART_2;
        data = UART2RxBuffer;
        RxAddress = &UART2RxBuffer;
    }
    if(UartNotifyCallback != NULL)
            UartNotifyCallback(uartid, UART_NOTIFY_RX, data);
    //Jason mark on 2019.05.27
    //__HAL_UART_FLUSH_DRREGISTER( UartHandle );
    HAL_UART_Receive_IT( UartHandle, RxAddress, 1 );

}

void HAL_UART_ErrorCallback( UART_HandleTypeDef *UartHandle )
{
    //Jason mark on 2019.05.27
    //__HAL_UART_FLUSH_DRREGISTER( UartHandle );
    HAL_UART_Receive_IT( UartHandle, &UART1RxBuffer, 1 );
}

void USART2_IRQHandler( void )
{
    HAL_UART_IRQHandler( &Uart2Handle );
}

void USART1_IRQHandler( void )
{
    HAL_UART_IRQHandler( &Uart1Handle );
}

/**
  * @brief UART MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef  GPIO_InitStruct;
    if(huart->Instance == USART1)
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable GPIO TX/RX clock */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* Enable USARTx clock */
        __HAL_RCC_USART1_CLK_ENABLE();

        /*##-2- Configure peripheral GPIO ##########################################*/
        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = GPIO_PIN_9;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_USART1;

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin = GPIO_PIN_10;
        //GPIO_InitStruct.Alternate = GPIO_AF4_USART1;

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /*##-3- Configure the NVIC for UART ########################################*/
        /* NVIC for USART */
        HAL_NVIC_SetPriority(USART1_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);

        /*enable rx interrupt*/
        //HAL_UART_Receive_IT( huart, &UART1RxBuffer, 1 );
    }
    else if(huart->Instance == USART2)
    {
        /*##-1- Enable peripherals and GPIO Clocks #################################*/
        /* Enable GPIO TX/RX clock */
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* Enable USARTx clock */
        __HAL_RCC_USART2_CLK_ENABLE();

        /*##-2- Configure peripheral GPIO ##########################################*/
        /* UART TX GPIO pin configuration  */
        GPIO_InitStruct.Pin       = GPIO_PIN_2;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_USART2;

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* UART RX GPIO pin configuration  */
        GPIO_InitStruct.Pin = GPIO_PIN_3;
        //GPIO_InitStruct.Alternate = GPIO_AF4_USART1;

        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /*##-3- Configure the NVIC for UART ########################################*/
        /* NVIC for USART */
        HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);

        /*enable rx interrupt*/
        //HAL_UART_Receive_IT( huart, &UART2RxBuffer, 1 );
    }
}

/**
  * @brief UART MSP De-Initialization
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param huart: UART handle pointer
  * @retval None
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        /*##-1- Reset peripherals ##################################################*/
        __HAL_RCC_USART1_FORCE_RESET();
        __HAL_RCC_USART1_RELEASE_RESET();
        __HAL_RCC_USART1_CLK_DISABLE( );

        /*##-2- Disable peripherals and GPIO Clocks #################################*/
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_10);

        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
    else if(huart->Instance == USART2)
    {
        /*##-1- Reset peripherals ##################################################*/
        __HAL_RCC_USART2_FORCE_RESET();
        __HAL_RCC_USART2_RELEASE_RESET();
        __HAL_RCC_USART2_CLK_DISABLE( );

        /*##-2- Disable peripherals and GPIO Clocks #################################*/
        /* Configure UART Tx as alternate function  */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2);
        /* Configure UART Rx as alternate function  */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_3);

        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }

}

bool CheckUartBusy(void)
{
    return UartBusy;
}
