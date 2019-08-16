#include "spi-board.h"
#include "gpio-board.h"
#include "board.h"
#include "stm32l0xx_hal_spi.h"

Gpio_t ioPin_cs;
SPI_HandleTypeDef SPI_BASE;
/**
  * @brief  Chip Select pin low
  */
#define SPI_CS_LOW()       GpioWrite(&ioPin_cs,  0)
/**
  * @brief  Chip Select pin high
  */
#define SPI_CS_HIGH()      GpioWrite(&ioPin_cs, 1) 

/**
  * @brief  DeInitializes the SPI interface.
  * @param  None
  * @retval None
  */
void SpiDeInit(void)
{
    
    Gpio_t ioPin;
    HAL_SPI_DeInit( &SPI_BASE);
    GpioInit( &ioPin, SPI_MOSI, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, SPI_MISO, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_DOWN, 0 );
    GpioInit( &ioPin, SPI_SCLK, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
    GpioInit( &ioPin, SPI_CS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 1 );
    
}

/**
  * @brief  Initializes the SPI interface
  * @param  None
  * @retval None
  */
void SpiInit(SPI_Number spi)
{
    if(spi == SPIInterface2)
        SPI_BASE.Instance = SPI2;
    else if(spi == SPIInterface1)
        SPI_BASE.Instance = SPI1;
    
    SPI_BASE.Init.NSS = SPI_NSS_SOFT;
    
    SPI_BASE.Init.Direction = SPI_DIRECTION_2LINES;
    SPI_BASE.Init.Mode = SPI_MODE_MASTER;
    SPI_BASE.Init.DataSize = SPI_DATASIZE_8BIT;
    SPI_BASE.Init.CLKPolarity = SPI_POLARITY_LOW;
    SPI_BASE.Init.CLKPhase = SPI_PHASE_1EDGE;
    SPI_BASE.Init.TIMode = SPI_TIMODE_DISABLE;
    SPI_BASE.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    SPI_BASE.Init.CRCPolynomial = 7;
    SPI_BASE.Init.FirstBit = SPI_FIRSTBIT_MSB;
    SPI_BASE.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;/*32MHZ/8*/
    
    HAL_SPI_Init( &SPI_BASE );
}

FlagStatus SpiGetFlag(uint16_t flag )
{
    FlagStatus bitstatus = RESET;

    // Check the status of the specified SPI flag
    if( ( SPI_BASE.Instance->SR & flag ) != ( uint16_t )RESET )
    {
        // SPI_I2S_FLAG is set
        bitstatus = SET;
    }
    else
    {
        // SPI_I2S_FLAG is reset
        bitstatus = RESET;
    }
    // Return the SPI_I2S_FLAG status
    return  bitstatus;
}

uint16_t SpiInOut(uint16_t outData )
{
//    while( SPI_I2S_GetFlagStatus( SPI_BASE, SPI_I2S_FLAG_TXE ) == RESET );
//    SPI_I2S_SendData( SPI_BASE, outData );
//    while( SPI_I2S_GetFlagStatus( SPI_BASE, SPI_I2S_FLAG_RXNE ) == RESET );
//    return SPI_I2S_ReceiveData( SPI_BASE );;
    uint8_t rxData = 0;
    uint8_t txData =( outData & 0xFF );
//    if( ( obj == NULL ) || ( obj->Spi.Instance ) == NULL )
//    {
//        assert_param( FAIL );
//    }

    
  if(HAL_SPI_TransmitReceive(&SPI_BASE, (uint8_t*)&txData, (uint8_t *)&rxData, 1, 1000) != HAL_OK)
  {
    /* Transfer error in transmission process */
//    while(1)
//    {
//    }
      return( rxData );
  }
  
  /*##-3- Wait for the end of the transfer ###################################*/  
  while (HAL_SPI_GetState(&SPI_BASE) != HAL_SPI_STATE_READY)
  {
  } 
//    __HAL_SPI_ENABLE( &SPI_BASE );

//    while( SpiGetFlag( SPI_FLAG_TXE ) == RESET );
//    SPI_BASE.Instance->DR = ( uint16_t ) ( outData & 0xFF );

//    while( SpiGetFlag( SPI_FLAG_RXNE ) == RESET );
//    rxData = ( uint16_t ) SPI_BASE.Instance->DR;

    return( rxData );
}

void SpiWriteBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;

    SPI_CS_LOW();

    SpiInOut(addr | 0x80 );
    for( i = 0; i < size; i++ )
    {
        SpiInOut(buffer[i] );
    }

    SPI_CS_HIGH();
}

void SpiReadBuffer( uint8_t addr, uint8_t *buffer, uint8_t size )
{
    uint8_t i;
    
    SPI_CS_LOW();

    SpiInOut(addr & 0x7F );

    for( i = 0; i < size; i++ )
    {
        buffer[i] = SpiInOut(0);
    }

    SPI_CS_HIGH();
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
//    Gpio_t ioPin_mosi;
//    Gpio_t ioPin_miso;
//    Gpio_t ioPin_sclk;
    GPIO_InitTypeDef  GPIO_InitStruct;

  if (hspi->Instance == SPI2)
  {
    /*##-1- Enable peripherals and GPIO Clocks #################################*/
    /* Enable GPIO TX/RX clock PB12/13/14/15*/
    __GPIOB_CLK_ENABLE();
    /* Enable SPI clock */
    __SPI2_CLK_ENABLE();

    /*##-2- Configure peripheral GPIO ##########################################*/
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_13;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
//    GpioInit( &ioPin_mosi, SPI_MOSI, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI2 );
//    GpioInit( &ioPin_miso, SPI_MISO, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI2 );
//    GpioInit( &ioPin_sclk, SPI_SCLK, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI2 );
    /*##-3- Configure the NVIC for SPI #########################################*/
    /* NVIC for SPI */
//    HAL_NVIC_SetPriority(SPIx_IRQn, 1, 0);
//    HAL_NVIC_EnableIRQ(SPIx_IRQn);
  }
  else if(hspi->Instance == SPI1)
  {
    /* Enable GPIO TX/RX clock PA4/5/6/7*/
    __GPIOA_CLK_ENABLE();
    /* Enable SPI clock */
    __SPI1_CLK_ENABLE();
      
          /*##-2- Configure peripheral GPIO ##########################################*/
    /* SPI SCK GPIO pin configuration  */
    GPIO_InitStruct.Pin       = GPIO_PIN_5;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI MISO GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI MOSI GPIO pin configuration  */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//      GpioInit( &ioPin_mosi, SPI_MOSI, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI1 );
//      GpioInit( &ioPin_miso, SPI_MISO, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI1 );
//      GpioInit( &ioPin_sclk, SPI_SCLK, PIN_ALTERNATE_FCT, PIN_PUSH_PULL, PIN_PULL_DOWN, GPIO_AF5_SPI1 );
  }
  GpioInit( &ioPin_cs, SPI_CS, PIN_OUTPUT, PIN_PUSH_PULL, PIN_PULL_UP, 1 );
}

/**
  * @brief SPI MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO and NVIC configuration to their default state
  * @param hspi: SPI handle pointer
  * @retval None
  */
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance == SPI2)
  {
    /*##-1- Reset peripherals ##################################################*/
    __SPI2_FORCE_RESET();
    __SPI2_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure SPI SCK as alternate function  */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_13);
    /* Configure SPI MISO as alternate function  */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_14);
    /* Configure SPI MOSI as alternate function  */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_15);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12);
    __SPI2_CLK_DISABLE();
    /*##-3- Disable the NVIC for SPI ###########################################*/
//    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  }
  else if(hspi->Instance == SPI1)
  {
    /*##-1- Reset peripherals ##################################################*/
    __SPI1_FORCE_RESET();
    __SPI1_RELEASE_RESET();

    /*##-2- Disable peripherals and GPIO Clocks ################################*/
    /* Configure SPI SCK as alternate function  */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5);
    /* Configure SPI MISO as alternate function  */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6);
    /* Configure SPI MOSI as alternate function  */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_7);
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
    __SPI1_CLK_DISABLE();
    /*##-3- Disable the NVIC for SPI ###########################################*/
//    HAL_NVIC_DisableIRQ(SPI2_IRQn);
  }
}