#include "i2c-board.h"
#include "gpio-board.h"


#define TIMEOUT_MAX                                 0x8000 
I2C_HandleTypeDef I2c1Handle;
I2cAddrSize I2cInternalAddrSize = I2C_ADDR_SIZE_8;
bool I2cInitialized = false;
/* I2C TIMING Register define when I2C clock source is SYSCLK */
//#define I2C_TIMING                                  0x00707CBB    // Standard Mode, 100kHz
//#define I2C_TIMING                                  0x00300F38    // Fast Mode, 400kHz
#define I2C_TIMING                                  0x00300F88    // Fast Mode, 200kHz -> Use this one

void I2C1Init(void)
{
    if( I2cInitialized == false )
    {
        I2cInitialized = true;
        I2c1Handle.Instance = I2C1;
        
#if defined (RPMA_ST_NODE)
        I2c1Handle.Init.Timing = I2C_TIMING;
        I2c1Handle.Init.OwnAddress1 = 0;
        I2c1Handle.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
        I2c1Handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
        I2c1Handle.Init.OwnAddress2 = 0;
        I2c1Handle.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
        I2c1Handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
        I2c1Handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
        HAL_I2C_Init( &I2c1Handle );
        HAL_I2CEx_ConfigAnalogFilter( &I2c1Handle, I2C_ANALOGFILTER_ENABLE);
#endif        
    }
}

void I2C1DeInit(void)
{
    I2cInitialized = false;
    HAL_I2C_DeInit(&I2c1Handle);
}

void I2cResetBus(void )
{
    I2cInitialized = false;
    I2C1Init();
}
uint8_t I2cRead(uint8_t deviceAddr, uint16_t addr, uint8_t *data )
{
    return( I2cReadBuffer(deviceAddr, addr, data, 1 ) );
}

uint8_t I2cReadBuffer(uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    uint8_t status = FAIL;
    uint16_t memAddSize = 0;

    if( I2cInternalAddrSize == I2C_ADDR_SIZE_8 )
    {
        memAddSize = I2C_MEMADD_SIZE_8BIT;
    }
    else
    {
        memAddSize = I2C_MEMADD_SIZE_16BIT;
    }
    status = ( HAL_I2C_Mem_Read( &I2c1Handle, deviceAddr, addr, memAddSize, buffer, size, 2000 ) == HAL_OK ) ? OK : FAIL;
    return status;
}

uint8_t I2cWrite(uint8_t deviceAddr, uint16_t addr, uint8_t data )
{
        if( I2cWriteBuffer(deviceAddr, addr, &data, 1 ) == FAIL )
        {
            // if first attempt fails due to an IRQ, try a second time
            if( I2cWriteBuffer(deviceAddr, addr, &data, 1 ) == FAIL )
            {
                return FAIL;
            }
            else
            {
                return OK;
            }
        }
        else
        {
            return OK;
        }
}

uint8_t I2cWriteBuffer( uint8_t deviceAddr, uint16_t addr, uint8_t *buffer, uint16_t size )
{
    uint8_t status = FAIL;
    uint16_t memAddSize = 0;

    if( I2cInternalAddrSize == I2C_ADDR_SIZE_8 )
    {
        memAddSize = I2C_MEMADD_SIZE_8BIT;
    }
    else
    {
        memAddSize = I2C_MEMADD_SIZE_16BIT;
    }
    status = ( HAL_I2C_Mem_Write( &I2c1Handle, deviceAddr, addr, memAddSize, buffer, size, 2000 ) == HAL_OK ) ? OK : FAIL;
    return status;
}

void I2cSetAddrSize(I2cAddrSize addrSize )
{
    I2cInternalAddrSize = addrSize;
}
/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will 
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */   
#define EE_FLAG_TIMEOUT         ( ( uint32_t )0x1000 )
#define EE_LONG_TIMEOUT         ( ( uint32_t )( 10 * EE_FLAG_TIMEOUT ) )

/* Maximum number of trials for I2cMcuWaitStandbyState( ) function */
#define EE_MAX_TRIALS_NUMBER     300

uint8_t I2cWaitStandbyState(uint8_t deviceAddr )
{
    uint8_t status = FAIL;
    status = ( HAL_I2C_IsDeviceReady( &I2c1Handle, deviceAddr, 300, 4096 ) == HAL_OK ) ? OK : FAIL;;
    return status;
}

/**
  * @brief I2C MSP Initialization 
  *        This function configures the hardware resources used in this example: 
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration  
  *           - DMA configuration for transmission request by peripheral 
  *           - NVIC configuration for DMA interrupt request enable
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
GPIO_InitTypeDef GPIO_InitStruct;
    if(hi2c->Instance==I2C1)
    {
      /* Enable GPIO clock */
      __GPIOB_CLK_ENABLE();
    /* USER CODE BEGIN I2C1_MspInit 0 */

    /* USER CODE END I2C1_MspInit 0 */
    
      /**I2C1 GPIO Configuration    
      PB6     ------> I2C1_SCL
      PB7     ------> I2C1_SDA 
      */
      GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      GPIO_InitStruct.Alternate = GPIO_AF1_I2C1;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

      /* Peripheral clock enable */
      __HAL_RCC_I2C1_CLK_ENABLE();
    /* USER CODE BEGIN I2C1_MspInit 1 */

    /* USER CODE END I2C1_MspInit 1 */
    }
}

/**
  * @brief I2C MSP De-Initialization 
  *        This function frees the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  *          - Revert GPIO, DMA and NVIC configuration to their default state
  * @param hi2c: I2C handle pointer
  * @retval None
  */
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
  if(hi2c->Instance==I2C1)
  {
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB6     ------> I2C1_SCL
    PB7     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);
  }
}
