/**
  ******************************************************************************
  * @file    FreeRTOS\FreeRTOS_LowPower\Src\stm32l0xx_it.c
  * @author  MCD Application Team
  * @version V1.7.0
  * @date    31-May-2016
  * @brief   Main Interrupt Service Routines.
  ******************************************************************************
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_it.h"
#include "stm32l0xx_hal.h"
#include "cmsis_os.h"
#include "eeprom-board.h"
#ifdef EARTAG_BOARD
#include "root.h"
#endif
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0+ Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
#ifdef EARTAG_BOARD
    uint8_t hardfault = 1;
    InternalEepromWriteBuffer(EEPROM_CRASH_HARDFAULT_ADDR, (uint8_t *)&hardfault, sizeof(hardfault));
    NVIC_SystemReset();
#endif
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
#ifdef EARTAG_BOARD
    uint8_t memManage = 1;
    InternalEepromWriteBuffer(EEPROM_CRASH_MEMORY_MANAGE_ADDR, (uint8_t *)&memManage, sizeof(memManage));
    NVIC_SystemReset();
#endif
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
#ifdef EARTAG_BOARD
    uint8_t busFault = 1;
    InternalEepromWriteBuffer(EEPROM_CRASH_BUSFAULT_ADDR, (uint8_t *)&busFault, sizeof(busFault));
    NVIC_SystemReset();
#endif
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
#ifdef EARTAG_BOARD
    uint8_t usageFault = 1;
    InternalEepromWriteBuffer(EEPROM_CRASH_USAGEFAULT_ADDR, (uint8_t *)&usageFault, sizeof(usageFault));
    NVIC_SystemReset();
#endif
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
  osSystickHandler();
}

/******************************************************************************/
/*                 STM32L0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l0xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
