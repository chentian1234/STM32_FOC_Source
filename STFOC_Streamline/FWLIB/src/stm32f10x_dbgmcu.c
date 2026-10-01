/**
  ******************************************************************************
  * @file    stm32f10x_dbgmcu.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file provides all the DBGMCU firmware functions.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_dbgmcu.h"

/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @defgroup DBGMCU 
  * @brief DBGMCU driver modules
  * @{
  */ 

/** @defgroup DBGMCU_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup DBGMCU_Private_Defines
  * @{
  */

#define IDCODE_DEVID_MASK    ((uint32_t)0x00000FFF)
/**
  * @}
  */

/** @defgroup DBGMCU_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @defgroup DBGMCU_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup DBGMCU_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup DBGMCU_Private_Functions
  * @{
  */

/**
  * @brief  Returns the device revision identifier.
  * @param  None
  * @retval Device revision identifier
  */
uint32_t DBGMCU_GetREVID(void)
{
   return(DBGMCU->IDCODE >> 16);
}

/**
  * @brief  Returns the device identifier.
  * @param  None
  * @retval Device identifier
  */
uint32_t DBGMCU_GetDEVID(void)
{
   return(DBGMCU->IDCODE & IDCODE_DEVID_MASK);
}

/**
  * @brief 配置指定外设及低功耗模式的行为
* 当MCU处于调试模式时使用。
* @param DBGMCU_Periph：指定外设和低功耗模式。
* 此参数可为以下值的任意组合：
*     @arg DBGMCU_SLEEP：在睡眠模式下保持调试器连接
*     @arg DBGMCU_STOP：在停止模式下保持调试器连接
*     @arg DBGMCU_STANDBY：在待机模式下保持调试器连接
*     @arg DBGMCU_IWDG_STOP：当核心停止时，调试IWDG停止
*     @arg DBGMCU_WWDG_STOP：当核心停止时，调试WWDG停止
*     @arg DBGMCU_TIM1_STOP：当核心停止时，TIM1计数器停止
*     @arg DBGMCU_TIM2_STOP：当核心停止时，TIM2计数器停止
*     @arg DBGMCU_TIM3_STOP：当核心停止时，TIM3计数器停止
*     @arg DBGMCU_TIM4_STOP：当核心停止时，TIM4计数器停止
*     @arg DBGMCU_CAN1_STOP：当核心停止时，调试CAN1停止
*     @arg DBGMCU_I2C1_SMBUS_TIMEOUT：当核心停止时，I2C1 SMBUS超时模式停止
*     @arg DBGMCU_I2C2_SMBUS_TIMEOUT：当核心停止时，I2C2 SMBUS超时模式停止
*     @arg DBGMCU_TIM5_STOP：当核心停止时，TIM5计数器停止
*     @arg DBGMCU_TIM6_STOP：当核心停止时，TIM6计数器停止  
*     @arg DBGMCU_TIM7_STOP：当核心停止时，TIM7计数器停止  
*     @arg DBGMCU_TIM8_STOP：当核心停止时，TIM8计数器停止  
*     @arg DBGMCU_CAN2_STOP：当核心停止时，Debug CAN2 停止  
*     @arg DBGMCU_TIM15_STOP：当核心停止时，TIM15计数器停止  
*     @arg DBGMCU_TIM16_STOP：当核心停止时，TIM16计数器停止  
*     @arg DBGMCU_TIM17_STOP：当核心停止时，TIM17计数器停止  
*     @arg DBGMCU_TIM9_STOP：当核心停止时，TIM9计数器停止  
*     @arg DBGMCU_TIM10_STOP：当核心停止时，TIM10计数器停止  
*     @arg DBGMCU_TIM11_STOP：当核心停止时，TIM11计数器停止  
*     @arg DBGMCU_TIM12_STOP：当核心停止时，TIM12计数器停止  
*     @arg DBGMCU_TIM13_STOP：当核心停止时，TIM13计数器停止  
*     @arg DBGMCU_TIM14_STOP：当核心停止时，TIM14计数器停止  
* @param NewState：指定外设在调试模式下的新状态。  
*   此参数可为：ENABLE 或 DISABLE
  * @retval 无
  */
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DBGMCU_PERIPH(DBGMCU_Periph));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    DBGMCU->CR |= DBGMCU_Periph;
  }
  else
  {
    DBGMCU->CR &= ~DBGMCU_Periph;
  }
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
