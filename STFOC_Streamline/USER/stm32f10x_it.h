/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.h 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   This file contains the headers of the interrupt handlers.
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

/*******************************************************************************
* 文件说明(中文):
*   本文件是中断服务函数(ISR)的声明头文件，声明 Cortex-M3 内核异常处理函数。
*   注意: 外设中断(如 ADC1_2_IRQHandler、TIM1_UP_IRQHandler、USART1_IRQHandler 等)
*   在本文件中并未声明，它们的定义分散在 stm32f10x_it.c 与 uart.c 中，
*   函数名与启动文件(startup_stm32f10x_xx.s)中的向量表一一对应。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_IT_H
#define __STM32F10x_IT_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void NMI_Handler(void);         // 不可屏蔽中断(NMI)处理函数
void HardFault_Handler(void);   // 硬件错误异常处理函数
void MemManage_Handler(void);   // 存储器管理异常处理函数
void BusFault_Handler(void);    // 总线错误异常处理函数
void UsageFault_Handler(void);  // 用法错误异常处理函数
void SVC_Handler(void);         // 系统服务调用(SVC)处理函数
void DebugMon_Handler(void);    // 调试监视器异常处理函数
void PendSV_Handler(void);      // 可挂起系统服务(PendSV，常用于 RTOS 任务切换)
void SysTick_Handler(void);     // 系统滴答定时器中断处理函数(本工程未使用)

#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_IT_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
