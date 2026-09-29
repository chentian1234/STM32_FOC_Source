/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_conf.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Library configuration file.
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
  *******************************************************************************/

/*******************************************************************************
* 文件说明(中文):
*   本文件是标准外设库(StdPeriph Library)的配置文件。
*   作用: 通过"是否包含某外设头文件"来启用/禁用对应外设驱动模块。
*   只有在此处 #include 了某外设的 .h，该外设的驱动(.c)才会被编译进工程。
*   最下方的 USE_FULL_ASSERT 用于决定是否启用库函数参数检查(assert_param)。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_CONF_H
#define __STM32F10x_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Uncomment/Comment the line below to enable/disable peripheral header file inclusion */
#include "stm32f10x_adc.h"      // 模数转换器(ADC): 电流/母线电压采样必需
#include "stm32f10x_bkp.h"      // 备份寄存器(BKP)
#include "stm32f10x_can.h"      // CAN 总线控制器
#include "stm32f10x_cec.h"      // CEC(消费电子控制，本工程未用)
#include "stm32f10x_crc.h"      // CRC 计算单元
#include "stm32f10x_dac.h"      // 数模转换器(DAC)
#include "stm32f10x_dbgmcu.h"   // 调试 MCU: 调试时冻结定时器/看门狗等
#include "stm32f10x_dma.h"      // 直接存储器访问(DMA)
#include "stm32f10x_exti.h"     // 外部中断/事件控制器(EXTI)
#include "stm32f10x_flash.h"    // 内部 Flash 编程/选项字节
#include "stm32f10x_fsmc.h"     // 外部存储器控制器(FSMC)
#include "stm32f10x_gpio.h"     // 通用输入输出(GPIO): LED/按键/PWM 引脚必需
#include "stm32f10x_i2c.h"      // I2C 总线
#include "stm32f10x_iwdg.h"     // 独立看门狗(IWDG)
#include "stm32f10x_pwr.h"      // 电源管理(PWR)
#include "stm32f10x_rcc.h"      // 复位与时钟控制(RCC): 时钟使能必需
#include "stm32f10x_rtc.h"      // 实时时钟(RTC)
#include "stm32f10x_sdio.h"     // SDIO 接口
#include "stm32f10x_spi.h"      // SPI 总线
#include "stm32f10x_tim.h"      // 定时器(TIM): FOC 的 PWM/时序核心(TIM1)
#include "stm32f10x_usart.h"    // 通用同步异步收发器(USART): 串口必需
#include "stm32f10x_wwdg.h"     // 窗口看门狗(WWDG)
#include "misc.h" /* High level functions for NVIC and SysTick (add-on to CMSIS functions) */   // NVIC/系统滴答定时器高层封装

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Uncomment the line below to expanse the "assert_param" macro in the
   Standard Peripheral Library drivers code */
//#define USE_FULL_ASSERT    1     // 取消注释即启用库函数参数断言检查(会增加代码量，仅调试时用)

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function which reports
  *         the name of the source file and the source line number of the call
  *         that failed. If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))   // 启用断言: 参数非法时调用 assert_failed 报告文件与行号
/* Exported functions ------------------------------------------------------- */
  void assert_failed(uint8_t* file, uint32_t line);   // 断言失败回调(需用户自行实现)
#else
  #define assert_param(expr) ((void)0)                    // 未启用断言: 宏展开为空，不产生任何代码
#endif /* USE_FULL_ASSERT */

#endif /* __STM32F10x_CONF_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
