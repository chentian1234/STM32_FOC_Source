/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_svpwm_ics.h
* Author             : IMS Systems Lab
* Date First Issued  : 30/05/07
* Description        : This file contains declaration of functions exported by 
*                      module stm32x_svpwm_ics.c
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
********************************************************************************
* THE PRESENT SOFTWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/

/*******************************************************************************
* 文件说明(中文) :
*   本头文件声明「ICS(Integrated Current Sensing, 集成电流采样) + SVPWM」模块对外接口。
*   ICS 指集成在功率驱动芯片内部的电流检测功能: 芯片直接输出与各相(下桥臂)电流
*   成正比的模拟电压，MCU 通过 ADC 通道(PHASE_A/B_ADC_CHANNEL)读取，从而在
*   固定时刻便可同时获得两相电流，无需像单电阻那样做移相处理。
*   对应的实现文件为 stm32f10x_svpwm_ics.c，由 ICS_SENSORS 宏使能编译。
*   上层电流环调用 SVPWM_IcsCalcDutyCycles() 计算占空比，
*   调用 SVPWM_IcsGetPhaseCurrentValues() 取回 Ia/Ib(q1.15 定标)。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_SVPWM_ICS_H
#define __STM32F10x_SVPWM_ICS_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "MC_pwm_ics_prm.h"
#include "MC_const.h"
#include "MC_Control_Param.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

void SVPWM_IcsInit(void);                                          // 初始化 TIM1/PWM、ADC1/ADC2 及 ICS 电流采样通道与中断(上电/启动时调用一次)
Curr_Components SVPWM_IcsGetPhaseCurrentValues(void);              // 由 ADC 注入值换算相电流 Ia/Ib(q1.15), 返回 (Ia, Ib) 两分量
void SVPWM_IcsCalcDutyCycles (Volt_Components Stat_Volt_Input);    // 由 α/β 电压指令直接计算三相占空比并写入 CCR1~CCR3
void SVPWM_IcsCurrentReadingCalibration(void);                     // 采集零电流时 A/B 相 ADC 偏置, 用于电流读数零点补偿
u8 SVPWMEOCEvent(void);                                           // ADC 注入转换结束中断内调用: 读取母线电压与温度采样值

#endif /* __STM32F10x_SVPWM_ICS_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
