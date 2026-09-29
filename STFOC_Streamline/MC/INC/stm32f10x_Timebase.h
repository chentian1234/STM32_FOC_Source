/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_Timebase.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : Contains the prototypes of the time base module related
*                      functions.
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
* 文件说明(中文):
*   本文件声明 500us 软定时器（时基）模块的接口。该时基由 SysTick 中断驱动，
*   为整机提供统一的 0.5ms 计时节拍，供以下用途：
*     - 主状态机(main.c)的延时与超时判定；
*     - 显示刷新节流(MC_Display.c)；
*     - 按键消抖；
*     - 无传感器启动超时保护；
*     - 速度环采样周期控制（周期性触发 FOC_CalcFluxTorqueRef）。
*   所有计数均以 0.5ms(500us) 为最小单位。
*******************************************************************************/

#ifndef __STM32F10x_TIMEBASE_H
#define __STM32F10x_TIMEBASE_H

void TB_Init(void);            // 初始化 SysTick 并把时基配置为 500us 中断
void TB_Wait(u16);            // 阻塞等待指定的 500us 个数（忙等，仅供初始化使用）
void TB_Set_Delay_500us(u16);   // 设置主状态机用延时（单位：500us）
bool TB_Delay_IsElapsed(void);   // 查询主状态机延时是否到期（TRUE=已到期）
void TB_Set_DisplayDelay_500us(u16);   // 设置显示模块用延时（单位：500us）
bool TB_DisplayDelay_IsElapsed(void);   // 查询显示延时是否到期（TRUE=已到期）
void TB_Set_DebounceDelay_500us(u8);   // 设置按键消抖延时（单位：500us）
bool TB_DebounceDelay_IsElapsed(void);   // 查询消抖延时是否到期（TRUE=已到期）
bool TB_StartUp_Timeout_IsElapsed(void);   // 查询启动超时是否到期（TRUE=已到期）
void TB_Set_StartUp_Timeout(u16);   // 设置启动超时（单位：ms，内部×2 换为 500us）
#endif //__STM32F10x_TIMEBASE_H

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
