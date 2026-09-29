/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_encoder.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file contains the software implementation for the
*                      encoder position and speed reading.
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

/* ==========================================================================
 * 文件说明(中文)
 *   本文件是增量式编码器(Encoder)位置/速度反馈模块的对外接口头文件，
 *   声明了编码器反馈处理函数（初始化、电角度/机械角度获取、转速获取、
 *   速度缓冲清零、启动对齐等），供 FOC(磁场定向控制)上层软件调用。
 *   编码器反馈原理: 编码器输出 A/B 两路正交方波与 Z 相零位脉冲，定时器
 *   以「编码器模式」对 A/B 的 4 倍频边沿计数，得到转子连续位置；对位置
 *   作差分即得机械转速。相关可调参数见 MC_encoder_param.h。
 * ========================================================================== */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_ENCODER_H
#define __STM32F10x_ENCODER_H

/* Includes ------------------------------------------------------------------*/
#include "MC_encoder_param.h"

/* Exported functions ------------------------------------------------------- */
void ENC_Init(void);                    // 中文: 初始化编码器定时器(编码器模式)、GPIO 与中断
s16 ENC_Get_Electrical_Angle(void);     // 中文: 获取转子电角度(s16 定标, 360 度 = 65536)
s16 ENC_Get_Mechanical_Angle(void);     // 中文: 获取转子机械角度(s16 定标, 360 度 = 65536)
void ENC_Clear_Speed_Buffer(void);      // 中文: 清空测速平均缓冲, 并置首测标志
void ENC_ResetEncoder(void);            // 中文: 将编码器计数器复位到对齐角度对应值
s16 ENC_Get_Mechanical_Speed(void);     // 中文: 获取平滑后的机械转速(0.1Hz 定标)
void ENC_Calc_Average_Speed(void);      // 中文: 计算并平滑更新机械转速, 含错误检测
bool ENC_ErrorOnFeedback(void);         // 中文: 查询测速反馈是否判定为故障(TRUE = 故障)
void ENC_Start_Up(void);                // 中文: 启动对齐流程, 按转子磁极定向后切入 RUN

#endif  /*__STM32F10x_ENCODER_H*/
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
