/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_encoder_param.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : Contains the list of project specific parameters related
*                      to the encoder speed and position feedback.
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 22/07/08 v2.0.1
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
 *   本文件是增量式编码器(Encoder)位置/速度反馈模块(stm32f10x_encoder.c)
 *   的工程参数配置头文件，集中定义了编码器反馈所需的全部可调参数，包括：
 *     - 由哪个 16 位定时器承载编码器 A/B 正交信号(TIM2/TIM3/TIM4)；
 *     - 编码器每转脉冲数 ENCODER_PPR 及其与定时器计数的换算关系；
 *     - 应用允许的机械转速上下限(rpm)及对应的速度校验门限；
 *     - 速度平均缓冲深度、连续错误判定次数(进入 FAULT 的判据)；
 *     - 启动对齐(Alignment)阶段的时长、对齐电流与对齐电角度。
 *   在 FOC(磁场定向控制)中，编码器提供转子连续位置(电角度/机械角度)与
 *   转速；本文件即这些测量的量纲/定标与门限定义处。
 *   注: 文件末段还定义了若干由上述基准量换算得出的派生宏(带 S16 后缀者为
 *       s16 电角度定标)，各行含义见其旁的中文注释。
 * ========================================================================== */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_ENCODER_PARAM_H
#define __MC_ENCODER_PARAM_H

#include "STM32F10x_MCconf.h"

/* PERIPHERAL SET-UP ---------------------------------------------------------*/
// 中文: 外设配置区。仅当启用编码器(ENCODER)或编码器反馈观测
//       (VIEW_ENCODER_FEEDBACK)时，才选择承载编码器的定时器。
#if ((defined ENCODER)||(defined VIEW_ENCODER_FEEDBACK))
/* Define here the 16-bit timer chosen to handle encoder feedback */
/*TIMER 2 is the mandatory selection when using STM32MC-KIT */
// 中文: 选择承载编码器反馈的 16 位定时器；三选一，只允许开启其中之一。
//       使用 STM32MC-KIT 时 TIM2 为强制选择。
#define TIMER2_HANDLES_ENCODER        // 中文: 由 TIM2 处理编码器(本工程启用)
//#define TIMER3_HANDLES_ENCODER      // 中文: 由 TIM3 处理编码器(备用, 未启用)
//#define TIMER4_HANDLES_ENCODER      // 中文: 由 TIM4 处理编码器(备用, 未启用)
#endif  // ENCODER


#if defined(TIMER2_HANDLES_ENCODER)
#define ENCODER_TIMER         TIM2          // Encoder unit connected to TIM2
                                            // 中文: 编码器接口定时器 = TIM2
#elif defined(TIMER3_HANDLES_ENCODER)
#define ENCODER_TIMER         TIM3          // Encoder unit connected to TIM3
                                            // 中文: 编码器接口定时器 = TIM3
#else // TIMER4_HANDLES_ENCODER
#define ENCODER_TIMER         TIM4          // Encoder unit connected to TIM4
                                            // 中文: 编码器接口定时器 = TIM4
#endif

/*****************************  Encoder settings ******************************/
#define ENCODER_PPR           (u16)(400)   // number of pulses per revolution
                                           // 中文: 编码器每转脉冲数(线数)PPR = 400。
                                           //       正交解码 4 倍频后，定时器每转计数 = 4*PPR = 1600。

/* Define here the absolute value of the application minimum and maximum speed 
                                                                   in rpm unit*/
// 中文: 应用允许的机械转速绝对值上下限，单位 rpm(转/分)。
#define MINIMUM_MECHANICAL_SPEED_RPM  (u32)60   //rpm
                                                // 中文: 机械转速下限 = 60 rpm
#define MAXIMUM_MECHANICAL_SPEED_RPM  (u32)36000 //rpm
                                                 // 中文: 机械转速上限 = 36000 rpm

/* Define here the number of consecutive error measurement to be detected 
   before going into FAULT state */
// 中文: 进入 FAULT(故障)状态前，需要连续检测到的速度测量错误次数。
#define MAXIMUM_ERROR_NUMBER (u8)25
                             // 中文: 连续 25 次速度测量异常即判定反馈故障
/* Computation Parameter*/
//Number of averaged speed measurement
// 中文: 速度平均所用的样本数(即平均缓冲的深度)。
#define SPEED_BUFFER_SIZE   8   // power of 2 required to ease computations
                                // 中文: 取 8(2 的幂，便于计算)，即对最近 8 次速度求平均

/*************************** Alignment settings *******************************/
//Alignemnt duration
// 中文: 启动对齐(Alignment)阶段的持续时间，单位 ms(毫秒)。
#define T_ALIGNMENT           (u16) 700    // Alignment time in ms
                                           // 中文: 对齐时长 = 700 ms

#define ALIGNMENT_ANGLE       (u16) 90 //Degrees [0..359]  
                                       // 中文: 对齐时施加的电角度，单位为「度」，范围 [0..359]
//  90?<-> Ia = I_ALIGNMENT, Ib = Ic =-I_ALIGNMENT/2) 
// 中文: 磁场对准到 90 度电角度时的相电流关系: Ia = I_ALIGNMENT，
//       Ib = Ic = -I_ALIGNMENT/2。

// With MB459 and ALIGNMENT_ANGLE equal to 90?
//final alignment phase current = (I_ALIGNMENT * 0.64)/(32767 * Rshunt) 
// 中文: 使用 MB459 功率板且 ALIGNMENT_ANGLE = 90 度时，
//       对齐阶段最终相电流 = (I_ALIGNMENT * 0.64)/(32767 * Rshunt)。
#define I_ALIGNMENT           (u16) 22000  
                              // 中文: 对齐电流给定(Q15 定标，22000 表示幅值 22000/32767)

//Do not be modified
// 中文: 以下为派生/换算宏，请勿修改。
#define T_ALIGNMENT_PWM_STEPS     (u32) ((T_ALIGNMENT * SAMPLING_FREQ)/1000) 
                                  // 中文: 对齐阶段总 PWM 步数 = 对齐时长(ms)*采样频率(Hz)/1000
#define ALIGNMENT_ANGLE_S16       (s16)((s32)(ALIGNMENT_ANGLE) * 65536/360)
                                  // 中文: 对齐电角度换算为 s16 定标(360 度 = 65536)
#define MINIMUM_MECHANICAL_SPEED  (u16)(MINIMUM_MECHANICAL_SPEED_RPM/6)
                                  // 中文: 机械转速下限换算为 0.1Hz 定标(除以 6 即 /60*10)
#define MAXIMUM_MECHANICAL_SPEED  (u16)(MAXIMUM_MECHANICAL_SPEED_RPM/6)
                                  // 中文: 机械转速上限换算为 0.1Hz 定标(除以 6 即 /60*10)
#endif  /*__MC_ENCODER_PARAM_H*/
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
