/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_hall.h
* Author             : IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : Contains the prototypes of exported functions for hall 
*                      sensor feedback processing.
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
 *   本文件是霍尔(Hall)传感器反馈模块的对外接口头文件。它声明了霍尔反馈
 *   处理函数（定时器初始化、转速/电角度获取、测量初始化、超时查询等），
 *   供 FOC(磁场定向控制)上层软件调用。
 *   霍尔反馈原理: 电机内嵌 3 个霍尔开关，按 120 度或 60 度电角度排布，
 *   每个电周期产生 6 个离散状态(扇区)；模块通过定时器的输入捕获(IC)测量
 *   相邻状态跳变的时间间隔，据此得到转子转速，并结合跳变序列确定转向与
 *   转子电角度。相关可调参数见 MC_hall_prm.h。
 * ========================================================================== */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HALL_H
#define __HALL_H

/////////////////////// PWM Peripheral Input clock ////////////////////////////
// 中文: CKTIM 为定时器(PWM/捕获)外设的输入时钟频率，单位 Hz。
//       本芯片运行在 72MHz，故 CKTIM = 72 000 000，对应计数分辨率为 1Hz。
//       霍尔测速公式 Frotor = K x (Fosc/(Capture x overflow)) 中的 Fosc 即取此值。
#define CKTIM	((u32)72000000uL) 	/* Silicon running at 72MHz Resolution: 1Hz */

/* Includes ------------------------------------------------------------------*/
#include "MC_hall_prm.h" 

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void HALL_HallTimerInit(void);         // 中文: 初始化承载霍尔信号的定时器与 GPIO/中断
s16  HALL_GetRotorFreq (void);         // 中文: 获取转子机械频率(rad/每 PWM 周期定标, s16)
s16  HALL_GetSpeed (void);             // 中文: 获取转子机械转速(0.1Hz 定标, s16)
void HALL_InitHallMeasure(void);       // 中文: 启动前清空速度 FIFO, 初始化测量流程
bool HALL_IsTimedOut(void);            // 中文: 查询霍尔信号是否超时/丢失(TRUE = 超时)
s16 HALL_GetElectricalAngle(void);     // 中文: 获取最新电角度(s16 定标, 360 度 = 65536)
void HALL_IncElectricalAngle(void);    // 中文: 每个 FOC 周期按转速积分更新电角度
void HALL_Init_Electrical_Angle(void); // 中文: 依据三路霍尔电平初始化转子电角度
void HALL_ClrTimeOut(void);            // 中文: 清除霍尔超时标志

#endif /* __HALL_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
