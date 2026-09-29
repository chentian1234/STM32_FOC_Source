/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_MClib.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file gathers the motor control header files which 
*                      are needed depending on configuration.
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
*   本文件是电机控制库的“总头文件”。它按配置开关(stm32f10x_MCconf.h)选择性地
*   包含所需的子模块头文件，并据此定义一组与硬件相关的抽象宏，使上层控制代码
*   无需关心底层是编码器/霍尔/无传感器、以及哪种电流采样方式：
*     GET_ELECTRICAL_ANGLE : 获取电角度(供 Park/反Park 变换使用)；
*     GET_SPEED_0_1HZ      : 获取转速反馈，单位 0.1Hz；
*     GET_SPEED_DPP        : 获取转速，单位 dpp(每 PWM 周期角度增量)；
*     GET_PHASE_CURRENTS   : 采样三相相电流；
*     CALC_SVPWM           : 由 αβ 电压计算三相 PWM 占空比。
*   注意：这三个 GET_* 宏对同一功能只能由一条分支定义，互斥由 MCconf.h 保证。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10xMCLIB_H
#define __STM32F10xMCLIB_H

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_MCconf.h"   // 配置开关（必须最先包含，决定后续分支）
#include "MC_type.h"            // FOC 自定义类型（Curr_Components 等）
#include "stm32f10x_lib.h"      // STM32 标准外设库

/* 霍尔传感器方案：位置/转速取自 stm32f10x_hall 模块 */
#if defined HALL_SENSORS 
#include "stm32f10x_hall.h"
#define GET_ELECTRICAL_ANGLE    HALL_GetElectricalAngle()   // 霍尔估算电角度
#define GET_SPEED_0_1HZ         HALL_GetSpeed()             // 霍尔估算转速(0.1Hz)
#define GET_SPEED_DPP           HALL_GetRotorFreq()         // 霍尔估算转子频率(dpp)
#endif

/* 编码器方案：位置/转速取自 stm32f10x_encoder 模块 */
#if defined ENCODER
#include "stm32f10x_encoder.h"
#define GET_ELECTRICAL_ANGLE    ENC_Get_Electrical_Angle()   // 编码器电角度
#define GET_SPEED_0_1HZ         ENC_Get_Mechanical_Speed()   // 编码器机械转速(0.1Hz)
#define GET_SPEED_DPP    (s16)((ENC_Get_Mechanical_Speed()*_0_1HZ_2_128DPP)/128)   // 机械转速→dpp(经 _0_1HZ_2_128DPP 换算，右移 7 位)
#endif

/* 无位置传感器方案：位置/转速取自反电动势状态观测器(STO) */
#if defined NO_SPEED_SENSORS 
#include "MC_State_Observer.h"
#include "MC_State_Observer_param.h"
#include "MC_State_Observer_interface.h"
#define GET_ELECTRICAL_ANGLE    STO_Get_Electrical_Angle()   // 观测器电角度
#define GET_SPEED_0_1HZ         STO_Get_Speed_Hz()           // 观测器转速(0.1Hz)
#define GET_SPEED_DPP           STO_Get_Speed()              // 观测器转速(dpp)
#endif

/* 观测器在线整定(OBSERVER_GAIN_TUNING)时也需包含观测器相关头文件 */
#if defined OBSERVER_GAIN_TUNING
#include "MC_State_Observer.h"
#include "MC_State_Observer_param.h"
#include "MC_State_Observer_interface.h"
#endif

/* 仅观察：无传感模式下额外读取霍尔反馈 */
#if defined VIEW_HALL_FEEDBACK
#include "stm32f10x_hall.h"
#endif

/* 仅观察：无传感模式下额外读取编码器反馈 */
#if defined VIEW_ENCODER_FEEDBACK
#include "stm32f10x_encoder.h"
#endif

/* 电流采样：隔离电流传感器(ICS)方案 */
#ifdef ICS_SENSORS
#include "stm32f10x_svpwm_ics.h"
#define GET_PHASE_CURRENTS SVPWM_IcsGetPhaseCurrentValues   // ICS 相电流采样
#define CALC_SVPWM SVPWM_IcsCalcDutyCycles                  // ICS 对应 SVPWM 计算
#endif

/* 电流采样：三电阻方案（本工程启用）*/
#ifdef THREE_SHUNT
#include "stm32f10x_svpwm_3shunt.h"
#define GET_PHASE_CURRENTS SVPWM_3ShuntGetPhaseCurrentValues   // 三电阻相电流采样
#define CALC_SVPWM SVPWM_3ShuntCalcDutyCycles                  // 三电阻对应 SVPWM 计算
#endif

/* 电流采样：单电阻方案 */
#ifdef SINGLE_SHUNT
#include "stm32f10x_svpwm_1shunt.h"
#define GET_PHASE_CURRENTS SVPWM_1ShuntGetPhaseCurrentValues   // 单电阻相电流采样
#define CALC_SVPWM SVPWM_1ShuntCalcDutyCycles                  // 单电阻对应 SVPWM 计算
#endif

/* DAC 调试输出功能 */
#ifdef DAC_FUNCTIONALITY
#include "stm32f10x_MCdac.h"
#endif

#include "MC_Clarke_Park.h"           // Clarke/Park 及反变换
#include "MC_FOC_Drive.h"             // FOC 电流环/速度环主体
#include "MC_FOC_Methods.h"           // FOC 高级方法(MTPA/弱磁等)
#include "MC_PID_regulators.h"        // PID 调节器
#include "MC_Control_Param.h"         // 控制参数(采样周期等)
#include "stm32f10x_Timebase.h"       // 500us 时基模块
#include "MC_Display.h"               // 显示模块
#include "MC_Keys.h"                  // 按键模块
#include "stm32f10x_lcd.h"            // LCD 驱动
#include "MC_PMSM_motor_param.h"      // 电机参数
#include "MC_MotorControl_Layer.h"    // 电机控制层(状态机/故障)

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __STM32F10xMCLIB_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
