/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_PMSM_motor_param.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file contains the PM motor parameters.
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 14/07/08 v2.0.1
* 28/08/08 v2.0.2
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
*   本文件定义所驱动 PMSM(永磁同步电机)的物理参数与控制相关派生常数，
*   包括极对数、定子电阻/电感、额定电流、最高转速、电压常数等，以及由它们
*   计算出的 BEMF 上限、转速上限、转速定标系数等宏。被电流/速度控制相关模块引用。
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_PARAM_H
#define __MOTOR_PARAM_H

// Number of motor pair of poles
#define	POLE_PAIR_NUM 	(u8) 2        /* Number of motor pole pairs */   // 电机极对数(用于电角度=机械角度×极对数)
#define RS               0.35            /* Stator resistance , ohm*/   // 定子相电阻，单位 Ω
#define LS               0.0006        /* Stator inductance , H */   // 定子相电感，单位 H

// When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the 
// PID for speed regulation (i.e. reference torque). 
// Whit MB459 board, the value must be calculated accordingly with formula:
// NOMINAL_CURRENT = (Nominal phase current (A, 0-to-peak)*32767* Rshunt) /0.64

#define NOMINAL_CURRENT             (s16)23727  //motor nominal current (0-pk)   // 额定相电流(0~峰值)，s16/Q1.15 定标
#define MOTOR_MAX_SPEED_RPM         (u32)3600   //maximum speed required          // 电机最高机械转速，单位 rpm
#define MOTOR_VOLTAGE_CONSTANT      4   //Volts RMS ph-ph /kRPM   // 电压常数:线电压有效值(V)/每千转
//Demagnetization current
#define ID_DEMAG    -NOMINAL_CURRENT   // 退磁限制电流(d 轴负向最大)，取额定电流负值

#ifdef IPMSM_MTPA
//MTPA parameters, to be defined only for IPMSM and if MTPA control is chosen
#define IQMAX (s16)(23687)   // q 轴电流上限(MTPA 时限制转矩电流)，s16
#define SEGDIV (s16)(2921)   // MTPA 分段角度除数
#define ANGC {-1412,-2572,-4576,-5200,-5564,-10551,-12664,-15567}   // MTPA 8 段角系数(斜率)
#define OFST {0,105,463,632,764,3012,4162,5997}   // MTPA 8 段角偏置
#else
#define IQMAX NOMINAL_CURRENT   // 非 MTPA 时 q 轴电流上限=额定电流
#endif

#ifdef FLUX_WEAKENING
#define FW_VOLTAGE_REF (s16)(985)   //Vs reference, tenth of a percent   // 弱磁电压参考(电压标么值，0.1% 单位)
#define FW_KP_GAIN (s16)(3000)      //proportional gain of flux weakening ctrl   // 弱磁控制比例增益
#define FW_KI_GAIN (s16)(5000)      //integral gain of flux weakening ctrl   // 弱磁控制积分增益
#define FW_KPDIV ((u16)(32768))     //flux weak ctrl P gain scaling factor   // 弱磁比例增益定标除数
#define FW_KIDIV ((u16)(32768))     //flux weak ctrl I gain scaling factor   // 弱磁积分增益定标除数
#endif

#ifdef FEED_FORWARD_CURRENT_REGULATION
#define CONSTANT1_Q (s32)(6215)    // 电流环前馈解耦常数(q 轴)
#define CONSTANT1_D (s32)(6215)    // 电流环前馈解耦常数(d 轴)
#define CONSTANT2 (s32)(6962)      // 电流环前馈解耦第二常数
#endif

/*not to be modified*/
/* 由电机参数派生的反电动势上限:MAX_BEMF_VOLTAGE = 最高转速×电压常数×sqrt(2)/(1000×sqrt(3))，
   即把线电压有效值折算为相电压峰值，单位 V。 */
#define MAX_BEMF_VOLTAGE  (u16)((MOTOR_MAX_SPEED_RPM*\
                           MOTOR_VOLTAGE_CONSTANT*SQRT_2)/(1000*SQRT_3))

#define MOTOR_MAX_SPEED_HZ (s16)((MOTOR_MAX_SPEED_RPM*10)/60)   // 最高转速换算为 0.1Hz 单位(电频率相关)

#define _0_1HZ_2_128DPP (u16)((POLE_PAIR_NUM*65536*128)/(SAMPLING_FREQ*10))   // 0.1Hz 转速→每采样周期数字增量的换算系数

#endif /*__MC_PMSM_MOTOR_PARAM_H*/
/**************** (c) 2008  STMicroelectronics ********************************/
