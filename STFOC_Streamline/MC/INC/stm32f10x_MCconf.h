/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : STM32F10x_MCconf.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : Motor Control Library configuration file.
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
*   本文件是电机控制库的总配置开关文件。通过“定义/注释”下列宏来裁剪整机功能，
*   编译器据此选择：电流采样方式、转子位置/转速检测方式、FOC 高级算法（MTPA/
*   弱磁/前馈）、制动、PID 微分项、PID 在线整定、DAC 输出等。文件末尾还有一组
*   #if 配置合法性检查，配置冲突或缺失时直接 #error/#warning 报错。
*   本精简工程当前启用：THREE_SHUNT（三电阻采样）、ENCODER（编码器）、
*   DIFFERENTIAL_TERM_ENABLED（PID 微分项）；其余开关均被注释关闭。
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_MCCONF_H
#define __STM32F10x_MCCONF_H


/********************   Current sampling technique definition   ***************/
/*   Define here the technique utilized for sampling the three-phase current  */
/*   电流采样方式定义：三选一（ICS/三电阻/单电阻），由下方合法性检查保证。*/

  /* Current sensing by ICS (Isolated current sensors) */
//#define ICS_SENSORS               // 隔离电流传感器采样（本工程未启用）

  /* Current sensing by Three Shunt resistors */
#define THREE_SHUNT                  // 三电阻采样（本工程启用）

  /* Current sensing by Single Shunt resistor */
//#define SINGLE_SHUNT              // 单电阻采样（本工程未启用）

/*******************  Position sensing technique definition  ******************/
/* Define here the type of rotor position sensing utilized for both Field     */
/*                    Oriented Control and speed regulation                   */
/*   转子位置/转速检测方式定义：三选一（编码器/霍尔/无传感器）。*/

  /* Position sensing by quadrature encoder */
#define ENCODER                      // 正交编码器检测（本工程启用）

  /* Position sensing by Hall sensors */
//#define HALL_SENSORS              // 霍尔传感器检测（本工程未启用）

  /* Sensorless position sensing  */
//#define NO_SPEED_SENSORS          // 无位置传感器（反电动势观测器，本工程未启用）
  /* When in sensorless operation define here if you also want to acquire any */
  /* position sensor information                                              */
  //#define VIEW_HALL_FEEDBACK     // 无传感时仍读取霍尔信息供观察（仅无传感模式可用）
//#define VIEW_ENCODER_FEEDBACK     // 无传感时仍读取编码器信息供观察（仅无传感模式可用）
  
  /* When in sensorless operation define here if you want to perform an       */
  /* alignment before the ramp-up                                             */
//#define NO_SPEED_SENSORS_ALIGNMENT   // 无传感强拖前先做对齐/预定位（仅无传感模式可用）

/************************** FOC methods **************************************/
  /* Internal Permanent Magnet Motors Maximum-Torque-per-Ampere strategy */
//#define IPMSM_MTPA                // 内置式永磁电机 MTPA（最大转矩每安培）策略（未启用）

  /* Flux weakening operations allowed */
//#define FLUX_WEAKENING            // 允许弱磁运行（未启用）

  /* Feed forward current regulation based on known motor parameters */
//#define FEED_FORWARD_CURRENT_REGULATION   // 基于已知电机参数的电流环前馈解耦（未启用）

/**************************   Brake technique   *******************************/
/*      Define here the if you want to enable brake resistor management       */
/*          MANDATORY in case of operation in flux weakening region           */
  
  /* Uncomment to enable brake resistor management feature */
//#define BRAKE_RESISTOR            // 制动电阻管理（弱磁运行时必需，本工程未启用）

/********************    PIDs differential terms enabling  ********************/
/*        Define here if you want to enable PIDs differential terms           */

  /* Uncomment to enable differential terms of PIDs */           
#define DIFFERENTIAL_TERM_ENABLED   // 启用 PID 微分项（本工程启用）
  
/********************  PIDs gains tuning operations enabling  *****************/
/*  Define here if you want to tune currents (Id, Iq) PIDs, Luenberger State  */
/*                             Observer and PLL gains                         */                              

  /* Uncomment to enable the generation of a square-wave shaped reference Iq */
//#define FLUX_TORQUE_PIDs_TUNING   // 用方波 Iq 参考在线整定电流/速度 PID（未启用）
  
  /* Uncomment to enable the tuning of Luenberger State Observer and PLL gains*/ 
//#define OBSERVER_GAIN_TUNING      // 在线整定 Luenberger 观测器与 PLL 增益（未启用）

/***********************   DAC functionality enabling  ************************/

  /*Uncomment to enable DAC functionality feature through TIM3 output channels*/
//#define DAC_FUNCTIONALITY         // 通过 TIM3 通道输出 DAC 调试波形（未启用）

/******************************************************************************/
/* Check-up of the configuration validity*/
/* 以下为配置合法性检查：冲突或多选/漏选时编译期报错。 */
/* 电流采样方式不能同时选两种（ICS 与三电阻冲突）。*/
#if ( (defined(ICS_SENSORS)) && (defined(THREE_SHUNT)) )
#error "Invalid configuration: Two current sampling techniques selected"
#endif

/* 电流采样方式不能同时选两种（ICS 与单电阻冲突）。*/
#if ( (defined(ICS_SENSORS)) && (defined(SINGLE_SHUNT)) )
#error "Invalid configuration: Two current sampling techniques selected"
#endif

/* 电流采样方式不能同时选两种（单电阻与三电阻冲突）。*/
#if ( (defined(SINGLE_SHUNT)) && (defined(THREE_SHUNT)) )
#error "Invalid configuration: Two current sampling techniques selected"
#endif

/* 电流采样方式必须选一种，否则报错。*/
#if ( (!defined(ICS_SENSORS)) && (!defined(THREE_SHUNT)) && (!defined(SINGLE_SHUNT)) )
#error "Invalid setup: No sampling technique selected"
#endif

/* 位置检测方式不能同时选两种（编码器/霍尔/无传感之间互斥）。*/
#if ( (defined(ENCODER)) && (defined (HALL_SENSORS)) || (defined(ENCODER)) &&\
    (defined (NO_SPEED_SENSORS)) || (defined(NO_SPEED_SENSORS)) && (defined\
    (HALL_SENSORS)))
#error "Invalid configuration: Two position sensing techniques selected"
#endif

/* 位置检测方式必须选一种，否则报错。*/
#if ( (!defined(ENCODER)) && (!defined (HALL_SENSORS)) && (!defined\
                                                            (NO_SPEED_SENSORS)))
#error "Invalid configuration: No position sensing technique selected"
#endif

/* 观察霍尔反馈仅在无传感器模式下有效。*/
#if ((defined VIEW_HALL_FEEDBACK) && (!defined NO_SPEED_SENSORS))
#error "Invalid configuration: VIEW_HALL_FEEDBACK supported only in\
                                                           sensorless operation"
#endif

/* 观察编码器反馈仅在无传感器模式下有效。*/
#if ((defined VIEW_ENCODER_FEEDBACK) && (!defined NO_SPEED_SENSORS))
#error "Invalid configuration: VIEW_ENCODER_FEEDBACK supported only in\
                                                           sensorless operation"
#endif

/* FLUX_TORQUE_PIDs_TUNING（PID 在线整定）不支持无传感器运行。*/
#if ((defined FLUX_TORQUE_PIDs_TUNING) && (defined NO_SPEED_SENSORS))
#error "Invalid configuration: FLUX_TORQUE_PIDs_TUNING not supported in\
                                                           sensorless operation"
#endif

/* 不能同时观察霍尔与编码器两种反馈。*/
#if ((defined VIEW_HALL_FEEDBACK) && (defined VIEW_ENCODER_FEEDBACK))
#error "Invalid configuration: Two position sensing techniques selected"
#endif

/* 非无传感器模式下若误开了对齐功能，则告警并强制取消该宏。*/
#if ((!defined NO_SPEED_SENSORS) && (defined NO_SPEED_SENSORS_ALIGNMENT))
#warning "No speed sensors alignment has been disabled"
#undef NO_SPEED_SENSORS_ALIGNMENT
#endif

#endif /* __STM32F10x_MCCONF_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
