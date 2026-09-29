/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Globals.h
* Author             : IMS Systems Lab  
* Date First Issued  : 21/11/07
* Description        : This file contains the declarations of the exported 
*                      variables of module "MC_globals.c".
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
* 模块说明(中文) : 全局变量池对外声明头文件（对应定义 MC_Globals.c）。
*                  声明 FOC 各模块共享的电气/磁/机械量、系统状态机状态、全局标志字
*                  与三个 PID 调节器实例及参考给定。
*                  定标约定：电流/电压等 s16 量一般为 Q15；角度以 65536=360°电角度；
*                  转速以 0.1Hz（Hz×10）定标。
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_GLOBALS_H
#define __MC_GLOBALS_H
/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_lib.h"
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/*Electrical, magnetic and mechanical variables*/

/* 定子三相电流 Ia、Ib（s16 Q15 标幺）。*/
extern Curr_Components Stat_Curr_a_b;              /*Stator currents Ia,Ib*/ 

/* Clarke 变换结果：Ialpha、Ibeta（静止 αβ 坐标系，s16 Q15）。*/
extern Curr_Components Stat_Curr_alfa_beta;        /*Ialpha & Ibeta, Clarke's  
                                                  transformations of Ia & Ib */

/* Park 变换结果：Iq、Id（随转子磁链同步旋转的 dq 坐标系，s16 Q15）。*/
extern Curr_Components Stat_Curr_q_d;         /*Iq & Id, Parke's transformations
                                                of Ialpha & Ibeta, */

/* 定子三相电压 Va、Vb（s16 Q15）。*/
extern Volt_Components Stat_Volt_a_b;              /*Stator voltages Va, Vb*/ 

/* 电流环输出：Vq、Vd（同步旋转坐标系电压给定，s16 Q15）。*/
extern Volt_Components Stat_Volt_q_d;         /*Vq & Vd, voltages on a reference
                                          frame synchronous with the rotor flux*/

/* 反 Park 变换结果：Valpha、Vbeta（静止 αβ 坐标系，送入 SVPWM，s16 Q15）。*/
extern Volt_Components Stat_Volt_alfa_beta;       /*Valpha & Vbeta, RevPark
                                                    transformations of Vq & Vd*/

/*Variable of convenience*/

/* 全局标志字（位掩码）：控制方式与各故障/状态位，详见 MC_const.h。*/
extern volatile u32 wGlobal_Flags;

/* 系统状态机状态（IDLE/INIT/START/RUN/STOP/BRAKE/WAIT/FAULT）。*/
extern volatile SystStatus_t State;

/* 转矩(q 轴电流) PID 调节器参数与状态。*/
extern PID_Struct_t       PID_Torque_InitStructure;
/* 磁链(d 轴电流) PID 调节器参数与状态。*/
extern PID_Struct_t       PID_Flux_InitStructure;
/* 速度 PID 调节器参数与状态。*/
extern PID_Struct_t       PID_Speed_InitStructure;

/* 磁链(d 轴)电流参考给定，s16 Q15。*/
extern volatile s16 hFlux_Reference;
/* 转矩(q 轴)电流参考给定，s16 Q15。*/
extern volatile s16 hTorque_Reference;
/* 速度参考给定，定标 0.1Hz（Hz×10），s16。*/
extern volatile s16 hSpeed_Reference;
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#endif /* __MC_GLOBALS_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
