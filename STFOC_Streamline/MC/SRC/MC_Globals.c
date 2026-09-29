/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Globals.c
* Author             : IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : This file contains the declarations of the global 
*                      variables utilized by the motor control library
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
* 模块说明(中文) : 全局变量池定义文件。集中定义 FOC 各模块共享的电气/磁/机械量、
*                  系统状态机状态、全局标志字与三个 PID 调节器实例及其参考给定。
*                  这些变量在电流环（Clarke/Park/反Park/SVPWM）与速度环之间传递，
*                  因此具有全局可见性（extern 声明见 MC_Globals.h）。
*                  定标约定：电流/电压等 s16 量一般为 Q15（±32767 对应 ±满量程）；
*                  角度类量以 65536 = 360° 电角度定标；转速以 0.1Hz（Hz×10）定标。
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MCconf.h"
#include "MC_const.h"
#include "MC_type.h"
#include "MC_Globals.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Electrical, magnetic and mechanical variables*/

/* 定子三相电流 Ia、Ib（s16 Q15 标幺，已由 ADC 换算）。*/
Curr_Components Stat_Curr_a_b;              /*Stator currents Ia,Ib*/ 

/* Clarke 变换结果：Ialpha、Ibeta（静止 αβ 坐标系，s16 Q15）。*/
Curr_Components Stat_Curr_alfa_beta;        /*Ialpha & Ibeta, Clarke's  
                                            transformations of Ia & Ib */

/* Park 变换结果：Iq、Id（随转子磁链同步旋转的 dq 坐标系，s16 Q15）。*/
Curr_Components Stat_Curr_q_d;              /*Iq & Id, Parke's transformations of 
                                            Ialpha & Ibeta, */

/* 定子三相电压 Va、Vb（s16 Q15）。*/
Volt_Components Stat_Volt_a_b;              /*Stator voltages Va, Vb*/ 

/* 电流环输出：Vq、Vd（与转子磁链同步旋转坐标系下的电压给定，s16 Q15）。*/
Volt_Components Stat_Volt_q_d;              /*Vq & Vd, voltages on a reference
                                            frame synchronous with the rotor flux*/

/* 反 Park 变换结果：Valpha、Vbeta（静止 αβ 坐标系电压，送入 SVPWM，s16 Q15）。*/
Volt_Components Stat_Volt_alfa_beta;        /*Valpha & Vbeta, RevPark transformations
                                             of Vq & Vd*/

/*Variable of convenience*/

/* 全局标志字（位掩码）：FIRST_START/SPEED_CONTROL/START_UP_FAILURE/SPEED_FEEDBACK/
   BRAKE_ON 及故障位 OVERHEAT/OVER_CURRENT/OVER_VOLTAGE/UNDER_VOLTAGE 等。*/
#ifdef FLUX_TORQUE_PIDs_TUNING
volatile u32 wGlobal_Flags = FIRST_START;
#else
volatile u32 wGlobal_Flags = FIRST_START | SPEED_CONTROL;
#endif

/* 系统状态机状态（IDLE/INIT/START/RUN/STOP/BRAKE/WAIT/FAULT）。*/
volatile SystStatus_t State;

/* 磁链(d 轴电流) PID 调节器参数与状态。*/
PID_Struct_t PID_Flux_InitStructure;
/* 磁链(d 轴)电流参考给定，s16 Q15。*/
volatile s16 hFlux_Reference;

/* 转矩(q 轴电流) PID 调节器参数与状态。*/
PID_Struct_t PID_Torque_InitStructure;
/* 转矩(q 轴)电流参考给定，s16 Q15。*/
volatile s16 hTorque_Reference;

/* 速度 PID 调节器参数与状态（其输出作为 q 轴电流参考）。*/
PID_Struct_t   PID_Speed_InitStructure;
/* 速度参考给定，定标 0.1Hz（Hz×10），s16。*/
volatile s16 hSpeed_Reference;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
