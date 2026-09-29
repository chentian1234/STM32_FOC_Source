/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_State_Observer_Interface.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : Contains the prototypes of the PMSM State Observer 
*                      related functions (module MC_State_Observer_Interface.c)
*
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
* 模块说明(中文) : 无位置传感器（Sensorless）状态观测器对外接口头文件。
*                  本模块对应实现文件 MC_State_Observer_Interface.c，位于 FOC
*                  控制链路的“转子位置/转速估计”环节：它把电机参数与观测器
*                  常数打包交给底层观测器（MC_State_Observer.c），对外提供
*                  转子电角度、机械角度、转速(Hz)、启动流程与转速可信度等接口，
*                  供 FOC 电流环（Park/反Park）与速度环使用。
*                  术语：反电动势观测器（B-EMF Observer）= 通过定子电压电流
*                  估算反电动势，从而推算出转子磁链位置与转速的算法。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_STATE_OBSERVER_INTERFACE_H
#define __MC_STATE_OBSERVER_INTERFACE_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* 获取转子机械角度。返回 s16，定标：65536 计数 = 360° 机械角；调用者通常用于显示。*/
s16 STO_Get_Mechanical_Angle(void);
/* 获取电机机械转速，单位 = 0.1Hz（即返回 Hz 值 ×10）；供速度环反馈使用。*/
s16 STO_Get_Speed_Hz(void);
/* 无传感器启动流程状态机（S_INIT/ALIGNMENT/RAMP_UP），按 PWM 周期在中断中调用；
   完成对齐、开环强拖，并在观测器收敛后把 State 切换为 RUN。*/
void STO_Start_Up(void);
/* 检查转速估计的可信度连续性：若连续 RELIABILITY_HYSTERESYS 次均不可信则返回 FALSE。*/
bool STO_Check_Speed_Reliability(void);
/* 观测器接口初始化：填充并下发观测器常数结构体，每次电机启动初始化时调用一次。*/
void STO_StateObserverInterface_Init(void);
#ifdef OBSERVER_GAIN_TUNING
/* 当用户通过上位界面修改观测器/PLL 增益后，更新观测器运行增益（仅增益整定模式编译）。*/
void STO_Obs_Gains_Update(void);
#endif
#endif 
/* __MC_STATE_OBSERVER_INTERFACE_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
