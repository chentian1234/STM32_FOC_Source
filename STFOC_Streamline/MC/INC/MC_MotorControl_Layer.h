/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_MotorControl_Layer.h
* Author             : IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : Export of public functions of Motor control layer 
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
* 模块说明(中文) : 电机控制层（MCL）对外接口头文件（对应实现 MC_MotorControl_Layer.c）。
*                  提供电机启动/停止时的外设与状态初始化、功率级/母线电压/温度检测、
*                  故障置位与清除、PID 积分项复位，以及（可选）制动电阻控制等接口。
*                  它是 main.c 状态机与 FOC 算法之间的“逻辑外设管理层”。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_MOTORCONTROLLAYER_H
#define __MC_MOTORCONTROLLAYER_H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
//Not to be modified
/* 母线过压阈值（ADC 计数值，u16）：由实际阈值电压 OVERVOLTAGE_THRESHOLD_V 按母线
   采样分压比换算得到（32768 对应满量程 3.3V）。母线电压高于此值判 OVER_VOLT。*/
#define OVERVOLTAGE_THRESHOLD  (u16)(OVERVOLTAGE_THRESHOLD_V*\
                                                 (BUS_ADC_CONV_RATIO*32768/3.3))
/* 母线欠压阈值（ADC 计数值，u16）：由实际阈值电压 UNDERVOLTAGE_THRESHOLD_V 换算。
   母线电压低于此值判 UNDER_VOLT。*/
#define UNDERVOLTAGE_THRESHOLD (u16)(UNDERVOLTAGE_THRESHOLD_V*\
                                                 (BUS_ADC_CONV_RATIO*32768/3.3))
/* Exported variables --------------------------------------------------------*/

/* 母线电压 ADC 原始采样值（u16，由 ADC 中断写入）。*/
extern u16 h_ADCBusvolt;
/* 功率级温度 NTC 的 ADC 原始采样值（u16，由 ADC 中断写入）。*/
extern u16 h_ADCTemp;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* 电机启动初始化：复位 PID 积分、初始化 FOC 与位置/电流采样外设，并开 PWM 输出。*/
void MCL_Init(void);
/* 功率级检查：过温/欠压检测，异常则置相应故障（过压由模拟看门狗处理）。*/
void MCL_ChkPowerStage(void);
/* 检查故障源是否已消失；对按键 SEL 请求清除，并返回是否可清除/恢复。*/
bool MCL_ClearFault(void);
/* 置位指定故障：关 PWM、记录故障标志、进入 FAULT 状态与故障菜单。*/
void MCL_SetFault(u16);
/* 过温检测（带迟滞），返回 TRUE 表示过温。*/
bool MCL_Chk_OverTemp(void);
/* 母线电压检查：返回 OVER_VOLT / UNDER_VOLT / NO_FAULT。*/
BusV_t MCL_Chk_BusVolt(void);
/* 计算母线电压，单位 V（u16）。*/
u16 MCL_Compute_BusVolt(void);
/* 计算功率级温度，单位 摄氏度（u8，含 14 偏移）。*/
u8 MCL_Compute_Temp(void);
/* 更新母线电压滑动平均值（在 ADC 中断或周期任务中调用）。*/
void MCL_Calc_BusVolt(void);
/* 返回母线电压的 s16 内部定标值（供观测器等使用）。*/
s16 MCL_Get_BusVolt(void);
/* 复位母线电压/温度平均值数组，避免复位后误报故障。*/
void MCL_Init_Arrays(void);
#ifdef BRAKE_RESISTOR
/* 初始化制动电阻开关 GPIO。*/
void MCL_Brake_Init(void);
/* 打开制动电阻（置位 GPIO）。*/
void MCL_Set_Brake_On(void);
/* 关闭制动电阻（复位 GPIO）。*/
void MCL_Set_Brake_Off(void);
#endif
#endif //__MC_MOTORCONTROLLAYER_H
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
