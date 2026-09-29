/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_type.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This header file provides structure type definitions that 
*                      are used throughout this motor control library.
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
*   本文件是 FOC 电机控制库的公共数据类型定义头文件，集中定义各模块之间传递的
*   数据结构：电流/电压正交分量、三角函数值、状态观测器常数、MTPA 常数、PI(D)
*   调节器参数结构体，以及系统状态枚举与母线电压状态枚举。
*   在 FOC 控制中的角色是提供统一的“类型契约”:
*   Clarke 变换(三相→两相静止 αβ)输出 Curr_Components；
*   Park 变换(αβ→旋转 dq)输入输出均为 Curr_Components；
*   反 Park 变换输出 Volt_Components；
*   Trig_Functions 输出 Trig_Components(正弦/余弦)；
*   PID_Regulator 使用 PID_Struct_t。
*   本文件被 MC_Clarke_Park、MC_PID_regulators、MC_FOC_Drive 等模块以及全局
*   变量头 MC_Globals.h 引用。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_TYPE_H
#define __MC_TYPE_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* 电流分量结构体 Curr_Components：一对正交电流分量。
   用途：Clarke 变换(三相→两相静止 αβ 坐标系)的输出(α、β)，
         或 Park 变换(αβ→同步旋转 dq 坐标系)的输出(q、d 轴电流)。
   定标：s16，Q1.15 格式(32767 对应电流满量程)，单位为按量程归一化的电流值。 */
typedef struct 
{
  s16 qI_Component1;    // 第一分量：α 轴或 q 轴电流，s16 / Q1.15
  s16 qI_Component2;    // 第二分量：β 轴或 d 轴电流，s16 / Q1.15
} Curr_Components;

/* 电压分量结构体 Volt_Components：一对正交电压分量。
   用途：电流环 PI 输出并经电压圆限制后的旋转 dq 电压(q、d)，
         或反 Park 变换(旋转 dq→静止 αβ)输出的静止 αβ 电压。
   定标：s16，Q1.15 格式，单位为与母线电压相关的归一化电压值。 */
typedef struct 
{
  s16 qV_Component1;    // 第一分量：q 轴或 α 轴电压，s16 / Q1.15
  s16 qV_Component2;    // 第二分量：d 轴或 β 轴电压，s16 / Q1.15
} Volt_Components;

/* 三角函数结构体 Trig_Components：保存某一电角度对应的正弦、余弦值。
   由 Trig_Functions() 通过查表(hSin_Cos_Table)得到，供 Park / 反 Park 使用。
   定标：s16，Q1.15 格式，取值范围 [-32767, 32767] 对应 [-1, 1]。 */
typedef struct
{
  s16 hCos;    // 电角度余弦 cos(theta)，s16 / Q1.15，范围 [-32767,32767]
  s16 hSin;    // 电角度正弦 sin(theta)，s16 / Q1.15，范围 [-32767,32767]
} Trig_Components;

typedef struct
{
 s16 hC1;                  // 观测器反馈常数 C1，s16
 s16 hC2;                  // 观测器反馈常数 C2，s16
 s16 hC3;                  // 观测器反馈常数 C3，s16
 s16 hC4;                  // 观测器反馈常数 C4，s16
 s16 hC5;                  // 观测器反馈常数 C5，s16
 s16 hC6;                  // 观测器反馈常数 C6，s16
 s16 hF1;                  // 观测器滤波常数 F1，s16
 s16 hF2;                  // 观测器滤波常数 F2，s16
 s16 hF3;                  // 观测器滤波常数 F3，s16
 s16 PLL_P;                // 锁相环(PLL)比例增益，s16
 s16 PLL_I;                // 锁相环(PLL)积分增益，s16
 s32 wMotorMaxSpeed_dpp;   // 电机最大转速数字量(每采样周期增量 dpp)，s32
 u16 hPercentageFactor;    // 百分比系数(0~32767 对应 0~100%)，u16
} StateObserver_Const; 

typedef struct
{
  s16 PLL_P;   // 锁相环比例增益更新量，s16
  s16 PLL_I;   // 锁相环积分增益更新量，s16
  s16 hC2;     // 观测器反馈常数 C2 更新量，s16
  s16 hC4;     // 观测器反馈常数 C4 更新量，s16
} StateObserver_GainsUpdate;

/* MTPA(最大转矩电流比控制)分段线性插值常数结构体：
   描述 d 轴电流参考-转矩角映射的分段直线近似参数。 */
typedef struct
{
 s16 hsegdiv;      // 角度分段除数(用于确定所属插值区间)，s16
 s32 wangc[8];     // 8 段角度系数(每段直线斜率)，s32
 s32 wofst[8];     // 8 段角度偏移(每段直线偏置)，s32
} MTPA_Const;

typedef struct 
{  
  s16 hKp_Gain;                     // 比例增益分子 Kp，s16
  u16 hKp_Divisor;                  // 比例项定标除数：比例项输出 = Kp*误差 / 本值，u16
  s16 hKi_Gain;                     // 积分增益分子 Ki，s16
  u16 hKi_Divisor;                  // 积分项定标除数：积分项输出 = 积分累加值 / 本值，u16
  s16 hLower_Limit_Output;     //Lower Limit for Output limitation   // 输出下限(饱和限制)，s16
  s16 hUpper_Limit_Output;     //Lower Limit for Output limitation   // 输出上限(饱和限制)，s16
  s32 wLower_Limit_Integral;   //Lower Limit for Integral term limitation  // 积分项下限(抗积分饱和 anti-windup)，s32
  s32 wUpper_Limit_Integral;   //Lower Limit for Integral term limitation  // 积分项上限(抗积分饱和 anti-windup)，s32
  s32 wIntegral;                    // 积分项累加值(Ki*误差 的逐周期累加)，s32
  // Actually used only if DIFFERENTIAL_TERM_ENABLED is enabled in
  //stm32f10x_MCconf.h
  s16 hKd_Gain;                     // 微分增益分子 Kd，s16
  u16 hKd_Divisor;                  // 微分项定标除数：微分项输出 = Kd*误差差分 / 本值，u16
  s32 wPreviousError;               // 上一次的误差值(用于计算误差差分)，s32
} PID_Struct_t;

/* 系统状态枚举 SystStatus_t：电机控制系统状态机的状态取值。 */
typedef enum 
{
IDLE, INIT, START, RUN, STOP, BRAKE, WAIT, FAULT   // 空闲/初始化/启动/运行/停机/制动/等待/故障
} SystStatus_t;

/* 母线电压状态枚举 BusV_t：直流母线电压监测结果。 */
typedef enum 
{
NO_FAULT, OVER_VOLT, UNDER_VOLT   // 正常/过压(>阈值)/欠压(<阈值)
} BusV_t;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MC_TYPE_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
