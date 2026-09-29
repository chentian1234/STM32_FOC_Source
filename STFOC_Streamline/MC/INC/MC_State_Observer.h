/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_State_Observer.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : Contains the prototypes of the PMSM Observer 
*                      related functions (module MC_State_Observer.c)
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
* 模块说明(中文) : PMSM 反电动势（B-EMF）状态观测器核心头文件。
*                  本模块声明底层观测器（MC_State_Observer.c）的对外函数：以电机
*                  模型为基础，用定子电压/电流估算反电动势，再由 PLL（锁相环）
*                  解算出转子电角度与转速。向上层（FOC 电流环/速度环）提供估计
*                  角度、估计转速与可信度标志。
*                  术语：
*                   - 反电动势观测器：由电压电流推算转子磁链位置的模型算法。
*                   - PLL 锁相环：由估计反电动势的相位误差经比例积分(PLL_P/PLL_I)
*                     校正，输出跟踪转子角度与频率的闭环结构。
*                   - Luenberger 观测器：一种用状态误差反馈校正的线性观测器结构。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_STATE_OBSERVER_H
#define __MC_STATE_OBSERVER_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* 观测器主计算：输入静止坐标系电压(Valfa,Vbeta)、电流(Ialfa,Ibeta)与母线电压，
   按 PWM 周期执行，估计反电动势并更新内部角度/转速状态及中间量。*/
void STO_Calc_Rotor_Angle(Volt_Components,Curr_Components,s16);
/* 由当前估计角度求差分得到估计转速（dpp 定标，见 _0_1HZ_2_128DPP），按速度采样周期调用。*/
void STO_Calc_Speed(void);
/* 返回估计的转子电角度，s16，定标 65536 = 360° 电角度；供 Park/反Park 使用。*/
s16 STO_Get_Electrical_Angle(void);
/* 返回估计转速，s16，定标为 dpp（每 PWM 周期的角度增量），供内部换算使用。*/
s16 STO_Get_Speed(void);
/* 返回转速估计是否可信（用于启动收敛判定与反馈丢失保护）。*/
bool STO_IsSpeed_Reliable(void);
/* 更新观测器与 PLL 增益（用户界面改参后调用）。*/
void STO_Gains_Update(StateObserver_GainsUpdate*);
/* 观测器复位初始化（清空内部积分/状态量）。*/
void STO_Init(void);
/* 初始化内部转速缓冲区（可信度/方差统计用）。*/
void STO_InitSpeedBuffer(void);
/* 用常数结构体初始化观测器系数与 PLL 增益（每次启动前调用）。*/
void STO_Gains_Init(StateObserver_Const*);
/* 返回估计的 alpha 轴定子电流（用于调试/上位观测），s16 Q15。*/
s16 STO_Get_wIalfa_est(void);
/* 返回估计的 beta 轴定子电流（用于调试/上位观测），s16 Q15。*/
s16 STO_Get_wIbeta_est(void);
/* 返回估计的 alpha 轴反电动势（用于调试/上位观测），s16 Q15。*/
s16 STO_Get_wBemf_alfa_est(void);
/* 返回估计的 beta 轴反电动势（用于调试/上位观测），s16 Q15。*/
s16 STO_Get_wBemf_beta_est(void);
#endif 
/* __MC_STATE_OBSERVER_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
