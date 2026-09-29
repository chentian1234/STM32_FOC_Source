/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name  	     : MC_FOC_Methods.h
* Author             : IMS Systems Lab 
* Date First Issued  : 29/05/08
* Description        : Contains the prototypes for the PMSM FOC methods module 
*                      related functions.
********************************************************************************
* History:
* 29/05/08 v2.0
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
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
*   本文件声明 PMSM FOC 高级控制方法的接口，包括:
*   - 磁链弱磁/磁链调节(FOC_FluxRegulator 系列)；
*   - MTPA(最大转矩电流比)查表给定(FOC_MTPA 系列)；
*   - 电流环前馈解耦(FOC_FF_CurrReg 系列)。
*   这些函数在需要相应功能时由 FOC 主流程调用；本精简工程中若未使能对应宏，
*   则仅保留声明。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_FOC_METHODS_H
#define __MC_FOC_METHODS_H 

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/* 磁链调节器主计算:输入测量电流/电压与电角度(theta)，输出更新的 dq 电流参考。 */
Curr_Components FOC_FluxRegulator (Curr_Components,Volt_Components,s16);
/* 磁链调节器初始化:绑定 PID 参数结构与初始 d 轴电流参考。 */
void FOC_FluxRegulator_Init(PID_Struct_t *, s16);
/* 磁链调节器刷新:根据电压/电流在线更新磁链参考，返回新的 d 轴电流参考。 */
s16 FOC_FluxRegulator_Update(s16, s16);
/* MTPA(最大转矩电流比):由 q 轴电流参考计算相应的 d 轴电流参考。 */
s16 FOC_MTPA(s16 hIqRef);
/* MTPA 初始化:装入分段插值常数与退磁电流限值。 */
void FOC_MTPA_Init(MTPA_Const MTPA_InitStructure_in, s16 hIdDemag_in);
/* 电流环前馈解耦初始化:装入三个前馈常数。 */
void FOC_FF_CurrReg_Init(s32 wConstant1Q, s32 wConstant1D, s32 wConstant2);
/* 电流环前馈解耦:根据电流、电压、转速与母线电压计算前馈补偿后的 dq 电压。 */
Volt_Components FOC_FF_CurrReg(Curr_Components, Volt_Components, s16 speed,s16 vbus);

#endif /* MC_FOC_METHODS_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
