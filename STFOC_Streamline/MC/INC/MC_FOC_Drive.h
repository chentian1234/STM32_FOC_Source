/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_FOC_Drive.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : Contains the prototypes for the PMSM FOC-drive module 
*                      related functions.
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
*   本文件声明 PMSM FOC 主驱动接口。在 FOC 控制中的角色是:
*   - FOC_Init:初始化 FOC 内部状态与 PID；
*   - FOC_Model:FOC 电流环主体(Clarke/Park→电流 PI→圆限制→反Park→SVPWM)；
*   - FOC_CalcFluxTorqueRef:速度环计算转矩/磁链电流参考；
*   - FOC_TorqueCtrl:转矩控制与接口初始化等。
*   被主控制流程(状态机/中断)调用。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_FOC_DRIVE_H
#define __MC_FOC_DRIVE_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
/* FOC_Init:初始化 FOC 模块内部状态(如 dq 电流参考清零)。 */
void FOC_Init(void);
/* FOC_Model:FOC 电流环每个 PWM 周期执行一次的主计算。 */
void FOC_Model(void);
/* FOC_CalcFluxTorqueRef:速度环计算并输出转矩/磁链(dq)电流参考。 */
void FOC_CalcFluxTorqueRef(void);
/* FOC_TorqueCtrl:转矩控制(开环/闭环转矩给定处理)。 */
void FOC_TorqueCtrl(void);
/* FOC_MTPAInterface_Init:初始化 MTPA 接口相关参数。 */
void FOC_MTPAInterface_Init(void);
/* FOC_FluxRegulatorInterface_Init:初始化磁链调节器接口相关参数。 */
void FOC_FluxRegulatorInterface_Init(void);

#endif /* __MC_FOC_DRIVE_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
