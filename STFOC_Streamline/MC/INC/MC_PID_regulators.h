/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_PID_regulators.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : Contains the prototypes of PI(D) related functions.
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
* 文件说明(中文):
*   本文件声明 PI(D) 调节器的对外接口。在 FOC 中用于三个闭环:
*   - 转矩环(电流 q 轴)、磁链环(电流 d 轴):由 FOC_Model 每个 PWM 周期调用；
*   - 速度环:由 FOC_CalcFluxTorqueRef 按速度环采样周期调用。
*   被 MC_FOC_Drive.c 调用，参数结构体 PID_Struct_t 定义于 MC_type.h。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
 
#ifndef __PI_REGULATORS__H
#define __PI_REGULATORS__H

/* Includes ------------------------------------------------------------------*/
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* PID_Init:初始化转矩/磁链/速度三个 PI(D) 结构体的增益、限幅与积分初值。 */
void PID_Init (PID_Struct_t *,PID_Struct_t *,PID_Struct_t *);
/* PID_Speed_Coefficients_update:根据电机转速(0.1Hz 分辨率)按分段线性插值刷新速度环 Kp/Ki/Kd。 */
void PID_Speed_Coefficients_update(s16, PID_Struct_t *);
/* PID_Regulator:执行一次 PI(D) 运算，返回限幅后的输出(s16)。 */
s16 PID_Regulator(s16, s16, PID_Struct_t *);

/* Exported variables ------------------------------------------------------- */

#endif 

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
