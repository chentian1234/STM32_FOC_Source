/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_svpwm_3shunt.h
* Author             : IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : This file contains declaration of functions exported by 
*                      module stm32x_svpwm_3shunt.c
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
* 文件说明(中文) : 本文件是 STM32F10x 三电阻(3-Shunt)电流采样 SVPWM 模块的对外
*                  接口头文件，声明了由 stm32f10x_svpwm_3shunt.c 实现的全部函数。
*                  在 FOC(磁场定向控制)系统中，本模块位于"功率级/采样层"：它把
*                  电流环输出的 α-β 定子电压指令换算为三相逆变桥的 SVPWM
*                  (空间矢量脉宽调制)占空比并写入 TIM1 比较寄存器；同时在三相
*                  下桥臂导通窗口内触发 ADC，完成三相电流采样，供 Clarke/Park
*                  变换使用。三电阻方案的优点是每一相电流都可直接测量，缺相时
*                  可由另外两相重构第三相。
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_SVPWM_3SHUNT_H
#define __STM32F10x_SVPWM_3SHUNT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "MC_pwm_3shunt_prm.h"
#include "MC_const.h"
#include "MC_Control_Param.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* SVPWM_3ShuntInit: 上电初始化例程，配置 TIM1(中心对齐 PWM、死区、刹车)与
   ADC1/ADC2(注入组)以及 NVIC，最后调用零电流校准例程。 */
void SVPWM_3ShuntInit(void);
/* SVPWM_3ShuntGetPhaseCurrentValues: 由 ADC 注入通道原始值减去零电流偏置，
   换算并饱和为 q1.15 格式的两相定子电流，返回 Curr_Components 结构。 */
Curr_Components SVPWM_3ShuntGetPhaseCurrentValues(void);
/* SVPWM_3ShuntCalcDutyCycles: 由 α-β 电压指令完成 SVPWM 扇区判断、T1/T2
   作用时间与三相占空比计算，并写入 TIM1->CCR1/2/3/4。 */
void SVPWM_3ShuntCalcDutyCycles (Volt_Components Stat_Volt_Input);
/* SVPWM_3ShuntCurrentReadingCalibration: 电机静止时采样 16 次求平均，得到
   三相电流通道的零电流(偏置)ADC 值。 */
void SVPWM_3ShuntCurrentReadingCalibration(void);
/* SVPWM_3ShuntAdvCurrentReading: 使能/禁止"高级"电流读取(由 CC4 触发 ADC 而不
   是更新事件触发)，cmd 取 ENABLE 或 DISABLE。 */
void SVPWM_3ShuntAdvCurrentReading(FunctionalState cmd);
/* SVPWMUpdateEvent: 更新(下溢)事件中断回调，重新使能 ADC 外部触发并清除多余
   采样标志，需赋值给 pSVPWM_UpdateEvent 函数指针。 */
void SVPWMUpdateEvent(void);
/* SVPWMEOCEvent: ADC 注入组转换结束中断回调，读取母线电压与温度采样值，并在
   电机运行期间关闭 ADC 外部触发，返回 1(u8)。 */
u8 SVPWMEOCEvent(void);

#endif /* __STM32F10x_SVPWM_3SHUNT_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
