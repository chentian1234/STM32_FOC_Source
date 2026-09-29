/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_MCdac.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : It contains prototypes and definitions necessary for 
*                      handling DAC functionality from LCD display and joystick
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

/* ============================================================================
 * 【中文文件说明】stm32f10x_MCdac.h —— FOC 调试用模拟量输出(DAC)模块头文件
 * 声明调试时把内部变量输出为模拟电压所需的接口与常量：
 *   - 观测变量编号宏(I_A..USER_2)：作为 MCDAC_Update_Value() 的槽位参数及通道变量选择编号；
 *   - 输出通道宏(DAC_CH1/DAC_CH2)：标识两路模拟输出；
 *   - 初始化、周期输出、写入变量、切换变量与取变量名共 5 个函数原型。
 * 实现见 stm32f10x_MCdac.c(实际以 TIM3 的 PWM + 外部 RC 滤波模拟 DAC 输出)。
 * ==========================================================================*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_MCDAC_H
#define __STM32F10x_MCDAC_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* 【中文】观测变量编号宏：与 hMeasurementArray[]/OutputVariableNames[] 的索引一一对应，
   既是 MCDAC_Update_Value() 的槽位参数，也是两个输出通道的变量选择编号。
   含义：I_A/I_B/I_ALPHA/I_BETA/I_Q/I_D 为电流量，I_Q_REF/I_D_REF 为电流给定；
         V_Q/V_D/V_ALPHA/V_BETA 为电压量；SENS_ANGLE/SENS_SPEED 为传感器实测角度/转速；
         LO_ANGLE/LO_SPEED 为观测器估算角度/转速；LO_I_A/LO_I_B 为观测相电流；
         LO_BEMF_A/LO_BEMF_B 为观测反电动势；USER_1/USER_2 为用户自定义变量。 */
#define I_A           (u8)(1)               
#define I_B           (u8)(2)
#define I_ALPHA       (u8)(3)
#define I_BETA        (u8)(4)
#define I_Q           (u8)(5)
#define I_D           (u8)(6)
#define I_Q_REF       (u8)(7)
#define I_D_REF       (u8)(8)
#define V_Q           (u8)(9)
#define V_D           (u8)(10)
#define V_ALPHA       (u8)(11)
#define V_BETA        (u8)(12)
#define SENS_ANGLE    (u8)(13)
#define SENS_SPEED    (u8)(14)
#define LO_ANGLE      (u8)(15)
#define LO_SPEED      (u8)(16)
#define LO_I_A        (u8)(17)
#define LO_I_B        (u8)(18)
#define LO_BEMF_A     (u8)(19)
#define LO_BEMF_B     (u8)(20)
#define USER_1        (u8)(21)
#define USER_2        (u8)(22)
/* 【中文】模拟输出通道编号：DAC_CH1 = 通道1(PB0/TIM3_CH3)，DAC_CH2 = 通道2(PB1/TIM3_CH4)，
   供 MCDAC_Output_Choice()/MCDAC_Output_Var_Name() 指定通道。 */
#define DAC_CH1       (u8)(1)
#define DAC_CH2       (u8)(2)

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* 【中文】对外接口：
   MCDAC_Init()            初始化 GPIO/TIM3，准备输出两路 PWM 模拟量；
   MCDAC_Update_Output()   周期调用，输出当前选中的变量；
   MCDAC_Update_Value()    由控制任务写入各观测变量值(bVariable=槽位, hValue=值)；
   MCDAC_Output_Choice()   切换某通道观测的变量(bStep=步进, bChannel=通道)；
   MCDAC_Output_Var_Name() 返回某通道当前变量的名称字符串(供 LCD 显示)。 */
void MCDAC_Init(void);
void MCDAC_Update_Output(void);
void MCDAC_Update_Value(u8,s16);
void MCDAC_Output_Choice(s8,u8);
u8 *MCDAC_Output_Var_Name(u8);

#endif 
/* __STM32F10x_MCDAC_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
