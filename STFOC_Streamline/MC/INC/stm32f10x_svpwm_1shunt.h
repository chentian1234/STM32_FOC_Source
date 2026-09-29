/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_svpwm_1shunt.h
* Author             : IMS Systems Lab
* Date First Issued  : 29/05/08
* Description        : This file contains declaration of functions exported by 
*                      module stm32x_svpwm_1shunt.c
********************************************************************************
* History:
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
* 文件说明(中文) :
*   本头文件声明「单电阻(1-shunt)电流采样 + SVPWM(空间矢量脉宽调制)」模块对外的接口。
*   单电阻电流采样: 在直流母线回路里只串一个采样电阻，通过在 PWM 周期的不同
*   有效矢量作用区间分时采集母线电流，再重构出三相电流 Ia/Ib/Ic。
*   对应的实现文件为 stm32f10x_svpwm_1shunt.c，由 SINGLE_SHUNT 宏使能编译。
*   上层电流环每个控制周期调用 SVPWM_1ShuntCalcDutyCycles() 计算三相占空比，
*   调用 SVPWM_1ShuntGetPhaseCurrentValues() 取回重构后的相电流(q1.15 定标)。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_SVPWM_1SHUNT_H
#define __STM32F10x_SVPWM_1SHUNT_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "MC_pwm_1shunt_prm.h"
#include "MC_const.h"
#include "MC_Control_Param.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

#define INVERT_NONE 0   // 不进行移相(占空比畸变): 处于规则区(REGULAR)，无需修正
#define INVERT_A 1      // 对 A 相做移相/畸变(将该相占空比下移 HTMIN 以拓宽采样窗口)
#define INVERT_B 2      // 对 B 相做移相/畸变
#define INVERT_C 3      // 对 C 相做移相/畸变

/* 记录本 PWM 周期两次电流采样点各自测量的是哪一相(含正负极性)，供重构相电流时使用 */
typedef struct
{
	u8 sampCur1;    // 第一个采样点对应的相/极性，取值见下方 SAMP_xxx 宏
	u8 sampCur2;    // 第二个采样点对应的相/极性，取值见下方 SAMP_xxx 宏
} CURRENTSAMPLEDTYPE;

#define SAMP_NO 0    // 该采样点无效/不使用
#define SAMP_IA 1    // 采样得到的是 +Ia (母线电流正向对应 A 相)
#define SAMP_IB 2    // 采样得到的是 +Ib
#define SAMP_IC 3    // 采样得到的是 +Ic
#define SAMP_NIA 4   // 采样得到的是 -Ia (需取反)
#define SAMP_NIB 5   // 采样得到的是 -Ib (需取反)
#define SAMP_NIC 6   // 采样得到的是 -Ic (需取反)
#define SAMP_OLDA 7  // 使用上一周期保存的 A 相电流值(本点无法测量，用旧值)
#define SAMP_OLDB 8  // 使用上一周期保存的 B 相电流值
#define SAMP_OLDC 9  // 使用上一周期保存的 C 相电流值

/* 本 PWM 周期最终写入定时器比较寄存器/采样触发寄存器的数值(单位: TIM1 计数时钟 CK_INT 个计数, 即 1/72MHz) */
typedef struct 
{
	u16 hTimePhA;   // A 相占空比对应比较值(写入 CCR1)
	u16 hTimePhB;   // B 相占空比对应比较值(写入 CCR2)
	u16 hTimePhC;   // C 相占空比对应比较值(写入 CCR3)
	u16 hTimeSmp1;  // 第一采样点触发的比较值(写入 CCR4, 触发 ADC 注入转换)
	u16 hTimeSmp2;  // 第二采样点触发的比较值(写入 CCR4, 触发 ADC 注入转换)
} DUTYVALUESTYPE;

#define MAX(a,b) ((a)>(b))?(a):(b)   // 取两数较大者的宏(注意未加括号，使用需谨慎)

/* DMA 突发(burst)写入 CCRx 时允许的最小计数值(单位: TIM1 计数 Ck)，
   防止某种移相下占空比过小而丢失，A/B/C 三相因上下桥臂时序不同取值略有差异 */
#define DMABURSTMIN_A 16    // A 相 DMA 突发写入时的最小比较值
#define DMABURSTMIN_B 23    // B 相 DMA 突发写入时的最小比较值
#define DMABURSTMIN_C 35    // C 相 DMA 突发写入时的最小比较值

/* DMA ENABLE mask */
#define CCR_ENABLE_Set          ((u32)0x00000001)   // DMA_CCR 寄存器的 EN 位置 1 (启动通道)
#define CCR_ENABLE_Reset        ((u32)0xFFFFFFFE)   // 掩码: 与运算后清 EN 位 (关闭通道)

/* 在 update 中断处理内为避免切换发生在危险时刻而对占空比做的限幅(单位: TIM1 计数 Ck) */
#define MINTIMCNTUPHAND ((u16)(60))    // update 处理允许的最小占空比比较值
#define MAXTIMCNTUPHAND ((u16)(220))   // update 处理允许的最大占空比比较值

#define MIDTIMCNTUPHAND ((MAXTIMCNTUPHAND+MINTTIMCNTUPHAND) >> 1)   // 上述区间的中点(用于三档限幅判断)

/* Exported functions ------------------------------------------------------- */
void SVPWM_1ShuntCalcDutyCycles (Volt_Components Stat_Volt_Input);   // 由 α/β 电压指令计算三相占空比、采样点并做单电阻移相修正(每个电流环周期调用)
Curr_Components SVPWM_1ShuntGetPhaseCurrentValues(void);              // 由 ADC 注入值重构三相电流, 返回 (Ia, Ib) 两分量(q1.15)

void SVPWM_1ShuntInit(void);                                          // 初始化 TIM1/PWM、ADC1/ADC2、DMA 及中断(上电或启动时调用一次)
void SVPWM_1ShuntCurrentReadingCalibration(void);                     // 采集零电流时 ADC 偏置, 用于电流读数零点补偿
void SVPWM_1ShuntAdvCurrentReading(FunctionalState cmd);              // 使能/关闭电流采样(启停电机时切换采样方式)

void SVPWMUpdateEvent(void);                                          // TIM1 更新中断(下溢)内调用的处理: 切换 PWM 通道模式/开关 DMA/刷新缓冲
u8 SVPWMEOCEvent(void);                                               // ADC 注入转换结束(第二 EOC)中断内调用: 读取母线电压与温度采样值

#endif /* __STM32F10x_SVPWM_1SHUNT_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
