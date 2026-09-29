/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_pwm_1shunt_prm.h
* Author             : IMS Systems Lab  
* Date First Issued  : Mar/07
* Description        : Contains the list of project specific parameters related
*                      to the single-shunt current reading.
********************************************************************************
* History:
* Mar/07 v1.0
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
* 文件说明(中文) : 单电阻(1-Shunt)电流采样方案的工程参数配置头文件。单电阻方案
*                  只在直流母线负极与地之间串一个采样电阻，通过在一个 PWM 周期
*                  内不同时刻采样母线电流，间接重构出三相电流，因此对 ADC 触发
*                  时刻(TAFTER/TBEFORE)与最小脉宽(TMIN)要求更严格。
*                  本文件定义 PWM 时钟/周期/死区、ADC 采样时间、电流/温度/母线
*                  电压通道等，量纲与 MC_Control_Param.h 中 PWM_FREQ/DEADTIME_NS
*                  一致；时间类宏均已折算为 TIM1 计数值(1us ≈ 72 计数)。
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_PWM_1SHUNT_PRM_H
#define __MC_PWM_1SHUNT_PRM_H

/////////////////////// PWM Peripheral Input clock ////////////////////////////
/* CKTIM: TIM1 定时器输入时钟频率，单位 Hz(72MHz，分辨率 1Hz)。 */
#define CKTIM	((u32)72000000uL) 	/* Silicon running at 72MHz Resolution: 1Hz */

////////////////////// PWM Frequency ///////////////////////////////////

/****	 Pattern type is center aligned  ****/

	/* PWM_PRSC: TIM1 时钟预分频寄存器值(PSC)，0 表示不分频。 */
	#define PWM_PRSC ((u8)0)

        /* Resolution: 1Hz */                            
	/* PWM_PERIOD: 中心对齐模式下的 ARR 值 = CKTIM/(2*PWM_FREQ)，
	   即 PWM 半周期对应的计数值。 */
	#define PWM_PERIOD ((u16) (CKTIM / (u32)(2 * PWM_FREQ *(PWM_PRSC+1)))) 
        
////////////////////////////// Deadtime Value /////////////////////////////////
	/* DEADTIME: 上下桥臂死区时间，单位 TIM1 计数(以 CKTIM/2=36MHz 计数)。 */
	#define DEADTIME  (u16)((unsigned long long)CKTIM/2 \
          *(unsigned long long)DEADTIME_NS/1000000000uL) 

///////////////////////////// Current reading parameters //////////////////////

/* PHASE_A_ADC_CHANNEL/GPIO: A 相电流采样 ADC 通道(ADC1_IN11)与引脚 PC1。 */
#define PHASE_A_ADC_CHANNEL     ADC_Channel_11
#define PHASE_A_GPIO_PORT       GPIOC
#define PHASE_A_GPIO_PIN        GPIO_Pin_1

/* PHASE_B_ADC_CHANNEL/GPIO: B 相电流采样 ADC 通道(ADC1_IN12)与引脚 PC2。 */
#define PHASE_B_ADC_CHANNEL     ADC_Channel_12
#define PHASE_B_GPIO_PORT       GPIOC
#define PHASE_B_GPIO_PIN        GPIO_Pin_2

/* PHASE_C_ADC_CHANNEL/GPIO: C 相电流采样 ADC 通道(ADC1_IN13)与引脚 PC3。 */
#define PHASE_C_ADC_CHANNEL     ADC_Channel_13
#define PHASE_C_GPIO_PORT       GPIOC
#define PHASE_C_GPIO_PIN        GPIO_Pin_3

/****************** Control related define ***************************/
/* CURRENT_COMPENSATION: 使能单电阻采样因最小脉宽限制导致的电流不可测区间
   补偿(用一个周期内另一时刻的采样值推算)。 */
#define CURRENT_COMPENSATION // Enable the current compensation

/* SAMPLING_TIME_NS: ADC 采样保持时间，单位 ns，当前选 700ns。 */
//#define SAMPLING_TIME_NS   200  //200ns
#define SAMPLING_TIME_NS   700  //700ns
//#define SAMPLING_TIME_NS  1200  //1.2us
//#define SAMPLING_TIME_NS  2450  //2.45us

/* 按 SAMPLING_TIME_NS 映射到 ADC 采样周期寄存器编码(同上：1.5/7.5/13.5/28.5 周期)。 */
#if (SAMPLING_TIME_NS == 200)
#define SAMPLING_TIME_CK  ADC_SampleTime_1Cycles5
#elif (SAMPLING_TIME_NS == 700)
#define SAMPLING_TIME_CK  ADC_SampleTime_7Cycles5
#elif (SAMPLING_TIME_NS == 1200)
#define SAMPLING_TIME_CK  ADC_SampleTime_13Cycles5
#elif (SAMPLING_TIME_NS == 2450)
#define SAMPLING_TIME_CK  ADC_SampleTime_28Cycles5
#else
#warning "Sampling time is not a possible value"
#endif

/* TRISE_NS: 电流上升沿建立时间，单位 ns(2.55us)。 */
#define TRISE_NS 2550     //2.55usec

/* 时间量由 ns 换算为 TIM1 计数(×72)：采样时间/上升沿/死区对应的计数。 */
#define SAMPLING_TIME (u16)(((u16)(SAMPLING_TIME_NS) * 72uL)/1000uL) 
#define TRISE (u16)((((u16)(TRISE_NS)) * 72uL)/1000uL)
#define TDEAD (u16)((DEADTIME_NS * 72uL)/1000uL)

/* TMIN: 单电阻方案要求的最小有效脉宽(死区+上升沿+采样时间+1)，单位计数，
   小于该宽度的矢量无法可靠采样；HTMIN = TMIN/2，用于占空比限幅。
   TSAMPLE: 采样窗口计数; TAFTER: 采样触发滞后(死区+上升沿); TBEFORE: 触发前移
   (采样时间+1)。 */
#define TMIN (((u16)(((DEADTIME_NS+((u16)(TRISE_NS))+((u16)(SAMPLING_TIME_NS)))*72uL)/1000ul))+1)
#define HTMIN (u16)(TMIN >> 1)
#define TSAMPLE SAMPLING_TIME
#define TAFTER ((u16)(((DEADTIME_NS+((u16)(TRISE_NS)))*72ul)/1000ul))
#define TBEFORE (((u16)(((((u16)(SAMPLING_TIME_NS)))*72ul)/1000ul))+1)

/* MAX_TRTS: 采样所需时间的 2 倍(上升沿或采样时间中的较大者乘 2)，单位计数，
   用于判断是否存在足以完成采样的窗口。 */
#if (TRISE_NS > SAMPLING_TIME_NS)
  #define MAX_TRTS (2 * TRISE)
#else
  #define MAX_TRTS (2 * SAMPLING_TIME)
#endif

/////////////////  Power Stage management Conversions setting ////////////////////////

/* TEMP_FDBK_CHANNEL: NTC 温度反馈 ADC 通道(ADC1_IN10)，引脚 PC0。 */
#define TEMP_FDBK_CHANNEL                 ADC_Channel_10
#define TEMP_FDBK_CHANNEL_GPIO_PORT       GPIOC
#define TEMP_FDBK_CHANNEL_GPIO_PIN        GPIO_Pin_0

/* BUS_VOLT_FDBK_CHANNEL: 直流母线电压反馈 ADC 通道(ADC1_IN3)，引脚 PA3。 */
#define BUS_VOLT_FDBK_CHANNEL             ADC_Channel_3
#define BUS_VOLT_FDBK_CHANNEL_GPIO_PORT   GPIOA
#define BUS_VOLT_FDBK_CHANNEL_GPIO_PIN    GPIO_Pin_3

#endif  /*__MC_PWM_1SHUNT_PRM_H*/
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
