/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_pwm_ics_prm.h
* Author             : IMS Systems Lab  
* Date First Issued  : 21/11/07
* Description        : Contains the list of project specific parameters related
*                      to the ICS current reading.
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
* 文件说明(中文) : ICS(内部电流传感器/Integrated Current Sensor)电流采样方案的
*                  工程参数配置头文件。ICS 方案通过功率驱动芯片内部的电流传感
*                  单元(如 L6235 类的镜像电流输出)获得相电流，无需外部采样电阻，
*                  但只有部分相电流可用(通常只能得到两相)，第三相需重构。
*                  本文件定义 PWM 时钟/周期/死区、ADC 采样时间与电流/温度/母线
*                  电压通道，量纲与 MC_Control_Param.h 中 PWM_FREQ/DEADTIME_NS
*                  一致；时间类宏以 TIM1 计数为单位(1us ≈ 72 计数)。
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_PWM_ICS_PRM_H
#define __MC_PWM_ICS_PRM_H

/////////////////////// PWM Peripheral Input clock ////////////////////////////
/* CKTIM: TIM1 定时器输入时钟频率，单位 Hz。原注释写 60MHz 有误，实际为 72000000。
   分辨率 1Hz，1 个计数 ≈ 13.9ns。 */
#define CKTIM	((u32)72000000uL) 	/* Silicon running at 60MHz Resolution: 1Hz */

////////////////////// PWM Frequency ///////////////////////////////////

/****	 Pattern type is center aligned  ****/

	/* PWM_PRSC: TIM1 时钟预分频寄存器值(PSC)，0 表示不分频。 */
	#define PWM_PRSC ((u8)0)

        /* Resolution: 1Hz */                            
	/* PWM_PERIOD: 中心对齐模式下的 ARR 值 = CKTIM/(2*PWM_FREQ)，即半周期计数。 */
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

/* SAMPLING_TIME_NS: ADC 采样保持时间，单位 ns，当前选 700ns。 */
//#define SAMPLING_TIME_NS   200  //200ns
#define SAMPLING_TIME_NS   700  //700ns
//#define SAMPLING_TIME_NS  1200  //1.2us
//#define SAMPLING_TIME_NS  2450  //2.45us

/* 按 SAMPLING_TIME_NS 映射到 ADC 采样周期寄存器编码(1.5/7.5/13.5/28.5 周期)。 */
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

/////////////////  Power Stage management Conversions setting ////////////////////////

/* TEMP_FDBK_CHANNEL: NTC 温度反馈 ADC 通道(ADC1_IN10)，引脚 PC0。 */
#define TEMP_FDBK_CHANNEL                 ADC_Channel_10
#define TEMP_FDBK_CHANNEL_GPIO_PORT       GPIOC
#define TEMP_FDBK_CHANNEL_GPIO_PIN        GPIO_Pin_0

/* BUS_VOLT_FDBK_CHANNEL: 直流母线电压反馈 ADC 通道(ADC1_IN3)，引脚 PA3。 */
#define BUS_VOLT_FDBK_CHANNEL             ADC_Channel_3
#define BUS_VOLT_FDBK_CHANNEL_GPIO_PORT   GPIOA
#define BUS_VOLT_FDBK_CHANNEL_GPIO_PIN    GPIO_Pin_3

#endif  /*__MC_PWM_ICS_PRM_H*/
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
