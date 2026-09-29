/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_pwm_3shunt_prm.h
* Author             : IMS Systems Lab  
* Date First Issued  : 21/11/07
* Description        : Contains the list of project specific parameters related
*                      to the three-shunt current reading.
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
* 文件说明(中文) : 三电阻(3-Shunt)电流采样方案的工程参数配置头文件。集中定义
*                  了 PWM 时钟/周期/死区、ADC 采样时间、噪声与上升沿裕量时间
*                  (TNOISE/TRISE)、ADC 触发提前/滞后窗口(TW_BEFORE/TW_AFTER)
*                  以及三相电流、温度、母线电压的 ADC 通道与 GPIO 引脚。
*                  这些量纲与定标直接决定 SVPWM 占空比与 ADC 触发时刻的计算，
*                  修改时需与硬件(MB459 功率板)及 MC_Control_Param.h 中的
*                  PWM_FREQ/DEADTIME_NS 保持一致。
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_PWM_3SHUNT_PRM_H
#define __MC_PWM_3SHUNT_PRM_H

/////////////////////// PWM Peripheral Input clock ////////////////////////////
/* CKTIM: TIM1 定时器输入时钟频率，单位 Hz。72MHz 系统时钟下为 72000000，
   分辨率 1Hz；TIM1 计数 1 个 tick 即 1/72MHz ≈ 13.9ns。 */
#define CKTIM	((u32)72000000uL) 	/* Silicon running at 72MHz Resolution: 1Hz */

////////////////////// PWM Frequency ///////////////////////////////////

/****	 Pattern type is center aligned  ****/

	/* PWM_PRSC: TIM1 时钟预分频寄存器值(PSC)，0 表示不分频。 */
	#define PWM_PRSC ((u8)0)

        /* Resolution: 1Hz */                            
	/* PWM_PERIOD: PWM 半周期(中心对齐模式下的 ARR 值)= CKTIM/(2*PWM_FREQ)。
	   因上下计数，实际 PWM 频率 = PWM_FREQ，PWM 周期 = 1/PWM_FREQ。
	   PWM_FREQ=14400Hz 时 PWM_PERIOD=2500 个计数。 */
	#define PWM_PERIOD ((u16) (CKTIM / (u32)(2 * PWM_FREQ *(PWM_PRSC+1)))) 
        
////////////////////////////// Deadtime Value /////////////////////////////////
	/* DEADTIME: 上下桥臂之间的死区时间，单位 TIM1 计数(以 CKTIM/2=36MHz 计数)。
	   由 DEADTIME_NS 纳秒值换算而来，用于防止同一桥臂上下管直通短路。 */
	#define DEADTIME  (u16)((unsigned long long)CKTIM/2 \
          *(unsigned long long)DEADTIME_NS/1000000000uL) 

///////////////////////////// Current reading parameters //////////////////////

/* PHASE_A_ADC_CHANNEL/GPIO: A 相电流采样所用的 ADC 通道(ADC1_IN11)与引脚 PC1。 */
#define PHASE_A_ADC_CHANNEL     ADC_Channel_11
#define PHASE_A_GPIO_PORT       GPIOC
#define PHASE_A_GPIO_PIN        GPIO_Pin_1

/* PHASE_B_ADC_CHANNEL/GPIO: B 相电流采样所用的 ADC 通道(ADC1_IN12)与引脚 PC2。 */
#define PHASE_B_ADC_CHANNEL     ADC_Channel_12
#define PHASE_B_GPIO_PORT       GPIOC
#define PHASE_B_GPIO_PIN        GPIO_Pin_2

/* PHASE_C_ADC_CHANNEL/GPIO: C 相电流采样所用的 ADC 通道(ADC1_IN13)与引脚 PC3。 */
#define PHASE_C_ADC_CHANNEL     ADC_Channel_13
#define PHASE_C_GPIO_PORT       GPIOC
#define PHASE_C_GPIO_PIN        GPIO_Pin_3

/* SAMPLING_TIME_NS: ADC 采样保持时间，单位 ns。当前选 700ns(约 7.5 个 ADCCLK
   周期)，需大于运放/网络的建立时间；下面按取值映射到对应的 ADC 采样周期宏。 */
//#define SAMPLING_TIME_NS   200  //200ns
#define SAMPLING_TIME_NS   700  //700ns
//#define SAMPLING_TIME_NS  1200  //1.2us
//#define SAMPLING_TIME_NS  2450  //2.45us

/* 根据 SAMPLING_TIME_NS 选择 ADC 采样周期寄存器编码：
   200ns→1.5 周期, 700ns→7.5 周期, 1200ns→13.5 周期, 2450ns→28.5 周期；
   若取值不在表中则编译告警。 */
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

/* TNOISE_NS: 开关噪声持续时间，单位 ns(2.55us)；TRISE_NS: 电流上升沿建立
   时间，单位 ns(2.55us)。二者用于计算采样点的延时裕量。 */
#define TNOISE_NS 2550     //2.55usec
#define TRISE_NS 2550     //2.55usec

/* 以下把上述时间量由 ns 换算为 TIM1 计数(×72 → 1us 对应 72 个 tick)：
   SAMPLING_TIME - ADC 采样窗口对应计数; TNOISE/TRISE - 噪声/上升沿对应计数;
   TDEAD - 死区对应计数。 */
#define SAMPLING_TIME (u16)(((u16)(SAMPLING_TIME_NS) * 72uL)/1000uL) 
#define TNOISE (u16)((((u16)(TNOISE_NS)) * 72uL)/1000uL)
#define TRISE (u16)((((u16)(TRISE_NS)) * 72uL)/1000uL)
#define TDEAD (u16)((DEADTIME_NS * 72uL)/1000uL)

/* MAX_TNTR_NS: 噪声时间与上升沿时间中的较大者，单位 ns。 */
#if (TNOISE_NS > TRISE_NS)
  #define MAX_TNTR_NS TNOISE_NS
#else
  #define MAX_TNTR_NS TRISE_NS
#endif

/* TW_AFTER: ADC 触发点相对下桥臂导通时刻的滞后窗口 = 死区 + max(噪声,上升沿)，
   单位 TIM1 计数；TW_BEFORE: 触发点前移窗口 = 采样时间 + 1 个计数。
   二者共同决定 CC4 触发 ADC 的时刻，保证采样落在下桥臂稳定导通窗口内。 */
#define TW_AFTER ((u16)(((DEADTIME_NS+MAX_TNTR_NS)*72ul)/1000ul))
#define TW_BEFORE (((u16)(((((u16)(SAMPLING_TIME_NS)))*72ul)/1000ul))+1)

/////////////////  Power Stage management Conversions setting ////////////////////////

/* TEMP_FDBK_CHANNEL: 功率板 NTC 温度反馈 ADC 通道(ADC1_IN10)，引脚 PC0。 */
#define TEMP_FDBK_CHANNEL                 ADC_Channel_10
#define TEMP_FDBK_CHANNEL_GPIO_PORT       GPIOC
#define TEMP_FDBK_CHANNEL_GPIO_PIN        GPIO_Pin_0

/* BUS_VOLT_FDBK_CHANNEL: 直流母线电压反馈 ADC 通道(ADC1_IN3)，引脚 PA3。 */
#define BUS_VOLT_FDBK_CHANNEL             ADC_Channel_3
#define BUS_VOLT_FDBK_CHANNEL_GPIO_PORT   GPIOA
#define BUS_VOLT_FDBK_CHANNEL_GPIO_PIN    GPIO_Pin_3

#endif  /*__MC_PWM_3SHUNT_PRM_H*/
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
