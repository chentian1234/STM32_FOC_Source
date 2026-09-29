/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Control_Param.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file gathers parameters related to:
*                      power devices, speed regulation frequency, PID controllers
*                      setpoints and constants, start-up ramp, lowest values for
*                      speed reading validation.
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
*   本文件集中配置 FOC 控制相关的各类参数与常数:
*   - 功率级:PWM 开关频率、死区、最大调制比档位；
*   - 电流环:ADC/电流环采样率(REP_RATE、SAMPLING_FREQ)；
*   - 保护阈值:过温/过压/欠压、母线 ADC 分压比；
*   - 速度环采样周期、速度/转矩/磁链 PID 初值与定标除数；
*   - 速度 PID 系数随转速分段线性插值的断点与斜率(alpha_Kx)。
*   被 FOC 库各模块引用。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_CONTROL_PARAM_H
#define __MC_CONTROL_PARAM_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/*********************** POWER DEVICES PARAMETERS ******************************/

/****	Power devices switching frequency  ****/
// PWM 开关频率(Hz)，中心对齐方式
#define PWM_FREQ ((u16) 14400) // in Hz  (N.b.: pattern type is center aligned)

/****    Deadtime Value   ****/
// 死区时间、无参无返回(单位 ns，范围 [0...3500])
#define DEADTIME_NS	((u16) 800)  //in nsec; range is [0...3500] 
                                                                    
/* 最大调制比选择:请根据所选 PWM 频率取消注释对应档位。
   本工程选用 96% 档(见下方已启用行)，其对应的电压圆上限 MAX_MODULE 定义在 MC_Clarke_Park.h。 */
/****      Uncomment the Max modulation index     ****/ 
/**** corresponding to the selected PWM frequency ****/
//#define MAX_MODULATION_100_PER_CENT     // up to 11.4 kHz PWM frequency 
//#define MAX_MODULATION_99_PER_CENT      // up to 11.8 kHz
//#define MAX_MODULATION_98_PER_CENT      // up to 12.2 kHz  
//#define MAX_MODULATION_97_PER_CENT      // up to 12.9 kHz  
#define MAX_MODULATION_96_PER_CENT      // up to 14.4 kHz    // 已启用:调制比 96%(与 14.4kHz PWM 匹配)
//#define MAX_MODULATION_95_PER_CENT      // up to 14.8 kHz
//#define MAX_MODULATION_94_PER_CENT      // up to 15.2 kHz  
//#define MAX_MODULATION_93_PER_CENT      // up to 16.7 kHz
//#define MAX_MODULATION_92_PER_CENT      // up to 17.1 kHz
//#define MAX_MODULATION_89_PER_CENT      // up to 17.5 kHz

/*********************** CURRENT REGULATION PARAMETERS ************************/

/****	ADC IRQ-HANDLER frequency, related to PWM  ****/
// 电流环重载率:电流环每 (REP_RATE+1)/(2*PWM_FREQ) 秒执行一次
#define REP_RATE (1)  // (N.b): Internal current loop is performed every 
                      //             (REP_RATE + 1)/(2*PWM_FREQ) seconds.
                      // REP_RATE has to be an odd number in case of three-shunt
                      // current reading; this limitation doesn't apply to ICS

//Not to be modified
// 电流环(ADC 中断)采样频率，由 PWM 频率与 REP_RATE 推算，分辨率 1Hz
#define SAMPLING_FREQ   ((u16)PWM_FREQ/((REP_RATE+1)/2))   // Resolution: 1Hz

/********************** POWER BOARD PROTECTIONS THRESHOLDS ********************/

// 散热器过温阈值(℃)
#define NTC_THRESHOLD_C           60  //°C on heatsink of MB459 board
// 温度滞环(℃):过温恢复的温度回落量
#define NTC_HYSTERIS_C             5   // Temperature hysteresis (°C)

// 母线过压阈值(V)
#define OVERVOLTAGE_THRESHOLD_V   350 //Volt on DC Bus of MB459 board
// 母线欠压阈值(V)
#define UNDERVOLTAGE_THRESHOLD_V  18  //Volt on DC Bus of MB459 board

// 直流母线电压的 ADC 分压比(用于由 ADC 值还原母线电压)
#define BUS_ADC_CONV_RATIO  0.008 /* DC bus voltage partitioning ratio*/

/*********************** SPEED LOOP SAMPLING TIME *****************************/
//Not to be modified
// 速度环(外环)采样周期档位:宏值 N 代表周期 = (N+1)×500us(即速度定时器计数值),
// 例如 2ms 档 N=3 → (3+1)×500us=2ms。下列各档由定时器配置决定,不可随意修改。
#define PID_SPEED_SAMPLING_500us      0     // min 500usec
#define PID_SPEED_SAMPLING_1ms        1     // (1+1)*500usec = 1msec
#define PID_SPEED_SAMPLING_2ms        3     // (3+1)*500usec = 2msec
#define PID_SPEED_SAMPLING_5ms        9		// (9+1)*500usec = 5msec		
#define PID_SPEED_SAMPLING_10ms       19	// (19+1)*500usec = 10msec
#define PID_SPEED_SAMPLING_20ms       39	// (39+1)*500usec = 20msec
#define PID_SPEED_SAMPLING_127ms      255   // max (255-1)*500us = 127 ms

//User should make his choice here below
// 用户选定的速度环采样周期:此处选择 2ms(即速度环每 2ms 执行一次速度 PI:FOC_CalcFluxTorqueRef)
#define PID_SPEED_SAMPLING_TIME   (u8)(PID_SPEED_SAMPLING_2ms)

/******************** SPEED PID-CONTROLLER INIT VALUES************************/

/* default values for Speed control loop */
// 速度环初始给定转速(单位:RPM,转/分)
#define PID_SPEED_REFERENCE_RPM   (s16)1500
// 速度环比例增益分子 Kp(真实比例增益 = Kp/SP_KPDIV)
#define PID_SPEED_KP_DEFAULT      (s16)1000
// 速度环积分增益分子 Ki(真实积分增益 = Ki/SP_KIDIV)
#define PID_SPEED_KI_DEFAULT      (s16)700
// 速度环微分增益分子 Kd(真实微分增益 = Kd/SP_KDDIV)
#define PID_SPEED_KD_DEFAULT      (s16)800

/* Speed PID parameter dividers          */
// 速度环 Kp 定标除数(把较大的增益分子缩小,避免定点运算溢出)
#define SP_KPDIV ((u16)(16))
// 速度环 Ki 定标除数
#define SP_KIDIV ((u16)(256))
// 速度环 Kd 定标除数
#define SP_KDDIV ((u16)(16))

/************** QUADRATURE CURRENTS PID-CONTROLLERS INIT VALUES **************/

// With MB459 phase current (A)= (PID_X_REFERENCE * 0.64)/(32767 * Rshunt)
// 相电流换算:实际相电流(A)=(PID 参考值 × 0.64)/(32767 × 采样电阻 Rshunt);
// 参考值为 s16 定标(32767 对应满量程),故称"标幺值/归一化"参考。

/* default values for Torque control loop */
// 转矩环(q 轴电流环)PID 初值:参考为 4500(标幺),KP/KI/KD 为增益分子
#define PID_TORQUE_REFERENCE   (s16)4500   //(N.b: that's the reference init  
                                       //value in both torque and speed control)
// 转矩环比例增益分子 Kp(真实增益 = Kp/TF_KPDIV)
#define PID_TORQUE_KP_DEFAULT  (s16)8000       
// 转矩环积分增益分子 Ki(真实增益 = Ki/TF_KIDIV)
#define PID_TORQUE_KI_DEFAULT  (s16)1000
// 转矩环微分增益分子 Kd(真实增益 = Kd/TF_KDDIV)
#define PID_TORQUE_KD_DEFAULT  (s16)3000

/* default values for Flux control loop */
// 磁链环(d 轴电流环)PID 初值:参考置 0 即 Id=0 控制(表贴式 PMSM 常用策略)
#define PID_FLUX_REFERENCE   (s16)0
// 磁链环比例增益分子 Kp(真实增益 = Kp/TF_KPDIV)
#define PID_FLUX_KP_DEFAULT  (s16)7500 
// 磁链环积分增益分子 Ki(真实增益 = Ki/TF_KIDIV)
#define PID_FLUX_KI_DEFAULT  (s16)1000
// 磁链环微分增益分子 Kd(真实增益 = Kd/TF_KDDIV)
#define PID_FLUX_KD_DEFAULT  (s16)3000

// Toruqe/Flux PID  parameter dividers
// 转矩/磁链(电流)环 PID 增益定标除数:取较大值使增益分子与内部定标匹配、避免定点溢出
#define TF_KPDIV ((u16)(8192))
#define TF_KIDIV ((u16)(4096))
#define TF_KDDIV ((u16)(8192))

/* 方波转矩给定周期:当 STM32F10x_MCconf.h 中打开 FLUX_TORQUE_PIDs_TUNING 宏时,
   系统会产生周期为 SQUARE_WAVE_PERIOD 毫秒的方波转矩参考,便于整定电流环 PID */
/* Define here below the period of the square waved reference torque generated
 when FLUX_TORQUE_PIDs_TUNING is uncommented in STM32F10x_MCconf.h          */
#define SQUARE_WAVE_PERIOD   (u16)2000 //in msec 

/* Ki/Kp/Kd 系数随转速分段线性插值:在 Fmin~F_1、F_1~F_2、F_2~Fmax 三段内,
   用该段斜率 alpha_Kx 把起点系数线性插值到终点系数(英文图解与公式见下)。 */
/*******           Ki, Kp, Kd COEFFICIENT CALCULATION       ********************/
/*******           		Speed control operation		     ***********************
		

              /|\               /
               |               /
  	       |	      /
               |             /
               |   _________/  
               |  /
               | /
	       |/_________________________
	   Fmin   F_1      F_2  Fmax      /
				
		                                                                

We assume a linear variation of Ki, Kp, Kd coefficients following
the motor speed. 2 intermediate frequencies ar set (see definition here after)
and 3 terms (Ki,Kp,Kd) associated with Fmin, F_1, F_2, Fmax 
(total: 4+4+4 terms); following linear coefficients are used to compute each term.

Example: 

Fmin = 500  <->	50 Hz 	(reminder -> mechanical frequency with 0.1 Hz resolution!)
Ki_min = 20	Kp_min = 40       Kd_min = 500 

F_1 = 2000 <->	200 Hz 	
Ki_1 = 80	Kp_1 = 1000        Kd_1 = 260 

then:
alpha_Ki_1 = (Ki_1-Ki_Fmin)/(F_1-Fmin) = 60/1500 = 0.04
alpha_Kp_1 = (Kp_1-Kp_Fmin)/(F_1-Fmin) = 960/1500 = 0.64
alpha_Kd_1 = (Kd_1-Kd_Fmin)/(F_1-Fmin) = -240/1500 = -0.16

** Result **
From Freq_Min to F_1, Ki, Kp, Kd will then obey to:
Ki = Ki_Fmin + alpha_Ki_1*(Freq_motor-Freq_Min)
Kp = Kp_Fmin + alpha_Kp_1*(Freq_motor-Freq_Min)
Kd = Kd_Fmin + alpha_Kd_1*(Freq_motor-Freq_Min)

		                                                                
*********************************************************************************/
//Settings for min frequency
// 最低转速断点:机械频率 1Hz(频率定标 0.1Hz,故 10 表示 1.0Hz)
#define Freq_Min         (u16)10 // 1 Hz mechanical
// 最低转速处的速度环系数:Ki=1000、Kp=2000、Kd=0
#define Ki_Fmin          (u16)1000 // Frequency min coefficient settings
#define Kp_Fmin          (u16)2000
#define Kd_Fmin          (u16)0

//Settings for intermediate frequency 1
// 中间断点 1:机械频率 5Hz;该处系数 Ki=2000、Kp=1000、Kd=0
#define F_1              (u16)50 // 5 Hz mechanical
#define Ki_F_1           (u16)2000 // Intermediate frequency 1 coefficient settings
#define Kp_F_1           (u16)1000
#define Kd_F_1           (u16)0

//Settings for intermediate frequency 2
// 中间断点 2:机械频率 20Hz;该处系数 Ki=1000、Kp=750、Kd=0
#define F_2              (u16)200 // 20 Hz mechanical
#define Ki_F_2           (u16)1000 // Intermediate frequency 2 coefficient settings
#define Kp_F_2           (u16)750
#define Kd_F_2           (u16)0

//Settings for max frequency
// 最高转速断点:机械频率 50Hz;该处系数 Ki=500、Kp=500、Kd=0
#define Freq_Max         (u16)500 // 50 Hz mechanical
#define Ki_Fmax          (u16)500 // Frequency max coefficient settings
#define Kp_Fmax          (u16)500
#define Kd_Fmax          (u16)0
                                                                             
/********************************************************************************/      
/* Do not modify */
/* 分段线性插值斜率 alpha_Kx(放大 1024 倍以保留小数精度,真实斜率 = alpha_Kx/1024):
   斜率 = (该段终点系数 - 起点系数) × 1024 / (终点频率 - 起点频率);由宏自动计算得出。
   例:alpha_Ki_1 对应 Fmin→F_1 段,alpha_Ki_2 对应 F_1→F_2 段,alpha_Ki_3 对应 F_2→Fmax 段。 */
/* linear coefficients */                                                                             
#define alpha_Ki_1		(s32)( ((s32)((s16)Ki_F_1-(s16)Ki_Fmin)*1024) / (s32)(F_1-Freq_Min) )
#define alpha_Kp_1		(s32)( ((s32)((s16)Kp_F_1-(s16)Kp_Fmin)*1024) / (s32)(F_1-Freq_Min) )
#define alpha_Kd_1		(s32)( ((s32)((s16)Kd_F_1-(s16)Kd_Fmin)*1024) / (s32)(F_1-Freq_Min) )

#define alpha_Ki_2		(s32)( ((s32)((s16)Ki_F_2-(s16)Ki_F_1)*1024) / (s32)(F_2-F_1) )
#define alpha_Kp_2		(s32)( ((s32)((s16)Kp_F_2-(s16)Kp_F_1)*1024) / (s32)(F_2-F_1) )
#define alpha_Kd_2		(s32)( ((s32)((s16)Kd_F_2-(s16)Kd_F_1)*1024) / (s32)(F_2-F_1) )

#define alpha_Ki_3		(s32)( ((s32)((s16)Ki_Fmax-(s16)Ki_F_2)*1024) / (s32)(Freq_Max-F_2) )
#define alpha_Kp_3		(s32)( ((s32)((s16)Kp_Fmax-(s16)Kp_F_2)*1024) / (s32)(Freq_Max-F_2) )
#define alpha_Kd_3		(s32)( ((s32)((s16)Kd_Fmax-(s16)Kd_F_2)*1024) / (s32)(Freq_Max-F_2) )


/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
#endif /* __MC_CONTROL_PARAM_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
