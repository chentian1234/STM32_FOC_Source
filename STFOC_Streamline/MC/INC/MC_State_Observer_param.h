/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_State_Observer_param.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : Contains the PMSM State Observer related parameters
*                      (module MC_State_Observer_Interface.c)
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
* 模块说明(中文) : PMSM 状态观测器参数配置头文件（对应实现 MC_State_Observer_Interface.c）。
*                  集中定义反电动势观测器与 PLL 锁相环的增益/系数、无传感器启动
*                  （对齐→开环强拖→切换闭环）的斜坡参数、以及转速可信度统计阈值。
*                  定标约定：观测器内部以 F1/F2 两个定点缩放因子把浮点系数转为
*                  整数运算；角度类量以 65536 = 360° 电角度定标；转速以 dpp
*                  （每 PWM 周期角度增量）或 0.1Hz（Hz×10）定标。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_STATE_OBSERVER_PARAM_H
#define __MC_STATE_OBSERVER_PARAM_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* 最大相电流幅值（0-峰值），单位 A；用于观测器系数 C4 的定标与内部饱和参考。*/
#define MAX_CURRENT 2.9             /* max current value, Amps */

// Values showed on LCD display must be here multiplied by 10 
/* 观测器增益 K1：电流估计误差的反馈增益（Luenberger 观测器）。定标为 s32；
   注意整定界面(LCD)显示的 K1 值需将本值乘 10 才是界面数值。*/
#define K1 (s32)(-12000)             /* State Observer Gain 1 */
// Values showed on LCD display must be here multiplied by 100 
/* 观测器增益 K2：反电动势估计误差的反馈增益（Luenberger 观测器）。定标为 s32；
   注意整定界面(LCD)显示的 K2 值需将本值乘 100 才是界面数值。*/
#define K2 (s32)(+85200)           /* State Observer Gain 2 */

/* PLL 比例增益 PLL_P：与电机最大转速、极对数成正比，与 PWM 采样频率成反比，
   使锁相环在最大转速附近仍具合适带宽。填入 StateObserver_Const.PLL_P。*/
#define PLL_KP_GAIN (s16)(532*MOTOR_MAX_SPEED_RPM*POLE_PAIR_NUM/SAMPLING_FREQ)

/* PLL 积分增益 PLL_I：决定锁相环消除稳态相位误差的速度；按极对数、最大转速
   及采样频率平方归一化换算。填入 StateObserver_Const.PLL_I。*/
#define PLL_KI_GAIN (s16)(1506742*POLE_PAIR_NUM/SAMPLING_FREQ\
                                        *MOTOR_MAX_SPEED_RPM/SAMPLING_FREQ)    
                                  
/* 启动(开环强拖)参数总述：启用无位置传感器时，电机先按固定频率斜坡以受控幅值与
   频率的定子电流“强拖”加速，待反电动势观测器收敛后再切入闭环。下面的速度/电流
   斜坡曲线描述该过程。*/
/******************* START-UP PARAMETERS ***************************************

              Speed /|\
FINAL_START_UP_SPEED |              /
                     |            /
                     |          /          
                     |        /	      
                     |      /          
                     |    / 
                     |  / 
                     |/_______________________________________                      
                     0          FREQ_START_UP_DURATION      t /               */

/* 频率(转速)斜坡持续时间，单位 ms；斜坡结束时转速达到 FINAL_START_UP_SPEED。*/
#define FREQ_START_UP_DURATION    (u16) 1500 //in msec
/* 开环强拖结束时的转子机械转速，单位 rpm；此时尝试切换到闭环速度控制。*/
#define FINAL_START_UP_SPEED      (u16) 2700 //Rotor mechanical speed (rpm)

/*
                 |I|/|\
                     |
      FINAL_I_STARTUP|       __________________    
                     |     /          
                     |    /	      
                     |   /          
                     |  / 
                     | / 
      FIRST_I_STARTUP|/ 
                     |_______________________________________________                      
                     0 I_START_UP_DURATION  FREQ_START_UP_DURATION t /        */
                                                                              
// With MB459 phase current = (X_I_START_UP * 0.64)/(32767 * Rshunt)
/* 强拖起始相电流给定（内部定标：×1024 参与的电流参考）；配合下方电流斜坡曲线使用。*/
#define FIRST_I_STARTUP           (u16) 8000  
/* 强拖结束相电流给定（内部定标）；本工程起止电流相同，故电流保持不变。*/
#define FINAL_I_STARTUP           (u16) 8000 
/* 电流斜坡持续时间，单位 ms（I_STARTUP_PWM_STEPS 由其换算得到）。*/
#define I_START_UP_DURATION       (u16) 350 //in msec

// Alignment settings 
#ifdef NO_SPEED_SENSORS_ALIGNMENT

/* 对齐(Alignment)设置：无传感器启动前先把转子强迫对齐到已知电角度，保证后续
   开环强拖有确定的初始相位。仅当定义了 NO_SPEED_SENSORS_ALIGNMENT 时生效。*/
//Alignemnt duration
/* 对齐持续时间，单位 ms；期间定子电流矢量固定在 SLESS_ALIGNMENT_ANGLE 角度。*/
#define SLESS_T_ALIGNMENT           (u16) 700    // Alignment time in ms

/* 对齐时施加的定子电流矢量电角度，单位 度[0..359]；90° 时对应
   Ia=SLESS_I_ALIGNMENT、Ib=Ic=-SLESS_I_ALIGNMENT/2。*/
#define SLESS_ALIGNMENT_ANGLE       (u16) 90 //Degrees [0..359]  
//  90° <-> Ia = SLESS_I_ALIGNMENT, Ib = Ic =-SLESS_I_ALIGNMENT/2) 

// With SLESS_ALIGNMENT_ANGLE equal to 90° final alignment 
// phase current = (SLESS_I_ALIGNMENT * 1.65/ Av)/(32767 * Rshunt)  
// being Av the voltage gain between Rshunt and A/D input
/* 对齐时的相电流幅值（内部定标，与硬件采样电阻/放大倍数相关）。*/
#define SLESS_I_ALIGNMENT           (u16) 22000 

#endif

/* 统计(可信度)参数总述：用于判定观测器转速估计是否稳定可信，作为启动收敛判据
   与“反馈丢失”保护的依据。*/
/**************************** STATISTIC PARAMETERS ****************************/
//Threshold for the speed measurement variance.   
/* 转速测量方差阈值（相对均值的比例，0.0625=6.25%）；低于此认为转速波动小、
   估计稳定。最终以 PERCENTAGE_FACTOR 换算为内部 0..127 百分比定标使用。*/
#define VARIANCE_THRESHOLD        0.0625  //Percentage of mean value

// Number of consecutive tests on speed variance to be passed before start-up is
// validated. Checked every PWM period
/* 启动收敛判定所需连续通过的检测次数；每 PWM 周期检测一次，连续满足条件才
   认为观测器已收敛（对应 bConvCounter 的阈值）。*/
#define NB_CONSECUTIVE_TESTS      (u16) 60
// Number of consecutive tests on speed variance before the variable containing
// speed reliability change status. Checked every SPEED_SAMPLING_TIME
/* 转速可信度状态翻转所需的连续不可信次数；每个速度采样周期检测一次，
   连续达到该次数才判为“反馈丢失”。*/
#define RELIABILITY_HYSTERESYS    (u8)  3
//Minimum Rotor speed to validate the start-up
/* 验证启动成功所需的最小转子机械转速，单位 rpm。*/
#define MINIMUM_SPEED_RPM             (u16) 580

/* 观测器内部定点缩放因子 F1=2048=2^11：把电流/电压相关的浮点系数转换为整数
   运算所需的尺度。*/
#define F1 (s16)(2048)
/* 观测器内部定点缩放因子 F2=8192=2^13：用于反电动势相关的系数缩放。*/
#define F2 (s16)(8192)

//The parameters below shouldn't be modified
/*max phase voltage, 0-peak Volts*/
/* 最大相电压，0-峰值，单位 V；由半母线电压 3.3/2 除以母线采样分压比得到。*/
#define MAX_VOLTAGE (s16)((3.3/2)/BUS_ADC_CONV_RATIO) 

/* 观测器系数 C1 = F1*Rs/(Ls*Fs)：定子电阻项（Rs 定子电阻 Ω，Ls 定子电感 H，
   Fs 采样频率 Hz），用于电流微分方程的离散化。*/
#define C1 (s32)((F1*RS)/(LS*SAMPLING_FREQ))
/* 观测器系数 C2 = F1*K1/Fs：电流估计误差的反馈项（含增益 K1）。*/
#define C2 (s32)((F1*K1)/SAMPLING_FREQ)
/* 观测器系数 C3 = F1*MaxBemf/(Ls*MaxCurrent*Fs)：与反电动势/电感相关的项。*/
#define C3 (s32)((F1*MAX_BEMF_VOLTAGE)/(LS*MAX_CURRENT*SAMPLING_FREQ))
/* 观测器系数 C4 = (K2*MaxCurrent/MaxBemf)*F2/Fs：反电动势估计误差的反馈项（含增益 K2）。*/
#define C4 (s32)((((K2*MAX_CURRENT)/(MAX_BEMF_VOLTAGE))*F2)/(SAMPLING_FREQ))
/* 观测器系数 C5 = F1*MaxVoltage/(Ls*MaxCurrent*Fs)：定子电压激励项。*/
#define C5 (s32)((F1*MAX_VOLTAGE)/(LS*MAX_CURRENT*SAMPLING_FREQ))

/* 电机最大转速对应的 dpp 定标值（每 PWM 周期角度增量，×65536 表示电角度，
   并留 1.2 倍裕量）；用于观测器内部速度限幅与可信度判定。*/
#define MOTOR_MAX_SPEED_DPP (s32)((1.2*MOTOR_MAX_SPEED_RPM*65536*POLE_PAIR_NUM)\
                                                            /(SAMPLING_FREQ*60))

/* 频率斜坡阶段(0→FINAL_START_UP_SPEED)持续的 PWM 周期数；由持续时间按采样
   频率换算（PWM_STEPS = ms*Fs/1000）。*/
#define FREQ_STARTUP_PWM_STEPS (u32) ((FREQ_START_UP_DURATION * SAMPLING_FREQ)\
                                                                          /1000) 
/* 每个 PWM 周期频率增量，定标 65536 = 360° 电角度（即电频率步长）；
   由目标转速与斜坡步数换算得到。*/
#define FREQ_INC (u16) ((FINAL_START_UP_SPEED*POLE_PAIR_NUM*65536/60)\
                                                        /FREQ_STARTUP_PWM_STEPS)
/* 电流斜坡阶段(I_START_UP_DURATION)持续的 PWM 周期数。*/
#define I_STARTUP_PWM_STEPS (u32) ((I_START_UP_DURATION * SAMPLING_FREQ)/1000) 
/* 每个 PWM 周期的电流增量（定标 ×1024），由起止电流差与斜坡步数换算。*/
#define I_INC (u16)((FINAL_I_STARTUP -FIRST_I_STARTUP)*1024/I_STARTUP_PWM_STEPS)
/* 方差阈值换算成观测器内部使用的百分比定标（0..127 对应 0..100%）。*/
#define PERCENTAGE_FACTOR    (u16)(VARIANCE_THRESHOLD*128)      
/* 启动验证所需的最小转速阈值，定标 0.1Hz（Hz×10）：rpm/6 ≈ 0.1Hz。*/
#define MINIMUM_SPEED        (u16) (MINIMUM_SPEED_RPM/6)
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MC_STATE_OBSERVER_PARAM_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
