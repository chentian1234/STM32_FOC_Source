/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_hall_prm.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : Contains the list of project specific parameters related
*                      to the Hall sensors speed feedback.
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

/* ==========================================================================
 * 文件说明(中文)
 *   本文件是霍尔(Hall)传感器速度/位置反馈模块(stm32f10x_hall.c)的工程参数
 *   配置头文件，集中定义了霍尔反馈所需的全部可调参数，包括：
 *     - 由哪个 16 位定时器承载霍尔信号(TIM2/TIM3/TIM4)；
 *     - 霍尔传感器的机械安装方式(120 度或 60 度电角度排布)；
 *     - 霍尔信号跳变沿相对 A 相反电动势(Bemf)峰值的电角度相位偏移；
 *     - 应用允许的转速上下限及越界时返回的哨兵值；
 *     - 定时器预分频上限、超时判定所需的溢出次数、速度滑动平均深度。
 *   在 FOC(磁场定向控制)中，霍尔传感器提供转子所处的 6 个离散扇区位置并
 *   据此推算转速；本文件即这些测量参数与门限的物理量纲/定标定义处。
 * ========================================================================== */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HALL_PRM_H
#define __HALL_PRM_H
/* Includes ------------------------------------------------------------------*/
#include "STM32F10x_MCconf.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* APPLICATION SPECIFIC DEFINE -----------------------------------------------*/
/* Define here the 16-bit timer chosen to handle hall sensors feedback  */
/* Timer 2 is the mandatory selection when using STM32MC-KIT  */
// 中文: 选择承载霍尔传感器反馈的 16 位定时器；三选一，只允许开启其中之一。
//       使用 STM32MC-KIT 评估板时，TIM2 为强制选择。
#define TIMER2_HANDLES_HALL        // 中文: 由 TIM2 采集霍尔信号(本工程启用)
//#define TIMER3_HANDLES_HALL      // 中文: 由 TIM3 采集霍尔信号(备用, 未启用)
//#define TIMER4_HANDLES_HALL      // 中文: 由 TIM4 采集霍尔信号(备用, 未启用)

/* HALL SENSORS PLACEMENT ----------------------------------------------------*/
// 中文: 霍尔传感器安装方式的枚举值。三个霍尔开关沿电角度圆周相隔 120 度
//       或 60 度放置，两种排布对应的霍尔状态跳变序列不同。
#define DEGREES_120 0              // 中文: 传感器相隔 120 电角度(本工程采用)
#define DEGREES_60  1              // 中文: 传感器相隔 60 电角度

/* Define here the mechanical position of the sensors with reference to an 
                                                             electrical cycle */ 
// 中文: 选择传感器相对一个电周期的机械安装方式；本工程采用 120 度排布。
#define HALL_SENSORS_PLACEMENT DEGREES_120

/* Define here in degrees the electrical phase shift between the low to high
transition of signal H1 and the maximum of the Bemf induced on phase A */

// 中文: H1 信号由低变高的跳变沿，相对 A 相 BEMF(反电动势)峰值之间的电角度
//       相位偏移，单位为「度」。用于把霍尔扇区中心对齐到正确的电角度。
//       本工程取 -60 度，表示霍尔跳变沿超前 BEMF 峰值 60 电角度。
#define HALL_PHASE_SHIFT (s16) -60 

/* APPLICATION SPEED DOMAIN AND ERROR/RANGE CHECKING -------------------------*/
// 中文: 应用转速范围与错误/越界检查参数区

/* Define here the rotor mechanical frequency above which speed feedback is not 
realistic in the application: this allows discriminating glitches for instance 
*/
// 中文: 定义应用允许的最高转子机械转速上限(rpm)。超过该值即认为反馈不合理，
//       可借此滤除毛刺/干扰造成的异常高速读数。
#define HALL_MAX_SPEED_FDBK_RPM          ((u32)30000) // Unit is rpm
                                                       // 中文: 单位 rpm(转/分)，此处为 30000 rpm

/* Define here the returned value if measured speed is > MAX_SPEED_FDBK_RPM
It could be 0 or FFFF depending on upper layer software management */
// 中文: 当测得转速 > HALL_MAX_SPEED_FDBK_RPM 时返回的哨兵值；其具体取值
//       由上层软件管理策略决定(可为 0 或 0xFFFF)。
#define HALL_MAX_SPEED               ((u16)5000) // Unit is 0.1Hz
                                                 // 中文: 单位 0.1Hz，即 5000 表示 500.0 Hz 机械频率
// With digit-per-PWM unit (here 2*PI rad = 0xFFFF):
// 中文: 采用「每 PWM 周期数字量」定标时(此处 2*PI 弧度 = 0xFFFF)表示的超速哨兵值。
#define HALL_MAX_PSEUDO_SPEED        ((s16)-32768)
                                     // 中文: s16 定标，代表最大伪转速哨兵(-32768 即 S16_MIN)

/* Define here the rotor mechanical frequency below which speed feedback is not 
realistic in the application: this allows to discriminate too low freq for 
instance */
// 中文: 定义应用允许的最低转子机械转速下限(rpm)。低于该值即认为反馈不合理，
//       用于剔除过低频率(例如启动瞬间的抖动)。
#define HALL_MIN_SPEED_FDBK_RPM          ((u16)60) // Unit is rpm
                                                    // 中文: 单位 rpm(转/分)，此处为 60 rpm

/* Max TIM prescaler ratio defining the lowest expected speed feedback */
// 中文: 定义能测得的最低转速所对应的定时器预分频比上限(取值 800)。
//       预分频比越大，定时器计数越慢，可测的捕获周期越长(对应转速越低)。
#define HALL_MAX_RATIO		((u16)800u)

/* Number of consecutive timer overflows without capture: this can indicate
that informations are lost or that speed is decreasing very sharply */
/* This is needed to implement hall sensors time-out. This duration depends on hall sensor
timer pre-scaler, which is variable; the time-out will be higher at low speed*/
// 中文: 连续定时器溢出但仍未捕获到霍尔跳变的次数上限。超过该次数即判定
//       「霍尔信号丢失」或「转速急剧下降」，用于实现霍尔反馈超时检测。
//       该超时时长取决于霍尔定时器的预分频比，且预分频比可变，因此在低速
//       时超时时长会更长。
#ifdef FLUX_TORQUE_PIDs_TUNING
#define HALL_MAX_OVERFLOWS       ((u16)4)   // 中文: PID 整定模式下的溢出次数阈值
#else
#define HALL_MAX_OVERFLOWS       ((u16)2)   // 中文: 正常运行模式下允许的连续溢出次数
#endif

/* ROLLING AVERAGE DEPTH -----------------------------------------------------*/
// 中文: 速度滑动平均(滚动平均)的深度，即用于平均的最近捕获次数(FIFO 长度)。
//       PID 整定模式下取 1(不做平均，直接反映瞬时值)；正常运行时取 6。
#ifdef FLUX_TORQUE_PIDs_TUNING
#define HALL_SPEED_FIFO_SIZE 	((u8)1)
#else
#define HALL_SPEED_FIFO_SIZE 	((u8)6)
#endif

/* Exported macro ------------------------------------------------------------*/
// 中文: 导出宏与导出函数声明区(本文件为纯参数配置，无函数声明)。
/* Exported functions ------------------------------------------------------- */

#endif /* __HALL_PRM_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
