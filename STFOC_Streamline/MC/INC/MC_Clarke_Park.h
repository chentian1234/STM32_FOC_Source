/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Clarke_Park.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This module implements the reference frame transformations
*                      needed for vector control: Clarke, Park and Reverse Park.
*                      It also performs the voltage circle limitation.
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 14/07/08 v2.0.1
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
*   本文件声明 FOC 矢量控制所需的坐标变换与电压圆限制接口:
*   - Clarke 变换:三相定子电流 → 两相静止 αβ 坐标系；
*   - Park 变换:静止 αβ → 转子磁链同步旋转 dq 坐标系；
*   - 反 Park 变换:旋转 dq → 静止 αβ；
*   - RevPark_Circle_Limitation:电压矢量圆限制(限制 dq 电压幅值)；
*   - Trig_Functions:由电角度查表得到 sin/cos。
*   这些函数被 MC_FOC_Drive.c 的 FOC_Model()(电流环，PWM 周期调用)使用。
*   头文件同时按调制比定义了 MAX_MODULE(电压圆半径上限)。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_CLARKE_PARK_H
#define __MC_CLARKE_PARK_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "MC_type.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* 电压矢量圆限制的上限 MAX_MODULE(电压模值上限)：
   将 dq 参考电压 (Vd,Vq) 的模值限制在以 MAX_MODULE 为半径的圆内，
   以满足 SVPWM(空间矢量脉宽调制)对调制比的要求，防止过调制。
   数值 = 32767 × 调制比百分比(由 MC_Control_Param.h 选择具体档位)。 */
#ifdef MAX_MODULATION_77_PER_CENT
#define MAX_MODULE      25230   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*77%    // 圆半径上限 = 32767×77%(调制比 77%) 
#endif

#ifdef MAX_MODULATION_79_PER_CENT
#define MAX_MODULE      25885   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*79%    // 圆半径上限 = 32767×79%(调制比 79%) 
#endif

#ifdef MAX_MODULATION_81_PER_CENT
#define MAX_MODULE      26541   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*81%    // 圆半径上限 = 32767×81%(调制比 81%) 
#endif

#ifdef MAX_MODULATION_83_PER_CENT
#define MAX_MODULE      27196   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*83%    // 圆半径上限 = 32767×83%(调制比 83%) 
#endif

#ifdef MAX_MODULATION_85_PER_CENT
#define MAX_MODULE      27851   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*85%    // 圆半径上限 = 32767×85%(调制比 85%)  
#endif

#ifdef MAX_MODULATION_87_PER_CENT
#define MAX_MODULE      28507   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*87%    // 圆半径上限 = 32767×87%(调制比 87%)  
#endif

#ifdef MAX_MODULATION_89_PER_CENT
#define MAX_MODULE      29162   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*89%    // 圆半径上限 = 32767×89%(调制比 89%)
#endif

#ifdef MAX_MODULATION_91_PER_CENT
#define MAX_MODULE      29817   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*91%    // 圆半径上限 = 32767×91%(调制比 91%)
#endif

#ifdef MAX_MODULATION_92_PER_CENT
#define MAX_MODULE      30145   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*92%    // 圆半径上限 = 32767×92%(调制比 92%)
#endif

#ifdef MAX_MODULATION_93_PER_CENT
#define MAX_MODULE      30473   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*93%    // 圆半径上限 = 32767×93%(调制比 93%)
#endif

#ifdef MAX_MODULATION_94_PER_CENT
#define MAX_MODULE      30800   //root(Vd^2+Vq^2) <= MAX_MODULE = 32767*94%    // 圆半径上限 = 32767×94%(调制比 94%)
#endif

#ifdef MAX_MODULATION_95_PER_CENT
#define MAX_MODULE      31128   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*95%    // 圆半径上限 = 32767×95%(调制比 95%)
#endif

#ifdef MAX_MODULATION_96_PER_CENT
#define MAX_MODULE      31456   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*96%    // 圆半径上限 = 32767×96%(调制比 96%,本项目所选)
#endif

#ifdef MAX_MODULATION_97_PER_CENT
#define MAX_MODULE      31783   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*97%    // 圆半径上限 = 32767×97%(调制比 97%)
#endif

#ifdef MAX_MODULATION_98_PER_CENT
#define MAX_MODULE      32111   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*98%    // 圆半径上限 = 32767×98%(调制比 98%)
#endif

#ifdef MAX_MODULATION_99_PER_CENT
#define MAX_MODULE      32439   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*99%    // 圆半径上限 = 32767×99%(调制比 99%)
#endif

#ifdef MAX_MODULATION_100_PER_CENT
#define MAX_MODULE      32767   // root(Vd^2+Vq^2) <= MAX_MODULE = 32767*100%    // 圆半径上限 = 32767×100%(调制比 100%)
#endif

/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Clarke 变换(三相→两相静止 αβ 坐标系)：输入三相定子电流，
   输出两相静止坐标系电流(α、β 分量)。 */
Curr_Components Clarke(Curr_Components);
/* Park 变换(静止 αβ→同步旋转 dq 坐标系)：输入 αβ 电流与电角度，
   输出 dq 旋转坐标系电流(q、d 分量)。第二个参数为 s16 电角度。 */
Curr_Components Park(Curr_Components,s16);
/* 电压圆限制：将全局 dq 参考电压的模值限制到 MAX_MODULE 以内(无参无返回)。 */
void RevPark_Circle_Limitation(void);
/* 反 Park 变换(同步旋转 dq→静止 αβ 坐标系)：输入 dq 电压，输出 αβ 电压。 */
Volt_Components Rev_Park(Volt_Components Volt_Input); 
/* 三角函数查表：输入 s16 电角度，返回该角度的 sin/cos(Trig_Components)。 */
Trig_Components Trig_Functions(s16 hAngle);

#endif //__MC_CLARKE_PARK_H
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
