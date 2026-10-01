/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_PID_regulators.c
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file contains the software implementation for the
                       PI(D) regulators.
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
*   本文件实现定点 PI(D) 调节器，供 FOC 的三个闭环使用:
*   - 转矩环(电流 q 轴)、磁链环(电流 d 轴):FOC_Model 每个 PWM 周期调用一次；
*   - 速度环:FOC_CalcFluxTorqueRef 按速度环采样周期调用。
*   PID 输出 = Kp*误差/Kp_Divisor + 积分累加/Ki_Divisor (+ Kd*误差差分/Kd_Divisor)。
*   所有增益均为定点整数，Kp_Divisor/Ki_Divisor/Kd_Divisor 为定标除数。
*   积分项含上下限以抗积分饱和(anti-windup)。
*   本文件被 MC_FOC_Drive.c 调用，依赖 MC_Globals.h 的全局变量与 MC_type.h 的
*   PID_Struct_t 类型。
*******************************************************************************/

/* Standard include ----------------------------------------------------------*/

#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "stm32f10x_type.h"
#include "MC_Globals.h"

#define PID_SPEED_REFERENCE  (u16)(PID_SPEED_REFERENCE_RPM/6)   // 速度参考换算:rpm→s16 内部定标(除以 6)

typedef signed long long s64;   // 64 位有符号整数，用于积分项中间运算防止溢出

/*******************************************************************************
* 功能说明 : 初始化 FOC 的三个 PI(D) 调节器(转矩环/磁链环/速度环)的
*            Kp/Ki/Kd 增益、定标除数、输出限幅、积分限幅，并清零积分项。
*            在系统初始化阶段调用一次。
* 参数     : PID_Torque - 转矩环(q 轴电流)参数结构体指针；
*            PID_Flux   - 磁链环(d 轴电流)参数结构体指针；
*            PID_Speed  - 速度环参数结构体指针。
* 返回     : 无。
* 备注     : 同时设置全局参考量 hTorque_Reference/hFlux_Reference/hSpeed_Reference；
*            速度环输出以 ±IQMAX 限幅(作为转矩电流参考上限)。
*******************************************************************************/
void PID_Init (PID_Struct_t *PID_Torque, PID_Struct_t *PID_Flux, PID_Struct_t *PID_Speed)
{
  hTorque_Reference = PID_TORQUE_REFERENCE;   // 设定转矩环初始参考(转矩电流参考)

  PID_Torque->hKp_Gain    = PID_TORQUE_KP_DEFAULT;// 转矩环比例项增益
  PID_Torque->hKp_Divisor = TF_KPDIV;   // 转矩环比例项定标除数(TF 通道)

  PID_Torque->hKi_Gain = PID_TORQUE_KI_DEFAULT;
  PID_Torque->hKi_Divisor = TF_KIDIV;   // 转矩环积分项定标除数
  
  PID_Torque->hKd_Gain = PID_TORQUE_KD_DEFAULT;
  PID_Torque->hKd_Divisor = TF_KDDIV;   // 转矩环微分项定标除数
  PID_Torque->wPreviousError = 0;
  
  PID_Torque->hLower_Limit_Output=S16_MIN;   // 输出下限(负满量程)
  PID_Torque->hUpper_Limit_Output= S16_MAX;   // 输出上限(正满量程)
  PID_Torque->wLower_Limit_Integral = S16_MIN * TF_KIDIV;   // 积分项下限 = S16_MIN*KiDiv(抗积分饱和)
  PID_Torque->wUpper_Limit_Integral = S16_MAX * TF_KIDIV;   // 积分项上限 = S16_MAX*KiDiv(抗积分饱和)
  PID_Torque->wIntegral = 0;
 
  /**************************************************/
  /************END PID Torque Regulator members*******/
  /**************************************************/

  /**************************************************/
  /************PID Flux Regulator members*************/
  /**************************************************/

  PID_Flux->wIntegral = 0;  // 清零积分项

  hFlux_Reference = PID_FLUX_REFERENCE;   // 设定磁链环(d 轴电流)初始参考

  PID_Flux->hKp_Gain    = PID_FLUX_KP_DEFAULT;
  PID_Flux->hKp_Divisor = TF_KPDIV;   // 磁链环比例项定标除数

  PID_Flux->hKi_Gain = PID_FLUX_KI_DEFAULT;
  PID_Flux->hKi_Divisor = TF_KIDIV;   // 磁链环积分项定标除数
  
  PID_Flux->hKd_Gain = PID_FLUX_KD_DEFAULT;
  PID_Flux->hKd_Divisor = TF_KDDIV;   // 磁链环微分项定标除数
  PID_Flux->wPreviousError = 0;
  
  PID_Flux->hLower_Limit_Output=S16_MIN;   // 输出下限(负满量程)
  PID_Flux->hUpper_Limit_Output= S16_MAX;   // 输出上限(正满量程)
  PID_Flux->wLower_Limit_Integral = S16_MIN * TF_KIDIV;   // 积分项下限(抗积分饱和)
  PID_Flux->wUpper_Limit_Integral = S16_MAX * TF_KIDIV;   // 积分项上限(抗积分饱和)
  PID_Flux->wIntegral = 0;
  
  /**************************************************/
  /************END PID Flux Regulator members*********/
  /**************************************************/

  /**************************************************/
  /************PID Speed Regulator members*************/
  /**************************************************/


  PID_Speed->wIntegral = 0;  // 清零积分项

  hSpeed_Reference = PID_SPEED_REFERENCE;   // 设定速度环初始参考(内部定标)

  PID_Speed->hKp_Gain    = PID_SPEED_KP_DEFAULT;
  PID_Speed->hKp_Divisor = SP_KPDIV;   // 速度环比例项定标除数(SP 通道)

  PID_Speed->hKi_Gain = PID_SPEED_KI_DEFAULT;
  PID_Speed->hKi_Divisor = SP_KIDIV;   // 速度环积分项定标除数
  
  PID_Speed->hKd_Gain = PID_SPEED_KD_DEFAULT;
  PID_Speed->hKd_Divisor = SP_KDDIV;   // 速度环微分项定标除数
  PID_Speed->wPreviousError = 0;
  
  PID_Speed->hLower_Limit_Output= -IQMAX;   // 速度环输出下限=-IQMAX(转矩电流参考下限)
  PID_Speed->hUpper_Limit_Output= IQMAX;   // 速度环输出上限=IQMAX(转矩电流参考上限)
  PID_Speed->wLower_Limit_Integral = -IQMAX * SP_KIDIV;   // 积分项下限(抗积分饱和)
  PID_Speed->wUpper_Limit_Integral = IQMAX * SP_KIDIV;   // 积分项上限(抗积分饱和)
  PID_Speed->wIntegral = 0;
  /**************************************************/
  /**********END PID Speed Regulator members*********/
  /**************************************************/

}

/*******************************************************************************
* 功能说明 : 根据电机转速对速度环的 Kp/Ki/Kd 做分段线性插值更新。
*            速度区间划分为 [Freq_Min, F_1]、[F_1, F_2]、[F_2, Freq_Max]
*            三段，段内按 alpha_Kx/1024 的斜率线性插值，区间外取端点常数。
*            在速度环每次执行前调用。
* 参数     : motor_speed - 机械转速，0.1Hz 分辨率(如 10Hz 对应 100)，
*                          内部先取绝对值再分段；s16。
*            PID_Struct  - 待更新的速度环 PID 参数结构体指针。
* 返回     : 无。
* 备注     : alpha_Kp_1/alpha_Ki_1 等来自 MC_Control_Param.h，已乘 1024 放大，
*            故插值时再除以 1024(即 alpha_Kx/1024 为每 0.1Hz 的系数增量)。
*******************************************************************************/
void PID_Speed_Coefficients_update(s16 motor_speed, PID_Struct_t *PID_Struct)
{
if ( motor_speed < 0)  
{
  motor_speed = (u16)(-motor_speed);   // 取绝对值，只用转速大小分段
}

if ( motor_speed <= Freq_Min )    // 转速低于 Freq_Min? 
{
  PID_Struct->hKp_Gain = Kp_Fmin;   // 转速<=Fmin:取最低速段常数 Kp
  PID_Struct->hKi_Gain = Ki_Fmin;   //                取最低速段常数 Ki

  #ifdef DIFFERENTIAL_TERM_ENABLED
  PID_Struct->hKd_Gain =Kd_Fmin;
  #endif
}
else if ( motor_speed <= F_1 )
{
  PID_Struct->hKp_Gain = Kp_Fmin + (s32)(alpha_Kp_1*(motor_speed - Freq_Min) / 1024);   // 第1段线性插值:Kp
  PID_Struct->hKi_Gain = Ki_Fmin + (s32)(alpha_Ki_1*(motor_speed - Freq_Min) / 1024);   // 第1段线性插值:Ki

  #ifdef DIFFERENTIAL_TERM_ENABLED
  PID_Struct->hKd_Gain = Kd_Fmin + (s32)(alpha_Kd_1*(motor_speed - Freq_Min) / 1024);
  #endif
}
else if ( motor_speed <= F_2 )
{
  PID_Struct->hKp_Gain = Kp_F_1 + (s32)(alpha_Kp_2 * (motor_speed-F_1) / 1024);   // 第2段线性插值:Kp
  PID_Struct->hKi_Gain = Ki_F_1 + (s32)(alpha_Ki_2 * (motor_speed-F_1) / 1024);   // 第2段线性插值:Ki

  #ifdef DIFFERENTIAL_TERM_ENABLED
  PID_Struct->hKd_Gain = Kd_F_1 + (s32)(alpha_Kd_2 * (motor_speed-F_1) / 1024);
  #endif
}
else if ( motor_speed <= Freq_Max )
{
  PID_Struct->hKp_Gain = Kp_F_2 + (s32)(alpha_Kp_3 * (motor_speed-F_2) / 1024);   // 第3段线性插值:Kp
  PID_Struct->hKi_Gain = Ki_F_2 + (s32)(alpha_Ki_3 * (motor_speed-F_2) / 1024);   // 第3段线性插值:Ki

  #ifdef DIFFERENTIAL_TERM_ENABLED
  PID_Struct->hKd_Gain = Kd_F_2 + (s32)(alpha_Kd_3 * (motor_speed-F_2) / 1024);
  #endif
}
else  // 转速超过 Freq_Max? 
{
  PID_Struct->hKp_Gain = Kp_Fmax;   // 转速>Fmax:取最高速段常数 Kp
  PID_Struct->hKi_Gain = Ki_Fmax;   //                取最高速段常数 Ki

  #ifdef DIFFERENTIAL_TERM_ENABLED
  PID_Struct->hKd_Gain = Kd_Fmax;
  #endif
}
}

/*******************************************************************************
* 功能说明 : 执行一次 PI(D) 运算:计算误差→比例项→积分项(带抗饱和限幅)
*            →(可选)微分项，求和并做输出限幅后返回。被电流环/速度环调用。
* 参数     : hReference       - 参考值(给定)，s16；
*            hPresentFeedback - 反馈值，s16；
*            PID_Struct       - 该环的 PID 参数结构体指针。
* 返回     : 限幅后的调节输出，s16(含义随环而定:电压或转矩电流参考)。
* 备注     : 若 Ki_Gain==0 则清积分(纯比例)；积分上下限 wUpper/wLower_Limit_Integral
*            用于抗积分饱和；输出上下限 hUpper/hLower_Limit_Output 用于最终限幅。
*            微分项仅在 DIFFERENTIAL_TERM_ENABLED 定义时参与运算。
*******************************************************************************/
s16 PID_Regulator(s16 hReference, s16 hPresentFeedback, PID_Struct_t *PID_Struct)
{
    s32 wError, wProportional_Term,wIntegral_Term, houtput_32;
    s64 dwAux; 
#ifdef DIFFERENTIAL_TERM_ENABLED    
    s32 wDifferential_Term;
#endif    
    // 计算误差
    wError= (s32)(hReference - hPresentFeedback);   // 误差 = 参考 - 反馈(s32 防溢出)
 
    // 计算比例项
    wProportional_Term = PID_Struct->hKp_Gain * wError;   // 比例项 = Kp*误差(尚未除以 KpDiv)

    // 计算积分项
    if (PID_Struct->hKi_Gain == 0)
    {
        PID_Struct->wIntegral = 0;   // Ki 为 0(纯比例):清零积分，避免残留
    }
    else
    { 
        wIntegral_Term = PID_Struct->hKi_Gain * wError;   // 本次积分增量 = Ki*误差
        dwAux = PID_Struct->wIntegral + (s64)(wIntegral_Term);   // 累加到积分项(64 位防溢出)
    
        if (dwAux > PID_Struct->wUpper_Limit_Integral)
        {
            PID_Struct->wIntegral = PID_Struct->wUpper_Limit_Integral;
        }
        else if (dwAux < PID_Struct->wLower_Limit_Integral)
        { 
            PID_Struct->wIntegral = PID_Struct->wLower_Limit_Integral;
        }
        else
        {
            PID_Struct->wIntegral = (s32)(dwAux);
        }
    }
    // 计算微分项
#ifdef DIFFERENTIAL_TERM_ENABLED
    {
        s32 wtemp;
  
        wtemp = wError - PID_Struct->wPreviousError;   // 误差差分(本次误差 - 上次误差)
        wDifferential_Term = PID_Struct->hKd_Gain * wtemp;   // 微分项 = Kd*误差差分
        PID_Struct->wPreviousError = wError;    // 保存本次误差供下周期使用
    }
    houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 
                PID_Struct->wIntegral/PID_Struct->hKi_Divisor + 
                wDifferential_Term/PID_Struct->hKd_Divisor);    // 输出 = P/KpDiv + I/KiDiv + D/KdDiv

#else  
    houtput_32 = (wProportional_Term/PID_Struct->hKp_Divisor+ 
                PID_Struct->wIntegral/PID_Struct->hKi_Divisor);    // 无微分:输出 = P/KpDiv + I/KiDiv
#endif
  
    if (houtput_32 >= PID_Struct->hUpper_Limit_Output)   // 超出上限?
    {
        return(PID_Struct->hUpper_Limit_Output);		  			 	// 输出上限钳位
    }
    else if (houtput_32 < PID_Struct->hLower_Limit_Output)   // 低于下限?
    {
        return(PID_Struct->hLower_Limit_Output);   // 输出下限钳位
    }
    else 
    {
        return((s16)(houtput_32)); 		// 在限幅范围内:转换为 s16 返回
    }
}		   

/******************** (C) COPYRIGHT 2008 STMicroelectronics *******************/