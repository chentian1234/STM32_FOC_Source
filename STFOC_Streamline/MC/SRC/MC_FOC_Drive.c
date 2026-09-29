/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_FOC_Drive.c
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file provides all the PMSM FOC drive functions.
* 
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 14/07/08 v2.0.1
* 28/08/08 v2.0.2
* 04/09/08 v2.0.3
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
*   本文件是 PMSM FOC 驱动的主体，实现 FOC 电流环与速度环的计算流程:
*   - FOC_Init            :初始化内部 dq 电流参考；
*   - FOC_Model           :电流环(每个 PWM 周期执行):相电流采样→Clarke→Park→
*                           电流 PI→电压圆限制→反 Park→SVPWM；
*   - FOC_CalcFluxTorqueRef:速度环(按速度环周期执行):速度 PI→转矩/磁链电流参考。
*   本文件被主控制流程调用，依赖 MC_Clarke_Park、MC_PID_regulators 提供的变换与
*   PI 运算，以及 MC_Globals.h 的全局变量。
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "MC_const.h"
#include "MC_FOC_Drive.h"
#include "MC_PMSM_motor_param.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define FW_KDDIV        1     // 弱磁(Flux Weakening)调节器 Kd 定标除数(本工程未启用弱磁)
#define FW_KD_GAIN      0     // 弱磁调节器微分增益(0 表示不使用微分)
#define FW_D_TERM_INIT  0     // 弱磁微分项初值
#define VOLTAGE_SAMPLING_BUFFER 128   // 母线电压采样缓冲区长度(用于电压滤波)
/* Private macro -------------------------------------------------------------*/
/* SATURATION_TO_S16(a):饱和宏，把变量 a 限制在 [-S16_MAX, S16_MAX] 范围内。 */
#define SATURATION_TO_S16(a)    if (a > S16_MAX)              \
                                {                             \
                                  a = S16_MAX;                \
                                }                             \
                                else if (a < -S16_MAX)        \
                                {                             \
                                  a = -S16_MAX;               \
                                }                             \
/* Private functions ---------------------------------------------------------*/
/* Private variable ----------------------------------------------------------*/
static volatile Curr_Components Stat_Curr_q_d_ref;       // 速度环输出的 dq 电流参考(volatile:可能被其他上下文访问)
static Curr_Components Stat_Curr_q_d_ref_ref;            // 电流环实际使用的 dq 电流参考(最终给定)




/*******************************************************************************
* 功能说明(中文) : 初始化 FOC 模块内部的 dq 电流参考(清零)。
*                  在系统初始化阶段调用一次。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 清零静态变量 Stat_Curr_q_d_ref 与 Stat_Curr_q_d_ref_ref。
*******************************************************************************/
void FOC_Init (void)
{
  Stat_Curr_q_d_ref_ref.qI_Component1 = 0;   // q 轴电流给定清零
  Stat_Curr_q_d_ref_ref.qI_Component2 = 0;   // d 轴电流给定清零
  
  Stat_Curr_q_d_ref.qI_Component1 = 0;   // q 轴电流参考(速度环输出)清零
  Stat_Curr_q_d_ref.qI_Component2 = 0;   // d 轴电流参考(速度环输出)清零
}

extern u16  hTimePhA, hTimePhB, hTimePhC, hTimePhD;   // 三相/第四路 PWM 定时器比较值(外部定义)
extern u8  bSector;                                     // SVPWM 扇区号(外部定义)
/*******************************************************************************
* 功能说明(中文) : FOC 电流环主体，每个 PWM 中断(电流环周期)调用一次。
*                  流程:采样相电流→Clarke(三相→αβ)→取电角度→Park(αβ→dq)→
*                  转矩/磁链 PI(得到 dq 电压)→电压圆限制→反Park(dq→αβ)→
*                  SVPWM 计算三相占空比；并把各中间量写入 uartdat 供上位机观测。
* 参数(中文)     : 无(使用全局变量)。
* 返回(中文)     : 无。
* 备注(中文)     : 依赖全局 Stat_Curr_a_b、Stat_Curr_alfa_beta、Stat_Curr_q_d、
*                  Stat_Volt_q_d、Stat_Volt_alfa_beta、PID_Torque/Flux_InitStructure、
*                  uartdat、SavePtr，以及外部 hTimePhA/B/C、bSector。
*******************************************************************************/
void FOC_Model(void)
{
    s16 ang;
    static s16 preang;
    
    
    Stat_Curr_a_b = GET_PHASE_CURRENTS();   // 采样三相相电流(宏，来自 MC_Globals.h)
    Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b);   // Clarke:三相→两相静止 αβ
    
    ang = GET_ELECTRICAL_ANGLE;             // 获取当前电角度(s16 定标，0x8000 对应 180°)
    Stat_Curr_q_d = Park(Stat_Curr_alfa_beta,ang);   // Park:αβ→旋转 dq，得到反馈 Iq/Id
    
    
    Stat_Volt_q_d.qV_Component1 = PID_Regulator(Stat_Curr_q_d_ref_ref.qI_Component1, 
                        Stat_Curr_q_d.qI_Component1, &PID_Torque_InitStructure);   // 转矩环 PI:输出 q 轴电压
    
    
    
    Stat_Volt_q_d.qV_Component2 = PID_Regulator(Stat_Curr_q_d_ref_ref.qI_Component2, 
                          Stat_Curr_q_d.qI_Component2, &PID_Flux_InitStructure);   // 磁链环 PI:输出 d 轴电压  
    
    
    RevPark_Circle_Limitation();            // 电压圆限制:限制 dq 电压模值不过调制
    Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);   // 反 Park:dq→αβ 电压
    CALC_SVPWM(Stat_Volt_alfa_beta);        // SVPWM:由 αβ 电压计算三相占空比(宏)
    
    
    uartdat[0] = Stat_Curr_a_b.qI_Component1;   // 以下把各中间量写入 uartdat 供上位机监测
    uartdat[1] = Stat_Curr_a_b.qI_Component2;
    uartdat[2] = - Stat_Curr_a_b.qI_Component1 - Stat_Curr_a_b.qI_Component2;
    
    uartdat[3] = Stat_Curr_alfa_beta.qI_Component1;
    uartdat[4] = Stat_Curr_alfa_beta.qI_Component2;
    
    uartdat[5] = ang;
    
    uartdat[6] = Stat_Curr_q_d.qI_Component1;
    uartdat[7] = Stat_Curr_q_d.qI_Component2;
    
    uartdat[8] = Stat_Volt_q_d.qV_Component1;
    uartdat[9] = Stat_Volt_q_d.qV_Component2;
    
    uartdat[10] = Stat_Volt_alfa_beta.qV_Component1;
    uartdat[11] = Stat_Volt_alfa_beta.qV_Component2;
    
    uartdat[12] = hTimePhA;
    uartdat[13] = hTimePhB;
    uartdat[14] = hTimePhC;
    uartdat[15] = bSector;
    
    if( SavePtr == 0 )
    {
        if( preang < 0 && ang >= 0 )   // 电角度由负穿越零(检测过零点)
        {
            SaveForms();   // 触发一次波形数据保存
        }
    }
    else
        SaveForms();
    
    preang = ang;   // 记录本次电角度供下一次比较
}


/*******************************************************************************
* 功能说明(中文) : 速度环计算转矩电流参考。用速度 PI 比较速度参考 hSpeed_Reference
*                  与实测转速 GET_SPEED_0_1HZ，输出转矩(q 轴)电流参考；
*                  d 轴电流参考置 0(Id=0 控制)，并同步到全局参考量。
*                  按速度环采样周期调用。
* 参数(中文)     : 无(使用全局变量)。
* 返回(中文)     : 无。
* 备注(中文)     : 更新全局 Stat_Curr_q_d_ref / Stat_Curr_q_d_ref_ref /
*                  hTorque_Reference / hFlux_Reference。
*******************************************************************************/
void FOC_CalcFluxTorqueRef(void)
{
    //                                              1500            hRot_Speed
    Stat_Curr_q_d_ref.qI_Component1 = PID_Regulator(hSpeed_Reference,GET_SPEED_0_1HZ,&PID_Speed_InitStructure);   // 速度环 PI:输出转矩(q 轴)电流参考

    Stat_Curr_q_d_ref.qI_Component2 = 0;   // d 轴电流参考=0(Id=0 控制)
    Stat_Curr_q_d_ref_ref = Stat_Curr_q_d_ref;   // 同步给电流环使用的最终参考
 
    hTorque_Reference = Stat_Curr_q_d_ref_ref.qI_Component1;   // 更新全局转矩参考
    hFlux_Reference = Stat_Curr_q_d_ref_ref.qI_Component2;     // 更新全局磁链参考
}
