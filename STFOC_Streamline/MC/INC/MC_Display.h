/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Display.h
* Author             : IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : Contains the prototypes of the LCD display module related
*                      routines.
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
* 中文文件说明(模块作用) :
*   本头文件定义 LCD 显示模块的"菜单索引"常量与对外函数原型。
*   bMenu_index(在 MC_Keys.c 中定义)保存当前菜单索引, 每个索引对应一个可调项/
*   显示页; MC_Display.c 的 ComputeVisualization() 据此决定 LCD 显示哪一屏内容,
*   并进一步决定按键(UP/DOWN)调节的对象。菜单项分组: 控制模式/参考转速/速度环
*   PID/转矩环 PID/磁链环 PID/功率级状态/转矩控制/故障显示等, 另含可选的观测器
*   增益、DAC、弱磁等扩展菜单(由相应宏条件编译)。
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DISPLAY_H
#define __DISPLAY_H


/* Exported constants --------------------------------------------------------*/
/* 以下为菜单索引(bMenu_index 的取值): 每个菜单对应 LCD 上的一屏显示内容与一个可调对象。 */

/* —— 速度控制相关菜单 —— */
#define CONTROL_MODE_MENU_1    (u8) 0    // 菜单0: 控制模式选择(速度控制模式, 显示目标/实测转速)
#define REF_SPEED_MENU         (u8) 1    // 菜单1: 参考转速设定(显示目标/实测转速)

/* —— 速度环 PID 参数菜单 —— */
#define P_SPEED_MENU           (u8) 2    // 菜单2: 速度环比例增益 Kp 调节
#define I_SPEED_MENU           (u8) 3    // 菜单3: 速度环积分增益 Ki 调节
#define D_SPEED_MENU           (u8) 4    // 菜单4: 速度环微分增益 Kd 调节(需开 DIFFERENTIAL_TERM_ENABLED)

/* —— 转矩环(q 轴电流) PID 参数菜单 —— */
#define P_TORQUE_MENU          (u8) 5    // 菜单5: 转矩环比例增益 Kp 调节
#define I_TORQUE_MENU          (u8) 6    // 菜单6: 转矩环积分增益 Ki 调节
#define D_TORQUE_MENU          (u8) 7    // 菜单7: 转矩环微分增益 Kd 调节(需开 DIFFERENTIAL_TERM_ENABLED)

/* —— 磁链环(d 轴电流) PID 参数菜单 —— */
#define P_FLUX_MENU            (u8) 8    // 菜单8: 磁链环比例增益 Kp 调节
#define I_FLUX_MENU            (u8) 9    // 菜单9: 磁链环积分增益 Ki 调节
#define D_FLUX_MENU            (u8) 10   // 菜单10: 磁链环微分增益 Kd 调节(需开 DIFFERENTIAL_TERM_ENABLED)

/* —— 功率级状态显示菜单 —— */
#define POWER_STAGE_MENU       (u8) 11   // 菜单11: 功率级状态(显示母线电压/功率级温度)

/* —— 转矩控制相关菜单 —— */
#define CONTROL_MODE_MENU_6    (u8) 12   // 菜单12: 控制模式选择(转矩控制模式)
#define IQ_REF_MENU            (u8) 13   // 菜单13: q 轴(转矩)电流参考给定调节
#define ID_REF_MENU            (u8) 14   // 菜单14: d 轴(磁链)电流参考给定调节

/* —— 故障/等待菜单 —— */
#define FAULT_MENU             (u8) 15   // 菜单15: 故障显示(依据 wGlobal_Flags 显示故障类型)
#define WAIT_MENU              (u8) 16   // 菜单16: 等待菜单(电机停转等待界面)

/* 反电动势观测器/PLL 增益整定菜单(索引17~20), 仅当定义 OBSERVER_GAIN_TUNING 时有效。 */
#ifdef OBSERVER_GAIN_TUNING
#define K1_MENU				   (u8) 17
#define K2_MENU				   (u8) 18
#define P_PLL_MENU			   (u8) 19
#define I_PLL_MENU			   (u8) 20
#endif

/* DAC 输出变量选择菜单(索引21~22), 仅当定义 DAC_FUNCTIONALITY 时有效。 */
#ifdef DAC_FUNCTIONALITY
#define DAC_PB0_MENU		           (u8) 21
#define DAC_PB1_MENU                       (u8) 22
#endif

/* 弱磁(Flux Weakening)电压环调节菜单(索引23~25), 仅当定义 FLUX_WEAKENING 时有效。 */
#ifdef FLUX_WEAKENING
#define P_VOLT_MENU             (u8)23
#define I_VOLT_MENU             (u8)24
#define TARGET_VOLT_MENU        (u8)25
#endif

/* Exported functions ------------------------------------------------------- */
/*******************************************************************************
* 功能说明(中文) : 显示主刷新函数。由 TB 显示时间基准定时(约每 200ms)触发, 依据全局
*                  菜单索引 bMenu_index 计算应显示的界面并刷新 LCD(数值/颜色高亮)。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 由主循环周期调用; 内部依赖 ComputeVisualization 与 5 位数字显示。
*******************************************************************************/
void Display_LCD(void);            // LCD 主刷新函数(周期性调用)
/*******************************************************************************
* 功能说明(中文) : 上电时在 LCD 上显示欢迎信息(产品名/版本/操作提示)。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 通常仅在系统初始化阶段调用一次。
*******************************************************************************/
void Display_Welcome_Message(void); // 上电欢迎界面显示

#endif  /*__DISPLAY_H*/
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
