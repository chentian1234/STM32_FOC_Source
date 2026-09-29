/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_const.h
* Author             : IMS Systems Lab  
* Date First Issued  : 21/11/07
* Description        : provides constants shared by all modules
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
*   本文件提供 FOC 库各模块共用的数学常数、状态/故障标志位以及正弦查表。
*   在 FOC 控制中的角色是:
*   - 提供 1/sqrt(3) 等 Clarke/反 Clarke 变换所需的定点定标常数；
*   - 提供 sin/cos 256 点查表(SIN_COS_TABLE)及查表偏移 OFFSET，
*     供 Trig_Functions 计算电角度的正弦/余弦；
*   - 提供系统状态与故障的标志位宏，供状态机与保护逻辑使用。
*   常数定标约定:q1.15 表示 Q1.15 定点格式，1.0 对应 32768(0x8000)。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MC_CONST_H
#define __MC_CONST_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define divSQRT_3	(s16) 0x49E6    /* 1/sqrt(3) in q1.15 format=0.5773315*/   // 1/sqrt(3) 的 Q1.15 定标值 0x49E6(约 0.577),Clarke 变换用
/* Offset for cos(Theta)=sin_cos_Table[index_sin+offset]*/
// OFFSET=64:256 点正弦表中 90° 对应的索引偏移(256/4),用于由 sin 查 cos
#define OFFSET 		64	
#define SQRT_3		1.732051   // sqrt(3) 浮点常数
#define PI              3.1416   // 圆周率 PI 浮点常数
#define SQRT_2          1.4142   // sqrt(2) 浮点常数

//to be deleted if not used by anyone
#define q0dot5            ((s16) 0x4000)       		//0.5 in 1.5   // Q1.15 定点表示的 0.5(即 0x4000)
#define qSqrtd2           ((s16) 0x6ED9) 			//sqrt3/2 in 1.15   // sqrt(3)/2 的 Q1.15 值(约 0.866)
#define	qSqrt_inverse            0x5A11		//sqrt(3)* Vdc_invT   // sqrt(3) 乘以母线电压倒数 的定点定标常数
#define	qSqrtd2_inverse          0x2D0f		//(sqrt(3))/2* Vdc_invT   // (sqrt(3)/2) 乘以母线电压倒数 的定点定标常数
#define	q3d2_inverse             0x4E0C			//3/2* Vdc_invT   // (3/2) 乘以母线电压倒数 的定点定标常数

#define SPEED_CONTROL           (u32)0x0001   // 状态标志位:速度控制使能
#define FIRST_START             (u32)0x0002   // 状态标志位:首次启动
#define START_UP_FAILURE        (u32)0x0004   // 状态标志位:启动失败
#define SPEED_FEEDBACK          (u32)0x0008   // 状态标志位:速度反馈有效
#define BRAKE_ON                (u32)0x0010   // 状态标志位:制动进行中
#define OVERHEAT                (u32)0x0100   // 故障标志位:过温
#define OVER_CURRENT            (u32)0x0200   // 故障标志位:过流
#define OVER_VOLTAGE            (u32)0x0400   // 故障标志位:过压
#define UNDER_VOLTAGE           (u32)0x0800   // 故障标志位:欠压

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Exported variables ------------------------------------------------------- */
/* 正弦查表数据(256 点，Q1.15)：覆盖电角度 0~90°。
   索引 i 对应角度 i*(90°/256)，表项 = sin(i*90°/256)*32768。
   通过本表配合 OFFSET=64 及象限对称性，可查得 0~360° 任意角的正弦/余弦。 */
#define SIN_COS_TABLE { \
0x0000,0x00C9,0x0192,0x025B,0x0324,0x03ED,0x04B6,0x057F,\
0x0648,0x0711,0x07D9,0x08A2,0x096A,0x0A33,0x0AFB,0x0BC4,\
0x0C8C,0x0D54,0x0E1C,0x0EE3,0x0FAB,0x1072,0x113A,0x1201,\
0x12C8,0x138F,0x1455,0x151C,0x15E2,0x16A8,0x176E,0x1833,\
0x18F9,0x19BE,0x1A82,0x1B47,0x1C0B,0x1CCF,0x1D93,0x1E57,\
0x1F1A,0x1FDD,0x209F,0x2161,0x2223,0x22E5,0x23A6,0x2467,\
0x2528,0x25E8,0x26A8,0x2767,0x2826,0x28E5,0x29A3,0x2A61,\
0x2B1F,0x2BDC,0x2C99,0x2D55,0x2E11,0x2ECC,0x2F87,0x3041,\
0x30FB,0x31B5,0x326E,0x3326,0x33DF,0x3496,0x354D,0x3604,\
0x36BA,0x376F,0x3824,0x38D9,0x398C,0x3A40,0x3AF2,0x3BA5,\
0x3C56,0x3D07,0x3DB8,0x3E68,0x3F17,0x3FC5,0x4073,0x4121,\
0x41CE,0x427A,0x4325,0x43D0,0x447A,0x4524,0x45CD,0x4675,\
0x471C,0x47C3,0x4869,0x490F,0x49B4,0x4A58,0x4AFB,0x4B9D,\
0x4C3F,0x4CE0,0x4D81,0x4E20,0x4EBF,0x4F5D,0x4FFB,0x5097,\
0x5133,0x51CE,0x5268,0x5302,0x539B,0x5432,0x54C9,0x5560,\
0x55F5,0x568A,0x571D,0x57B0,0x5842,0x58D3,0x5964,0x59F3,\
0x5A82,0x5B0F,0x5B9C,0x5C28,0x5CB3,0x5D3E,0x5DC7,0x5E4F,\
0x5ED7,0x5F5D,0x5FE3,0x6068,0x60EB,0x616E,0x61F0,0x6271,\
0x62F1,0x6370,0x63EE,0x646C,0x64E8,0x6563,0x65DD,0x6656,\
0x66CF,0x6746,0x67BC,0x6832,0x68A6,0x6919,0x698B,0x69FD,\
0x6A6D,0x6ADC,0x6B4A,0x6BB7,0x6C23,0x6C8E,0x6CF8,0x6D61,\
0x6DC9,0x6E30,0x6E96,0x6EFB,0x6F5E,0x6FC1,0x7022,0x7083,\
0x70E2,0x7140,0x719D,0x71F9,0x7254,0x72AE,0x7307,0x735E,\
0x73B5,0x740A,0x745F,0x74B2,0x7504,0x7555,0x75A5,0x75F3,\
0x7641,0x768D,0x76D8,0x7722,0x776B,0x77B3,0x77FA,0x783F,\
0x7884,0x78C7,0x7909,0x794A,0x7989,0x79C8,0x7A05,0x7A41,\
0x7A7C,0x7AB6,0x7AEE,0x7B26,0x7B5C,0x7B91,0x7BC5,0x7BF8,\
0x7C29,0x7C59,0x7C88,0x7CB6,0x7CE3,0x7D0E,0x7D39,0x7D62,\
0x7D89,0x7DB0,0x7DD5,0x7DFA,0x7E1D,0x7E3E,0x7E5F,0x7E7E,\
0x7E9C,0x7EB9,0x7ED5,0x7EEF,0x7F09,0x7F21,0x7F37,0x7F4D,\
0x7F61,0x7F74,0x7F86,0x7F97,0x7FA6,0x7FB4,0x7FC1,0x7FCD,\
0x7FD8,0x7FE1,0x7FE9,0x7FF0,0x7FF5,0x7FF9,0x7FFD,0x7FFE}

#endif /* __MC_CONST_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
