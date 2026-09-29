/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_type.h
* Author             : MCD Application Team
* Version            : V2.0
* Date               : 05/23/2008
* Description        : This file contains all the common data types used for the
*                      STM32F10x firmware library.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED 
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*******************************************************************************/

/*******************************************************************************
* 文件说明(中文):
*   本文件定义工程中常用的自定义数据类型。
*   主要提供布尔类型 bool(FALSE/TRUE)，以及各整型上下限常量宏
*   (U8_MAX/S16_MIN/U32_MAX 等)，便于书写边界判断。
*   文件中的 s8/s16/s32、u8/u16/u32 等 ST 旧版类型定义被 #if 0 屏蔽，
*   实际由标准外设库(CMSIS)中的 stdint 类型(uint8_t/int16_t 等)承担。
*   位宽约定: u8=8 位, u16=16 位, u32=32 位；s 为有符号、u 为无符号。
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F10x_TYPE_H
#define __STM32F10x_TYPE_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
#if 0   // 以下旧版类型定义被屏蔽: 现由标准库 stdint.h 的类型(uint8_t/int16_t 等)替代
typedef signed long  s32;       // 有符号 32 位整数
typedef signed short s16;       // 有符号 16 位整数
typedef signed char  s8;        // 有符号 8 位整数

typedef signed long  const sc32;  /* Read Only */    // 只读: 有符号 32 位常量
typedef signed short const sc16;  /* Read Only */    // 只读: 有符号 16 位常量
typedef signed char  const sc8;   /* Read Only */    // 只读: 有符号 8 位常量

typedef volatile signed long  vs32;      // 易变(volatile)有符号 32 位，常用于寄存器/共享变量
typedef volatile signed short vs16;      // 易变有符号 16 位
typedef volatile signed char  vs8;       // 易变有符号 8 位

typedef volatile signed long  const vsc32;  /* Read Only */   // 只读易变有符号 32 位
typedef volatile signed short const vsc16;  /* Read Only */   // 只读易变有符号 16 位
typedef volatile signed char  const vsc8;   /* Read Only */   // 只读易变有符号 8 位

typedef unsigned long  u32;     // 无符号 32 位整数
typedef unsigned short u16;     // 无符号 16 位整数
typedef unsigned char  u8;      // 无符号 8 位整数

typedef unsigned long  const uc32;  /* Read Only */   // 只读无符号 32 位
typedef unsigned short const uc16;  /* Read Only */   // 只读无符号 16 位
typedef unsigned char  const uc8;   /* Read Only */   // 只读无符号 8 位

typedef volatile unsigned long  vu32;    // 易变无符号 32 位
typedef volatile unsigned short vu16;    // 易变无符号 16 位
typedef volatile unsigned char  vu8;     // 易变无符号 8 位

typedef volatile unsigned long  const vuc32;  /* Read Only */   // 只读易变无符号 32 位
typedef volatile unsigned short const vuc16;  /* Read Only */   // 只读易变无符号 16 位
typedef volatile unsigned char  const vuc8;   /* Read Only */   // 只读易变无符号 8 位



typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;    // 标志/中断状态: RESET=0, SET=1

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;  // 使能状态: DISABLE=0, ENABLE=1
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))   // 参数检查宏

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;         // 返回状态: ERROR=0, SUCCESS=1
#endif

typedef enum {FALSE = 0, TRUE = !FALSE} bool;   // 布尔类型: FALSE=0, TRUE=1









// 各整型取值范围的常量宏(注: 使用了上面被屏蔽的 u8/s8 等类型名，实际工程未使用)
#define U8_MAX     ((u8)255)                // 无符号 8 位最大值  255
#define S8_MAX     ((s8)127)                // 有符号 8 位最大值  127
#define S8_MIN     ((s8)-128)               // 有符号 8 位最小值 -128
#define U16_MAX    ((u16)65535u)            // 无符号 16 位最大值 65535
#define S16_MAX    ((s16)32767)             // 有符号 16 位最大值 32767
#define S16_MIN    ((s16)-32768)            // 有符号 16 位最小值 -32768
#define U32_MAX    ((u32)4294967295uL)      // 无符号 32 位最大值 4294967295
#define S32_MAX    ((s32)2147483647)        // 有符号 32 位最大值 2147483647
#define S32_MIN    ((s32)-2147483648)       // 有符号 32 位最小值 -2147483648

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __STM32F10x_TYPE_H */

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
