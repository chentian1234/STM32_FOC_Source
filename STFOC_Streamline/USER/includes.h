/*******************************************************************************
* 文件说明(中文):
*   本文件是本工程的"公共汇总头文件"。各 .c 源文件只需包含 includes.h，
*   即可一次性引入标准外设库、应用层(LED/串口)与本工程电机控制库的头文件，
*   避免在每个源文件里重复书写大量 #include。
*******************************************************************************/

#ifndef _INCLUDES_H_
#define _INCLUDES_H_



#include "stm32f10x.h"          // STM32F10x 标准外设库总头文件(寄存器定义、外设类型)
#include "led.h"                // 应用层: 状态指示灯驱动
#include "uart.h"               // 应用层: 串口(USART1)通信驱动
#include "UartProcess.h"        // 应用层: 串口命令处理/波形读取

#include "stm32f10x_type.h"     // 自定义整型与布尔类型定义(s16/u16/u32 等)

#include "stm32f10x_lib.h"      // 标准外设库汇总头文件
#include "stm32f10x_MClib.h"    // 电机控制库(MC library)汇总头文件
#include "MC_Globals.h"         // 电机控制库全局变量与类型声明



#endif
