/*******************************************************************************
* 文件说明(中文):
*   本文件是"printf 重定向到串口 1"模块的头文件。
*   只对外声明串口 1 初始化函数 Uart1Init()；printf 的重定向通过实现标准库
*   的 putchar() 完成(见 Uart1Printf.c)。
*******************************************************************************/

#ifndef __UART1PRINTF_H
#define __UART1PRINTF_H

#include "stm32f10x.h"      // 标准外设库头文件
#include <stdio.h>          // 标准输入输出库，声明 printf/putchar 等


void Uart1Init( void );     // 初始化 USART1(115200-8-N-1)，供 printf 调试输出使用


#endif
