/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Keys.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file contains the prototypes for the push-buttons
*                      and joystick unit module related functions.
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
*   本头文件声明"按键/摇杆"人机交互模块对外提供的接口与按键代码常量。
*   该模块通过轮询读取五向摇杆(上/下/左/右/选择)与用户按键的 GPIO 电平，
*   完成去抖动后向主循环返回按键代码，用于电机启停控制与本地菜单浏览。
*   其中各按键代码(NOKEY/SEL/RIGHT/LEFT/UP/DOWN/KEY_HOLD)是按键扫描的返回值，
*   也是菜单/显示等模块判读当前按键操作的依据。
*******************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MCx_KEYS_H
#define __MCx_KEYS_H

/* Exported constants --------------------------------------------------------*/
// These are defines for reading the joystick and SEL button
/* 以下为按键扫描函数的返回值(按键代码), 用于标识本次扫描到的按键事件。 */
#define  NOKEY      (u8)0    // 无键按下(空闲/松开状态)
#define  SEL        (u8)1    // 选择/确认键(SEL 键或用户按键)被按下
#define  RIGHT      (u8)2    // 摇杆"右键"被按下(菜单中用于移动/返回)
#define  LEFT       (u8)3    // 摇杆"左键"被按下(菜单中用于移动)
#define  UP         (u8)4    // 摇杆"上键"被按下(增加给定值)
#define  DOWN       (u8)5    // 摇杆"下键"被按下(减小给定值)
#define  KEY_HOLD   (u8)6    // 按键保持(长按): 同一按键在连续扫描中始终被按下

/* Exported variables ------------------------------------------------------- */
extern u8 bMenu_index;   // 当前菜单索引(取值见 MC_Display.h 的 xxx_MENU 宏), 决定 LCD 显示内容与按键操作对象

/* Exported functions ------------------------------------------------------- */
/*******************************************************************************
* 功能说明(中文) : 初始化摇杆五向键(上/下/左/右/选择)与用户按键所在的 GPIO,
*                  统一配置为浮空输入模式。系统上电时调用一次。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 使能 GPIOA/GPIOB/GPIOC/GPIOD/GPIOE 的 APB2 时钟;
*                  引脚分配见 MC_Keys.c 中的 KEY_xxx_PORT/BIT 宏。
*******************************************************************************/
void KEYS_process(void);   // 轮询扫描按键(含去抖/长按识别)并执行按键功能, 由主循环周期调用
void KEYS_Init(void);      // 按键/摇杆 GPIO 初始化, 上电调用一次
/*******************************************************************************
* 功能说明(中文) : 返回最近一次按键扫描/处理得到的按键代码, 供显示等模块查询。
* 参数(中文)     : 无。
* 返回(中文)     : bKey - 按键代码(见上方 NOKEY/SEL/RIGHT/LEFT/UP/DOWN/KEY_HOLD)。
* 备注(中文)     : 返回值来自 MC_Keys.c 内的静态变量 bKey。
*******************************************************************************/
u8 KEYS_ExportbKey(void);
#endif //__MCx_KEYS_H

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
