/**
  ******************************************************************************
  * @file    stm3210b_eval_lcd.h
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   This file contains all the functions prototypes for the stm3210b_eval_lcd
  *          firmware driver.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* ============================================================================
 * 【中文文件说明】stm32f10x_lcd.h —— LCD(TFT 液晶)显示驱动头文件
 * 本文件源自 STM3210B-EVAL 评估板例程(宏保护名沿用 stm3210b_eval_lcd)。
 * 作用：为 FOC 调试界面提供彩色 TFT 显示屏的驱动接口声明，主要包含：
 *   (1) LCD 控制引脚(NCS/RS/NWR)与 SPI 接口(SCK/MISO/MOSI)的硬件映射宏；
 *   (2) 三款兼容 LCD 控制器(ILI9320 / SPFD5408 / HX8312)的寄存器编号宏 R0~R239；
 *   (3) RGB565 颜色宏、文本行坐标(Line0~Line9)与绘制方向(Horizontal/Vertical)宏；
 *   (4) 初始化、清屏、光标、字符/字符串、图形、窗口及底层时序等接口函数原型。
 * 说明：屏幕分辨率 240x320，字符点阵 16x24，每屏可显示 10 行、每行 20 个字符；
 *       运行数据由 MC_Display 模块组织后，通过本驱动的显示接口输出到屏幕。
 * ==========================================================================*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM3210B_EVAL_LCD_H
#define __STM3210B_EVAL_LCD_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
   /*
typedef u8 uint8_t;
typedef u16 uint16_t;
typedef u32 uint32_t;
typedef s32 int32_t;
*/
/* 【中文】__IO 宏：等价于 volatile，用于修饰会随硬件自行改变的变量或寄存器，
   防止编译器把对硬件寄存器的重复访问优化掉。 */
#define     __IO    volatile

/** @addtogroup Utilities
  * @{
  */
  
/** @addtogroup STM3210B_EVAL_LCD
  * @{
  */ 


/** @defgroup STM3210B_EVAL_LCD_Exported_Types
  * @{
  */ 
/**
  * @}
  */ 



/** @defgroup STM3210B_EVAL_LCD_Exported_Constants
  * @{
  */ 

/**
 * @brief Uncomment the line below if you want to use LCD_DrawBMP function to
 *        display a bitmap picture on the LCD. This function assumes that the bitmap
 *        file is loaded in the SPI Flash (mounted on STM3210B-EVAL board), however
 *        user can tailor it according to his application hardware requirement.     
 */
/* 【中文】USE_LCD_DrawBMP：若需要在 LCD 上显示位图(位图文件预存在板上 SPI Flash 中)，
   则取消下一行的注释以启用 LCD_DrawBMP() 函数。 */
/*#define USE_LCD_DrawBMP*/

/**
 * @brief Uncomment the line below if you want to use user defined Delay function
 *        (for precise timing), otherwise default _delay_ function defined within
 *         this driver is used (less precise timing).  
 */
/* 【中文】USE_Delay：若需要精确定时，取消下面 USE_Delay 的注释，此时 _delay_ 会被
   重定义为 main.h 中用户提供的 Delay()(例如基于 SysTick 的 10ms 基准)；
   否则使用本驱动内部精度较低的软件延时函数 delay()。 */
/* #define USE_Delay */

#ifdef USE_Delay
#include "main.h"
 
  #define _delay_     Delay  /* !< User can provide more timing precise _delay_ function
                                   (with 10ms time base), using SysTick for example */
#else
  #define _delay_     delay      /* !< Default _delay_ function with less precise timing */
#endif                                     


/** 
  * @brief  LCD Control pins  
  * 【中文】LCD 控制引脚(均为 MCU 普通 GPIO，软件直接控制电平)：
  *   NCS 片选      : PB2 ，低电平有效，选中 LCD 控制器；
  *   NWR 写选通    : PD15，低电平有效(HX8312 并行时序下的写信号)；
  *   RS  寄存器选择: PD7 ，低=命令/寄存器号，高=数据(HX8312 并行时序使用)。
  *   使用 GPIOD 与 GPIOB，需同时使能二者的 APB2 时钟。
  */ 
#define LCD_NCS_PIN            GPIO_Pin_2                  
#define LCD_NCS_GPIO_PORT      GPIOB                       
#define LCD_NCS_GPIO_CLK       RCC_APB2Periph_GPIOB  
#define LCD_NWR_PIN            GPIO_Pin_15
#define LCD_RS_PIN             GPIO_Pin_7
#define LCD_NWR_GPIO_PORT      GPIOD                       
#define LCD_RS_GPIO_PORT       GPIOD
#define LCD_RSNWR_GPIO_CLK     RCC_APB2Periph_GPIOD 

/** 
  * @brief  LCD SPI Interface pins 
  * 【中文】LCD 串行接口引脚：使用 SPI2(主模式)，
  *   SCK=PB13、MISO=PB14、MOSI=PB15，SPI2 时钟挂在 APB1 总线上。
  *   ILI9320/SPFD5408 采用 8 位 SPI 字节传输；
  *   HX8312 采用 16 位数据宽度的并行模拟时序(借助 SPI 发送数据)。
  */ 
#define LCD_SPI_SCK_PIN        GPIO_Pin_13                 
#define LCD_SPI_MISO_PIN       GPIO_Pin_14                 
#define LCD_SPI_MOSI_PIN       GPIO_Pin_15                 
#define LCD_SPI_GPIO_PORT      GPIOB                       
#define LCD_SPI_GPIO_CLK       RCC_APB2Periph_GPIOB  
#define LCD_SPI			           SPI2
#define LCD_SPI_CLK		         RCC_APB1Periph_SPI2

/** 
  * @brief  LCD Registers  
  * 【中文】LCD 控制器寄存器"编号"宏(Rxx 的值即写命令时要下发的寄存器地址)：
  *   三款控制器(ILI9320/SPFD5408/HX8312)共用这一套编号，但同一编号含义可能不同。
  *   常用：R0 器件代号/振荡器控制；R3 显存写入方向与扫描模式；
  *        R7 显示开关(262K 色+显示 ON)；R22/R34 GRAM 数据口；
  *        R32/R33 ILI9320 的 GRAM 水平/垂直地址；R80~R83 GRAM 显示窗口范围；
  *        R229/R231/R239 ILI9320 内部时序寄存器。
  */ 
#define R0             0x00
#define R1             0x01
#define R2             0x02
#define R3             0x03
#define R4             0x04
#define R5             0x05
#define R6             0x06
#define R7             0x07
#define R8             0x08
#define R9             0x09
#define R10            0x0A
#define R12            0x0C
#define R13            0x0D
#define R14            0x0E
#define R15            0x0F
#define R16            0x10
#define R17            0x11
#define R18            0x12
#define R19            0x13
#define R20            0x14
#define R21            0x15
#define R22            0x16
#define R23            0x17
#define R24            0x18
#define R25            0x19
#define R26            0x1A
#define R27            0x1B
#define R28            0x1C
#define R29            0x1D
#define R30            0x1E
#define R31            0x1F
#define R32            0x20
#define R33            0x21
#define R34            0x22
#define R36            0x24
#define R37            0x25
#define R40            0x28
#define R41            0x29
#define R43            0x2B
#define R45            0x2D
#define R48            0x30
#define R49            0x31
#define R50            0x32
#define R51            0x33
#define R52            0x34
#define R53            0x35
#define R54            0x36
#define R55            0x37
#define R56            0x38
#define R57            0x39
#define R59            0x3B
#define R60            0x3C
#define R61            0x3D
#define R62            0x3E
#define R63            0x3F
#define R64            0x40
#define R65            0x41
#define R66            0x42
#define R67            0x43
#define R68            0x44
#define R69            0x45
#define R70            0x46
#define R71            0x47
#define R72            0x48
#define R73            0x49
#define R74            0x4A
#define R75            0x4B
#define R76            0x4C
#define R77            0x4D
#define R78            0x4E
#define R79            0x4F
#define R80            0x50
#define R81            0x51
#define R82            0x52
#define R83            0x53
#define R96            0x60
#define R97            0x61
#define R106           0x6A
#define R118           0x76
#define R128           0x80
#define R129           0x81
#define R130           0x82
#define R131           0x83
#define R132           0x84
#define R133           0x85
#define R134           0x86
#define R135           0x87
#define R136           0x88
#define R137           0x89
#define R139           0x8B
#define R140           0x8C
#define R141           0x8D
#define R143           0x8F
#define R144           0x90
#define R145           0x91
#define R146           0x92
#define R147           0x93
#define R148           0x94
#define R149           0x95
#define R150           0x96
#define R151           0x97
#define R152           0x98
#define R153           0x99
#define R154           0x9A
#define R157           0x9D
#define R192           0xC0
#define R193           0xC1
#define R227           0xE3
#define R229           0xE5
#define R231           0xE7
#define R239           0xEF


/** 
  * @brief  LCD color  
  * 【中文】RGB565 颜色宏：16 位色，格式为 R(5)G(6)B(5)，
  *   即高 5 位红、中 6 位绿、低 5 位蓝。例：Red=0xF800、Green=0x07E0、Blue=0x001F。
  * 下方 Line0~Line9 为逐行显示的行起始坐标(单位:像素行，每行为 24 像素高)；
  * Horizontal/Vertical 为 LCD_DrawLine 的方向参数(0=水平，1=垂直)。
  */ 
#define White          0xFFFF
#define Black          0x0000
#define Grey           0xF7DE
#define Blue           0x001F
#define Blue2          0x051F
#define Red            0xF800
#define Magenta        0xF81F
#define Green          0x07E0
#define Cyan           0x7FFF
#define Yellow         0xFFE0
#define Line0          0
#define Line1          24
#define Line2          48
#define Line3          72
#define Line4          96
#define Line5          120
#define Line6          144
#define Line7          168
#define Line8          192
#define Line9          216
#define Horizontal     0x00
#define Vertical       0x01
/**
  * @}
  */ 

/** @defgroup STM3210B_EVAL_LCD_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 



/** @defgroup STM3210B_EVAL_LCD_Exported_Functions
  * @{
  */ 
/* 【中文】以下为本驱动对外导出的接口函数：
 *   初始化   : LCD_Init(按 LCDType 下发初始化序列)、STM3210B_LCD_Init(探测并初始化)；
 *   颜色设置 : LCD_SetTextColor(文字色)、LCD_SetBackColor(背景色)；
 *   清屏/光标: LCD_Clear(全屏填充)、LCD_ClearLine(清某一行)、LCD_SetCursor(定位)；
 *   字符显示 : LCD_DrawChar(字形点阵)、LCD_DisplayChar(单字符)、LCD_DisplayStringLine(整行字符串)；
 *   图形绘制 : LCD_DrawLine/LCD_DrawRect/LCD_DrawCircle/LCD_DrawMonoPict/LCD_DrawBMP；
 *   显示窗口 : LCD_SetDisplayWindow、LCD_WindowModeDisable；
 *   底层时序 : LCD_nCS_StartByte/LCD_WriteRegIndex/LCD_WriteReg/LCD_ReadReg/
 *              LCD_WriteRAM_Prepare/LCD_WriteRAMWord/LCD_WriteRAM；
 *   电源/开关: LCD_PowerOn/LCD_DisplayOn/LCD_DisplayOff；
 *   硬件配置 : LCD_CtrlLinesConfig/LCD_CtrlLinesWrite/LCD_SPIConfig。 */
void LCD_Init(void);
void STM3210B_LCD_Init(void);
void LCD_SetTextColor(__IO uint16_t Color);
void LCD_SetBackColor(__IO uint16_t Color);
void LCD_ClearLine(uint8_t Line);
void LCD_Clear(uint16_t Color);
void LCD_SetCursor(uint8_t Xpos, uint16_t Ypos);
void LCD_DrawChar(uint8_t Xpos, uint16_t Ypos, const uint16_t *c);
void LCD_DisplayChar(uint8_t Line, uint16_t Column, uint8_t Ascii);
void LCD_DisplayStringLine(uint8_t Line, uint8_t *ptr);
void LCD_SetDisplayWindow(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void LCD_WindowModeDisable(void);
void LCD_DrawLine(uint8_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction);
void LCD_DrawRect(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void LCD_DrawCircle(uint8_t Xpos, uint16_t Ypos, uint16_t Radius);
void LCD_DrawMonoPict(const uint32_t *Pict);
void LCD_DrawBMP(uint32_t BmpAddress);

void LCD_nCS_StartByte(uint8_t Start_Byte);
void LCD_WriteRegIndex(uint8_t LCD_Reg);
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
void LCD_WriteRAM_Prepare(void);
void LCD_WriteRAMWord(uint16_t RGB_Code);
uint16_t LCD_ReadReg(uint8_t LCD_Reg);
void LCD_WriteRAM(uint16_t RGB_Code);
void LCD_PowerOn(void);
void LCD_DisplayOn(void);
void LCD_DisplayOff(void);


void LCD_CtrlLinesConfig(void);
void LCD_CtrlLinesWrite(GPIO_TypeDef* GPIOx, uint16_t CtrlPins, BitAction BitVal);
void LCD_SPIConfig(void);

/**
  * @}
  */ 
  
#ifdef __cplusplus
}
#endif

#endif /* __STM3210B_EVAL_LCD_H */


/**
  * @}
  */ 

/**
  * @}
  */ 
  
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
