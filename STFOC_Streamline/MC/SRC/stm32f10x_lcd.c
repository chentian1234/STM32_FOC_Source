/**
  ******************************************************************************
  * @file    stm3210b_eval_lcd.c
  * @author  MCD Application Team
  * @version V3.1.2
  * @date    09/28/2009
  * @brief   This file includes the LCD driver for AM-240320LTNQW00H (LCD_HX8312),
  *          AM-240320L8TNQW00H (LCD_ILI9320), AM-240320LDTNQW00H (LCD_SPFD5408B)
  *          Liquid Crystal Display Module of STM3210B-EVAL board.
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
 * 【中文文件说明】stm32f10x_lcd.c —— LCD(TFT 液晶)显示驱动实现
 * 本文件源自 STM3210B-EVAL 评估板例程，为 FOC 调试界面提供 240x320 彩色 TFT
 * 的驱动实现。它同时兼容三款 LCD 控制器(ILI9320 / SPFD5408 / HX8312)：
 *   - SPI 方式: ILI9320 与 SPFD5408 通过 SPI2(8 位)下发"起始字节+寄存器号/数据"；
 *   - 并行方式: HX8312 复用 SPI2 发送 16 位，并配合 NWR/RS/NCS 控制线模拟并口时序；
 *   - 上电后由 STM3210B_LCD_Init() 读回控制器 ID 自动识别型号，写入全局 LCDType。
 * 主要功能：初始化、颜色设置、光标定位、清屏、字符/字符串显示、点/线/矩形/圆/位图，
 *          以及显示窗口设置、显示开关、底层寄存器与显存(GRAM)读写时序。
 * 说明：FOC 运行数据由 MC_Display 模块组织，再经 LCD_SetTextColor / LCD_ClearLine /
 *       LCD_DisplayStringLine / LCD_DisplayChar 等接口输出到屏幕用于实时观测。
 * 注意：本文件当前未加入编译目标，仅供静态阅读参考。
 * ==========================================================================*/

/* Includes ------------------------------------------------------------------*/
/* 【中文】仅包含本模块头文件；其内部又包含 stm32f10x_lib.h，
   提供 GPIO/SPI/RCC 等标准外设库声明与 uint8_t/uint16_t 等类型。 */
#include "stm32f10x_lcd.h"

/** @addtogroup Utilities
  * @{
  */
  
/** @defgroup STM3210B_EVAL_LCD 
  * @briefThis file includes the LCD driver for AM-240320LTNQW00H (LCD_HX8312),
  *          AM-240320L8TNQW00H (LCD_ILI9320), AM-240320LDTNQW00H (LCD_SPFD5408B)
  *          Liquid Crystal Display Module of STM3210B-EVAL board.
  * @{
  */ 

/** @defgroup STM3210B_EVAL_LCD_Private_Types
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup FONTS_Exported_Types
  * @{
  */ 
     /* ASCII Table: each character is 16 column (16dots large) 
        and 24 raw (24 dots high) */
     /* 【中文字模说明】ASCII_Table[] 为可显示字符 0x20~0x7E 的点阵字模表，
        每个字符占用 24 个 uint16_t(共 48 字节)：
          - 每个 uint16_t 表示字符的一行(横向 16 个像素)，bit0~bit15 对应第 0~15 列；
          - 24 个 uint16_t 自上而下依次表示 24 行，故字模尺寸为 16(宽) x 24(高) 点；
          - 位=1 的像素用 TextColor 画，位=0 的像素用 BackColor 画(见 LCD_DrawChar)。
        表中字符严格按 ASCII 码 0x20(空格) 起递增排列，因此字模偏移量为
        (Ascii - 32) * 24，见 LCD_DisplayChar()。
        其中 '<' '>' '^' '|' 四个字符被绘图改为左/右/上/下方向箭头，供界面图标使用。
        单位/量纲：每个元素为一行像素的位图数据(无物理量纲，非电流/电压等物理量)。 */
     const uint16_t ASCII_Table[] =
       {
/** 
  * @brief        Space ' '  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '!'  
  */ 
         0x0000, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180,
         0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0000, 0x0000,
         0x0180, 0x0180, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '"'  
  */ 
         0x0000, 0x0000, 0x00CC, 0x00CC, 0x00CC, 0x00CC, 0x00CC, 0x00CC,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '#'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0C60, 0x0C60,
         0x0C60, 0x0630, 0x0630, 0x1FFE, 0x1FFE, 0x0630, 0x0738, 0x0318,
         0x1FFE, 0x1FFE, 0x0318, 0x0318, 0x018C, 0x018C, 0x018C, 0x0000,
/** 
  * @brief        '$'  
  */ 
         0x0000, 0x0080, 0x03E0, 0x0FF8, 0x0E9C, 0x1C8C, 0x188C, 0x008C,
         0x0098, 0x01F8, 0x07E0, 0x0E80, 0x1C80, 0x188C, 0x188C, 0x189C,
         0x0CB8, 0x0FF0, 0x03E0, 0x0080, 0x0080, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '%'  
  */ 
         0x0000, 0x0000, 0x0000, 0x180E, 0x0C1B, 0x0C11, 0x0611, 0x0611,
         0x0311, 0x0311, 0x019B, 0x018E, 0x38C0, 0x6CC0, 0x4460, 0x4460,
         0x4430, 0x4430, 0x4418, 0x6C18, 0x380C, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '&'  
  */ 
         0x0000, 0x01E0, 0x03F0, 0x0738, 0x0618, 0x0618, 0x0330, 0x01F0,
         0x00F0, 0x00F8, 0x319C, 0x330E, 0x1E06, 0x1C06, 0x1C06, 0x3F06,
         0x73FC, 0x21F0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '''  
  */ 
         0x0000, 0x0000, 0x000C, 0x000C, 0x000C, 0x000C, 0x000C, 0x000C,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '('  
  */ 
         0x0000, 0x0200, 0x0300, 0x0180, 0x00C0, 0x00C0, 0x0060, 0x0060,
         0x0030, 0x0030, 0x0030, 0x0030, 0x0030, 0x0030, 0x0030, 0x0030,
         0x0060, 0x0060, 0x00C0, 0x00C0, 0x0180, 0x0300, 0x0200, 0x0000,
/** 
  * @brief        ')'  
  */ 
         0x0000, 0x0020, 0x0060, 0x00C0, 0x0180, 0x0180, 0x0300, 0x0300,
         0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
         0x0300, 0x0300, 0x0180, 0x0180, 0x00C0, 0x0060, 0x0020, 0x0000,
/** 
  * @brief        '*'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x00C0, 0x00C0,
         0x06D8, 0x07F8, 0x01E0, 0x0330, 0x0738, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '+'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0180, 0x0180,
         0x0180, 0x0180, 0x0180, 0x3FFC, 0x3FFC, 0x0180, 0x0180, 0x0180,
         0x0180, 0x0180, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        ','  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0180, 0x0180, 0x0100, 0x0100, 0x0080, 0x0000, 0x0000,
/** 
  * @brief        '-'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x07E0, 0x07E0, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '.'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0x00C0, 0x00C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '/'  
  */ 
         0x0000, 0x0C00, 0x0C00, 0x0600, 0x0600, 0x0600, 0x0300, 0x0300,
         0x0300, 0x0380, 0x0180, 0x0180, 0x0180, 0x00C0, 0x00C0, 0x00C0,
         0x0060, 0x0060, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '0'  
  */ 
         0x0000, 0x03E0, 0x07F0, 0x0E38, 0x0C18, 0x180C, 0x180C, 0x180C,
         0x180C, 0x180C, 0x180C, 0x180C, 0x180C, 0x180C, 0x0C18, 0x0E38,
         0x07F0, 0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '1'  
  */ 
         0x0000, 0x0100, 0x0180, 0x01C0, 0x01F0, 0x0198, 0x0188, 0x0180,
         0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180,
         0x0180, 0x0180, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '2'  
  */ 
         0x0000, 0x03E0, 0x0FF8, 0x0C18, 0x180C, 0x180C, 0x1800, 0x1800,
         0x0C00, 0x0600, 0x0300, 0x0180, 0x00C0, 0x0060, 0x0030, 0x0018,
         0x1FFC, 0x1FFC, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '3'  
  */ 
         0x0000, 0x01E0, 0x07F8, 0x0E18, 0x0C0C, 0x0C0C, 0x0C00, 0x0600,
         0x03C0, 0x07C0, 0x0C00, 0x1800, 0x1800, 0x180C, 0x180C, 0x0C18,
         0x07F8, 0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '4'  
  */ 
         0x0000, 0x0C00, 0x0E00, 0x0F00, 0x0F00, 0x0D80, 0x0CC0, 0x0C60,
         0x0C60, 0x0C30, 0x0C18, 0x0C0C, 0x3FFC, 0x3FFC, 0x0C00, 0x0C00,
         0x0C00, 0x0C00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '5'  
  */ 
         0x0000, 0x0FF8, 0x0FF8, 0x0018, 0x0018, 0x000C, 0x03EC, 0x07FC,
         0x0E1C, 0x1C00, 0x1800, 0x1800, 0x1800, 0x180C, 0x0C1C, 0x0E18,
         0x07F8, 0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '6'  
  */ 
         0x0000, 0x07C0, 0x0FF0, 0x1C38, 0x1818, 0x0018, 0x000C, 0x03CC,
         0x0FEC, 0x0E3C, 0x1C1C, 0x180C, 0x180C, 0x180C, 0x1C18, 0x0E38,
         0x07F0, 0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '7'  
  */ 
         0x0000, 0x1FFC, 0x1FFC, 0x0C00, 0x0600, 0x0600, 0x0300, 0x0380,
         0x0180, 0x01C0, 0x00C0, 0x00E0, 0x0060, 0x0060, 0x0070, 0x0030,
         0x0030, 0x0030, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '8'  
  */ 
         0x0000, 0x03E0, 0x07F0, 0x0E38, 0x0C18, 0x0C18, 0x0C18, 0x0638,
         0x07F0, 0x07F0, 0x0C18, 0x180C, 0x180C, 0x180C, 0x180C, 0x0C38,
         0x0FF8, 0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '9'  
  */ 
         0x0000, 0x03E0, 0x07F0, 0x0E38, 0x0C1C, 0x180C, 0x180C, 0x180C,
         0x1C1C, 0x1E38, 0x1BF8, 0x19E0, 0x1800, 0x0C00, 0x0C00, 0x0E1C,
         0x07F8, 0x01F0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        ':'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0180, 0x0180,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0180, 0x0180, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        ';'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0180, 0x0180,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0180, 0x0180, 0x0100, 0x0100, 0x0080, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '<'  MODIFIED TO DRAW AN ARROW POINTING LEFT
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x1000, 0x1C00, 0x0F80, 0x03E0, 0x7FF8, 0x7FF8, 0x7FF8, 0x03E0, 
         0x0F80, 0x1C00, 0x1000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '='  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x1FF8, 0x0000, 0x0000, 0x0000, 0x1FF8, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '>'  MODIFIED TO DRAW AN ARROW POINTING RIGHT
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0008, 0x0038, 0x01F0, 0x07C0, 0x1FFE, 0x1FFE, 0x1FFE, 0x07C0, 
         0x01F0, 0x0038, 0x0008, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         
/** 
  * @brief        '?'  
  */ 
         0x0000, 0x03E0, 0x0FF8, 0x0C18, 0x180C, 0x180C, 0x1800, 0x0C00,
         0x0600, 0x0300, 0x0180, 0x00C0, 0x00C0, 0x00C0, 0x0000, 0x0000,
         0x00C0, 0x00C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '@'  
  */ 
         0x0000, 0x0000, 0x07E0, 0x1818, 0x2004, 0x29C2, 0x4A22, 0x4411,
         0x4409, 0x4409, 0x4409, 0x2209, 0x1311, 0x0CE2, 0x4002, 0x2004,
         0x1818, 0x07E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'A'  
  */ 
         0x0000, 0x0380, 0x0380, 0x06C0, 0x06C0, 0x06C0, 0x0C60, 0x0C60,
         0x1830, 0x1830, 0x1830, 0x3FF8, 0x3FF8, 0x701C, 0x600C, 0x600C,
         0xC006, 0xC006, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'B'  
  */ 
         0x0000, 0x03FC, 0x0FFC, 0x0C0C, 0x180C, 0x180C, 0x180C, 0x0C0C,
         0x07FC, 0x0FFC, 0x180C, 0x300C, 0x300C, 0x300C, 0x300C, 0x180C,
         0x1FFC, 0x07FC, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'C'  
  */ 
         0x0000, 0x07C0, 0x1FF0, 0x3838, 0x301C, 0x700C, 0x6006, 0x0006,
         0x0006, 0x0006, 0x0006, 0x0006, 0x0006, 0x6006, 0x700C, 0x301C,
         0x1FF0, 0x07E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'D'  
  */ 
         0x0000, 0x03FE, 0x0FFE, 0x0E06, 0x1806, 0x1806, 0x3006, 0x3006,
         0x3006, 0x3006, 0x3006, 0x3006, 0x3006, 0x1806, 0x1806, 0x0E06,
         0x0FFE, 0x03FE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'E'  
  */ 
         0x0000, 0x3FFC, 0x3FFC, 0x000C, 0x000C, 0x000C, 0x000C, 0x000C,
         0x1FFC, 0x1FFC, 0x000C, 0x000C, 0x000C, 0x000C, 0x000C, 0x000C,
         0x3FFC, 0x3FFC, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'F'  
  */ 
         0x0000, 0x3FF8, 0x3FF8, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018,
         0x1FF8, 0x1FF8, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018,
         0x0018, 0x0018, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'G'  
  */ 
         0x0000, 0x0FE0, 0x3FF8, 0x783C, 0x600E, 0xE006, 0xC007, 0x0003,
         0x0003, 0xFE03, 0xFE03, 0xC003, 0xC007, 0xC006, 0xC00E, 0xF03C,
         0x3FF8, 0x0FE0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'H'  
  */ 
         0x0000, 0x300C, 0x300C, 0x300C, 0x300C, 0x300C, 0x300C, 0x300C,
         0x3FFC, 0x3FFC, 0x300C, 0x300C, 0x300C, 0x300C, 0x300C, 0x300C,
         0x300C, 0x300C, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'I'  
  */ 
         0x0000, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180,
         0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180,
         0x0180, 0x0180, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'J'  
  */ 
         0x0000, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0600,
         0x0600, 0x0600, 0x0600, 0x0600, 0x0600, 0x0618, 0x0618, 0x0738,
         0x03F0, 0x01E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'K'  
  */ 
         0x0000, 0x3006, 0x1806, 0x0C06, 0x0606, 0x0306, 0x0186, 0x00C6,
         0x0066, 0x0076, 0x00DE, 0x018E, 0x0306, 0x0606, 0x0C06, 0x1806,
         0x3006, 0x6006, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'L'  
  */ 
         0x0000, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018,
         0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018,
         0x1FF8, 0x1FF8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'M'  
  */ 
         0x0000, 0xE00E, 0xF01E, 0xF01E, 0xF01E, 0xD836, 0xD836, 0xD836,
         0xD836, 0xCC66, 0xCC66, 0xCC66, 0xC6C6, 0xC6C6, 0xC6C6, 0xC6C6,
         0xC386, 0xC386, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'N'  
  */ 
         0x0000, 0x300C, 0x301C, 0x303C, 0x303C, 0x306C, 0x306C, 0x30CC,
         0x30CC, 0x318C, 0x330C, 0x330C, 0x360C, 0x360C, 0x3C0C, 0x3C0C,
         0x380C, 0x300C, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'O'  
  */ 
         0x0000, 0x07E0, 0x1FF8, 0x381C, 0x700E, 0x6006, 0xC003, 0xC003,
         0xC003, 0xC003, 0xC003, 0xC003, 0xC003, 0x6006, 0x700E, 0x381C,
         0x1FF8, 0x07E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'P'  
  */ 
         0x0000, 0x0FFC, 0x1FFC, 0x380C, 0x300C, 0x300C, 0x300C, 0x300C,
         0x180C, 0x1FFC, 0x07FC, 0x000C, 0x000C, 0x000C, 0x000C, 0x000C,
         0x000C, 0x000C, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'Q'  
  */ 
         0x0000, 0x07E0, 0x1FF8, 0x381C, 0x700E, 0x6006, 0xE003, 0xC003,
         0xC003, 0xC003, 0xC003, 0xC003, 0xE007, 0x6306, 0x3F0E, 0x3C1C,
         0x3FF8, 0xF7E0, 0xC000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'R'  
  */ 
         0x0000, 0x0FFE, 0x1FFE, 0x3806, 0x3006, 0x3006, 0x3006, 0x3806,
         0x1FFE, 0x07FE, 0x0306, 0x0606, 0x0C06, 0x1806, 0x1806, 0x3006,
         0x3006, 0x6006, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'S'  
  */ 
         0x0000, 0x03E0, 0x0FF8, 0x0C1C, 0x180C, 0x180C, 0x000C, 0x001C,
         0x03F8, 0x0FE0, 0x1E00, 0x3800, 0x3006, 0x3006, 0x300E, 0x1C1C,
         0x0FF8, 0x07E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'T'  
  */ 
         0x0000, 0x7FFE, 0x7FFE, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180,
         0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180,
         0x0180, 0x0180, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'U'  
  */ 
         0x0000, 0x300C, 0x300C, 0x300C, 0x300C, 0x300C, 0x300C, 0x300C,
         0x300C, 0x300C, 0x300C, 0x300C, 0x300C, 0x300C, 0x300C, 0x1818,
         0x1FF8, 0x07E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'V'  
  */ 
         0x0000, 0x6003, 0x3006, 0x3006, 0x3006, 0x180C, 0x180C, 0x180C,
         0x0C18, 0x0C18, 0x0E38, 0x0630, 0x0630, 0x0770, 0x0360, 0x0360,
         0x01C0, 0x01C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'W'  
  */ 
         0x0000, 0x6003, 0x61C3, 0x61C3, 0x61C3, 0x3366, 0x3366, 0x3366,
         0x3366, 0x3366, 0x3366, 0x1B6C, 0x1B6C, 0x1B6C, 0x1A2C, 0x1E3C,
         0x0E38, 0x0E38, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'X'  
  */ 
         0x0000, 0xE00F, 0x700C, 0x3018, 0x1830, 0x0C70, 0x0E60, 0x07C0,
         0x0380, 0x0380, 0x03C0, 0x06E0, 0x0C70, 0x1C30, 0x1818, 0x300C,
         0x600E, 0xE007, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'Y'  
  */ 
         0x0000, 0xC003, 0x6006, 0x300C, 0x381C, 0x1838, 0x0C30, 0x0660,
         0x07E0, 0x03C0, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180, 0x0180,
         0x0180, 0x0180, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'Z'  
  */ 
         0x0000, 0x7FFC, 0x7FFC, 0x6000, 0x3000, 0x1800, 0x0C00, 0x0600,
         0x0300, 0x0180, 0x00C0, 0x0060, 0x0030, 0x0018, 0x000C, 0x0006,
         0x7FFE, 0x7FFE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '['  
  */ 
         0x0000, 0x03E0, 0x03E0, 0x0060, 0x0060, 0x0060, 0x0060, 0x0060,
         0x0060, 0x0060, 0x0060, 0x0060, 0x0060, 0x0060, 0x0060, 0x0060,
         0x0060, 0x0060, 0x0060, 0x0060, 0x0060, 0x03E0, 0x03E0, 0x0000,
/** 
  * @brief        '\'  
  */ 
         0x0000, 0x0030, 0x0030, 0x0060, 0x0060, 0x0060, 0x00C0, 0x00C0,
         0x00C0, 0x01C0, 0x0180, 0x0180, 0x0180, 0x0300, 0x0300, 0x0300,
         0x0600, 0x0600, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        ']'  
  */ 
         0x0000, 0x03E0, 0x03E0, 0x0300, 0x0300, 0x0300, 0x0300, 0x0300,
         0x0300, 0x0300, 0x0300, 0x0300, 0x0300, 0x0300, 0x0300, 0x0300,
         0x0300, 0x0300, 0x0300, 0x0300, 0x0300, 0x03E0, 0x03E0, 0x0000,
/** 
  * @brief        '^'  MODIFIED TO DRAW AN ARROW UP POINTING
  */ 
         0x0000, 0x0180, 0x01C0, 0x01C0, 0x03E0, 0x03E0, 0x03E0, 0x07F0,
         0x07F0, 0x0DD8, 0x0DD8, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 
         0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x0000,
/** 
  * @brief        '_'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0xFFFF, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '''  
  */ 
         0x0000, 0x000C, 0x000C, 0x000C, 0x000C, 0x000C, 0x000C, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'a'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03F0, 0x07F8,
         0x0C1C, 0x0C0C, 0x0F00, 0x0FF0, 0x0CF8, 0x0C0C, 0x0C0C, 0x0F1C,
         0x0FF8, 0x18F0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'b'  
  */ 
         0x0000, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x03D8, 0x0FF8,
         0x0C38, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x0C38,
         0x0FF8, 0x03D8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'c'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03C0, 0x07F0,
         0x0E30, 0x0C18, 0x0018, 0x0018, 0x0018, 0x0018, 0x0C18, 0x0E30,
         0x07F0, 0x03C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'd'  
  */ 
         0x0000, 0x1800, 0x1800, 0x1800, 0x1800, 0x1800, 0x1BC0, 0x1FF0,
         0x1C30, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1C30,
         0x1FF0, 0x1BC0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'e'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03C0, 0x0FF0,
         0x0C30, 0x1818, 0x1FF8, 0x1FF8, 0x0018, 0x0018, 0x1838, 0x1C30,
         0x0FF0, 0x07C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'f'  
  */ 
         0x0000, 0x0F80, 0x0FC0, 0x00C0, 0x00C0, 0x00C0, 0x07F0, 0x07F0,
         0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0,
         0x00C0, 0x00C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'g'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0DE0, 0x0FF8,
         0x0E18, 0x0C0C, 0x0C0C, 0x0C0C, 0x0C0C, 0x0C0C, 0x0C0C, 0x0E18,
         0x0FF8, 0x0DE0, 0x0C00, 0x0C0C, 0x061C, 0x07F8, 0x01F0, 0x0000,
/** 
  * @brief        'h'  
  */ 
         0x0000, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x07D8, 0x0FF8,
         0x1C38, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818,
         0x1818, 0x1818, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'i'  
  */ 
         0x0000, 0x00C0, 0x00C0, 0x0000, 0x0000, 0x0000, 0x00C0, 0x00C0,
         0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0,
         0x00C0, 0x00C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'j'  
  */ 
         0x0000, 0x00C0, 0x00C0, 0x0000, 0x0000, 0x0000, 0x00C0, 0x00C0,
         0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0,
         0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00F8, 0x0078, 0x0000,
/** 
  * @brief        'k'  
  */ 
         0x0000, 0x000C, 0x000C, 0x000C, 0x000C, 0x000C, 0x0C0C, 0x060C,
         0x030C, 0x018C, 0x00CC, 0x006C, 0x00FC, 0x019C, 0x038C, 0x030C,
         0x060C, 0x0C0C, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'l'  
  */ 
         0x0000, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0,
         0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0,
         0x00C0, 0x00C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'm'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3C7C, 0x7EFF,
         0xE3C7, 0xC183, 0xC183, 0xC183, 0xC183, 0xC183, 0xC183, 0xC183,
         0xC183, 0xC183, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'n'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0798, 0x0FF8,
         0x1C38, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818,
         0x1818, 0x1818, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'o'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03C0, 0x0FF0,
         0x0C30, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x0C30,
         0x0FF0, 0x03C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'p'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03D8, 0x0FF8,
         0x0C38, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x0C38,
         0x0FF8, 0x03D8, 0x0018, 0x0018, 0x0018, 0x0018, 0x0018, 0x0000,
/** 
  * @brief        'q'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1BC0, 0x1FF0,
         0x1C30, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1C30,
         0x1FF0, 0x1BC0, 0x1800, 0x1800, 0x1800, 0x1800, 0x1800, 0x0000,
/** 
  * @brief        'r'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x07B0, 0x03F0,
         0x0070, 0x0030, 0x0030, 0x0030, 0x0030, 0x0030, 0x0030, 0x0030,
         0x0030, 0x0030, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        's'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x03E0, 0x03F0,
         0x0E38, 0x0C18, 0x0038, 0x03F0, 0x07C0, 0x0C00, 0x0C18, 0x0E38,
         0x07F0, 0x03E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        't'  
  */ 
         0x0000, 0x0000, 0x0080, 0x00C0, 0x00C0, 0x00C0, 0x07F0, 0x07F0,
         0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0,
         0x07C0, 0x0780, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'u'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1818, 0x1818,
         0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1818, 0x1C38,
         0x1FF0, 0x19E0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'v'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x180C, 0x0C18,
         0x0C18, 0x0C18, 0x0630, 0x0630, 0x0630, 0x0360, 0x0360, 0x0360,
         0x01C0, 0x01C0, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'w'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x41C1, 0x41C1,
         0x61C3, 0x6363, 0x6363, 0x6363, 0x3636, 0x3636, 0x3636, 0x1C1C,
         0x1C1C, 0x1C1C, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'x'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x381C, 0x1C38,
         0x0C30, 0x0660, 0x0360, 0x0360, 0x0360, 0x0360, 0x0660, 0x0C30,
         0x1C38, 0x381C, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        'y'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x3018, 0x1830,
         0x1830, 0x1870, 0x0C60, 0x0C60, 0x0CE0, 0x06C0, 0x06C0, 0x0380,
         0x0380, 0x0380, 0x0180, 0x0180, 0x01C0, 0x00F0, 0x0070, 0x0000,
/** 
  * @brief        'z'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x1FFC, 0x1FFC,
         0x0C00, 0x0600, 0x0300, 0x0180, 0x00C0, 0x0060, 0x0030, 0x0018,
         0x1FFC, 0x1FFC, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
/** 
  * @brief        '{'  
  */ 
         0x0000, 0x0300, 0x0180, 0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x00C0,
         0x00C0, 0x0060, 0x0060, 0x0030, 0x0060, 0x0040, 0x00C0, 0x00C0,
         0x00C0, 0x00C0, 0x00C0, 0x00C0, 0x0180, 0x0300, 0x0000, 0x0000,
/** 
  * @brief        '|'  MODIFIED TO DRAW AN ARROW DOWN POINTING
  */ 
         0x0000, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 
         0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x01C0, 0x0DD8, 0x0DD8, 0x07F0, 
         0x07F0, 0x03E0, 0x03E0, 0x03E0, 0x01C0, 0x01C0, 0x0180, 0x0000,
         
/** 
  * @brief        '}'  
  */ 
         0x0000, 0x0060, 0x00C0, 0x01C0, 0x0180, 0x0180, 0x0180, 0x0180,
         0x0180, 0x0300, 0x0300, 0x0600, 0x0300, 0x0100, 0x0180, 0x0180,
         0x0180, 0x0180, 0x0180, 0x0180, 0x00C0, 0x0060, 0x0000, 0x0000,
/** 
  * @brief        '~'  
  */ 
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x10F0, 0x1FF8, 0x0F08, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
         0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
      };
/**
  * @}
  */


/** @defgroup STM3210B_EVAL_LCD_Private_Defines
  * @{
  */ 
/* 【中文】LCD 控制器型号标识：读控制器 ID 寄存器(R0)得到的返回值，
   用于自动识别所插屏的型号(ILI9320 / SPFD5408 / HX8312)。 */
#define LCD_ILI9320     0x9320
#define LCD_HX8312      0x8312
#define LCD_SPFD5408    0x5408
/* 【中文】SPI 命令字节：高 6 位是固定起始字节 0x70，低 2 位表示操作类型：
   SET_INDEX=00(写寄存器号)、READ_STATUS=01(读状态)、
   LCD_WRITE_REG=10(写寄存器)、LCD_READ_REG=11(读寄存器)。 */
#define START_BYTE      0x70
#define SET_INDEX       0x00
#define READ_STATUS     0x01
#define LCD_WRITE_REG   0x02
#define LCD_READ_REG    0x03
/**
  * @}
  */ 


/** @defgroup STM3210B_EVAL_LCD_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM3210B_EVAL_LCD_Private_Variables
  * @{
  */ 
  /* Global variables to set the written text color */
/* 【中文】全局绘图颜色(16 位 RGB565)：
   TextColor 文字/线条前景色，初值 0x0000(黑)；
   BackColor 背景色，初值 0xFFFF(白)。
   由 LCD_SetTextColor()/LCD_SetBackColor() 修改，供 LCD_DrawChar/LCD_DrawLine 等使用。 */
static __IO uint16_t TextColor = 0x0000, BackColor = 0xFFFF;

/* 【中文】当前 LCD 控制器型号，初值 ILI9320；由 STM3210B_LCD_Init() 探测后确定。
   所有底层读写函数都依据它选择对应的传输时序(SPI 字节流或 16 位并口模拟)。 */
static __IO uint32_t LCDType = LCD_ILI9320;

/**
  * @}
  */ 


/** @defgroup STM3210B_EVAL_LCD_Private_Function_Prototypes
  * @{
  */ 
/* 【中文】私有函数原型：
   delay()              软件延时(仅当 USE_Delay 未定义时才编译本函数)；
   LCD_WriteRegHX8312()  HX8312 控制器寄存器写(16 位并口模拟时序)；
   LCD_WriteRegILI9320() ILI9320/SPFD5408 控制器寄存器写(SPI 字节时序)。 */
#ifndef USE_Delay
static void delay(__IO uint32_t nCount);
#endif /* USE_Delay*/

static void LCD_WriteRegHX8312(uint8_t LCD_Reg, uint8_t LCD_RegValue);
static void LCD_WriteRegILI9320(uint8_t LCD_Reg, uint16_t LCD_RegValue);
/**
  * @}
  */ 


/** @defgroup STM3210B_EVAL_LCD_Private_Functions
  * @{
  */ 


/**
  * @brief  Setups the LCD.
  * 功能说明(中文) : 依据全局变量 LCDType 所选型号，向 LCD 写入完整的上电初始化寄存器
  *                  序列(软启动、电源上电、伽马校正、GRAM 范围、面板控制、显示开等)，
  *                  使屏幕进入可用状态。由 STM3210B_LCD_Init() 调用(可能被多次调用)。
  * 参数(中文)     : 无。
  * 返回(中文)     : 无。
  * 备注(中文)     : 依赖全局变量 LCDType；内部用 _delay_() 做毫秒级等待，确保内部
  *                  电源与振荡器稳定；直接操作 SPI2 及 NCS/RS/NWR 控制引脚。
  * @param  None
  * @retval None
  */
void LCD_Init(void)
{ 
/* Configure the LCD Control pins --------------------------------------------*/
  LCD_CtrlLinesConfig();
  
/* Configure the LCD_SPI interface ----------------------------------------------*/
  LCD_SPIConfig();
  
  if(LCDType == LCD_ILI9320)  // 【中文】按探测到的控制器型号分支：以下为 ILI9320 初始化
  {
    _delay_(5); /* Delay 50 ms */  // 【中文】上电延时 50ms，等电源稳定后再发初始化命令
    /* Start Initial Sequence ------------------------------------------------*/
    /* 【中文】以下是 ILI9320 初始化序列，顺序不可乱：先软启动与扫描/时序设置，再上电升压
       (含较长放电延时)，然后做伽马校正、设定 GRAM 范围与面板参数，最后置显示开；
       各步之间的 _delay_() 用于满足控制器内部电源稳定的等待时间。 */
    LCD_WriteReg(R229, 0x8000); /* Set the internal vcore voltage */
    LCD_WriteReg(R0,  0x0001); /* Start internal OSC. */
    LCD_WriteReg(R1,  0x0100); /* set SS and SM bit */
    LCD_WriteReg(R2,  0x0700); /* set 1 line inversion */
    LCD_WriteReg(R3,  0x1030); /* set GRAM write direction and BGR=1. */
    LCD_WriteReg(R4,  0x0000); /* Resize register */
    LCD_WriteReg(R8,  0x0202); /* set the back porch and front porch */
    LCD_WriteReg(R9,  0x0000); /* set non-display area refresh cycle ISC[3:0] */
    LCD_WriteReg(R10, 0x0000); /* FMARK function */
    LCD_WriteReg(R12, 0x0000); /* RGB interface setting */
    LCD_WriteReg(R13, 0x0000); /* Frame marker Position */
    LCD_WriteReg(R15, 0x0000); /* RGB interface polarity */
    /* Power On sequence -----------------------------------------------------*/
    LCD_WriteReg(R16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(R17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
    LCD_WriteReg(R18, 0x0000); /* VREG1OUT voltage */
    LCD_WriteReg(R19, 0x0000); /* VDV[4:0] for VCOM amplitude */
    _delay_(20);               /* Dis-charge capacitor power voltage (200ms) */
    LCD_WriteReg(R16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(R17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
    _delay_(5);                /* Delay 50 ms */
    LCD_WriteReg(R18, 0x0139); /* VREG1OUT voltage */
    _delay_(5);                /* Delay 50 ms */
    LCD_WriteReg(R19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
    LCD_WriteReg(R41, 0x0013); /* VCM[4:0] for VCOMH */
    _delay_(5);                /* Delay 50 ms */
    LCD_WriteReg(R32, 0x0000); /* GRAM horizontal Address */
    LCD_WriteReg(R33, 0x0000); /* GRAM Vertical Address */
    /* Adjust the Gamma Curve ------------------------------------------------*/
    LCD_WriteReg(R48, 0x0006);
    LCD_WriteReg(R49, 0x0101);
    LCD_WriteReg(R50, 0x0003);
    LCD_WriteReg(R53, 0x0106);
    LCD_WriteReg(R54, 0x0b02);
    LCD_WriteReg(R55, 0x0302);
    LCD_WriteReg(R56, 0x0707);
    LCD_WriteReg(R57, 0x0007);
    LCD_WriteReg(R60, 0x0600);
    LCD_WriteReg(R61, 0x020b);
  
    /* Set GRAM area ---------------------------------------------------------*/
    LCD_WriteReg(R80, 0x0000); /* Horizontal GRAM Start Address */
    LCD_WriteReg(R81, 0x00EF); /* Horizontal GRAM End Address */
    LCD_WriteReg(R82, 0x0000); /* Vertical GRAM Start Address */
    LCD_WriteReg(R83, 0x013F); /* Vertical GRAM End Address */
    LCD_WriteReg(R96,  0x2700); /* Gate Scan Line */
    LCD_WriteReg(R97,  0x0001); /* NDL,VLE, REV */
    LCD_WriteReg(R106, 0x0000); /* set scrolling line */
    /* Partial Display Control -----------------------------------------------*/
    LCD_WriteReg(R128, 0x0000);
    LCD_WriteReg(R129, 0x0000);
    LCD_WriteReg(R130, 0x0000);
    LCD_WriteReg(R131, 0x0000);
    LCD_WriteReg(R132, 0x0000);
    LCD_WriteReg(R133, 0x0000);
    /* Panel Control ---------------------------------------------------------*/
    LCD_WriteReg(R144, 0x0010);
    LCD_WriteReg(R146, 0x0000);
    LCD_WriteReg(R147, 0x0003);
    LCD_WriteReg(R149, 0x0110);
    LCD_WriteReg(R151, 0x0000);
    LCD_WriteReg(R152, 0x0000);
    /* Set GRAM write direction and BGR = 1 */
    /* I/D=01 (Horizontal : increment, Vertical : decrement) */
    /* AM=1 (address is updated in vertical writing direction) */
    LCD_WriteReg(R3, 0x1018);
    LCD_WriteReg(R7, 0x0173); /* 262K color and display ON */
  }
  else if(LCDType == LCD_SPFD5408)
  {   
    /* Start Initial Sequence --------------------------------------------------*/
    LCD_WriteReg(R227, 0x3008); /* Set internal timing */
    LCD_WriteReg(R231, 0x0012); /* Set internal timing */
    LCD_WriteReg(R239, 0x1231); /* Set internal timing */
    LCD_WriteReg(R1, 0x0100);   /* Set SS and SM bit */
    LCD_WriteReg(R2, 0x0700);   /* Set 1 line inversion */
    LCD_WriteReg(R3, 0x1030);   /* Set GRAM write direction and BGR=1. */
    LCD_WriteReg(R4, 0x0000);   /* Resize register */
    LCD_WriteReg(R8, 0x0202);   /* Set the back porch and front porch */
    LCD_WriteReg(R9, 0x0000);   /* Set non-display area refresh cycle ISC[3:0] */
    LCD_WriteReg(R10, 0x0000);  /* FMARK function */
    LCD_WriteReg(R12, 0x0000);  /* RGB interface setting */
    LCD_WriteReg(R13, 0x0000);  /* Frame marker Position */
    LCD_WriteReg(R15, 0x0000);  /* RGB interface polarity */
    /* Power On sequence -------------------------------------------------------*/
    LCD_WriteReg(R16, 0x0000);  /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(R17, 0x0000);  /* DC1[2:0], DC0[2:0], VC[2:0] */
    LCD_WriteReg(R18, 0x0000);  /* VREG1OUT voltage */
    LCD_WriteReg(R19, 0x0000);  /* VDV[4:0] for VCOM amplitude */
    _delay_(20);                /* Dis-charge capacitor power voltage (200ms) */  
    LCD_WriteReg(R17, 0x0007);	/* DC1[2:0], DC0[2:0], VC[2:0] */
    _delay_(5);                 /* Delay 50 ms */
    LCD_WriteReg(R16, 0x12B0);	/* SAP, BT[3:0], AP, DSTB, SLP, STB */
    _delay_(5);	                /* Delay 50 ms */
    LCD_WriteReg(R18, 0x01BD);  /* External reference voltage= Vci */
    _delay_(5);                 /* Delay 50 ms */ 
    LCD_WriteReg(R19, 0x1400);       /* VDV[4:0] for VCOM amplitude */
    LCD_WriteReg(R41, 0x000E);  /* VCM[4:0] for VCOMH */
    _delay_(5);                 /* Delay 50 ms */
    LCD_WriteReg(R32, 0x0000);  /* GRAM horizontal Address */
    LCD_WriteReg(R33, 0x013F);  /* GRAM Vertical Address */
    /* Adjust the Gamma Curve --------------------------------------------------*/
    LCD_WriteReg(R48, 0x0007);
    LCD_WriteReg(R49, 0x0302);
    LCD_WriteReg(R50, 0x0105);
    LCD_WriteReg(R53, 0x0206);
    LCD_WriteReg(R54, 0x0808);
    LCD_WriteReg(R55, 0x0206);
    LCD_WriteReg(R56, 0x0504);
    LCD_WriteReg(R57, 0x0007);
    LCD_WriteReg(R60, 0x0105);
    LCD_WriteReg(R61, 0x0808);
    /* Set GRAM area -----------------------------------------------------------*/
    LCD_WriteReg(R80, 0x0000);  /* Horizontal GRAM Start Address */
    LCD_WriteReg(R81, 0x00EF);  /* Horizontal GRAM End Address */
    LCD_WriteReg(R82, 0x0000);  /* Vertical GRAM Start Address */
    LCD_WriteReg(R83, 0x013F);  /* Vertical GRAM End Address */
    LCD_WriteReg(R96,  0xA700); /* Gate Scan Line */
    LCD_WriteReg(R97,  0x0001); /* NDL,VLE, REV */
    LCD_WriteReg(R106, 0x0000); /* Set scrolling line */
    /* Partial Display Control -------------------------------------------------*/
    LCD_WriteReg(R128, 0x0000);
    LCD_WriteReg(R129, 0x0000);
    LCD_WriteReg(R130, 0x0000);
    LCD_WriteReg(R131, 0x0000);
    LCD_WriteReg(R132, 0x0000);
    LCD_WriteReg(R133, 0x0000);
    /* Panel Control -----------------------------------------------------------*/
    LCD_WriteReg(R144, 0x0010);
    LCD_WriteReg(R146, 0x0000);
    LCD_WriteReg(R147, 0x0003);
    LCD_WriteReg(R149, 0x0110);
    LCD_WriteReg(R151, 0x0000);
    LCD_WriteReg(R152, 0x0000);
    /* Set GRAM write direction and BGR = 1
       I/D=01 (Horizontal : increment, Vertical : decrement)
       AM=1 (address is updated in vertical writing direction) */
    LCD_WriteReg(R3, 0x1018);
    LCD_WriteReg(R7, 0x0112);   /* 262K color and display ON */
  }
  else if(LCDType == LCD_HX8312)
  {
    /* Enable the LCD Oscillator ---------------------------------------------*/
    LCD_WriteReg(R1, 0x10);
    LCD_WriteReg(R0, 0xA0);
    LCD_WriteReg(R3, 0x01);
    _delay_(1); /* Delay 10 ms */
    LCD_WriteReg(R3, 0x00);
    LCD_WriteReg(R43, 0x04);
  
    LCD_WriteReg(R40, 0x18);
    LCD_WriteReg(R26, 0x05);
    LCD_WriteReg(R37, 0x05);
    LCD_WriteReg(R25, 0x00);
    
    /* LCD Power On ----------------------------------------------------------*/
    LCD_WriteReg(R28, 0x73);
    LCD_WriteReg(R36, 0x74);
    LCD_WriteReg(R30, 0x01);
    LCD_WriteReg(R24, 0xC1);
    _delay_(1); /* Delay 10 ms */
    LCD_WriteReg(R24, 0xE1);
    LCD_WriteReg(R24, 0xF1);
    _delay_(6); /* Delay 60 ms */
    LCD_WriteReg(R24, 0xF5);
    _delay_(6); /* Delay 60 ms */
    LCD_WriteReg(R27, 0x09);
    _delay_(1); /* Delay 10 ms */
    LCD_WriteReg(R31, 0x11);
    LCD_WriteReg(R32, 0x0E);
    LCD_WriteReg(R30, 0x81);
    _delay_(1); /* Delay 10 ms */
    
    /* Chip Set --------------------------------------------------------------*/
    LCD_WriteReg(R157, 0x00);
    LCD_WriteReg(R192, 0x00);
   
    LCD_WriteReg(R14, 0x00);
    LCD_WriteReg(R15, 0x00);
    LCD_WriteReg(R16, 0x00);
    LCD_WriteReg(R17, 0x00);
    LCD_WriteReg(R18, 0x00);
    LCD_WriteReg(R19, 0x00);
    LCD_WriteReg(R20, 0x00);
    LCD_WriteReg(R21, 0x00);
    LCD_WriteReg(R22, 0x00);
    LCD_WriteReg(R23, 0x00);
   
    LCD_WriteReg(R52, 0x01);
    LCD_WriteReg(R53, 0x00);
    LCD_WriteReg(R75, 0x00);
    LCD_WriteReg(R76, 0x00);
    LCD_WriteReg(R78, 0x00);
    LCD_WriteReg(R79, 0x00);
    LCD_WriteReg(R80, 0x00);
  
    LCD_WriteReg(R60, 0x00);
    LCD_WriteReg(R61, 0x00);
    LCD_WriteReg(R62, 0x01);
    LCD_WriteReg(R63, 0x3F);
    LCD_WriteReg(R64, 0x02);
    LCD_WriteReg(R65, 0x02);
    LCD_WriteReg(R66, 0x00);
    LCD_WriteReg(R67, 0x00);
    LCD_WriteReg(R68, 0x00);
    LCD_WriteReg(R69, 0x00);
    LCD_WriteReg(R70, 0xEF);
    LCD_WriteReg(R71, 0x00);
    LCD_WriteReg(R72, 0x00);
    LCD_WriteReg(R73, 0x01);
    LCD_WriteReg(R74, 0x3F);
  
    LCD_WriteReg(R29, 0x08);  /* R29:Gate scan direction setting */
  
    LCD_WriteReg(R134, 0x00);
    LCD_WriteReg(R135, 0x30);
    LCD_WriteReg(R136, 0x02);
    LCD_WriteReg(R137, 0x05);
  
    LCD_WriteReg(R141, 0x01);  /* R141:Register set-up mode for one line clock */
    LCD_WriteReg(R139, 0x20);  /* R139:One line SYSCLK number in one-line */
    LCD_WriteReg(R51, 0x01);  /* R51:N line inversion setting */
    LCD_WriteReg(R55, 0x01);  /* R55:Scanning method setting */
    LCD_WriteReg(R118, 0x00);
   
    /* Gamma Set -------------------------------------------------------------*/
    LCD_WriteReg(R143, 0x10);
    LCD_WriteReg(R144, 0x67);
    LCD_WriteReg(R145, 0x07);
    LCD_WriteReg(R146, 0x65);
    LCD_WriteReg(R147, 0x07);
    LCD_WriteReg(R148, 0x01);
    LCD_WriteReg(R149, 0x76);
    LCD_WriteReg(R150, 0x56);
    LCD_WriteReg(R151, 0x00);
    LCD_WriteReg(R152, 0x06);
    LCD_WriteReg(R153, 0x03);
    LCD_WriteReg(R154, 0x00);
  
    /* Display On ------------------------------------------------------------*/
    LCD_WriteReg(R1, 0x50);
    LCD_WriteReg(R5, 0x04);
    LCD_WriteReg(R0, 0x80);
    LCD_WriteReg(R59, 0x01);
    _delay_(4);  /* Delay 40 ms */
    LCD_WriteReg(R0, 0x20);
  }  
}


/**
  * @brief  Initializes the LCD.
  * 功能说明(中文) : 上电初始化并自动识别 LCD 控制器型号。先按默认型号初始化，再读回
  *                  ID 寄存器(R0)核对；不符则切换型号重新初始化，依次尝试
  *                  ILI9320 -> SPFD5408 -> HX8312。
  * 参数(中文)     : 无。
  * 返回(中文)     : 无。
  * 备注(中文)     : 探测结果写入全局变量 LCDType，供后续所有底层函数使用；
  *                  一般应在系统启动时调用一次。
  * @param  None
  * @retval None
  */
void STM3210B_LCD_Init(void)
{
  /* Setups the LCD */
  LCD_Init();
  /* Try to read new LCD controller ID 0x9320 */
  if (LCD_ReadReg(R0) == LCD_ILI9320)
  {
    LCDType = LCD_ILI9320;
  }
  else
  {
    LCDType = LCD_SPFD5408;
    /* Setups the LCD */
    LCD_Init();
    /* Try to read new LCD controller ID 0x5408 */
    if (LCD_ReadReg(R0) != LCD_SPFD5408)
    {
      LCDType = LCD_HX8312;
      /* Setups the LCD */
      LCD_Init();
    }
  }
}


/**
  * @brief  Sets the Text color.
  * 功能说明(中文) : 设置后续字符/线条绘制使用的前景(文字)颜色。
  * 参数(中文)     : Color - RGB565 颜色值(16 位，格式 R5G6B5)，如 Red=0xF800。
  * 返回(中文)     : 无。
  * 备注(中文)     : 修改静态全局变量 TextColor，对后续绘制立即生效，不影响已显示内容。
  * @param  Color: specifies the Text color code RGB(5-6-5).
  * @retval None
  */
void LCD_SetTextColor(__IO uint16_t Color)
{
  TextColor = Color;
}


/**
  * @brief  Sets the Background color.
  * 功能说明(中文) : 设置字符点阵中"灭点"及清屏/清行时使用的背景颜色。
  * 参数(中文)     : Color - RGB565 颜色值(16 位，格式 R5G6B5)，如 White=0xFFFF。
  * 返回(中文)     : 无。
  * 备注(中文)     : 修改静态全局变量 BackColor，对后续绘制立即生效。
  * @param  Color: specifies the Background color code RGB(5-6-5).
  * @retval None
  */
void LCD_SetBackColor(__IO uint16_t Color)
{
  BackColor = Color;
}


/**
  * @brief  Clears the selected line.
  * 功能说明(中文) : 用 20 个空格覆盖指定文本行，实现该行的清除。MC_Display 刷新界面时
  *                  用它擦除不再需要的行。
  * 参数(中文)     : Line - 文本行号(Line0~Line9，单位:行，每行 24 像素高)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 实为调用 LCD_DisplayStringLine 写入空格串，按当前 BackColor 显示背景。
  * @param  Line: the Line to be cleared. 
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..9
  * @retval None
  */
void LCD_ClearLine(uint8_t Line)
{
  LCD_DisplayStringLine(Line, "                    ");
}


/**
  * @brief  Clears the hole LCD.
  * 功能说明(中文) : 用指定颜色填充整屏显存，即清屏。
  * 参数(中文)     : Color - 填充用的 RGB565 颜色值(16 位)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 循环 240*320=76800 次写显存，耗时较长；调用前先把光标定位到显存
  *                  起始点，并对 ILI9320/SPFD5408 进入 GRAM 写准备状态。
  * @param  Color: the color of the background.
  * @retval None
  */
void LCD_Clear(uint16_t Color)
{
  uint32_t index = 0;
  
  LCD_SetCursor(0x00, 0x013F); 
  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  }
  for(index = 0; index < 76800; index++)   // 【中文】76800 = 240(行) x 320(列) 个像素
  {
    LCD_WriteRAM(Color);
  }
  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET); 
  }  
}


/**
  * @brief  Sets the cursor position.
  * 功能说明(中文) : 设置显存(GRAM)当前读写地址，后续写显存即从该点开始。
  * 参数(中文)     : Xpos - 行坐标(单位:像素行)；Ypos - 列坐标(单位:像素列)。
  * 返回(中文)     : 无。
  * 备注(中文)     : ILI9320/SPFD5408 用 R32/R33(水平/垂直地址)；
  *                  HX8312 用 R66/R67/R68(R67 为列地址高 1 位，R68 为低 8 位)。
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position. 
  * @retval None
  */
void LCD_SetCursor(uint8_t Xpos, uint16_t Ypos)
{
  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    LCD_WriteReg(R32, Xpos);  // 【中文】R32 = GRAM 水平(行)地址
    LCD_WriteReg(R33, Ypos);  // 【中文】R33 = GRAM 垂直(列)地址
  }
  else if(LCDType == LCD_HX8312)
  {
    LCD_WriteReg(R66, Xpos);
    LCD_WriteReg(R67, ((Ypos & 0x100)>> 8));
    LCD_WriteReg(R68, (Ypos & 0xFF));
  }
}


/**
  * @brief  Draws a character on LCD.
  * 功能说明(中文) : 在指定位置绘制一个 16x24 点阵字符。逐行(共 24 行)取出字模，按位
  *                  扫描本行的 16 列，位=1 画前景色、位=0 画背景色。
  * 参数(中文)     : Xpos - 起始像素行(单位:行)；
  *                  Ypos - 起始像素列(单位:列)；
  *                  c    - 指向该字符 24 个字模字的常量指针。
  * 返回(中文)     : 无。
  * 备注(中文)     : 每画完一行需重新定位光标并(对 ILI9320/SPFD5408)结束一次 GRAM 写。 
  * @param  Xpos: the Line where to display the character shape.
  * @param  Ypos: start column address.
  * @param  c: pointer to the character data.
  * @retval None
  */
void LCD_DrawChar(uint8_t Xpos, uint16_t Ypos, const uint16_t *c)
{
  uint32_t index = 0, i = 0;
  uint8_t Xaddress = 0;
   
  Xaddress = Xpos;
  
  LCD_SetCursor(Xaddress, Ypos);
  
  for(index = 0; index < 24; index++)
  {
    if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
    {
      LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    }
    for(i = 0; i < 16; i++)   // 【中文】逐位扫描本行的 16 个像素
    {
      if((c[index] & (1 << i)) == 0x00)   // 【中文】该位为 0 表示为"灭点"
      {
        LCD_WriteRAM(BackColor);   // 【中文】灭点用背景色绘制
      }
      else
      {
        LCD_WriteRAM(TextColor);
      }
    }
    if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
    {
      LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
    }  
    Xaddress++;
    LCD_SetCursor(Xaddress, Ypos);
  }
}


/**
  * @brief  Displays one character (16dots width, 24dots height).
  * 功能说明(中文) : 按 ASCII 码显示单个字符。先把 ASCII 减去 32 得到字模索引，再定位到
  *                  ASCII_Table 中该字符的 24 字字模并调用 LCD_DrawChar 绘制。
  * 参数(中文)     : Line   - 显示行(Line0~Line9)；
  *                  Column - 起始列(单位:像素列)；
  *                  Ascii  - 字符 ASCII 码，取值范围 0x20~0x7E。
  * 返回(中文)     : 无。
  * 备注(中文)     : 偏移 (Ascii-32)*24 必须与字模表布局一致；供 MC_Display 模块调用。
  * @param  Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..9
  * @param  Column: start column address.
  * @param  Ascii: character ascii code, must be between 0x20 and 0x7E.
  * @retval None
  */
void LCD_DisplayChar(uint8_t Line, uint16_t Column, uint8_t Ascii)
{
  Ascii -= 32;   // 【中文】去掉 ASCII 表前 32 个不可显示字符(字模从 0x20 开始)
  LCD_DrawChar(Line, Column, &ASCII_Table[Ascii * 24]);   // 【中文】每字符占 24 个字，定位其字模首地址
}


/**
  * @brief  Displays a maximum of 20 char on the LCD.
  * 功能说明(中文) : 在指定行从右向左显示字符串(最多 20 个字符)。每显示一个字符后列坐标
  *                  减 16，直到遇到字符串结束符(0)或达到 20 个字符。
  * 参数(中文)     : Line - 显示行(Line0~Line9)；ptr - 待显示字符串指针(以 0 结尾)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 起始列为 319(屏幕最右)，故文本自右向左排列；是 MC_Display 的主刷新接口。
  * @param  Line: the Line where to display the character shape .
  *   This parameter can be one of the following values:
  *     @arg Linex: where x can be 0..9
  * @param  *ptr: pointer to string to display on LCD.
  * @retval None
  */
void LCD_DisplayStringLine(uint8_t Line, uint8_t *ptr)
{
  uint32_t i = 0;
  uint16_t refcolumn = 319;
  /* Send the string character by character on lCD */
  while ((*ptr != 0) & (i < 20))
  {
    /* Display one character on LCD */
    LCD_DisplayChar(Line, refcolumn, *ptr);
    /* Decrement the column position by 16 */
    refcolumn -= 16;
    /* Point on the next character */
    ptr++;
    /* Increment the character counter */
    i++;
  }
}


/**
  * @brief  Sets a display window
  * 功能说明(中文) : 设置显存显示窗口(矩形区域)，之后写入的像素被限制在该窗口内。
  * 参数(中文)     : Xpos   - 窗口水平方向位置(单位:像素行)；
  *                  Ypos   - 窗口垂直方向位置(单位:像素列)；
  *                  Height - 窗口高度(单位:像素行)；
  *                  Width  - 窗口宽度(单位:像素列)。
  * 返回(中文)     : 无。
  * 备注(中文)     : ILI9320/SPFD5408 用 R80~R83 设 GRAM 起止地址；HX8312 用 R69~R74；
  *                  最后把光标定位到 (Xpos,Ypos)。
  * @param  Xpos: specifies the X buttom left position.
  * @param  Ypos: specifies the Y buttom left position.
  * @param  Height: display window height.
  * @param  Width: display window width.
  * @retval None
  */
void LCD_SetDisplayWindow(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
{
  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    /* Horizontal GRAM Start Address */
    if(Xpos >= Height)
    {
      LCD_WriteReg(R80, (Xpos - Height + 1));
    }
    else
    {
      LCD_WriteReg(R80, 0);
    }
    /* Horizontal GRAM End Address */
    LCD_WriteReg(R81, Xpos);
    /* Vertical GRAM Start Address */
    if(Ypos >= Width)
    {
      LCD_WriteReg(R82, (Ypos - Width + 1));
    }  
    else
    {
      LCD_WriteReg(R82, 0);
    }
    /* Vertical GRAM End Address */
    LCD_WriteReg(R83, Ypos);
  }
  else if(LCDType == LCD_HX8312)
  {  
    LCD_WriteReg(R1, 0xD0);
    LCD_WriteReg(R5, 0x14);
  
    LCD_WriteReg(R69, (Xpos - Height + 1));
    LCD_WriteReg(R70, Xpos);
 
    LCD_WriteReg(R71, (((Ypos - Width + 1) & 0x100)>> 8));
    LCD_WriteReg(R72, ((Ypos - Width + 1) & 0xFF));
    LCD_WriteReg(R73, ((Ypos & 0x100)>> 8));
    LCD_WriteReg(R74, (Ypos & 0xFF));
  }
  LCD_SetCursor(Xpos, Ypos);
}


/**
  * @brief  Disables LCD Window mode.
  * 功能说明(中文) : 关闭显示窗口模式，恢复为整屏(240x320)可写。
  * 参数(中文)     : 无。
  * 返回(中文)     : 无。
  * 备注(中文)     : ILI9320/SPFD5408 把窗口设为整屏并恢复写方向(R3=0x1018)；HX8312 恢复 R1/R5。
  * @param  None
  * @retval None
  */
void LCD_WindowModeDisable(void)
{
  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    LCD_SetDisplayWindow(239, 0x13F, 240, 320);
    LCD_WriteReg(R3, 0x1018);
  }
  else if(LCDType == LCD_HX8312)
  {
    LCD_WriteReg(R1, 0x50);
    LCD_WriteReg(R5, 0x04); 
  }
    
}


/**
  * @brief  Displays a line.
  * 功能说明(中文) : 以当前前景色绘制水平或垂直直线。
  * 参数(中文)     : Xpos/Ypos - 起点坐标(行/列，单位:像素)；
  *                  Length    - 线长(单位:像素)；
  *                  Direction - 方向，Horizontal(水平) 或 Vertical(垂直)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 水平线连续写显存；垂直线逐点重设光标再写；颜色取自 TextColor。
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Length: line length.
  * @param  Direction: line direction.
  *   This parameter can be one of the following values: Vertical or Horizontal.
  * @retval None
  */
void LCD_DrawLine(uint8_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction)
{
  uint32_t i = 0;
  
  LCD_SetCursor(Xpos, Ypos);
  if(Direction == Horizontal)
  { 
    if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
    {
      LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    }
    for(i = 0; i < Length; i++)
    {
      LCD_WriteRAM(TextColor);
    }
    if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
    {
      LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
    }
  }
  else
  {
   for(i = 0; i < Length; i++)
    {
      if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
      {
        LCD_WriteRAMWord(TextColor);
      }
      else  if(LCDType == LCD_HX8312)
      {
        LCD_WriteRAM(TextColor);
      }
      Xpos++;
      LCD_SetCursor(Xpos, Ypos);
    }
  }
}


/**
  * @brief  Displays a rectangle.
  * 功能说明(中文) : 绘制矩形边框，由上/下两条水平线与左/右两条垂直线组成。
  * 参数(中文)     : Xpos/Ypos   - 矩形参考角坐标(行/列，单位:像素)；
  *                  Height/Width- 矩形高度/宽度(单位:像素)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 内部四次调用 LCD_DrawLine，只画边框、不填充内部。
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Height: display rectangle height.
  * @param  Width: display rectangle width.
  * @retval None
  */
void LCD_DrawRect(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
{
  LCD_DrawLine(Xpos, Ypos, Width, Horizontal);
  LCD_DrawLine((Xpos + Height), Ypos, Width, Horizontal);
  
  LCD_DrawLine(Xpos, Ypos, Height, Vertical);
  LCD_DrawLine(Xpos, (Ypos - Width + 1), Height, Vertical);
}


/**
  * @brief  Displays a circle.
  * 功能说明(中文) : 用 Bresenham 中点画圆算法绘制圆形边框，利用八分对称一次算出圆周上
  *                  的八个对称点。
  * 参数(中文)     : Xpos/Ypos - 圆心坐标(行/列，单位:像素)；Radius - 半径(单位:像素)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 决策变量 D 控制下一像素的步进方向；颜色取自 TextColor。
  * @param  Xpos: specifies the X position.
  * @param  Ypos: specifies the Y position.
  * @param  Radius
  * @retval None
  */
void LCD_DrawCircle(uint8_t Xpos, uint16_t Ypos, uint16_t Radius)
{
  int32_t  D;/* Decision Variable */ 
  uint32_t  CurX;/* Current X Value */
  uint32_t  CurY;/* Current Y Value */ 
  
  D = 3 - (Radius << 1);   // 【中文】Bresenham 画圆算法决策变量的初值
  CurX = 0;
  CurY = Radius;
  
  while (CurX <= CurY)
  {
    LCD_SetCursor(Xpos + CurX, Ypos + CurY);
    if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
    {
      LCD_WriteRAMWord(TextColor);
    }
    else if(LCDType == LCD_HX8312)
    {
      LCD_WriteRAM(TextColor);
    }
    LCD_SetCursor(Xpos + CurX, Ypos - CurY);

    if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
    {
      LCD_WriteRAMWord(TextColor);
    }
    else if(LCDType == LCD_HX8312)
    {
      LCD_WriteRAM(TextColor);
    }
    LCD_SetCursor(Xpos - CurX, Ypos + CurY);

    if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
    {
      LCD_WriteRAMWord(TextColor);
    }
    else if(LCDType == LCD_HX8312)
    {
      LCD_WriteRAM(TextColor);
    }
    LCD_SetCursor(Xpos - CurX, Ypos - CurY);

    if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
    {
      LCD_WriteRAMWord(TextColor);
    }
    else if(LCDType == LCD_HX8312)
    {
      LCD_WriteRAM(TextColor);
    }
    LCD_SetCursor(Xpos + CurY, Ypos + CurX);
    if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
    {
      LCD_WriteRAMWord(TextColor);
    }
    else if(LCDType == LCD_HX8312)
    {
      LCD_WriteRAM(TextColor);
    }
    LCD_SetCursor(Xpos + CurY, Ypos - CurX);

    if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
    {
      LCD_WriteRAMWord(TextColor);
    }
    else if(LCDType == LCD_HX8312)
    {
      LCD_WriteRAM(TextColor);
    }
    LCD_SetCursor(Xpos - CurY, Ypos + CurX);

    if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
    {
      LCD_WriteRAMWord(TextColor);
    }
    else if(LCDType == LCD_HX8312)
    {
      LCD_WriteRAM(TextColor);
    }
    LCD_SetCursor(Xpos - CurY, Ypos - CurX);

    if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
    {
      LCD_WriteRAMWord(TextColor);
    }
    else if(LCDType == LCD_HX8312)
    {
      LCD_WriteRAM(TextColor);
    }
    if (D < 0)
    { 
      D += (CurX << 2) + 6;
    }
    else
    {
      D += ((CurX - CurY) << 2) + 10;
      CurY--;
    }
    CurX++;
  }
}


/**
  * @brief  Displays a monocolor picture.
  * 功能说明(中文) : 显示一幅单色位图。将位图数据按位展开，位=1 画前景色、位=0 画背景色，
  *                  整屏共 2400x32=76800 个像素。
  * 参数(中文)     : Pict - 指向位图数据数组的指针(每个 uint32_t 含 32 个像素)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 光标定位于 (0,319) 起逐点写入；依赖 TextColor/BackColor。
  * @param  Pict: pointer to the picture array.
  * @retval None
  */
void LCD_DrawMonoPict(const uint32_t *Pict)
{
  uint32_t index = 0, i = 0;
  LCD_SetCursor(0, 319); 

  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  }

  for(index = 0; index < 2400; index++)   // 【中文】2400 x 32 = 76800 像素，正好覆盖整屏
  {
    for(i = 0; i < 32; i++)
    {
      if((Pict[index] & (1 << i)) == 0x00)
      {
        LCD_WriteRAM(BackColor);
      }
      else
      {
        LCD_WriteRAM(TextColor);
      }
    }
  }

  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
  }
}

#ifdef USE_LCD_DrawBMP 
/**
  * @brief  Displays a bitmap picture loaded in the SPI Flash.
  * 功能说明(中文) : 显示存放在板上 SPI Flash 中的 BMP 位图。先读出位图大小与数据偏移，
  *                  把 SPI1 切到 16 位模式，从 Flash 流式读取像素并写入 LCD 显存。
  * 参数(中文)     : BmpAddress - 位图在 SPI Flash 中的首地址(单位:字节偏移)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 仅在宏 USE_LCD_DrawBMP 定义时编译；临时把 SPI1 改为 16 位传输，
  *                  完成后恢复为 8 位；依赖 SPI Flash 读写函数。
  * @param  BmpAddress: Bmp picture address in the SPI Flash.
  * @retval None
  */
void LCD_DrawBMP(uint32_t BmpAddress)
{
  uint32_t i = 0, size = 0;
  /* Read bitmap size */
  SPI_FLASH_BufferRead((uint8_t*)&size, BmpAddress + 2, 4);
  /* get bitmap data address offset */
  SPI_FLASH_BufferRead((uint8_t*)&i, BmpAddress + 10, 4);
  
  size = (size - i)/2;
  SPI_FLASH_StartReadSequence(BmpAddress + i);
  /* Disable SPI1  */
  SPI_Cmd(SPI1, DISABLE);
  /* SPI in 16-bit mode */
  SPI_DataSizeConfig(SPI1, SPI_DataSize_16b);
  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);
  
  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    /* Set GRAM write direction and BGR = 1 */
    /* I/D=00 (Horizontal : decrement, Vertical : decrement) */
    /* AM=1 (address is updated in vertical writing direction) */
    LCD_WriteReg(R3, 0x1008);
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
  }
  
  /* Read bitmap data from SPI Flash and send them to LCD */
  for(i = 0; i < size; i++)
  {
    LCD_WriteRAM(__REV16(SPI_FLASH_SendHalfWord(0xA5A5)));
  }
  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
  }

  /* Deselect the FLASH: Chip Select high */
  SPI_FLASH_CS_HIGH();
  /* Disable SPI1  */
  SPI_Cmd(SPI1, DISABLE);
  /* SPI in 8-bit mode */
  SPI_DataSizeConfig(SPI1, SPI_DataSize_8b);
  /* Enable SPI1  */
  SPI_Cmd(SPI1, ENABLE);

  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    /* Set GRAM write direction and BGR = 1 */
    /* I/D = 01 (Horizontal : increment, Vertical : decrement) */
    /* AM = 1 (address is updated in vertical writing direction) */
    LCD_WriteReg(R3, 0x1018);
  }
}
#endif /* USE_LCD_DrawBMP */
 


/**
  * @brief  Reset LCD control line(/CS) and Send Start-Byte
  * 功能说明(中文) : 拉低片选(NCS)选中 LCD，并通过 SPI 发送起始字节，标志一次命令/数据
  *                  传输开始；发送后轮询 SPI 忙标志直到数据发送完毕。
  * 参数(中文)     : Start_Byte - 起始命令字节(高 6 位固定 0x70，低 2 位为操作类型)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 依赖全局 LCD_SPI(SPI2)与 NCS 控制线；是所有底层访问的第一步。
  * @param  Start_Byte: the Start-Byte to be sent
  * @retval None
  */
void LCD_nCS_StartByte(uint8_t Start_Byte)
{
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_RESET);

  SPI_I2S_SendData(LCD_SPI, Start_Byte);   // 【中文】经 SPI2 发送命令/起始字节

  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)   // 【中文】轮询 SPI 忙标志(BSY)等待本字节发完
  {
  }
}


/**
  * @brief  Writes index to select the LCD register.
  * 功能说明(中文) : 向 LCD 写入选中的寄存器编号(索引)。先发起始字节+SET_INDEX 命令，
  *                  再发 16 位索引(高字节为 0，低字节为寄存器号)。
  * 参数(中文)     : LCD_Reg - 目标寄存器编号(Rxx 宏，取值 0x00~0xFF)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 写完拉高片选；每步发送后需轮询 SPI 忙标志，确保时序正确。
  * @param  LCD_Reg: address of the selected register.
  * @retval None
  */
void LCD_WriteRegIndex(uint8_t LCD_Reg)
{
  /* Reset LCD control line(/CS) and Send Start-Byte */
  LCD_nCS_StartByte(START_BYTE | SET_INDEX);

  /* Write 16-bit Reg Index (High Byte is 0) */
  SPI_I2S_SendData(LCD_SPI, 0x00);   // 【中文】寄存器索引的高字节固定为 0

  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
  {
  }

  SPI_I2S_SendData(LCD_SPI, LCD_Reg);

  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
  {
  }

  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
}


/**
  * @brief  Writes to the selected LCD ILI9320 register.
  * 功能说明(中文) : ILI9320/SPFD5408 的寄存器写实现：先写 16 位寄存器号，再按"起始字节
  *                  +写寄存器命令"发送 16 位数据(先高字节后低字节)。
  * 参数(中文)     : LCD_Reg - 目标寄存器编号(0x00~0xFF)；
  *                  LCD_RegValue - 16 位寄存器值(按 R5G6B5 或控制器定义)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 私有函数，经 LCD_WriteReg 分发调用；内部两次轮询 SPI 忙标志。
  * @param  LCD_Reg: address of the selected register.
  * @param  LCD_RegValue: value to write to the selected register.
  * @retval None
  */
static void LCD_WriteRegILI9320(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
  /* Write 16-bit Index (then Write Reg) */
  LCD_WriteRegIndex(LCD_Reg);

  /* Write 16-bit Reg */
  /* Reset LCD control line(/CS) and Send Start-Byte */
  LCD_nCS_StartByte(START_BYTE | LCD_WRITE_REG);

  SPI_I2S_SendData(LCD_SPI, LCD_RegValue>>8);

  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
  {
  }

  SPI_I2S_SendData(LCD_SPI, (LCD_RegValue & 0xFF));

  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
  {
  }

  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
}


/**
  * @brief  Reads the selected LCD Register.
  * 功能说明(中文) : 读回指定 LCD 寄存器的 16 位值(如读 R0 得到控制器 ID)。先把 SPI 分频
  *                  设为 4 降低速率，发读命令后丢弃若干无效字节，再读回高/低字节拼接。
  * 参数(中文)     : LCD_Reg - 目标寄存器编号(0x00~0xFF)。
  * 返回(中文)     : 该寄存器的 16 位值(高字节在前)。
  * 备注(中文)     : 起始字节后 5 个字节为无效/填充读取；读操作较慢，读完恢复 SPI 分频。
  * @param  LCD_Reg: address of the selected register.
  * @retval LCD Register Value.
  */
uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{
  uint16_t tmp = 0;
  uint8_t i = 0;
  
  /* LCD_SPI prescaler: 4 */
  /* 【中文】读操作需降低速率：先清除 CR1 的波特率位[5:3]，再设为 2 分频(值 100 表示 /4)。
     说明：本驱动轮询的是 SPI 的忙标志(BSY)，并非 LCD 控制器的忙标志(BF)。 */
  LCD_SPI->CR1 &= 0xFFC7;
  LCD_SPI->CR1 |= 0x0008;
  /* Write 16-bit Index (then Read Reg) */
  LCD_WriteRegIndex(LCD_Reg);
  /* Read 16-bit Reg */
  /* Reset LCD control line(/CS) and Send Start-Byte */
  LCD_nCS_StartByte(START_BYTE | LCD_READ_REG);
  
  for(i = 0; i < 5; i++)
  {
    SPI_I2S_SendData(LCD_SPI, 0xFF);
    while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
    {
    }
    /* One byte of invalid dummy data read after the start byte */
    while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET)
    {    
    }
    SPI_I2S_ReceiveData(LCD_SPI); 
  }

  SPI_I2S_SendData(LCD_SPI, 0xFF);

  /* Read upper byte */
  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
  {
  }

  /* Read lower byte */
  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET)
  {
  }
  tmp = SPI_I2S_ReceiveData(LCD_SPI);
  
  
  SPI_I2S_SendData(LCD_SPI, 0xFF);
  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
  {
  }

  /* Read lower byte */
  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET)
  {
  }

  tmp = ((tmp & 0xFF) << 8) | SPI_I2S_ReceiveData(LCD_SPI);
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);

  /* LCD_SPI prescaler: 2 */
  LCD_SPI->CR1 &= 0xFFC7;

  return tmp;
}


/**
  * @brief  Prepare to write to the LCD RAM.
  * 功能说明(中文) : 让 LCD 进入写显存(GRAM)状态：选中 GRAM 数据寄存器 R34 并发出写命令，
  *                  之后连续发送的数值即依次写入显存像素。
  * 参数(中文)     : 无。
  * 返回(中文)     : 无。
  * 备注(中文)     : 仅对 ILI9320/SPFD5408 使用；HX8312 通过 LCD_WriteRAM 直接写。
  * @param  None
  * @retval None
  */
void LCD_WriteRAM_Prepare(void)
{
  LCD_WriteRegIndex(R34); /* Select GRAM Reg */

  /* Reset LCD control line(/CS) and Send Start-Byte */
  LCD_nCS_StartByte(START_BYTE | LCD_WRITE_REG);
}


/**
  * @brief  Writes 1 word to the LCD RAM.
  * 功能说明(中文) : 向显存写入一个像素(字)。内部先进入 GRAM 写准备，写一个像素后拉高片选。
  * 参数(中文)     : RGB_Code - 像素颜色，RGB565 格式(16 位)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 仅用于 ILI9320/SPFD5408；每写一个点都重新发起一次写准备，效率较低。
  * @param  RGB_Code: the pixel color in RGB mode (5-6-5).
  * @retval None
  */
void LCD_WriteRAMWord(uint16_t RGB_Code)
{

  LCD_WriteRAM_Prepare();

  LCD_WriteRAM(RGB_Code);

  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
}


/**
  * @brief  Writes to the selected LCD HX8312 register.
  * 功能说明(中文) : HX8312 的寄存器写实现：拉低 NWR/RS/NCS 组成并口写时序，把寄存器号与
  *                  8 位数据拼成一个 16 位字经 SPI 发出，再拉高片选。
  * 参数(中文)     : LCD_Reg - 目标寄存器编号(0x00~0xFF)；
  *                  LCD_RegValue - 8 位寄存器值(高 8 位为寄存器号)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 私有函数，经 LCD_WriteReg 分发调用；控制线需在 LCD_CtrlLinesConfig 中配好。
  * @param  LCD_Reg: address of the selected register.
  * @param  LCD_RegValue: value to write to the selected register.
  * @retval None
  */
static void LCD_WriteRegHX8312(uint8_t LCD_Reg, uint8_t LCD_RegValue)
{
  uint16_t tmp = 0;
  
  LCD_CtrlLinesWrite(LCD_NWR_GPIO_PORT, LCD_NWR_PIN, Bit_RESET);
  LCD_CtrlLinesWrite(LCD_RS_GPIO_PORT, LCD_RS_PIN, Bit_RESET);
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_RESET);
  
  tmp = LCD_Reg << 8;
  tmp |= LCD_RegValue;
   
  SPI_I2S_SendData(LCD_SPI, tmp);

  while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
  {
  }
  
  LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
}


/**
  * @brief  Writes to the selected LCD register.
  * 功能说明(中文) : 寄存器写统一入口，按当前 LCDType 分派到 ILI9320/SPFD5408 或 HX8312 的
  *                  具体实现。
  * 参数(中文)     : LCD_Reg - 目标寄存器编号(0x00~0xFF)；
  *                  LCD_RegValue - 寄存器值(16 位；HX8312 仅取低 8 位)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 型号未知(LCDType 既非上述型号)时不做任何操作。
  * @param  LCD_Reg: address of the selected register.
  * @param  LCD_RegValue: value to write to the selected register.
  * @retval None
  */
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    LCD_WriteRegILI9320(LCD_Reg, LCD_RegValue);
  }
  else if(LCDType == LCD_HX8312)
  {
    LCD_WriteRegHX8312(LCD_Reg, ((uint8_t)LCD_RegValue));
  }
}


/**
  * @brief  Writes to the LCD RAM.
  * 功能说明(中文) : 向 LCD 显存写入一个像素(16 位颜色)。按型号分派：
  *                  ILI9320/SPFD5408 先发高字节再发低字节；HX8312 用并行模拟时序一次发 16 位。
  * 参数(中文)     : RGB_Code - 像素颜色，RGB565 格式(16 位)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 写显存前通常需先调用 LCD_WriteRAM_Prepare()(ILI9320/SPFD5408)。
  * @param  RGB_Code: the pixel color in RGB mode (5-6-5).
  * @retval None
  */
void LCD_WriteRAM(uint16_t RGB_Code)
{
  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    SPI_I2S_SendData(LCD_SPI, RGB_Code >> 8);   // 【中文】先发颜色高字节
    while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
    {
    }
    SPI_I2S_SendData(LCD_SPI, RGB_Code & 0xFF);
    while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
    {
    }
  }

  if(LCDType == LCD_HX8312)
  {
    LCD_CtrlLinesWrite(LCD_NWR_GPIO_PORT, LCD_NWR_PIN, Bit_RESET);
    LCD_CtrlLinesWrite(LCD_RS_GPIO_PORT, LCD_RS_PIN, Bit_SET);
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_RESET);
    SPI_I2S_SendData(LCD_SPI, RGB_Code);

    while(SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET)
    {
    }
  
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
  }
}


/**
  * @brief  Power on the LCD.
  * 功能说明(中文) : 执行 LCD 上电时序(电源稳定、升压电路与 VCOM 电压设置)，并点亮显示，
  *                  用于从断电/待机状态恢复。
  * 参数(中文)     : 无。
  * 返回(中文)     : 无。
  * 备注(中文)     : 依赖 LCDType；内部含 _delay_() 毫秒级等待以保证电源稳定。
  * @param  None
  * @retval None
  */
void LCD_PowerOn(void)
{
  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    /* Power On sequence ---------------------------------------------------------*/
    LCD_WriteReg(R16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(R17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
    LCD_WriteReg(R18, 0x0000); /* VREG1OUT voltage */
    LCD_WriteReg(R19, 0x0000); /* VDV[4:0] for VCOM amplitude */
    _delay_(20);               /* Dis-charge capacitor power voltage (200ms) */
    LCD_WriteReg(R16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(R17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
    _delay_(5);                /* Delay 50 ms */
    LCD_WriteReg(R18, 0x0139); /* VREG1OUT voltage */
    _delay_(5);                /* delay 50 ms */
    LCD_WriteReg(R19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
    LCD_WriteReg(R41, 0x0013); /* VCM[4:0] for VCOMH */
    _delay_(5);                /* delay 50 ms */
    LCD_WriteReg(R7, 0x0173);  /* 262K color and display ON */
  }
  else if(LCDType == LCD_HX8312)
  {  
    /* Power On Set */
    LCD_WriteReg(R28, 0x73);
    LCD_WriteReg(R36, 0x74);
    LCD_WriteReg(R30, 0x01);
    LCD_WriteReg(R24, 0xC1);
    _delay_(1); /* Delay 10 ms */
    LCD_WriteReg(R24, 0xE1);
    LCD_WriteReg(R24, 0xF1);
    _delay_(6); /* Delay 60 ms */
    LCD_WriteReg(R24, 0xF5);
    _delay_(6); /* Delay 60 ms */
    LCD_WriteReg(R27, 0x09);
    _delay_(1); /* Delay 10 ms */
    LCD_WriteReg(R31, 0x11);
    LCD_WriteReg(R32, 0x0E);
    LCD_WriteReg(R30, 0x81);
    _delay_(1); /* Delay 10 ms */
  }
}


/**
  * @brief  Enables the Display.
  * 功能说明(中文) : 打开 LCD 显示(不重新初始化，仅切换显示开关寄存器)，原显存内容保留。
  * 参数(中文)     : 无。
  * 返回(中文)     : 无。
  * 备注(中文)     : ILI9320/SPFD5408 置 R7=0x0173；HX8312 需先设相关寄存器再开显示。
  * @param  None
  * @retval None
  */
void LCD_DisplayOn(void)
{
  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    /* Display On */
    LCD_WriteReg(R7, 0x0173); /* 262K color and display ON */
  }
  else if(LCDType == LCD_HX8312)
  {  
    LCD_WriteReg(R1, 0x50);
    LCD_WriteReg(R5, 0x04);
    /* Display On */
    LCD_WriteReg(R0, 0x80);
    LCD_WriteReg(R59, 0x01);
    _delay_(4); /* Delay 40 ms */
    LCD_WriteReg(R0, 0x20);
  }  
}


/**
  * @brief  Disables the Display.
  * 功能说明(中文) : 关闭 LCD 显示(仅切换显示开关寄存器)，显存内容保留。
  * 参数(中文)     : 无。
  * 返回(中文)     : 无。
  * 备注(中文)     : ILI9320/SPFD5408 置 R7=0；HX8312 需先关显示再延时。
  * @param  None
  * @retval None
  */
void LCD_DisplayOff(void)
{
  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    /* Display Off */
    LCD_WriteReg(R7, 0x0);
  }
  else if(LCDType == LCD_HX8312)
  { 
    /* Display Off */
    LCD_WriteReg(R0, 0xA0);
    _delay_(4);  /* Delay 40 ms */
    LCD_WriteReg(R59, 0x00);
  } 
}


/**
  * @brief  Configures LCD control lines in Output Push-Pull mode.
  * 功能说明(中文) : 配置 LCD 控制引脚 GPIO(NCS、RS、NWR)为推挽输出模式并使能相应时钟，
  *                  最后把 NWR 与 RS 置为高电平(空闲态)。
  * 参数(中文)     : 无。
  * 返回(中文)     : 无。
  * 备注(中文)     : 应在 LCD 初始化早期调用；NCS 在 GPIOB，RS/NWR 在 GPIOD。
  * @param  None
  * @retval None
  */
void LCD_CtrlLinesConfig(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(LCD_RSNWR_GPIO_CLK | LCD_NCS_GPIO_CLK, ENABLE);

  /* Configure NCS in Output Push-Pull mode */
  GPIO_InitStructure.GPIO_Pin = LCD_NCS_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(LCD_NCS_GPIO_PORT, &GPIO_InitStructure);
  
  /* Configure NWR(RNW), RS in Output Push-Pull mode */
  GPIO_InitStructure.GPIO_Pin = LCD_RS_PIN | LCD_NWR_PIN;
  GPIO_Init(LCD_RS_GPIO_PORT, &GPIO_InitStructure);
  
  LCD_CtrlLinesWrite(LCD_NWR_GPIO_PORT, LCD_NWR_PIN, Bit_SET);
  LCD_CtrlLinesWrite(LCD_RS_GPIO_PORT, LCD_RS_PIN, Bit_SET);
}


/**
  * @brief  Sets or reset LCD control lines.
  * 功能说明(中文) : 置位/复位指定的 LCD 控制引脚(对 GPIO 端口按位写)，用于软件模拟并口时序。
  * 参数(中文)     : GPIOx    - 目标 GPIO 端口(GPIOB 或 GPIOD)；
  *                  CtrlPins - 控制引脚位掩码(NCS/NWR/RS 之一或多个)；
  *                  BitVal   - 写入电平，Bit_RESET(清零) 或 Bit_SET(置位)。
  * 返回(中文)     : 无。
  * 备注(中文)     : 对 LCD_CtrlLinesConfig 中配置的引脚直接操作。
  * @param  GPIOx: where x can be B or D to select the GPIO peripheral.
  * @param  CtrlPins: the Control line.
  *   This parameter can be:
  *     @arg LCD_NCS_PIN: Chip Select pin
  *     @arg LCD_NWR_PIN: Read/Write Selection pin
  *     @arg LCD_RS_PIN: Register/RAM Selection pin
  * @param  BitVal: specifies the value to be written to the selected bit.
  *   This parameter can be:
  *     @arg Bit_RESET: to clear the port pin
  *     @arg Bit_SET: to set the port pin
  * @retval None
  */
void LCD_CtrlLinesWrite(GPIO_TypeDef* GPIOx, uint16_t CtrlPins, BitAction BitVal)
{
  /* Set or Reset the control line */
  GPIO_WriteBit(GPIOx, CtrlPins, BitVal);
}


/**
  * @brief  Configures the LCD_SPI interface.
  * 功能说明(中文) : 配置 LCD 的 SPI 接口：使能 GPIO 与 SPI2 时钟，把 SCK/MISO/MOSI 配为
  *                  复用推挽输出，并把 SPI2 设为全双工主模式后使能。
  * 参数(中文)     : 无。
  * 返回(中文)     : 无。
  * 备注(中文)     : 依据 LCDType 选择参数：ILI9320/SPFD5408 用 8 位、CPOL=1/CPHA=2Edge；
  *                  HX8312 用 16 位、CPOL=0/CPHA=1Edge；波特率预分频固定为 2(最快)。
  * @param  None
  * @retval None
  */
void LCD_SPIConfig(void)
{
  SPI_InitTypeDef    SPI_InitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_APB2PeriphClockCmd(LCD_SPI_GPIO_CLK, ENABLE);

  /* Enable SPI clock  */
  RCC_APB1PeriphClockCmd(LCD_SPI_CLK, ENABLE);

  /* Configure SPI pins: SCK, MISO and MOSI */
  GPIO_InitStructure.GPIO_Pin = LCD_SPI_SCK_PIN | LCD_SPI_MISO_PIN | LCD_SPI_MOSI_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(LCD_SPI_GPIO_PORT, &GPIO_InitStructure);

  SPI_I2S_DeInit(LCD_SPI);
  
  /* SPI Config */
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

  if((LCDType == LCD_ILI9320) || (LCDType == LCD_SPFD5408))
  {
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;   // 【中文】ILI9320/SPFD5408：8 位数据
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;         // 【中文】时钟空闲电平为高
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;        // 【中文】在第 2 个时钟边沿采样数据
  }
  else if(LCDType == LCD_HX8312)
  {
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  }
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;

  SPI_Init(LCD_SPI, &SPI_InitStructure);

  /* SPI enable */
  SPI_Cmd(LCD_SPI, ENABLE);
}

#ifndef USE_Delay
/**
  * @brief  Inserts a delay time.
  * 功能说明(中文) : 软件空循环实现粗略延时(仅在未定义 USE_Delay 时编译)，供初始化时序使用。
  * 参数(中文)     : nCount - 延时系数(单位:约 10ms/计数，取决于主频)，循环 100000*nCount 次。
  * 返回(中文)     : 无。
  * 备注(中文)     : 精度较低且非精确；如需精确定时请定义 USE_Delay 改用外部 Delay()。
  * @param  nCount: specifies the delay time length.
  * @retval None
  */
static void delay(__IO uint32_t nCount)
{
  __IO uint32_t index = 0; 
  for(index = (100000 * nCount); index != 0; index--)
  {
  }
}
#endif /* USE_Delay*/
/**
  * @}
  */ 


/**
  * @}
  */ 

/**
  * @}
  */ 
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
