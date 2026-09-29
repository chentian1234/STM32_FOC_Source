/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Display.c
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file contains the software implementation of the
*                      display routines
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 14/07/08 v2.0.1
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
*   本文件实现 LCD 显示模块, 属演示板的人机界面(HMI)层:
*   1) Display_Welcome_Message() : 上电显示欢迎信息;
*   2) Display_LCD()             : 主刷新函数, 依据菜单索引 bMenu_index 选择界面并
*                                  刷新 LCD(约每 200ms 一次), 实现多页菜单显示;
*   3) Display_5DigitSignedNumber(): 在指定行/起始字符处显示 5 位带符号十进制数;
*   4) ComputeVisualization()    : 由菜单索引映射到"显示页编号 VISUALIZATION_x"。
*   显示的物理量来源: 转速(编码器/霍尔/无感)、Iq/Id 电流分量、转矩/磁链参考、
*   母线电压与温度等, 数值经定标换算后送 LCD 显示; 颜色高亮用于指示当前可调对象。
*   与 MC_Keys 模块配合: bMenu_index 由按键操作切换, 显示内容随其变化。
*******************************************************************************/
/* Standard include ----------------------------------------------------------*/
#include "stm32f10x_lib.h"  

/* Include of other module interface headers ---------------------------------*/
/* Local includes ------------------------------------------------------------*/

#include "stm32f10x_MClib.h"
#include "MC_Globals.h"

extern u8 bMenu_index;   // 全局菜单索引(定义于 MC_Keys.c), 决定当前显示页与可调对象

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BLINKING_TIME   5  // 5 * timebase_display_5 ms     // 文本闪烁计数阈值(以显示刷新时间基准为单位)

/* 显示页编号: ComputeVisualization() 把菜单索引映射为下列"显示页", 每个编号在
   Display_LCD() 的 switch 中对应一屏显示内容。 */
#define VISUALIZATION_1   (u8)1    // 页1: 速度控制页(目标/实测转速)
#define VISUALIZATION_2   (u8)2    // 页2: 速度环 PID 参数页
#define VISUALIZATION_3   (u8)3    // 页3: 转矩环(q 轴电流) PID 参数页
#define VISUALIZATION_4   (u8)4    // 页4: 磁链环(d 轴电流) PID 参数页
#define VISUALIZATION_5   (u8)5    // 页5: 功率级状态页(母线电压/温度)
#define VISUALIZATION_6   (u8)6    // 页6: 转矩控制页(Iq/Id 目标与实测、转速)
#define VISUALIZATION_7   (u8)7    // 页7: 故障显示页
#define VISUALIZATION_8   (u8)8    // 页8: 停机等待页("Motor is stopping")
#define VISUALIZATION_9   (u8)9    // 页9: 观测器/PLL 增益整定页
#define VISUALIZATION_10  (u8)10   // 页10: DAC 输出变量显示页
#ifdef FLUX_WEAKENING
#define VISUALIZATION_11  (u8)11   // 页11: 弱磁(Flux Weakening)电压环参数页
#endif

/* LCD 行内字符位置常量: CHAR_n 表示某一行"从左往右第 n 个字符"(从 0 开始),
   用于把这些数字对齐显示在固定的字符格上。 */
#define CHAR_0            (u8)0 //First character of the line starting from the left
#define CHAR_1            (u8)1 
#define CHAR_2            (u8)2
#define CHAR_3            (u8)3
#define CHAR_4            (u8)4
#define CHAR_5            (u8)5
#define CHAR_6            (u8)6
#define CHAR_7            (u8)7
#define CHAR_8            (u8)8
#define CHAR_9            (u8)9
#define CHAR_10           (u8)10
#define CHAR_11           (u8)11
#define CHAR_12           (u8)12
#define CHAR_13           (u8)13
#define CHAR_14           (u8)14
#define CHAR_15           (u8)15
#define CHAR_16           (u8)16
#define CHAR_17           (u8)17

#ifdef OBSERVER_GAIN_TUNING 
#define CHAR_18           (u8)18
#endif

#ifdef DAC_FUNCTIONALITY
#define CHAR_19           (u8)19
#endif

/* Private macro -------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void Display_5DigitSignedNumber(u8, u8, s16);   // 在指定行/起始字符显示 5 位带符号数(私有)
u8 ComputeVisualization(u8 );                   // 菜单索引 → 显示页编号 映射(私有)

/* Private variables ---------------------------------------------------------*/
volatile static u16 hTimebase_Blinking;         // 闪烁计数(以显示刷新基准为单位, volatile 供中断/主循环共享)
static u8 bPrevious_Visualization = 0;          // 上一次的显示页编号, 用于判断是否需要重画静态文本
static u8 bPresent_Visualization;               // 当前的显示页编号(由 ComputeVisualization 得到)

#ifdef FLUX_WEAKENING
/* 弱磁控制显示所需的外部变量。 */
extern s16 hFW_V_Ref;    // 弱磁目标电压参考(百分数)
extern s16 hFW_P_Gain;   // 弱磁电压环比例增益
extern s16 hFW_I_Gain;   // 弱磁电压环积分增益
extern s16 hVMagn;       // 电机电压矢量幅值(用于换算显示已施加电压百分比)
#endif

#ifdef OBSERVER_GAIN_TUNING
/* 观测器/PLL 增益显示所需的外部变量。 */
extern volatile s32 wK1_LO;         // 观测器增益 K1(低字)
extern volatile s32 wK2_LO;         // 观测器增益 K2(低字)
extern volatile s16 hPLL_P_Gain, hPLL_I_Gain;   // 锁相环 P/I 增益
#endif

/*******************************************************************************
* Function Name  : Display_Welcome_Message
* Description    : Welcome message on LCD after power-up
* Input          : None
* Output         : None
* Return         : None
* 功能说明(中文) : 上电后在 LCD 上显示欢迎信息, 包括产品名、固件版本与操作提示
*                  (<> 表示摇杆左右移动, ^| 表示摇杆上下修改)。初始化阶段调用一次。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 直接从常量字符串指针写屏, 不依赖 bMenu_index; 不刷新数值。
*******************************************************************************/
void Display_Welcome_Message(void)
{
  u8 *ptr = " STM32 Motor Control";   // 第0行: 产品名
  
  LCD_DisplayStringLine(Line0, ptr);  // 在 LCD 第0行显示该字符串
  
  ptr = "  PMSM FOC ver 2.0  ";       // 第1行: 固件版本
  LCD_DisplayStringLine(Line1, ptr);  // 在 LCD 第1行显示该字符串
    
  ptr = " <> Move  ^| Change ";       // 第9行: 操作提示(左右移动/上下修改)
  LCD_DisplayStringLine(Line9, ptr);            
}  

/*******************************************************************************
* Function Name  : Display_LCD
* Description    : Display routine for LCD management
* Input          : None
* Output         : None
* Return         : None
* 功能说明(中文) : LCD 主刷新函数。每约 200ms(显示时间基准到期)执行一次: 先由
*                  ComputeVisualization(bMenu_index) 得到当前"显示页编号", 再按该页
*                  重画静态文本并刷新动态数值, 用颜色(Red)高亮当前可调对象。
*                  菜单结构(由 bMenu_index 决定显示页):
*                    页1 速度控制页(菜单0/1)、页2 速度环PID(菜单2~4)、
*                    页3 转矩环PID(菜单5~7)、页4 磁链环PID(菜单8~10)、
*                    页5 功率级状态(菜单11)、页6 转矩控制(菜单12~14)、
*                    页7 故障(菜单15)、页9 观测器整定、页10 DAC、页11 弱磁(可选)。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 由主循环周期调用; 数值来源见各分支(转速/Iq/Id/参考量/母线电压/温度);
*                  State==WAIT 时强制显示"停机等待"页(页8)。
*******************************************************************************/
void Display_LCD(void)
{          
  if (TB_DisplayDelay_IsElapsed() == TRUE)   // 显示刷新时间基准到期?(约每 200ms 一次)
  { 
    TB_Set_DisplayDelay_500us(500);  //  refresh LCD every 400*5 = 200 ms     // 重新装载 500*500us = 250ms 刷新延时

    bPrevious_Visualization = bPresent_Visualization;   // 记录上一页, 用于判断是否需要重画静态文字

    bPresent_Visualization = ComputeVisualization(bMenu_index);   // 依据当前菜单索引得到本页显示页编号
  
    switch(bPresent_Visualization)   // 按显示页编号分派到各自的界面刷新逻辑
    {
      u8 *ptr;    // 指向待显示字符串的指针
      s16 temp;   // 待显示的数值(转速/Iq/Id/电压等, 已换算为带符号显示量)
            
      case(VISUALIZATION_1):   // 页1: 速度控制页(显示目标转速/实测转速, 单位 rpm)
        if (bPresent_Visualization != bPrevious_Visualization)   // 仅当切换页面时重画静态文本
        { 
#ifdef NO_SPEED_SENSORS          
          ptr = "   Sensorless Demo  ";
          LCD_DisplayStringLine(Line2,ptr);
#else          
          LCD_ClearLine(Line2);
#endif          
          LCD_ClearLine(Line3); 
          
          LCD_ClearLine(Line4); 
                 
          ptr = " Target     Measured";
          LCD_DisplayStringLine(Line5,ptr); 
          
          ptr = "       (rpm)        ";
          LCD_DisplayStringLine(Line7,ptr); 
          
          LCD_ClearLine(Line6);        
          
          LCD_ClearLine(Line8);
          
          ptr = " <> Move  ^| Change ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
        
        if(bMenu_index == CONTROL_MODE_MENU_1)
        {
          LCD_SetTextColor(Red);
        }        
        
        ptr = " Speed control mode";        
        LCD_DisplayStringLine(Line3,ptr);
        
        if(bMenu_index == CONTROL_MODE_MENU_1)
        {
          LCD_SetTextColor(Blue);
        }
        else //REF_SPEED_MENU
        {
          LCD_SetTextColor(Red);
        }
          
        //Compute target speed in rpm
        // 目标转速换算为 rpm: hSpeed_Reference 为 0.1Hz 定标, 乘 6 相当于 (Hz*60) 的换算(单对极演示)
        temp = (s16)(hSpeed_Reference * 6);                 
        Display_5DigitSignedNumber(Line7, CHAR_0, temp);     // 第7行第0字符起显示"目标转速(rpm)"
        
        if(bMenu_index != CONTROL_MODE_MENU_1)
        {
          LCD_SetTextColor(Blue);
        }
         
        //Compute measured speed in rpm
        // 实测转速换算为 rpm: 依速度反馈方式三选一(编码器/霍尔/无感), 均乘 6 换为 rpm
#ifdef ENCODER
        temp = (s16)(ENC_Get_Mechanical_Speed() * 6);   // 编码器实测机械转速(0.1Hz)→rpm
#endif        
#if defined HALL_SENSORS
        temp = (s16)(HALL_GetSpeed() * 6);              // 霍尔传感器实测转速(0.1Hz)→rpm
#endif
#if defined NO_SPEED_SENSORS        
        temp = (s16)(STO_Get_Speed_Hz() * 6);           // 无感观测器估算转速(0.1Hz)→rpm
#endif        
        Display_5DigitSignedNumber(Line7, CHAR_13, temp);   // 第7行第13字符起显示"实测转速(rpm)"
      
      break;
          
      case(VISUALIZATION_2):   // 页2: 速度环 PID 参数页(Kp/Ki/Kd, 高亮当前调节项)
        if (bPresent_Visualization != bPrevious_Visualization)
        {           
          ptr = "       Speed        ";
          LCD_DisplayStringLine(Line2,ptr);
          
          ptr = "    P     I     D   ";
          LCD_DisplayStringLine(Line3,ptr); 
          
          LCD_ClearLine(Line4);
          LCD_ClearLine(Line5);
           
          ptr = " Target        (rpm)";
          LCD_DisplayStringLine(Line6,ptr); 
          
          ptr = " Measured      (rpm)";
          LCD_DisplayStringLine(Line7,ptr);
                    
          LCD_ClearLine(Line8);
          
          ptr = " <> Move  ^| Change ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
        
        switch(bMenu_index)
        {
          case(P_SPEED_MENU):
            LCD_SetTextColor(Red);            
            temp = PID_Speed_InitStructure.hKp_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            LCD_SetTextColor(Blue);
            
            temp = PID_Speed_InitStructure.hKi_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            
#ifdef DIFFERENTIAL_TERM_ENABLED            
            temp = PID_Speed_InitStructure.hKd_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {
                LCD_DisplayChar(Line4, (u16)(320 -(16*(18-i))),'-');
              }
            }
#endif         
 
         break;
            
          case(I_SPEED_MENU):                                 
            temp = PID_Speed_InitStructure.hKp_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            
            LCD_SetTextColor(Red);   
            temp = PID_Speed_InitStructure.hKi_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            LCD_SetTextColor(Blue);
            
#ifdef DIFFERENTIAL_TERM_ENABLED            
            temp = PID_Speed_InitStructure.hKd_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {
                LCD_DisplayChar(Line4, (u16)(320 -(16*(18-i))),'-');
              }
            }
#endif
	      break;
          
#ifdef DIFFERENTIAL_TERM_ENABLED
            case(D_SPEED_MENU):
              temp = PID_Speed_InitStructure.hKp_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_1, temp);
              
              temp = PID_Speed_InitStructure.hKi_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_7, temp);
              
              LCD_SetTextColor(Red);
              temp = PID_Speed_InitStructure.hKd_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_13, temp);
              LCD_SetTextColor(Blue);
          
            break;
#endif
        default:
          break;
        }
        //Independently from the menu, this visualization must display current 
        //and measured speeds
        
        //Display target speed in rpm
        temp = (s16)(hSpeed_Reference * 6);          
        Display_5DigitSignedNumber(Line6, CHAR_9, temp);
        
        //Compute measured speed in rpm
#ifdef ENCODER
        temp = (s16)(ENC_Get_Mechanical_Speed() * 6);
#elif defined HALL_SENSORS
        temp = (s16)(HALL_GetSpeed() * 6);
#elif defined NO_SPEED_SENSORS        
        temp = (s16)(STO_Get_Speed_Hz() * 6);
#endif
        Display_5DigitSignedNumber(Line7, CHAR_9, temp);         
      break;
      
      case(VISUALIZATION_3):   // 页3: 转矩环(q 轴电流) PID 参数页 + 目标/实测 Iq
        if (bPresent_Visualization != bPrevious_Visualization)
        {           
          ptr = "       Torque       ";
          LCD_DisplayStringLine(Line2,ptr);
          
          ptr = "    P     I     D   ";
          LCD_DisplayStringLine(Line3,ptr); 
          
          LCD_ClearLine(Line4);
          LCD_ClearLine(Line5);
           
          ptr = " Target         (Iq)";
          LCD_DisplayStringLine(Line6,ptr); 
          
          ptr = " Measured       (Iq)";
          LCD_DisplayStringLine(Line7,ptr);
          
          LCD_ClearLine(Line8);
          
          ptr = " <> Move  ^| Change ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
        
        switch(bMenu_index)
        {
          case(P_TORQUE_MENU):
            LCD_SetTextColor(Red);            
            temp = PID_Torque_InitStructure.hKp_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            LCD_SetTextColor(Blue);
            
            temp = PID_Torque_InitStructure.hKi_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            
#ifdef DIFFERENTIAL_TERM_ENABLED            
            temp = PID_Torque_InitStructure.hKd_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {
                LCD_DisplayChar(Line4, (u16)(320 -(16*(18-i))),'-');
              }
            }
#endif       
         break;
            
          case(I_TORQUE_MENU):                                 
            temp = PID_Torque_InitStructure.hKp_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            
            LCD_SetTextColor(Red);   
            temp = PID_Torque_InitStructure.hKi_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            LCD_SetTextColor(Blue);
            
#ifdef DIFFERENTIAL_TERM_ENABLED             
            temp = PID_Torque_InitStructure.hKd_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {
                LCD_DisplayChar(Line4, (u16)(320 -(16*(18-i))),'-');
              }
            }
#endif
           break;
          
#ifdef DIFFERENTIAL_TERM_ENABLED 
            case(D_TORQUE_MENU):
              temp = PID_Torque_InitStructure.hKp_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_1, temp);
              
              temp = PID_Torque_InitStructure.hKi_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_7, temp);
              
              LCD_SetTextColor(Red);
              temp = PID_Torque_InitStructure.hKd_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_13, temp);
              LCD_SetTextColor(Blue);
              
            break;
#endif
        default:
          break;
        }
        //Independently from the menu, this visualization must display current 
        //and measured Iq
        
        temp = hTorque_Reference;          // 转矩(q 轴)电流参考给定(s16 Q15)
        Display_5DigitSignedNumber(Line6, CHAR_9, temp);   // 第6行显示"目标 Iq"

        temp = Stat_Curr_q_d.qI_Component1;                // 实测 q 轴电流分量 Iq(Q15)
        Display_5DigitSignedNumber(Line7, CHAR_9, temp);        // 第7行显示"实测 Iq"
      break;
     
       case(VISUALIZATION_4):   // 页4: 磁链环(d 轴电流) PID 参数页 + 目标/实测 Id
        if (bPresent_Visualization != bPrevious_Visualization)
        {           
          ptr = "        Flux        ";
          LCD_DisplayStringLine(Line2,ptr);
          
          ptr = "    P     I     D   ";
          LCD_DisplayStringLine(Line3,ptr); 
          
          LCD_ClearLine(Line4);
          LCD_ClearLine(Line5);
           
          ptr = " Target         (Id)";
          LCD_DisplayStringLine(Line6,ptr); 
          
          ptr = " Measured       (Id)";
          LCD_DisplayStringLine(Line7,ptr);
          
          LCD_ClearLine(Line8);
          
          ptr = " <> Move  ^| Change ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
        
        switch(bMenu_index)
        {
          case(P_FLUX_MENU):
            LCD_SetTextColor(Red);            
            temp = PID_Flux_InitStructure.hKp_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            LCD_SetTextColor(Blue);
            
            temp = PID_Flux_InitStructure.hKi_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            
#ifdef DIFFERENTIAL_TERM_ENABLED            
            temp = PID_Flux_InitStructure.hKd_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {
                LCD_DisplayChar(Line4, (u16)(320 -(16*(18-i))),'-');
              }
            }
#endif       
          break;
            
          case(I_FLUX_MENU):                                 
            temp = PID_Flux_InitStructure.hKp_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            
            LCD_SetTextColor(Red);   
            temp = PID_Flux_InitStructure.hKi_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            LCD_SetTextColor(Blue);
            
#ifdef DIFFERENTIAL_TERM_ENABLED             
            temp = PID_Flux_InitStructure.hKd_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_13, temp);
#else        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {
                LCD_DisplayChar(Line4, (u16)(320 -(16*(18-i))),'-');
              }
            }
#endif
           break;
            
#ifdef DIFFERENTIAL_TERM_ENABLED 
            case(D_FLUX_MENU):
              temp = PID_Flux_InitStructure.hKp_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_1, temp);
              
              temp = PID_Flux_InitStructure.hKi_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_7, temp);
              
              LCD_SetTextColor(Red);
              temp = PID_Flux_InitStructure.hKd_Gain;
              Display_5DigitSignedNumber(Line4, CHAR_13, temp);
              LCD_SetTextColor(Blue);
              
            break;
#endif
        default:
          break;
        }
        //Independently from the menu, this visualization must display current 
        //and measured Id
        
        temp = hFlux_Reference;          // 磁链(d 轴)电流参考给定(s16 Q15)
        Display_5DigitSignedNumber(Line6, CHAR_9, temp);   // 第6行显示"目标 Id"

        temp = Stat_Curr_q_d.qI_Component2;                // 实测 d 轴电流分量 Id(Q15)
        Display_5DigitSignedNumber(Line7, CHAR_9, temp);   // 第7行显示"实测 Id"
      break;

#ifdef FLUX_WEAKENING      
      case(VISUALIZATION_11):   // 页11: 弱磁(Flux Weakening)电压环参数页(目标/实测电压百分比)
        if (bPresent_Visualization != bPrevious_Visualization)
        {           
          ptr = "Flux Weakening Ctrl ";
          LCD_DisplayStringLine(Line2,ptr);
          
          ptr = "    P     I         ";
          LCD_DisplayStringLine(Line3,ptr); 
          
          LCD_ClearLine(Line4);
          LCD_ClearLine(Line5);
           
          ptr = " Target        (Vs%)";
          LCD_DisplayStringLine(Line6,ptr); 
          
          ptr = " Measured      (Vs%)";
          LCD_DisplayStringLine(Line7,ptr);
          
          LCD_ClearLine(Line8);
          
          ptr = " <> Move  ^| Change ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
        
        switch(bMenu_index)
        {
          case(P_VOLT_MENU):
            LCD_SetTextColor(Red);            
            temp = hFW_P_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            
            LCD_SetTextColor(Blue);            
            temp = hFW_I_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            temp = hFW_V_Ref;
            Display_5DigitSignedNumber(Line6, CHAR_9, temp);
            LCD_DrawRect(161,97,1,2);            
            
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {
                LCD_DisplayChar(Line4, (u16)(320 -(16*(18-i))),'-');
              }
            }
 
         break;
            
          case(I_VOLT_MENU):                                 
            temp = hFW_P_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            temp = hFW_V_Ref;
            Display_5DigitSignedNumber(Line6, CHAR_9, temp);            
            
            LCD_SetTextColor(Red);   
            temp = hFW_I_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            LCD_SetTextColor(Blue);
            LCD_DrawRect(161,97,1,2); 
        
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {
                LCD_DisplayChar(Line4, (u16)(320 -(16*(18-i))),'-');
              }
            }

	      break;
              
          case(TARGET_VOLT_MENU):
            LCD_SetTextColor(Red);            
            temp = hFW_V_Ref;
            Display_5DigitSignedNumber(Line6, CHAR_9, temp);
            LCD_DrawRect(161,97,1,2);
            
            LCD_SetTextColor(Blue);
            temp = hFW_P_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_1, temp);
            temp = hFW_I_Gain;
            Display_5DigitSignedNumber(Line4, CHAR_7, temp);
            
            {
              u32 i=0;
              for( i=0; i<5; i++)
              {
                LCD_DisplayChar(Line4, (u16)(320 -(16*(18-i))),'-');
              }
            }
 
         break;              
          
        default:
          break;
        }
        //Independently from the menu, this visualization must display current 
        //and measured voltage level
        
        //Compute applied voltage in s16
        // 已施加电压换算为百分比: 电压矢量幅值 hVMagn(Q15) 乘 1000 再除以调制圆半径 MAX_MODULE
        temp = (s16)(((s32)(hVMagn)*1000)/MAX_MODULE);        
        Display_5DigitSignedNumber(Line7, CHAR_9, temp);   // 第7行显示"实测电压百分比 Vs%"
        LCD_DrawRect(185,97,1,2);   // 在百分比数值旁画小标记
      break;
#endif      
      
      case(VISUALIZATION_5):   // 页5: 功率级状态页(母线电压 V / 功率级温度 摄氏度)
        if (bPresent_Visualization != bPrevious_Visualization)
        {           
          LCD_ClearLine(Line2);
          
          ptr = " Power Stage Status ";          
          LCD_DisplayStringLine(Line3, ptr); 
          
          LCD_ClearLine(Line4);
          
          ptr = "  DC bus =     Volt ";          
          LCD_DisplayStringLine(Line5, ptr); 
          
          LCD_ClearLine(Line6);
          
          ptr = "  T =      Celsius  ";          
          LCD_DisplayStringLine(Line7, ptr); 
          
          LCD_ClearLine(Line8);
          
          ptr = " <> Move            ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
      
        temp = MCL_Compute_BusVolt();        // 读取母线电压(单位: V, 3 位十进制)
        // 下面把电压的百/十/个位分别取模并 +0x30 转为 ASCII 数字, 写到第5行固定字符位
        LCD_DisplayChar(Line5, 320-16*CHAR_11, (u8)(((temp%1000)/100)+0x30));   // 百位
        LCD_DisplayChar(Line5, 320-16*CHAR_12, (u8)(((temp%100)/10)+0x30));     // 十位
        LCD_DisplayChar(Line5, 320-16*CHAR_13, (u8)((temp%10)+0x30));           // 个位
        
        temp = MCL_Compute_Temp();     // 读取功率级温度(单位: 摄氏度, 3 位十进制)
        // 温度百/十/个位取模后 +0x30 转 ASCII, 写到第7行固定字符位
        LCD_DisplayChar(Line7, 320-16*CHAR_6, (u8)(((temp%1000)/100)+0x30));
        LCD_DisplayChar(Line7, 320-16*CHAR_7, (u8)(((temp%100)/10)+0x30));
        LCD_DisplayChar(Line7, 320-16*CHAR_8, (u8)((temp%10)+0x30));
      
      break;    
        
      case(VISUALIZATION_6):   // 页6: 转矩控制页(Iq/Id 目标与实测、实测转速)
        if (bPresent_Visualization != bPrevious_Visualization)
        {           
#ifdef NO_SPEED_SENSORS          
          ptr = "   Sensorless Demo  ";
          LCD_DisplayStringLine(Line2,ptr);
#else          
          LCD_ClearLine(Line2);
#endif           
          LCD_ClearLine(Line3); 
          
          ptr = "     Target Measured";
          LCD_DisplayStringLine(Line4,ptr);
          
          ptr = "Iq                  ";
          LCD_DisplayStringLine(Line5,ptr); 
          
          ptr = "Id                  ";
          LCD_DisplayStringLine(Line6,ptr);
          
          ptr = "Speed (rpm)         ";
          LCD_DisplayStringLine(Line7,ptr);
          
          LCD_ClearLine(Line8);
          
          ptr = " <> Move  ^| Change ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
        
        switch(bMenu_index)
        {
          case(CONTROL_MODE_MENU_6):
            LCD_SetTextColor(Red);
            ptr = "Torque control mode ";        
            LCD_DisplayStringLine(Line3,ptr);  
            LCD_SetTextColor(Blue);
            
            temp = hTorque_Reference;
            Display_5DigitSignedNumber(Line5, CHAR_5, temp);
 
            temp = hFlux_Reference; 
            Display_5DigitSignedNumber(Line6, CHAR_5, temp);         
          break;   
          
          case(IQ_REF_MENU):
            ptr = "Torque control mode ";
            LCD_DisplayStringLine(Line3,ptr); 
            
            LCD_SetTextColor(Red);
            temp = hTorque_Reference;
            Display_5DigitSignedNumber(Line5, CHAR_5, temp);
            LCD_SetTextColor(Blue);
            
            temp = hFlux_Reference; 
            Display_5DigitSignedNumber(Line6, CHAR_5, temp); 
          break;
            
          case(ID_REF_MENU):
            ptr = "Torque control mode ";
            LCD_DisplayStringLine(Line3,ptr); 
            
            temp = hTorque_Reference;
            Display_5DigitSignedNumber(Line5, CHAR_5, temp);
            
            LCD_SetTextColor(Red);
            temp = hFlux_Reference; 
            Display_5DigitSignedNumber(Line6, CHAR_5, temp); 
            LCD_SetTextColor(Blue);
          break;
         default: 
          break;
        }            
        temp =Stat_Curr_q_d.qI_Component1;                 // 实测 q 轴电流 Iq(Q15)
        Display_5DigitSignedNumber(Line5, CHAR_13, temp);  // 第5行"实测"列显示 Iq
        
        temp =Stat_Curr_q_d.qI_Component2;                 // 实测 d 轴电流 Id(Q15)
        Display_5DigitSignedNumber(Line6, CHAR_13, temp);  // 第6行"实测"列显示 Id

        //Compute measured speed in rpm
        // 实测转速换算为 rpm(编码器/霍尔/无感三选一, 乘 6 由 0.1Hz 换为 rpm)
#ifdef ENCODER
        temp = (s16)(ENC_Get_Mechanical_Speed() * 6);   // 编码器机械转速(0.1Hz)→rpm
#elif defined HALL_SENSORS
        temp = (s16)(HALL_GetSpeed() * 6);              // 霍尔转速(0.1Hz)→rpm
#elif defined NO_SPEED_SENSORS        
        temp = (s16)(STO_Get_Speed_Hz() * 6);           // 无感估算转速(0.1Hz)→rpm
#endif 
        Display_5DigitSignedNumber(Line7, CHAR_13, temp);   // 第7行"实测"列显示转速(rpm)
      break;
        
      case(VISUALIZATION_7):   // 页7: 故障显示页(依据 wGlobal_Flags 显示故障类型与相关数值)
        if (bPresent_Visualization != bPrevious_Visualization)
        {  
          LCD_ClearLine(Line2);
          
          LCD_SetTextColor(Red);
          ptr = "    !!! FAULT !!!   ";
          LCD_DisplayStringLine(Line3,ptr);
          LCD_SetTextColor(Blue);
         
        // 依据全局故障标志 wGlobal_Flags 的位掩码, 判定并显示具体故障类型(优先级从上到下)
          if ( (wGlobal_Flags & UNDER_VOLTAGE) == UNDER_VOLTAGE)   // 母线欠压?
          {           
            ptr = " Bus Under Voltage  ";
            LCD_DisplayStringLine(Line4, ptr);                                   
          }
          else if ( (wGlobal_Flags & OVER_CURRENT) ==  OVER_CURRENT)   // 过流?
            {
              ptr = "   Over Current    ";
              LCD_DisplayStringLine(Line4, ptr); 
            }
          else if ( (wGlobal_Flags & OVERHEAT) ==  OVERHEAT)   // 过温?
            {
              ptr = "   Over Heating    ";
              LCD_DisplayStringLine(Line4, ptr);                             
            }
          else if ( (wGlobal_Flags & OVER_VOLTAGE) ==  OVER_VOLTAGE)   // 母线过压?
            {
              ptr = "  Bus Over Voltage  ";
              LCD_DisplayStringLine(Line4, ptr);               
            }
          else if ( (wGlobal_Flags & START_UP_FAILURE) ==  START_UP_FAILURE)   // 启动失败?
          {
             ptr = "  Start-up failed   ";
             LCD_DisplayStringLine(Line4, ptr);    
          }      
          else if ( (wGlobal_Flags & SPEED_FEEDBACK) ==  SPEED_FEEDBACK)   // 速度反馈出错?
          {
             ptr = "Error on speed fdbck";
             LCD_DisplayStringLine(Line4, ptr);     
          }  
          LCD_ClearLine(Line5);
          LCD_ClearLine(Line7);  
        } 

        if ((wGlobal_Flags & ( OVERHEAT | UNDER_VOLTAGE | OVER_VOLTAGE)) == 0)   // 非温/压类故障 → 提示按键返回菜单
        { 
          LCD_ClearLine(Line6);
          ptr = "   Press 'Key' to   ";
          LCD_DisplayStringLine(Line8,ptr);
          
          ptr = "   return to menu   ";
          LCD_DisplayStringLine(Line9,ptr);
        }
        else
        {
          if ((wGlobal_Flags & (UNDER_VOLTAGE | OVER_VOLTAGE)) ==0)    
          {//Under or over voltage
             if (bPresent_Visualization != bPrevious_Visualization)
             { 
               LCD_ClearLine(Line6);
             }
             temp = MCL_Compute_Temp(); 
             ptr = "       T =";  
             LCD_DisplayStringLine(Line6, ptr);
             LCD_DisplayChar(Line6, 320-16*CHAR_11, (u8)(((temp%1000)/100)+0x30));
             LCD_DisplayChar(Line6, 320-16*CHAR_12, (u8)(((temp%100)/10)+0x30));
             LCD_DisplayChar(Line6, 320-16*CHAR_13, (u8)((temp%10)+0x30));
             LCD_DisplayChar(Line6, 320-16*CHAR_14, ' ');
             LCD_DisplayChar(Line6, 320-16*CHAR_15, 'C');
          }
          else 
          {           
            if (bPresent_Visualization != bPrevious_Visualization)
            { 
              LCD_ClearLine(Line6);         
            }
            ptr = "  DC bus =";            
            LCD_DisplayStringLine(Line6, ptr); 
            temp = MCL_Compute_BusVolt();        
            LCD_DisplayChar(Line6, 320-16*CHAR_11, (u8)(((temp%1000)/100)+0x30));
            LCD_DisplayChar(Line6, 320-16*CHAR_12, (u8)(((temp%100)/10)+0x30));
            LCD_DisplayChar(Line6, 320-16*CHAR_13, (u8)((temp%10)+0x30)); 
            LCD_DisplayChar(Line6, 320-16*CHAR_14, ' ');
            LCD_DisplayChar(Line6, 320-16*CHAR_15, 'V');             
          }          
          LCD_ClearLine(Line8);
          LCD_ClearLine(Line9);
        }
      break;
     
      case(VISUALIZATION_8):     // 页8: 停机等待页(电机停转中提示, State==WAIT 时强制显示)
        if (bPresent_Visualization != bPrevious_Visualization)
        {  
          LCD_ClearLine(Line2);
          
          ptr = " Motor is stopping  ";
          LCD_DisplayStringLine(Line3,ptr);
          
          ptr = "   please wait...   ";
          LCD_DisplayStringLine(Line4,ptr);
          
          LCD_ClearLine(Line5);
          LCD_ClearLine(Line6);
          LCD_ClearLine(Line7);
          LCD_ClearLine(Line8);
          LCD_ClearLine(Line9);
        } 
      break;

#ifdef OBSERVER_GAIN_TUNING      
      case(VISUALIZATION_9):   // 页9: 观测器/PLL 增益整定页(显示 K1/K2 与 PLL 的 P/I 增益)
        if (bPresent_Visualization != bPrevious_Visualization)
        {           
          ptr = "   Observer Gains   ";
          LCD_DisplayStringLine(Line2,ptr);
          
          ptr = "     K1       K2    ";
          LCD_DisplayStringLine(Line3,ptr); 
          
          LCD_ClearLine(Line4);
           
          ptr = "      PLL Gains     ";
          LCD_DisplayStringLine(Line5,ptr); 
          
          ptr = "     P        I     ";
          LCD_DisplayStringLine(Line6,ptr);
          
          LCD_ClearLine(Line7);
          
          LCD_ClearLine(Line8);
          
          ptr = " <> Move  ^| Change ";          
          LCD_DisplayStringLine(Line9, ptr); 
        }
        
        switch(bMenu_index)
        {
          case(K1_MENU):
            LCD_SetTextColor(Red);            
            temp = wK1_LO/10;
            Display_5DigitSignedNumber(Line4, CHAR_3, temp);
            
            LCD_SetTextColor(Blue);
            temp = wK2_LO/100;
            Display_5DigitSignedNumber(Line4, CHAR_12, temp);
                      
            temp = hPLL_P_Gain;
            Display_5DigitSignedNumber(Line7, CHAR_3, temp);  
            
            temp = hPLL_I_Gain;
            Display_5DigitSignedNumber(Line7, CHAR_12, temp);
          break;
                     
           case(K2_MENU):              
            temp = wK1_LO/10;
            Display_5DigitSignedNumber(Line4, CHAR_3, temp);
            
            LCD_SetTextColor(Red);  
            temp = wK2_LO/100;
            Display_5DigitSignedNumber(Line4, CHAR_12, temp);
            
            LCD_SetTextColor(Blue);           
            temp = hPLL_P_Gain;
            Display_5DigitSignedNumber(Line7, CHAR_3, temp);  
            
            temp = hPLL_I_Gain;
            Display_5DigitSignedNumber(Line7, CHAR_12, temp);
          break;
            
          case(P_PLL_MENU):
            temp = wK1_LO/10;
            Display_5DigitSignedNumber(Line4, CHAR_3, temp);
             
            temp = wK2_LO/100;
            Display_5DigitSignedNumber(Line4, CHAR_12, temp);
            
            LCD_SetTextColor(Red);           
            temp = hPLL_P_Gain;
            Display_5DigitSignedNumber(Line7, CHAR_3, temp);  
            
            LCD_SetTextColor(Blue); 
            temp = hPLL_I_Gain;
            Display_5DigitSignedNumber(Line7, CHAR_12, temp);
          break;
            
          case(I_PLL_MENU):
            temp = wK1_LO/10;
            Display_5DigitSignedNumber(Line4, CHAR_3, temp);
             
            temp = wK2_LO/100;
            Display_5DigitSignedNumber(Line4, CHAR_12, temp);
                       
            temp = hPLL_P_Gain;
            Display_5DigitSignedNumber(Line7, CHAR_3, temp);  
            
            LCD_SetTextColor(Red);
            temp = hPLL_I_Gain;
            Display_5DigitSignedNumber(Line7, CHAR_12, temp);                  
            LCD_SetTextColor(Blue); 
          break;  
        default:
           break;
        }
        break;
#endif
      
#ifdef DAC_FUNCTIONALITY      
    case(VISUALIZATION_10):   // 页10: DAC 输出变量显示页(显示 PB0/PB1 当前 DAC 输出的是哪个变量)
      if (bPresent_Visualization != bPrevious_Visualization)
      {           
        LCD_ClearLine(Line2);
        
        ptr = "    Signal on PB0   ";
        LCD_DisplayStringLine(Line3,ptr); 
        
        LCD_ClearLine(Line4);
         
        LCD_ClearLine(Line5);
        
        ptr = "    Signal on PB1   ";
        LCD_DisplayStringLine(Line6,ptr);
        
        LCD_ClearLine(Line7);
        
        LCD_ClearLine(Line8);
        
        ptr = " <> Move  ^| Change ";          
        LCD_DisplayStringLine(Line9, ptr); 
      }
      
      switch(bMenu_index)
      {
        case(DAC_PB0_MENU):
          LCD_SetTextColor(Red);
          ptr = MCDAC_Output_Var_Name(DAC_CH1);
          LCD_DisplayStringLine(Line4, ptr);
          
          LCD_SetTextColor(Blue);
          ptr = MCDAC_Output_Var_Name(DAC_CH2);
          LCD_DisplayStringLine(Line7, ptr);
        break;
                   
         case(DAC_PB1_MENU):              
          ptr = MCDAC_Output_Var_Name(DAC_CH1);
          LCD_DisplayStringLine(Line4, ptr);
          
          LCD_SetTextColor(Red);
          ptr = MCDAC_Output_Var_Name(DAC_CH2);
          LCD_DisplayStringLine(Line7, ptr);
          LCD_SetTextColor(Blue);
        break;
        
        default:
         break;
      }
      break;      
#endif
    default:
      break;      
    }
  }
}
          
/*******************************************************************************
* Function Name  : Display_5DigitSignedNumber
* Description    : It Displays a 5 digit signed number in the specified line, 
*                  starting from a specified element of LCD display matrix 
* Input          : Line, starting point in LCD dysplay matrix, 5 digit signed
*                  number 
* Output         : None
* Return         : None
* 功能说明(中文) : 在指定行(Line)、从指定字符位置(bFirstchar)起, 显示一个 5 位带符号十进制数。
*                  先写符号位('-' 或空格), 再逐位写数字(低位在前), 最后写万位。
*                  被 Display_LCD 用于显示转速/电流/电压/温度等各类数值。
* 参数(中文)     : Line       - LCD 行号(Line0~Line9);
*                  bFirstchar - 起始字符位置(0 起, 从左往右);
*                  number     - 待显示的带符号 16 位整数(已换算好的显示量)。
* 返回(中文)     : 无。
* 备注(中文)     : 只显示 5 位, 超出范围的高位会截断; 列坐标用 320-16*n 换算为像素 x。
*******************************************************************************/

void Display_5DigitSignedNumber(u8 Line, u8 bFirstchar, s16 number)
{ u32 i;        // 逐位显示的循环计数
  u16 h_aux=1;  // 位权系数: 依次为 1,10,100,1000

  if (number<0)     // 负数: 先显示符号 '-', 并取绝对值
  {
    LCD_DisplayChar(Line,(u16)( 320-16*bFirstchar), '-');
    number = -number;
  }
  else 
  {
    LCD_DisplayChar(Line,(u16)( 320-16*bFirstchar), ' ');   // 非负: 符号位写空格对齐
  }
      
  for (i=0; i<4; i++)   // 依次输出个/十/百/千位(从右往左的 4 位)
  {
    LCD_DisplayChar(Line, (u16)(320 -(16*(bFirstchar+5-i))),
                                        (u8)(((number%(10*h_aux))/h_aux)+0x30));   // 取第 i 位并 +0x30 转 ASCII          
    h_aux *= 10;    // 位权 ×10
  }
  LCD_DisplayChar(Line,(u16)(320-(16*(bFirstchar+1))), (u8)(((number/10000))+0x30));   // 最高位(万位)
}    

/*******************************************************************************
* Function Name  : ComputeVisualization
* Description    : Starting from the value of the bMenuIndex, this function 
*                  extract the information about the present menu to be 
*                  displayed on LCD
* Input          : bMenuIndex variable 
* Output         : Present visualization
* Return         : None
* 功能说明(中文) : 把"菜单索引 bMenu_index"映射为"显示页编号 VISUALIZATION_x"。多个菜单
*                  索引可映射到同一显示页(如 P/I/D 三个 PID 菜单都显示同一 PID 页, 靠颜色
*                  高亮区分当前调节项)。若状态机处于 WAIT, 则强制返回停机等待页(页8)。
* 参数(中文)     : bLocal_MenuIndex - 当前菜单索引(见 MC_Display.h 的 xxx_MENU 宏)。
* 返回(中文)     : u8 - 显示页编号(VISUALIZATION_1 ~ VISUALIZATION_11)。
* 备注(中文)     : 纯映射函数, 不访问硬件; 未识别的菜单索引默认返回页1。
*******************************************************************************/

u8 ComputeVisualization(u8 bLocal_MenuIndex)
{  
  u8 bTemp;   // 计算得到的显示页编号

    switch(bLocal_MenuIndex)   // 菜单索引 → 显示页 的映射
    {
      case(CONTROL_MODE_MENU_1):
        bTemp = VISUALIZATION_1;
      break;
      case(REF_SPEED_MENU):
        bTemp = VISUALIZATION_1;
      break;
      
      case(P_SPEED_MENU):
        bTemp = VISUALIZATION_2; 
      break;
      case(I_SPEED_MENU):
        bTemp = VISUALIZATION_2; 
      break;
#ifdef DIFFERENTIAL_TERM_ENABLED
      case(D_SPEED_MENU):
       bTemp = VISUALIZATION_2; 
      break;
#endif        

      case(P_TORQUE_MENU):
        bTemp = VISUALIZATION_3; 
      break; 
      case(I_TORQUE_MENU):
        bTemp = VISUALIZATION_3; 
      break; 
#ifdef DIFFERENTIAL_TERM_ENABLED
      case(D_TORQUE_MENU):
        bTemp = VISUALIZATION_3; 
      break; 
#endif        
         
      case(P_FLUX_MENU):
         bTemp = VISUALIZATION_4; 
      break; 
      case(I_FLUX_MENU):
         bTemp = VISUALIZATION_4; 
      break; 
#ifdef DIFFERENTIAL_TERM_ENABLED
      case(D_FLUX_MENU):
         bTemp = VISUALIZATION_4; 
      break; 
#endif
      
#ifdef FLUX_WEAKENING
      case(P_VOLT_MENU):
        bTemp = VISUALIZATION_11; 
      break;
      case(I_VOLT_MENU):
        bTemp = VISUALIZATION_11;
      case(TARGET_VOLT_MENU):
        bTemp = VISUALIZATION_11;        
      break;
#endif      
             
      case(POWER_STAGE_MENU):
        bTemp = VISUALIZATION_5;
      break;
        
      case(CONTROL_MODE_MENU_6):
        bTemp = VISUALIZATION_6;
      break;
      case(IQ_REF_MENU):
        bTemp = VISUALIZATION_6;
      break;
      case(ID_REF_MENU):
        bTemp = VISUALIZATION_6;
      break;  
      
      case(FAULT_MENU):
        bTemp = VISUALIZATION_7;
      break;      
     
#ifdef OBSERVER_GAIN_TUNING  
      case(K1_MENU):
         bTemp = VISUALIZATION_9;
       break;   
       case(K2_MENU):
         bTemp = VISUALIZATION_9;
       break;
       case(P_PLL_MENU):
         bTemp = VISUALIZATION_9;
       break;
       case(I_PLL_MENU):
        bTemp = VISUALIZATION_9;
       break;
#endif

#ifdef DAC_FUNCTIONALITY
      case(DAC_PB0_MENU):
         bTemp = VISUALIZATION_10;
       break;   
       case(DAC_PB1_MENU):
         bTemp = VISUALIZATION_10;
       break;
#endif             
      default:
        bTemp = VISUALIZATION_1;
      break;      
    }    
      
    if (State == WAIT)     // 若状态机处于 WAIT(电机停转等待) → 强制显示停机等待页(页8)
    {
      bTemp = VISUALIZATION_8;
    }  
    
    return (bTemp);        // 返回最终显示页编号
}
      
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
