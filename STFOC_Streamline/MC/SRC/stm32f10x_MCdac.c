/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_MCdac.c
* Author             : IMS Systems Lab 
* Date First Issued  : 07/06/07
* Description        : This module manages all the necessary function to 
*                      implement DAC functionality
********************************************************************************
* History:
* 28/11/07 v1.0
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

/* ============================================================================
 * 【中文文件说明】stm32f10x_MCdac.c —— FOC 调试用"模拟量输出(DAC)"模块实现
 * 本模块把电机控制中关心的内部变量(相电流、Iq/Id、电压、角度、转速、观测器量等)
 * 输出为两路模拟电压，便于用示波器实时观测波形。
 * 实现方式：并未使用 STM32 内建 DAC 外设，而是用 TIM3 的 CH3/CH4 产生 PWM(PB0/PB1)，
 *           经外部 RC 低通滤波后还原为 0~3.3V 模拟电压；因此会占用 TIM3 定时器资源。
 * 工作流程：
 *   1) MCDAC_Init()           : 配置 PB0/PB1 与 TIM3(PWM 输出)，上电调用一次；
 *   2) MCDAC_Update_Value()   : 各控制任务把变量写进观测缓存 hMeasurementArray[]；
 *   3) MCDAC_Output_Choice()  : 用旋钮/按键切换两个通道各自观测的变量；
 *   4) MCDAC_Update_Output()  : 周期调用，把选中变量定标后写入 CH3/CH4 比较值；
 *   5) MCDAC_Output_Var_Name(): 返回变量名称，供 LCD 界面显示。
 * 注意：本文件当前未加入编译目标，仅供静态阅读参考。
 * ==========================================================================*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "stm32f10x_MCdac.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* 【中文】观测变量缓存数组(索引 0 未使用，1..22 依次对应 I_A/I_B/.../USER_2)。
   元素均为 s16 定标值(单位随变量而定，如电流为 Q15 标幺、角度为标幺值等)，
   由 MCDAC_Update_Value() 写入、MCDAC_Update_Output() 读出送到模拟通道。 */
s16 hMeasurementArray[23];

/* 【中文】观测变量显示名称表：与 hMeasurementArray 索引一一对应
   (每个名称定长 20 字符)，供 MCDAC_Output_Var_Name() 返回给界面显示。 */
u8 *OutputVariableNames[23] ={
  "0                   ","Ia                  ","Ib                  ",
  "Ialpha              ","Ibeta               ","Iq                  ",
  "Id                  ","Iq ref              ","Id ref              ",
  "Vq                  ","Vd                  ","Valpha              ",
  "Vbeta               ","Measured El Angle   ","Measured Rotor Speed",
  "Observed El Angle   ","Observed Rotor Speed","Observed Ialpha     ",
  "Observed Ibeta      ","Observed B-emf alpha","Observed B-emf beta ",
  "User 1              ","User 2              "};

/* 【中文】OutputVar[]：可选观测变量编号表(把"序号"映射到 hMeasurementArray 的索引)；
   max_out_var_num：可选变量个数(即序号上限)。下面按编译宏组合取不同子集：
   带编码器/霍尔且处于无传感器或观测器整定模式时，可选全部 23 项；
   仅无传感器时跳过第 13/14 项(实测角度/转速)；
   仅编码器/霍尔时跳过观测器相关项。 */
#if (defined ENCODER || defined VIEW_ENCODER_FEEDBACK || defined HALL_SENSORS\
  || defined VIEW_HALL_FEEDBACK) && (defined NO_SPEED_SENSORS ||\
  defined OBSERVER_GAIN_TUNING)

u8 OutputVar[23]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22};
u8 max_out_var_num = 22;

#elif (defined NO_SPEED_SENSORS)
u8 OutputVar[21]={0,1,2,3,4,5,6,7,8,9,10,11,12,15,16,17,18,19,20,21,22};
u8 max_out_var_num = 20;

#elif (defined ENCODER || defined HALL_SENSORS)
u8 OutputVar[17]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,21,22};
u8 max_out_var_num = 16;
#endif

u8 bChannel_1_variable=1;   // 【中文】模拟通道 1 当前观测的变量序号(默认 1 = Ia)
u8 bChannel_2_variable=1;   // 【中文】模拟通道 2 当前观测的变量序号(默认 1 = Ia)

/*******************************************************************************
* Function Name : MCDAC_Configuration
* Description : provides a short description of the function
* Input : details the input parameters.
* Output : details the output parameters.
* Return : details the return value.
* 功能说明(中文) : 初始化调试用"模拟量输出"通道。把 PB0/PB1 配为复用推挽输出，配置 TIM3 为
*                  向上计数、ARR=0x800(2048)、CH3/CH4 为 PWM1 模式，并使能计数器。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 本模块不使用 STM32 内建 DAC 外设，而是用 TIM3 的 PWM(PB0/PB1)经外部
*                  RC 低通滤波还原为模拟电压；会占用 TIM3 资源，注意与其它模块冲突。
*******************************************************************************/
void MCDAC_Init (void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  GPIO_InitTypeDef GPIO_InitStructure; 
  TIM_OCInitTypeDef TIM_OCInitStructure;
  
  /* Enable GPIOB */
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_StructInit(&GPIO_InitStructure);
 
  /* Configure PB.00 as alternate function output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;   // 【中文】PB0/PB1 即 TIM3_CH3/CH4 复用输出
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Enable TIM3 clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  TIM_DeInit(TIM3);

  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

  TIM_OCStructInit(&TIM_OCInitStructure);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0x800;          // 【中文】ARR=0x800(2048)，PWM 分辨率 11 位
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;         // 【中文】预分频 0，计数时钟 = 72MHz
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* Output Compare PWM Mode configuration: Channel3 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;                
  TIM_OCInitStructure.TIM_Pulse = 0x400; //Dummy value;    // 【中文】初始占空比占位值(约 50%)
  TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  
  /* Output Compare PWM Mode configuration: Channel4 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                   
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0x400; //Dummy value;    
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);   // 【中文】关闭 CCR 预装载，写比较值立即可用
  
  /* Enable TIM3 counter */
  TIM_Cmd(TIM3, ENABLE);
}

/*******************************************************************************
* Function Name : MCDAC_Output
* Description : provides a short description of the function
* Input : details the input parameters.
* Output : details the output parameters.
* Return : details the return value.
* 功能说明(中文) : 把两个通道当前选中的观测变量值，经定标后写入 TIM3 的 CH3/CH4 比较寄存器，
*                  更新 PWM 占空比，从而在 PB0/PB1 输出正比于变量值的模拟电压。
* 参数(中文)     : 无(通道所观测的变量由全局 bChannel_1_variable/bChannel_2_variable 选择)。
* 返回(中文)     : 无。
* 备注(中文)     : 需周期性调用(如任务或 PWM 中断)；实际输出的是 PWM 比较值而非 DAC 码。
*******************************************************************************/
void MCDAC_Update_Output(void)
{
    /* 【中文】定标公式说明：s16 取值范围 -32768..32767，先加 32768 做偏置变为无符号，
       再除以 32 得到 0..2047 的比较值，正好对应 TIM3 周期 0x800(2048)；
       于是有符号测量值被线性映射为 0~100% 的 PWM 占空比(再经 RC 滤波成模拟电压)。 */
    TIM_SetCompare3(TIM3, ((u16)((s16)((hMeasurementArray[OutputVar[bChannel_1_variable]]+32768)/32))));   // 【中文】通道1：变量值 -> TIM3_CH3 比较值
    TIM_SetCompare4(TIM3, ((u16)((s16)((hMeasurementArray[OutputVar[bChannel_2_variable]]+32768)/32))));   // 【中文】通道2：变量值 -> TIM3_CH4 比较值
}

/*******************************************************************************
* Function Name : MCDAC_Send_Output_Value
* Description : provides a short description of the function
* Input : details the input parameters.
* Output : details the output parameters.
* Return : details the return value.
* 功能说明(中文) : 把一个 16 位有符号测量值写入观测缓存 hMeasurementArray 的指定槽位，
*                  供 MCDAC_Update_Output() 取出并输出到模拟通道观测。
* 参数(中文)     : bVariable - 变量槽位编号(即 .h 中的 I_A..USER_2，取值 1..22)；
*                  hValue    - 变量值(s16，定标随变量而定，如电流 Q15 标幺、角度标幺等)。
* 返回(中文)     : 无。
* 备注(中文)     : 本函数为各控制任务写入观测量的统一入口，仅做缓存不做转换。
*******************************************************************************/
void MCDAC_Update_Value(u8 bVariable, s16 hValue)
{
  hMeasurementArray[bVariable] = hValue;   // 【中文】按槽位保存最新变量值，供输出函数读取
}

/*******************************************************************************
* Function Name : MCDAC_Output_Choice
* Description : provides a short description of the function
* Input : details the input parameters.
* Output : details the output parameters.
* Return : details the return value.
* 功能说明(中文) : 切换某个模拟输出通道所观测的变量序号(通常由旋钮/按键调用，实现变量滚动选择)。
* 参数(中文)     : bStep    - 步进量(s8，可为负：+1 下一个，-1 上一个)；
*                  bChannel - 通道号：DAC_CH1(1) 或 DAC_CH2(2)。
* 返回(中文)     : 无。
* 备注(中文)     : 序号在 1..max_out_var_num 之间循环(越界自动回绕)；实际变量经 OutputVar[] 映射。
*******************************************************************************/
void MCDAC_Output_Choice(s8 bStep, u8 bChannel)
{
  if (bChannel == DAC_CH1)
  {
     bChannel_1_variable += bStep;   // 【中文】通道1 变量序号步进(可为负，实现前后滚动)
     if (bChannel_1_variable > max_out_var_num)
     {
       bChannel_1_variable = 1;
     }
     else if (bChannel_1_variable == 0)
     {
       bChannel_1_variable = max_out_var_num;
     }
  }
  else
  {
     bChannel_2_variable += bStep;   // 【中文】通道2 变量序号步进
     if (bChannel_2_variable > max_out_var_num)
     {
       bChannel_2_variable = 1;
     }
     else if (bChannel_2_variable == 0)
     {
       bChannel_2_variable = max_out_var_num;
     }
  }
}

/*******************************************************************************
* Function Name : MCDAC_Output_Var_Name
* Description : provides a short description of the function
* Input : details the input parameters.
* Output : details the output parameters.
* Return : details the return value.
* 功能说明(中文) : 返回某通道当前观测变量的显示名称(定长 20 字符字符串)，供 LCD 界面显示。
* 参数(中文)     : bChannel - 通道号：DAC_CH1(1) 或 DAC_CH2(2)。
* 返回(中文)     : 指向该变量名称字符串的指针(u8 *)。
* 备注(中文)     : 名称取自 OutputVariableNames[]，并经 OutputVar[] 映射到实际测量索引。
*******************************************************************************/
u8 *MCDAC_Output_Var_Name(u8 bChannel)
{
  u8 *temp;
  if (bChannel == DAC_CH1)
  {
    temp = OutputVariableNames[OutputVar[bChannel_1_variable]];   // 【中文】先经 OutputVar 映射，再取名称字符串
  }
  else
  {
    temp = OutputVariableNames[OutputVar[bChannel_2_variable]];   // 【中文】先经 OutputVar 映射，再取名称字符串
  }
  return (temp);
}
    
  

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
