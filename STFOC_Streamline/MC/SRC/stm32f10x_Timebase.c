/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_Timebase.c
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This module handles time base. It used in display and 
*                      fault management, speed regulation, motor ramp-up  
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
*   本文件实现整机统一的 500us 软时基模块。核心是 SysTick 中断服务程序
*   (SysTick_Handler)：内核时钟 72MHz、重装载值 36000，故每 500us(0.5ms)
*   进入一次中断。中断里递减各软定时器计数，并完成两项周期性任务：
*     (1) 速度反馈周期到达时调用 ENC_Calc_Average_Speed() 计算平均转速；
*     (2) 速度环采样周期到达时(且电机处于 RUN 状态)调用 FOC_CalcFluxTorqueRef()
*         执行速度环/PID，输出转矩电流参考。
*   本模块为延时、超时、消抖、显示节流等提供统一计时基准。
*   术语：时基(Timebase)——为系统提供等间隔时间节拍的定时机制。
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"

/* Include of other module interface headers ---------------------------------*/
/* Local includes ------------------------------------------------------------*/
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "stm32f10x_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* 5ms 时基预分频值(定时器预分频寄存器值，实际分频为 31+1=32)。仅供 TIM 时基使用。*/
#define TB_Prescaler_5ms    31    // ((31+1)*(9374+1)/60000000) sec -> 5 ms 
/* 5ms 时基自动重装载值，配合 60MHz 时钟得到 5ms 周期。*/
#define TB_AutoReload_5ms    9374

/* 500us 时基预分频值(实际分频为 29+1=30)。*/
#define TB_Prescaler_500us    29    // ((29+1)*(999+1)/60000000) sec -> 500 us 
/* 500us 时基自动重装载值，配合 60MHz 时钟得到 500us 周期。*/
#define TB_AutoReload_500us    999

/* SysTick 中断抢占优先级(数值越小优先级越高)。*/
#define SYSTICK_PRE_EMPTION_PRIORITY 3
/* SysTick 中断子优先级。*/
#define SYSTICK_SUB_PRIORITY 0

/* 速度环采样周期（单位：500us 计数值），取自电机/控制配置 PID_SPEED_SAMPLING_TIME。*/
#define SPEED_SAMPLING_TIME   PID_SPEED_SAMPLING_TIME

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 无传感器/开环启动剩余超时计数（单位：500us），减到 0 表示启动超时。*/
static u16 hStart_Up_TimeLeft_500us =0;
/* 主状态机延时计数（单位：500us），由 SysTick 递减，volatile 因中断访问。*/
static volatile u16 hTimebase_500us = 0;
/* 显示模块延时计数（单位：500us），volatile 因中断访问。*/
static volatile u16 hTimebase_display_500us = 0;
/* 按键消抖延时计数（单位：500us），volatile 因中断访问。*/
static volatile u16 hKey_debounce_500us = 0;
/* 速度环采样倒计时（单位：500us），到 0 时执行一次速度环；全局可被外部修改。*/
volatile u8 bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;
/* 速度反馈计算倒计时（单位：500us），到 0 时计算平均转速。*/
static u16 hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;

#ifdef FLUX_TORQUE_PIDs_TUNING  
/* PID 在线整定用的方波周期计数(定义 FLUX_TORQUE_PIDs_TUNING 时启用)。*/
static u16 hTorqueSwapping = SQUARE_WAVE_PERIOD; 
#endif
/*******************************************************************************
* Function Name  : TB_Init
* Description    : TimeBase peripheral initialization. The base time is set to 
*                  500usec and the related interrupt is enabled  
* Input          : None
* Output         : None
* Return         : None
* 功能说明(中文) : 时基初始化。把 SysTick 时钟源选为 AHB(HCLK)，并配置为每
*                  500us 产生一次中断（72MHz 下重装载值 36000），从而建立整机
*                  500us 计时节拍。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 其余被注释掉的 SysTick 配置行保留原样，当前仅用 SysTick_Config
*                  一条即完成时钟源、重装载、计数使能与中断使能。
*******************************************************************************/
void TB_Init(void)
{   
  /* Select AHB clock(HCLK) as SysTick clock source */
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);   // 选 HCLK(72MHz) 作为 SysTick 时钟源
  /* SysTick interrupt each 500usec with Core clock equal to 72MHz */
  SysTick_Config(36000);   // 重装载 36000 → 72MHz/36000=2kHz，即每 500us 中断一次
  //SysTick_SetReload(36000);
  /* Enable SysTick Counter */
  //SysTick_CounterCmd(SysTick_Counter_Enable);

  /*NVIC_SystemHandlerPriorityConfig(SystemHandler_SysTick, 
                            SYSTICK_PRE_EMPTION_PRIORITY, SYSTICK_SUB_PRIORITY); */
  /* Enable SysTick interrupt */
  //SysTick_ITConfig(ENABLE);
}

/*******************************************************************************
* Function Name  : TB_Wait
* Description    : The function wait for a delay to be over.   
* Input          : None
* Output         : None
* Return         : None
* 功能说明(中文) : 阻塞式延时。把 'time' 装入 500us 计数变量后忙等，直到 SysTick
*                  中断把它递减到 0，实现 time×0.5ms 的延时。
* 参数(中文)     : time — 延时长度，单位 500us（如 time=10 → 5ms）。
* 返回(中文)     : 无。
* 备注(中文)     : 忙等期间 CPU 空转，仅供初始化等非实时场景使用。
*******************************************************************************/
void TB_Wait(u16 time)
{
hTimebase_500us = time;    // delay = 'time' value * 0.5ms   // 装载计数值，每个 500us 减 1
while (hTimebase_500us != 0) // wait and do nothing!   // 忙等，直到计数减到 0
{}  

}

/*******************************************************************************
* Function Name  : TB_Set_Delay_500us
* Description    : Set delay utilized by main.c state machine.   
* Input          : Time out value
* Output         : None
* Return         : None
* 功能说明(中文) : 设置供 main.c 主状态机使用的非阻塞延时计数。
* 参数(中文)     : hDelay — 延时长度，单位 500us。
* 返回(中文)     : 无。
* 备注(中文)     : 设置后由 SysTick 递减，调用方用 TB_Delay_IsElapsed 查询到期。
*******************************************************************************/
void TB_Set_Delay_500us(u16 hDelay)
{
  hTimebase_500us = hDelay;   // 装载延时计数（单位 500us）
}  

/*******************************************************************************
* Function Name  : TB_Delay_IsElapsed
* Description    : Check if the delay set by TB_Set_Delay_500us is elapsed.   
* Input          : None
* Output         : True if delay is elapsed, false otherwise 
* Return         : None
* 功能说明(中文) : 查询由 TB_Set_Delay_500us 设置的延时是否到期。
* 参数(中文)     : 无。
* 返回(中文)     : bool — TRUE 表示已到期(计数为 0)，FALSE 表示仍在延时中。
* 备注(中文)     : 非阻塞轮询方式使用，不改变计数。
*******************************************************************************/
bool TB_Delay_IsElapsed(void)
{
 if (hTimebase_500us == 0)   // 计数为 0 表示延时结束
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
}  

/*******************************************************************************
* Function Name  : TB_Set_DisplayDelay_500us
* Description    : Set Delay utilized by MC_Display.c module.   
* Input          : Time out value
* Output         : None
* Return         : None
* 功能说明(中文) : 设置供 MC_Display.c 显示模块使用的独立延时计数。
* 参数(中文)     : hDelay — 延时长度，单位 500us。
* 返回(中文)     : 无。
* 备注(中文)     : 使用独立计数变量，与主状态机延时互不干扰。
*******************************************************************************/
void TB_Set_DisplayDelay_500us(u16 hDelay)
{
  hTimebase_display_500us = hDelay;   // 装载显示延时计数（单位 500us）
}  

/*******************************************************************************
* Function Name  : TB_DisplayDelay_IsElapsed
* Description    : Check if the delay set by TB_Set_DisplayDelay_500us is elapsed.   
* Input          : None
* Output         : True if delay is elapsed, false otherwise 
* Return         : None
* 功能说明(中文) : 查询由 TB_Set_DisplayDelay_500us 设置的显示延时是否到期。
* 参数(中文)     : 无。
* 返回(中文)     : bool — TRUE 表示已到期，FALSE 表示仍在延时中。
* 备注(中文)     : 供显示刷新节流使用，非阻塞轮询。
*******************************************************************************/
bool TB_DisplayDelay_IsElapsed(void)
{
 if (hTimebase_display_500us == 0)   // 计数为 0 表示延时结束
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 

/*******************************************************************************
* Function Name  : TB_Set_DebounceDelay_500us
* Description    : Set Delay utilized by MC_Display.c module.   
* Input          : Time out value
* Output         : None
* Return         : None
* 功能说明(中文) : 设置按键消抖延时计数（用于按键/显示模块的去抖动）。
* 参数(中文)     : hDelay — 消抖时间，单位 500us（u8，最大约 127.5ms）。
* 返回(中文)     : 无。
* 备注(中文)     : 使用独立计数变量 hKey_debounce_500us。
*******************************************************************************/
void TB_Set_DebounceDelay_500us(u8 hDelay)
{
  hKey_debounce_500us = hDelay;   // 装载按键消抖计数（单位 500us）
}  

/*******************************************************************************
* Function Name  : TB_DebounceDelay_IsElapsed
* Description    : Check if the delay set by TB_Set_DebounceDelay_500us is elapsed.   
* Input          : None
* Output         : True if delay is elapsed, false otherwise 
* Return         : None
* 功能说明(中文) : 查询由 TB_Set_DebounceDelay_500us 设置的消抖延时是否到期。
* 参数(中文)     : 无。
* 返回(中文)     : bool — TRUE 表示已到期，FALSE 表示仍在消抖中。
* 备注(中文)     : 到期表示按键电平已稳定，可以采信该按键状态。
*******************************************************************************/
bool TB_DebounceDelay_IsElapsed(void)
{
 if (hKey_debounce_500us == 0)   // 计数为 0 表示消抖结束
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 

/*******************************************************************************
* Function Name  : TB_Set_StartUp_Timeout(STARTUP_TIMEOUT)
* Description    : Set Start up time out and initialize Start_up torque in  
*                  torque control.   
* Input          : Time out value
* Output         : None
* Return         : None
* 功能说明(中文) : 设置无传感器/开环启动的超时保护时间。启动过程若在该时间内
*                  未完成(观测器未收敛)，则超时后可判定启动失败。
* 参数(中文)     : hTimeout — 超时时间，单位 ms。
* 返回(中文)     : 无。
* 备注(中文)     : 内部乘以 2 换算为 500us 计数（因为时基单位为 0.5ms）。
*******************************************************************************/
void TB_Set_StartUp_Timeout(u16 hTimeout)
{
  hStart_Up_TimeLeft_500us = 2*hTimeout;   // ms → 500us 计数（×2）
}  

/*******************************************************************************
* Function Name  : TB_StartUp_Timeout_IsElapsed
* Description    : Set Start up time out.   
* Input          : None
* Output         : True if start up time out is elapsed, false otherwise 
* Return         : None
* 功能说明(中文) : 查询启动超时是否已到。
* 参数(中文)     : 无。
* 返回(中文)     : bool — TRUE 表示启动超时(计数为 0)，FALSE 表示尚未超时。
* 备注(中文)     : 配合 TB_Set_StartUp_Timeout 使用，用于启动失败保护。
*******************************************************************************/
bool TB_StartUp_Timeout_IsElapsed(void)
{
 if (hStart_Up_TimeLeft_500us == 0)   // 计数为 0 表示启动超时
 {
   return (TRUE);
 }
 else 
 {
   return (FALSE);
 }
} 


/*******************************************************************************
* Function Name  : SysTickHandler
* Description    : This function handles SysTick Handler.
* Input          : None
* Output         : None
* Return         : None
* 功能说明(中文) : SysTick 中断服务程序，每 500us 触发一次。职责：
*                  (1) 若无则递减各软定时器计数（主状态机延时、显示延时、按键消抖、
*                      启动超时、速度反馈周期、速度环周期）；
*                  (2) 速度反馈周期到达时调用 ENC_Calc_Average_Speed() 计算平均转速；
*                  (3) 速度环周期到达且电机处于 RUN 状态时调用 FOC_CalcFluxTorqueRef()
*                      执行速度环，更新转矩电流参考。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 中断内应保持简短；速度环与速度反馈均按各自周期节流执行。
*******************************************************************************/
void SysTick_Handler(void)
{ 
    if (hTimebase_500us != 0)   // 主状态机延时递减
    {
        hTimebase_500us --;
    }
    
    if (hTimebase_display_500us != 0)   // 显示延时递减
    {
        hTimebase_display_500us --;
    }
    
    if (hKey_debounce_500us != 0)   // 按键消抖计数递减
    {
        hKey_debounce_500us --;
    }
    
    if (hStart_Up_TimeLeft_500us != 0)   // 启动超时计数递减
    {
        hStart_Up_TimeLeft_500us--;
    }
    
    
    
    if (hSpeedMeas_Timebase_500us !=0)   // 速度反馈周期未到则递减
    {
        hSpeedMeas_Timebase_500us--;
    }
    else
    {
        hSpeedMeas_Timebase_500us = SPEED_SAMPLING_TIME;   // 重装速度反馈周期
    
        //ENC_Calc_Average_Speed must be called ONLY every SPEED_MEAS_TIMEBASE ms
        ENC_Calc_Average_Speed();   // 周期到达：计算平均转速（编码器/霍尔反馈）
    }
    
    
    if (bPID_Speed_Sampling_Time_500us != 0 )   // 速度环周期未到则递减
    {
        bPID_Speed_Sampling_Time_500us --;
    }
    else
    { 
        bPID_Speed_Sampling_Time_500us = PID_SPEED_SAMPLING_TIME;        // 重装速度环周期

        if (State == RUN)   // 仅在电机运行态执行速度环
        {    
            //PID_Speed_Coefficients_update(XXX_Get_Speed(),PID_Speed_InitStructure);
            FOC_CalcFluxTorqueRef();   // 执行速度环：更新转矩/磁链电流参考        
        }
    }
}

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
