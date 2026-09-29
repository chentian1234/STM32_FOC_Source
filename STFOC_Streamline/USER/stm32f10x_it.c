/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/*******************************************************************************
* 文件说明(中文):
*   本文件集中实现各中断服务函数(ISR)。
*   前一部分是 Cortex-M3 内核异常(复位相关/NMI/硬件错误等)，多数为空实现或死循环；
*   后一部分是本工程实际使用的三个外设中断:
*     ADC1_2_IRQHandler : ADC 注入转换完成中断，执行 FOC 电流环，是最关键的实时中断；
*     TIM1_BRK_IRQHandler: TIM1 刹车(过流)中断，用于故障保护；
*     TIM1_UP_IRQHandler : TIM1 更新中断，触发 SVPWM 状态更新。
*   另有 uart.c 中的 USART1_IRQHandler 处理串口收发。
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"   // 内核异常处理函数声明
#include "includes.h"       // 工程公共头文件(含外设库与电机控制库)

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  * 中文说明 : 不可屏蔽中断(NMI)处理。一般由严重硬件故障(如时钟失效)触发；
  *            本工程为空实现，不做任何处理。
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  * 中文说明 : 硬件错误异常处理。进入后死循环(便于调试器定位错误现场)，
  *            不会退出，也不会返回。
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  * 中文说明 : 存储器管理异常处理(如非法访问 MPU 保护区域)；进入后死循环。
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  * 中文说明 : 总线错误异常处理(如访问非法地址)；进入后死循环。
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  * 中文说明 : 用法错误异常处理(如非法指令、非对齐访问)；进入后死循环。
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  * 中文说明 : 系统服务调用(SVC)异常处理；本工程为空实现。
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  * 中文说明 : 调试监视器异常处理；本工程为空实现。
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  * 中文说明 : 可挂起系统服务(PendSV)异常处理，通常用于 RTOS 任务切换；
  *            本工程未使用 RTOS，为空实现。
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  * 中文说明 : 系统滴答定时器中断处理函数。下面整段被注释掉: 原实现是翻转 PC9(LED4)。
  *            本工程未使用 SysTick 定时中断(周期任务由 TIM/时间基准承担)。
  
void SysTick_Handler(void)
{
    GPIOC->ODR ^= 1<<9;
}*/

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  * 中文说明 : 外设中断处理函数模板(占位示例)，整段被注释，未使用。
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */
/*******************************************************************************
* 功能说明(中文) : ADC1/ADC2 注入通道转换完成中断服务函数，是本工程最关键的实时中断。
*                  由 TIM1 在 PWM 特定时刻触发三相电流采样，转换完成后进入此中断:
*                  (1) 判定并清除 ADC 注入转换完成(JEOC)标志；
*                  (2) 若 SVPWM 更新事件有效，计算母线电压，并按状态机 State 执行:
*                        RUN   状态 -> FOC_Model()   完成电流环(Clarke/Park/PI/SVPWM)；
*                        START 状态 -> ENC_Start_Up() 执行启动流程；
*                  (3) 每次中断翻转 LED1，用于观测中断是否在运行。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 执行周期 = PWM 载波频率(每个 PWM 周期一次)，对时序极敏感，
*                  中断内代码须尽量短小高效。
*******************************************************************************/
void ADC1_2_IRQHandler(void)
{
    //if(ADC_GetITStatus(ADC1, ADC_IT_JEOC) == SET))
    if((ADC1->SR & ADC_FLAG_JEOC) == ADC_FLAG_JEOC)     // 注入组转换完成标志是否置位?
    {
        //It clear JEOC flag
        ADC1->SR = ~(u32)ADC_FLAG_JEOC;                 // 写 0 清除 JEOC 标志
      
        if (SVPWMEOCEvent())                            // SVPWM 的电流采样结束事件是否有效?
        {    
            MCL_Calc_BusVolt();                         // 计算/更新母线电压(用于电压前馈与保护)
            switch (State)                              // 按电机控制状态机分派
            {
            case RUN:                                   // 运行态: 执行 FOC 电流环
                FOC_Model();       
                break;       
    
            case START:                                 // 启动态: 执行编码器启动流程
                ENC_Start_Up();       
                break; 
    
            default:
                break;
            }
        }
    }
    Led1Toggle();                                       // 翻转 LED1，作为该中断的心跳指示
    
}


/*******************************************************************************
* 功能说明(中文) : TIM1 刹车(Break)中断服务函数，由过流/故障信号触发。
*                  进入后立即上报过流故障(OVER_CURRENT)，并清除刹车中断标志。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 属于硬件保护通道: 触发即由硬件封锁 PWM 输出并置故障标志，
*                  需上层执行清故障流程后恢复。
*******************************************************************************/
void TIM1_BRK_IRQHandler(void)
{
  MCL_SetFault(OVER_CURRENT);               // 置"过流"故障
  TIM_ClearITPendingBit(TIM1, TIM_IT_Break);    // 清除 TIM1 刹车中断标志
}



/*******************************************************************************
* 功能说明(中文) : TIM1 更新(Update)中断服务函数。PWM 计数器溢出(每个 PWM 周期)
*                  时触发: 先清除更新标志，再调用 SVPWMUpdateEvent() 完成 SVPWM 状态更新。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 在定义了 ICS_SENSORS 的配置下不执行 SVPWMUpdateEvent()；
*                  本中断与 ADC 注入中断共同构成 FOC 的关键时序。
*******************************************************************************/
void TIM1_UP_IRQHandler(void)
{
  // Clear Update Flag
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);     // 清除 TIM1 更新中断标志

#ifndef ICS_SENSORS  
  SVPWMUpdateEvent();                       // 执行 SVPWM 更新事件(采样时刻/状态推进)
#endif
}









/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
