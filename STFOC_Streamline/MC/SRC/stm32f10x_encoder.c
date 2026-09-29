/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_encoder.c 
* Author             : IMS Systems Lab  
* Date First Issued  : 21/11/07
* Description        : This file contains the software implementation for the
*                      encoder unit
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
/* ==========================================================================
 * 文件说明(中文)
 *   本模块实现基于「增量式编码器」的转子位置/速度反馈, 供 FOC(磁场定向控制)
 *   使用。核心要点:
 *     - 编码器输出 A/B 两路正交方波(与 Z 相零位脉冲), 定时器工作在「编码器
 *       模式」, 对 A/B 边沿计数实现「正交解码 4 倍频」, 得到转子连续位置;
 *     - 定时器计数达到 4*ENCODER_PPR(每转)后回绕, 用溢出中断统计圈数, 从而
 *       把 16 位计数扩展为多圈位置;
 *     - 对位置做差分即得机械转速(机械频率 = 电频率 / 极对数);
 *     - 用滑动平均平滑转速, 并在 RUN 状态下做上下限饱和与反馈错误检测;
 *     - 启动时执行对齐(Alignment), 把转子定向到已知电角度后再切入 RUN。
 *   术语说明:
 *     - 正交解码 4 倍频: A/B 两相边沿都计数, 每线每转产生 4 个计数脉冲, 分辨率
 *       提高到 4*ENCODER_PPR 计数/转;
 *     - 极对数(POLE_PAIR_NUM): 机械 1 转 = POLE_PAIR_NUM 个电周期,
 *       即 电角度 = 机械角度 x 极对数。
 * ========================================================================== */
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_encoder.h"
#include "stm32f10x_it.h"
#include "MC_Globals.h"
#include "stm32f10x_MClib.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
// 中文: COUNTER_RESET —— 与「对齐电角度 ALIGNMENT_ANGLE」对应的编码器计数初值。
//       先按 4 倍频把角度换成计数: (ALIGNMENT_ANGLE*4*ENCODER_PPR/360)-1;
//       由于编码器计数值换算为电角度时会乘以极对数, 故此处再除以 POLE_PAIR_NUM。
//       启动对齐完成后写入计数器, 使位置基准对齐到该电角度。
#define COUNTER_RESET       (u16) ((((s32)(ALIGNMENT_ANGLE)*4*ENCODER_PPR/360)\
                                                              -1)/POLE_PAIR_NUM)
#define ICx_FILTER          (u8) 8 // 8<-> 670nsec
                                   // 中文: 输入捕获数字滤波, 0x08 对应约 670ns

#define SPEED_SAMPLING_FREQ (u16)(2000/(SPEED_SAMPLING_TIME+1))
                            // 中文: 速度采样频率(Hz); 由速度采样时间(ms)换算, 采样周期约 SPEED_SAMPLING_TIME ms

#define TIMx_PRE_EMPTION_PRIORITY 2  // 中文: 编码器定时器中断的抢占优先级
#define TIMx_SUB_PRIORITY 0          // 中文: 编码器定时器中断的子优先级

#define SPEED_SAMPLING_TIME   PID_SPEED_SAMPLING_TIME
                              // 中文: 速度采样时间, 取自 PID 参数(单位 ms)

/* Private macro -------------------------------------------------------------*/
// To avoid obvious initialization errors...
// 中文: 防止明显的初始化配置错误: 若同时选择了两个定时器承载编码器, 直接编译报错。
#if  ( (defined(TIMER2_HANDLES_ENCODER) && defined(TIMER3_HANDLES_ENCODER)) \
    || (defined(TIMER2_HANDLES_ENCODER) && defined(TIMER4_HANDLES_ENCODER)) \
    || (defined(TIMER3_HANDLES_ENCODER) && defined(TIMER4_HANDLES_ENCODER)))
  #error "Invalid encoder setup: 2 timers selected"
#endif

#ifdef ENCODER

// Warning message if encoder unit not connected
#ifndef TIMER2_HANDLES_ENCODER
#ifndef TIMER3_HANDLES_ENCODER
#ifndef TIMER4_HANDLES_ENCODER
#warning "Encoder not selected"
#endif
#endif
#endif
#endif  // ENCODER

/* Private functions ---------------------------------------------------------*/
s16 ENC_Calc_Rot_Speed(void);          // 中文: 计算本次机械转速(0.1Hz 定标, 内部使用)

/* Private variables ---------------------------------------------------------*/
static s16 hPrevious_angle, hSpeed_Buffer[SPEED_BUFFER_SIZE], hRot_Speed;
                          // 中文: hPrevious_angle 上次计数值; hSpeed_Buffer 测速平均缓冲; hRot_Speed 平滑后转速
static u8 bSpeed_Buffer_Index = 0;   // 中文: 测速平均缓冲的写入下标(循环)
static volatile u16 hEncoder_Timer_Overflow; 
                          // 中文: 编码器定时器计数溢出(回绕)次数, 用于扩展为多圈位置; 中断中递增
static bool bIs_First_Measurement = TRUE;  // 中文: 首测标志(首次测量被丢弃, 因无历史基准)
static bool bError_Speed_Measurement = FALSE; // 中文: 速度反馈故障标志(连续错误超限时置位)

/*******************************************************************************
* Function Name  : ENC_Init
* Description    : General Purpose Timer x set-up for encoder speed/position 
*                  sensors
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

/* 功能说明(中文) : 按「编码器模式」初始化通用定时器, 采集编码器 A/B 正交信号。
 *                 上电初始化阶段调用一次(非周期性)。
 * 参数(中文)     : 无
 * 返回(中文)     : 无
 * 备注(中文)     : 定时器由 ENCODER_TIMER 宏选定; 计数周期设为 4*ENCODER_PPR-1
 *                 (即 4 倍频后每转的计数); 配置 TI12 编码器模式; 使能 Update 中断
 *                 统计计数回绕, 从而扩展为多圈位置; 计数初值 = COUNTER_RESET。
 *******************************************************************************/
void ENC_Init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_ICInitTypeDef TIM_ICInitStructure;
  // 中文: 编码器初始化局部结构体; 依所选定时器配置对应 GPIO、NVIC 与定时器参数。
  
#if defined(TIMER2_HANDLES_ENCODER)   // Encoder unit connected to TIM2, 4X mode
    
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
    
  /* TIM2 clock source enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /* Enable GPIOA, clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  GPIO_StructInit(&GPIO_InitStructure);
  /* Configure PA.00,01 as encoder input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Enable the TIM2 Update Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#elif defined(TIMER3_HANDLES_ENCODER) // Encoder unit connected to TIM3, 4X mode
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* TIM3 clock source enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /* Enable GPIOA, clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  
  GPIO_StructInit(&GPIO_InitStructure);
  /* Configure PA.06,07 as encoder input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Enable the TIM3 Update Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

#elif defined(TIMER4_HANDLES_ENCODER) // Encoder unit connected to TIM4, 4X mode
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  
  /* TIM4 clock source enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  /* Enable GPIOA, clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  
  GPIO_StructInit(&GPIO_InitStructure);
  /* Configure PB.06,07 as encoder input */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Enable the TIM4 Update Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIMx_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIMx_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
#endif

  /* Timer configuration in Encoder mode */
  TIM_DeInit(ENCODER_TIMER);
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
  TIM_TimeBaseStructure.TIM_Period = (4*ENCODER_PPR)-1;  
  // 中文: 计数周期 = 4*ENCODER_PPR-1, 即 4 倍频后每转计数 4*ENCODER_PPR 次即回绕; 时钟不分频
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
  TIM_TimeBaseInit(ENCODER_TIMER, &TIM_TimeBaseStructure);
 
  // 中文: 配置 TI12 编码器模式: 对 A/B 两相的上升沿都计数, 等效「正交解码 4 倍频」
  TIM_EncoderInterfaceConfig(ENCODER_TIMER, TIM_EncoderMode_TI12, 
                             TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStructure);
  
  TIM_ICInitStructure.TIM_ICFilter = ICx_FILTER;
  // 中文: 设置输入滤波器(抗抖动/毛刺)。
  TIM_ICInit(ENCODER_TIMER, &TIM_ICInitStructure);
  
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInit(ENCODER_TIMER, &TIM_ICInitStructure);
      
 // Clear all pending interrupts
 // 中文: 清除所有挂起的更新中断标志。
  TIM_ClearFlag(ENCODER_TIMER, TIM_FLAG_Update);
  TIM_ITConfig(ENCODER_TIMER, TIM_IT_Update, ENABLE);
  // 中文: 使能更新(溢出)中断, 用于统计计数回绕圈数以扩展多圈位置。
  //Reset counter
  TIM2->CNT = COUNTER_RESET;
  
  // 中文: 使能编码器定时器, 开始对 A/B 正交信号计数。
  TIM_Cmd(ENCODER_TIMER, ENABLE);
}

/*******************************************************************************
* Function Name  : ENC_Get_Electrical_Angle
* Description    : Returns the absolute electrical Rotor angle 
* Input          : None
* Output         : None
* Return         : Rotor electrical angle: 0 -> 0 degrees, 
*                                          S16_MAX-> 180 degrees, 
*                                          S16_MIN-> -180 degrees
*                  Mechanical angle can be derived calling this function and 
*                  dividing by POLE_PAIR_NUM
*******************************************************************************/
/* 功能说明(中文) : 返回转子绝对电角度。
 * 参数(中文)     : 无
 * 返回(中文)     : s16 —— 电角度: 0 -> 0 度, S16_MAX -> 180 度, S16_MIN -> -180 度
 * 备注(中文)     : 机械角度 x 极对数 = 电角度; 即本结果除以 POLE_PAIR_NUM 得机械角度。
 *******************************************************************************/
s16 ENC_Get_Electrical_Angle(void)
{
  s32 temp;
  
  temp = (s32)(TIM_GetCounter(ENCODER_TIMER)) * (s32)(U32_MAX / (4*ENCODER_PPR));         
  temp *= POLE_PAIR_NUM;  
  return((s16)(temp/65536)); // s16 result
}

/*******************************************************************************
* Function Name  : ENC_Get_Mechanical_Angle
* Description    : Returns the absolute mechanical Rotor angle 
* Input          : None
* Output         : None
* Return         : Rotor mechanical angle: 0 -> 0 degrees, S16_MAX-> 180 degrees, 
                                            S16_MIN-> -180 degrees
*******************************************************************************/
/* 功能说明(中文) : 返回转子绝对机械角度。
 * 参数(中文)     : 无
 * 返回(中文)     : s16 —— 机械角度: 0 -> 0 度, S16_MAX -> 180 度, S16_MIN -> -180 度
 * 备注(中文)     : 与电角度相差 POLE_PAIR_NUM 倍(电角度 = 机械角度 x 极对数)。
 *******************************************************************************/
s16 ENC_Get_Mechanical_Angle(void)
{
  s32 temp;
  
  temp = (s32)(TIM_GetCounter(ENCODER_TIMER)) * (s32)(U32_MAX / (4*ENCODER_PPR)) ;
  return((s16)(temp/65536)); // s16 result
}

/*******************************************************************************
* Function Name  : ENC_ResetEncoder
* Description    : Write the encoder counter with the value corresponding to
*                  ALIGNMENT_ANGLE
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/* 功能说明(中文) : 把编码器计数器写入与 ALIGNMENT_ANGLE 对齐角度对应的值。
 * 参数(中文)     : 无
 * 返回(中文)     : 无
 * 备注(中文)     : 启动对齐完成后调用, 将位置基准归位到对齐点;
 *                 当前实现硬编码 TIM2->CNT(与 TIMER2_HANDLES_ENCODER 的选择一致)。
 *******************************************************************************/
void ENC_ResetEncoder(void)
{
  //Reset counter
  TIM2->CNT = COUNTER_RESET;
}

             
/*******************************************************************************
* Function Name  : ENC_Clear_Speed_Buffer
* Description    : Clear speed buffer used for average speed calculation  
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/* 功能说明(中文) : 清空用于平均的速度缓冲, 并置「首测」标志。
 * 参数(中文)     : 无
 * 返回(中文)     : 无
 * 备注(中文)     : 在对齐阶段结束时调用, 丢弃对齐期间的速度采样, 避免污染平均。
 *******************************************************************************/
void ENC_Clear_Speed_Buffer(void)
{   
  u32 i;

  for (i=0;i<SPEED_BUFFER_SIZE;i++)
  {
    hSpeed_Buffer[i] = 0;
  }
  bIs_First_Measurement = TRUE;
}

/*******************************************************************************
* Function Name  : ENC_Calc_Rot_Speed
* Description    : Compute return latest speed measurement 
* Input          : None
* Output         : s16
* Return         : Return motor speed in 0.1 Hz resolution. Since the encoder is
                   used as speed sensor, this routine will return the mechanical
                   speed of the motor (NOT the electrical frequency)
                   Mechanical frequency is equal to electrical frequency/(number 
                   of pair poles).
*******************************************************************************/
/* 功能说明(中文) : 计算本次测得的电机机械转速(非电频率)。
 *                 每个速度环采样周期调用一次(周期 = SPEED_SAMPLING_TIME)。
 * 参数(中文)     : 无
 * 返回(中文)     : s16 —— 机械转速, 单位 0.1Hz。机械频率 = 电频率/极对数。
 * 备注(中文)     : 两次读取计数值与溢出计数以得到角度增量(考虑正/反转与计数
 *                 回绕), 再按 1/采样时间换算为转速; 首次测量被丢弃。
 *******************************************************************************/
s16 ENC_Calc_Rot_Speed(void)
{   
    s32 wDelta_angle;
    u16 hEnc_Timer_Overflow_sample_one, hEnc_Timer_Overflow_sample_two;
    u16 hCurrent_angle_sample_one, hCurrent_angle_sample_two;
    signed long long temp;
    s16 haux;
    
    if (!bIs_First_Measurement)
    // 中文: 非首次测量才实际计算(首次无历史基准, 会被丢弃)
    {
        // 1st reading of overflow counter    
        hEnc_Timer_Overflow_sample_one = hEncoder_Timer_Overflow; 
        // 1st reading of encoder timer counter
        hCurrent_angle_sample_one = ENCODER_TIMER->CNT;
        // 2nd reading of overflow counter
        hEnc_Timer_Overflow_sample_two = hEncoder_Timer_Overflow;  
        // 2nd reading of encoder timer counter
        hCurrent_angle_sample_two = ENCODER_TIMER->CNT;      
        
        // Reset hEncoder_Timer_Overflow and read the counter value for the next
        // measurement
        hEncoder_Timer_Overflow = 0;
        haux = ENCODER_TIMER->CNT;   
        
        if (hEncoder_Timer_Overflow != 0) 
        {
            haux = ENCODER_TIMER->CNT; 
            hEncoder_Timer_Overflow = 0;            
        }
        
        if (hEnc_Timer_Overflow_sample_one != hEnc_Timer_Overflow_sample_two)
        { //Compare sample 1 & 2 and check if an overflow has been generated right 
            //after the reading of encoder timer. If yes, copy sample 2 result in 
            //sample 1 for next process 
            hCurrent_angle_sample_one = hCurrent_angle_sample_two;
            hEnc_Timer_Overflow_sample_one = hEnc_Timer_Overflow_sample_two;
        }
        
        if ( (ENCODER_TIMER->CR1 & TIM_CounterMode_Down) == TIM_CounterMode_Down)  
        {// encoder timer down-counting
            // 中文: 定时器为减计数(反转): 角增量 = 本次-上次-溢出圈数*每转计数
            wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle - 
                (hEnc_Timer_Overflow_sample_one) * (4*ENCODER_PPR));
        }
        else  
        {//encoder timer up-counting
            // 中文: 定时器为增计数(正转): 角增量 = 本次-上次+溢出圈数*每转计数
            wDelta_angle = (s32)(hCurrent_angle_sample_one - hPrevious_angle + 
                (hEnc_Timer_Overflow_sample_one) * (4*ENCODER_PPR));
        }
        
        // speed computation as delta angle * 1/(speed sempling time)
        // 中文: 转速 = 角增量 x 采样频率; 乘 10 得 0.1Hz 定标, 再除以每转计数(4*PPR)。
        temp = (signed long long)(wDelta_angle * SPEED_SAMPLING_FREQ);                                                                
        temp *= 10;  // 0.1 Hz resolution
        // 中文: 乘 10 得到 0.1Hz 分辨率
        temp /= (4*ENCODER_PPR);
        // 中文: 除以每转计数(4*ENCODER_PPR), 把计数增量换算为「转」
    
    } //is first measurement, discard it
    // 中文: 首次测量仅建立基准, 结果置 0 并清零溢出计数。
    else
    {
        bIs_First_Measurement = FALSE;
        // 中文: 清除首测标志, 下次开始输出真实转速
        temp = 0;
        hEncoder_Timer_Overflow = 0;
        haux = ENCODER_TIMER->CNT;       
        // Check if Encoder_Timer_Overflow is still zero. In case an overflow IT 
        // occured it resets overflow counter and wPWM_Counter_Angular_Velocity
        if (hEncoder_Timer_Overflow != 0) 
        {
            haux = ENCODER_TIMER->CNT; 
            hEncoder_Timer_Overflow = 0;            
        }
    }
    
    hPrevious_angle = haux;  
    
    return((s16) temp);
}

/*******************************************************************************
* Function Name  : ENC_Get_Mechanical_Speed
* Description    : Export the value of the smoothed motor speed computed in 
*                  ENC_Calc_Average_Speed function  
* Input          : None
* Output         : s16
* Return         : Return motor speed in 0.1 Hz resolution. This routine 
                   will return the average mechanical speed of the motor.
*******************************************************************************/
/* 功能说明(中文) : 导出 ENC_Calc_Average_Speed 计算出的平滑机械转速。
 * 参数(中文)     : 无
 * 返回(中文)     : s16 —— 平均机械转速, 单位 0.1Hz
 * 备注(中文)     : 结果保存在内部变量 hRot_Speed 中。
 *******************************************************************************/
s16 ENC_Get_Mechanical_Speed(void)
{
  return(hRot_Speed);
}

/*******************************************************************************
* Function Name  : ENC_Calc_Average_Speed
* Description    : Compute smoothed motor speed based on last SPEED_BUFFER_SIZE
                   informations and store it variable  
* Input          : None
* Output         : s16
* Return         : Return rotor speed in 0.1 Hz resolution. This routine 
                   will return the average mechanical speed of the motor.
*******************************************************************************/
/* 功能说明(中文) : 基于最近 SPEED_BUFFER_SIZE 次测速值计算平滑后的机械转速。
 *                 速度环每个采样周期调用一次。
 * 参数(中文)     : 无
 * 返回(中文)     : 无(结果写入 hRot_Speed, s16, 0.1Hz 定标)
 * 备注(中文)     : 在 RUN 状态下对测速值做上下限饱和并统计连续错误, 连续错误
 *                 达到 MAXIMUM_ERROR_NUMBER 次则置反馈故障标志。
 *******************************************************************************/
void ENC_Calc_Average_Speed(void)
{   
    s32 wtemp;
    u16 hAbstemp;
    u32 i;
    u8 static bError_counter;
    
    wtemp = ENC_Calc_Rot_Speed();
    hAbstemp = ( wtemp < 0 ? - wtemp :  wtemp);
    
    /* Checks for speed measurement errors when in RUN State and saturates if 
                                                necessary*/  
    // 中文: 仅在 RUN(运行)状态下校验测速范围, 用于反馈故障检测
    if (State == RUN)
    {    
        if(hAbstemp < MINIMUM_MECHANICAL_SPEED)
        // 中文: 测速低于下限: 饱和到下限值并累计一次错误
        { 
            if (wtemp < 0)
            {
                wtemp = -(s32)(MINIMUM_MECHANICAL_SPEED);
            }
            else
            {
                wtemp = MINIMUM_MECHANICAL_SPEED;
            }
            bError_counter++;
        }
        else  if (hAbstemp > MAXIMUM_MECHANICAL_SPEED) 
        {
            if (wtemp < 0)
            {
                wtemp = -(s32)(MAXIMUM_MECHANICAL_SPEED);
            }
            else
            {
                wtemp = MAXIMUM_MECHANICAL_SPEED;
            }
            bError_counter++;
        }
        else
        { 
            bError_counter = 0;
        }
        
        if (bError_counter >= MAXIMUM_ERROR_NUMBER)
        // 中文: 连续错误达到阈值 -> 置反馈故障标志
        {
            bError_Speed_Measurement = TRUE;
        }
        else
        {
            bError_Speed_Measurement = FALSE;
        }
    }
    else
    {
        bError_Speed_Measurement = FALSE;
        bError_counter = 0;
    }
    
    /* Compute the average of the read speeds */
    // 中文: 计算缓冲内所有测速值的平均作为平滑转速。
    
    hSpeed_Buffer[bSpeed_Buffer_Index] = (s16)wtemp;
    // 中文: 把本次测速值写入平均缓冲(循环写)
    bSpeed_Buffer_Index++;
    
    if (bSpeed_Buffer_Index == SPEED_BUFFER_SIZE) 
    {
        bSpeed_Buffer_Index = 0;
    }
    
    wtemp=0;
    
    for (i=0;i<SPEED_BUFFER_SIZE;i++)
    {
        wtemp += hSpeed_Buffer[i];
    }
    wtemp /= SPEED_BUFFER_SIZE;
    
    // 中文: 得到平滑后的机械转速(0.1Hz 定标)
    hRot_Speed = ((s16)(wtemp));
}

/*******************************************************************************
* Function Name  : ENC_ErrorOnFeedback
* Description    : Check for possible errors on speed measurement when State is 
*                  RUN. After MAXIMUM_ERROR_NUMBER consecutive speed measurement
*                  errors, the function return TRUE, else FALSE.
*                  Function return 
* Input          : None
* Output         : s16
* Return         : boolean variable
*******************************************************************************/
/* 功能说明(中文) : 检查测速反馈是否判定为故障。
 * 参数(中文)     : 无
 * 返回(中文)     : bool —— RUN 状态下连续 MAXIMUM_ERROR_NUMBER 次测速错误返回 TRUE
 * 备注(中文)     : 非 RUN 状态一律返回 FALSE(不报错)。
 *******************************************************************************/
bool ENC_ErrorOnFeedback(void)
{
 return(bError_Speed_Measurement); 
}

/*******************************************************************************
* Function Name : ENC_Start_Up
* Description   : The purpose of this function is to perform the alignment of 
*                 PMSM torque and flux regulation during the alignment phase.
* Input : details the input parameters.
* Output : details the output parameters.
* Return : details the return value.
*******************************************************************************/
/* 功能说明(中文) : 启动对齐(Alignment)流程: 对 PMSM 做转矩/磁链定向, 使转子
 *                 对齐到已知电角度, 随后切入 RUN 状态。
 *                 每个 FOC 周期调用, 直到对齐完成。
 * 参数(中文)     : 无
 * 返回(中文)     : 无
 * 备注(中文)     : 仅首次启动(置 FIRST_START 标志)执行一次; 期间按
 *                 T_ALIGNMENT_PWM_STEPS 逐步加大对齐电流 I_ALIGNMENT, 完成后
 *                 复位编码器、清零速度缓冲并进入 RUN。
 *******************************************************************************/
void ENC_Start_Up(void)
{
    static u32 wTimebase=0;
  
    if ( (wGlobal_Flags & FIRST_START) == FIRST_START)
    // 中文: 仅首次上电启动需要执行对齐流程
    {
        // First Motor start-up, alignment must be performed
        // 中文: 处于首次启动的对齐阶段, 记录已发出的对齐 PWM 周期数
        wTimebase++;
        // 中文: 对齐尚未结束: 磁链给定按 PWM 周期线性爬升, 逐步建立定向磁场
        if(wTimebase <= T_ALIGNMENT_PWM_STEPS)
        {                  
            hFlux_Reference = I_ALIGNMENT * wTimebase / T_ALIGNMENT_PWM_STEPS;               
            // 中文: 对齐阶段仅施加 d 轴(磁链)电流, 转矩电流给定保持 0
            hTorque_Reference = 0;
        
            Stat_Curr_a_b = GET_PHASE_CURRENTS(); 
            Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b); 
            Stat_Curr_q_d = Park(Stat_Curr_alfa_beta, ALIGNMENT_ANGLE_S16);  
            /*loads the Torque Regulator output reference voltage Vqs*/   
            Stat_Volt_q_d.qV_Component1 = PID_Regulator(hTorque_Reference, 
                        Stat_Curr_q_d.qI_Component1, &PID_Torque_InitStructure);   
        
            /*loads the Flux Regulator output reference voltage Vds*/
            Stat_Volt_q_d.qV_Component2 = PID_Regulator(hFlux_Reference, 
                          Stat_Curr_q_d.qI_Component2, &PID_Flux_InitStructure); 
  
            RevPark_Circle_Limitation();

            /*Performs the Reverse Park transformation,
            i.e transforms stator voltages Vqs and Vds into Valpha and Vbeta on a 
            stationary reference frame*/
  
            Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);

            /*Valpha and Vbeta finally drive the power stage*/ 
            CALC_SVPWM(Stat_Volt_alfa_beta);
        }
        else
        {
            wTimebase = 0;              
            ENC_ResetEncoder();          
            // 中文: 对齐结束: 复位编码器位置基准(上一行)、清零输出电压并恢复转矩/磁链给定
            Stat_Volt_q_d.qV_Component1 = Stat_Volt_q_d.qV_Component2 = 0;
            hTorque_Reference = PID_TORQUE_REFERENCE;
            hFlux_Reference = PID_FLUX_REFERENCE;
            wGlobal_Flags &= ~FIRST_START;   // alignment done only once 
            //Clear the speed acquisition of the alignment phase
            // 中文: 清除对齐阶段采集的速度样本, 避免污染后续测速滑动平均
            ENC_Clear_Speed_Buffer();
#ifdef ENCODER
            State = RUN;
#endif
        }
    }
    else
    {
#ifdef ENCODER
        State = RUN;
#endif
    }
} 

/*******************************************************************************
* Function Name  : TIMx_IRQHandler
* Description    : This function handles TIMx Update interrupt request.
                   Encoder unit connected to TIMx (x = 2,3 or 4)
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
#if defined(TIMER2_HANDLES_ENCODER)
/* 功能说明(中文) : 编码器定时器的更新(溢出)中断服务程序, 统计计数回绕次数。
 * 参数(中文)     : 无
 * 返回(中文)     : 无(中断服务例程)
 * 备注(中文)     : 计数从一端回绕到另一端(正转过 0 / 反转过 ARR)时触发一次,
 *                 每触发一次 hEncoder_Timer_Overflow 加 1(饱和到 U16_MAX),
 *                 用于把 16 位计数值扩展为多圈位置。
 *******************************************************************************/
  void TIM2_IRQHandler(void)
  {  
    /* Clear the interrupt pending flag */
    TIM_ClearFlag(ENCODER_TIMER, TIM_FLAG_Update);
    
    if (hEncoder_Timer_Overflow != U16_MAX)  
    {
     hEncoder_Timer_Overflow++;
    }
  }
#elif defined(TIMER3_HANDLES_ENCODER)

/* 功能说明(中文) : 编码器定时器(TIM3)更新中断服务程序, 统计计数回绕次数。
 * 参数(中文)     : 无
 * 返回(中文)     : 无(中断服务例程)
 * 备注(中文)     : 仅当选用 TIM3 承载编码器时才编译; 本工程选用 TIM2, 此分支不参与编译。
 *******************************************************************************/
  void TIM3_IRQHandler(void)
  {
    TIM_ClearFlag(ENCODER_TIMER, TIM_FLAG_Update);
    if (Encoder_Timer_Overflow != U16_MAX)  
    {
     Encoder_Timer_Overflow++;
    }
  
  }

#elif defined(TIMER4_HANDLES_ENCODER)
    
/* 功能说明(中文) : 编码器定时器(TIM4)更新中断服务程序, 统计计数回绕次数。
 * 参数(中文)     : 无
 * 返回(中文)     : 无(中断服务例程)
 * 备注(中文)     : 仅当选用 TIM4 承载编码器时才编译; 本工程选用 TIM2, 此分支不参与编译。
 *******************************************************************************/
  void TIM4_IRQHandler(void)
  {
    TIM_ClearFlag(ENCODER_TIMER, TIM_FLAG_Update);
    if (Encoder_Timer_Overflow != U16_MAX)  
    {
     Encoder_Timer_Overflow++;
    }  	
  }

#endif // TIMER4_HANDLES_ENCODER

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
