/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : STM32x_svpwm_ics.c
* Author             : IMS Systems Lab
* Date First Issued  : 21/11/07
* Description        : ICS current reading and PWM generation module 
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 09/07/08 v2.0.1
* 14/07/08 v2.0.2
* 17/07/08 v2.0.3
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
* 文件说明(中文) :
*   本文件实现「ICS(Integrated Current Sensing, 集成电流采样)+ SVPWM(空间矢量
*   脉宽调制)」控制模块，仅在 ICS_SENSORS 宏被定义时编译。
*   ICS 采样的基本思想: 功率驱动芯片内部已集成电流检测电路，直接输出与 A/B 相
*   (一般都对应下桥臂)电流成正比的模拟电压；MCU 通过 ADC1 的注入通道同时采样
*   A 相(以及母线电压)，ADC2 同时采样 B 相(以及温度)，再经偏置相减得到相电流。
*   相比单电阻采样，ICS 无需在占空比上做移相/矢量修正，任何占空比下均可直接
*   观测两相电流，实现更简单。
*   本模块与 TIM1 的配合:
*     - TIM1 工作在中央对齐 PWM 模式(中心对齐 1)，产生三相互补 PWM 并带死区；
*     - TIM1 的 TRGO 选择为 Update 事件(下溢)，作为 ADC 注入转换的触发源；
*     - ADC 注入转换完成后产生 JEOC 中断，在 SVPWMEOCEvent() 中软件启动
*       软件触发的注入转换以读取电流(此处为兼容上层框架的写法)。
*******************************************************************************/

#include "STM32F10x_MCconf.h"

#ifdef ICS_SENSORS

/* Includes-------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_svpwm_ics.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define NB_CONVERSIONS 16   // 校准阶段连续采样次数(取平均以降低偏置噪声)

#define SQRT_3		1.732051                          // √3 的浮点常量(用于 Clark/矢量长度换算)
#define T		(PWM_PERIOD * 4)                  // 时间基准 T = PWM 周期 × 4(用于占空比定标, 单位: 计数)
#define T_SQRT3         (u16)(T * SQRT_3)             // T×√3, 预计算避免运行时乘法(用于 wUAlpha 定标)

#define SECTOR_1	(u32)1   // 扇区 1
#define SECTOR_2	(u32)2   // 扇区 2
#define SECTOR_3	(u32)3   // 扇区 3
#define SECTOR_4	(u32)4   // 扇区 4
#define SECTOR_5	(u32)5   // 扇区 5
#define SECTOR_6	(u32)6   // 扇区 6

/* 注入通道号需左移到 ADC 注入序列寄存器(JSQR)的对应位; A/B 相位于高 5 位的第 10~14 位 */
#define PHASE_A_MSK       (u32)((u32)(PHASE_A_ADC_CHANNEL) << 10)   // A 相通道号左移 10 位后的掩码
#define PHASE_B_MSK       (u32)((u32)(PHASE_B_ADC_CHANNEL) << 10)   // B 相通道号左移 10 位后的掩码

/* 温度/母线电压位于注入序列寄存器的高 5 位(第 15~19 位), 故左移 15 */
#define TEMP_FDBK_MSK     (u32)((u32)(TEMP_FDBK_CHANNEL) <<15)      // 温度通道号左移 15 位
#define BUS_VOLT_FDBK_MSK (u32)((u32)(BUS_VOLT_FDBK_CHANNEL) <<15)  // 母线电压通道号左移 15 位
#define SEQUENCE_LENGHT    0x00100000                              // JSQR 中序列长度字段的置位掩码(长度=2)

#define ADC_PRE_EMPTION_PRIORITY 1   // ADC1_2 中断抢占优先级
#define ADC_SUB_PRIORITY 1           // ADC1_2 中断子优先级

#define BRK_PRE_EMPTION_PRIORITY 0   // TIM1 刹车(Break)中断抢占优先级(最高)
#define BRK_SUB_PRIORITY 1           // TIM1 刹车中断子优先级

#define LOW_SIDE_POLARITY  TIM_OCIdleState_Reset   // 空闲/刹车时下桥臂输出的电平(复位=低电平关断)

#define ADC_RIGHT_ALIGNMENT 3   // ADC 结果右对齐时需右移的位数说明(实际取用左对齐值整体右移处理)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static u16 hPhaseAOffset;   // A 相电流采样通道的零电流偏置(ADC 原始码的累加平均, 无电流时对应值)
static u16 hPhaseBOffset;   // B 相电流采样通道的零电流偏置
    
/* Private function prototypes -----------------------------------------------*/

void SVPWM_IcsInjectedConvConfig(void);   // 配置 ADC1 注入通道: A 相 + 母线电压, 触发源为 TIM1 TRGO

/*******************************************************************************
* Function Name  : SVPWM_IcsInit
* Description    : It initializes PWM and ADC peripherals
* 功能说明(中文) : 上电/启动时调用一次, 初始化 TIM1 三相 PWM 输出(含死区/刹车)、
*                  ADC1/ADC2 注入转换通道(ICS 电流采样 + 温度/母线电压采样)以及
*                  ADC1_2、TIM1_BRK 中断。完成后上层即可调用占空比计算与电流读取。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 依赖 MC_pwm_ics_prm.h 的参数(PWM_PERIOD/DEADTIME/通道定义等);
*                  内部会调用 SVPWM_IcsCurrentReadingCalibration() 做零电流偏置校准;
*                  TIM1 采用中央对齐模式1, ADCCLK = PCLK2/6 = 12MHz。
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_IcsInit(void)
{ 
  ADC_InitTypeDef ADC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM1_TimeBaseStructure;
  TIM_OCInitTypeDef TIM1_OCInitStructure;
  TIM_BDTRInitTypeDef TIM1_BDTRInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

  /* ADC1, ADC2, DMA, GPIO, TIM1 clocks enabling -----------------------------*/
  
  /* ADCCLK = PCLK2/6 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);   // ADC 时钟 = 72MHz/6 = 12MHz(不超过 14MHz 上限)

  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);   // 使能 DMA1 时钟(ICS 版本未实际使用 DMA)

  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);   // 使能 ADC1 时钟(采样 A 相/母线电压)

  /* Enable ADC2 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);   // 使能 ADC2 时钟(采样 B 相/温度)
  
  /* Enable GPIOA-GPIOE clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | 
           RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                                                  RCC_APB2Periph_GPIOE, ENABLE);
   
  /* Enable TIM1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  
  /* ADC1, ADC2, PWM pins configurations -------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  /****** Configure phase A ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = PHASE_A_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(PHASE_A_GPIO_PORT, &GPIO_InitStructure);
  /****** Configure phase B ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = PHASE_B_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(PHASE_B_GPIO_PORT, &GPIO_InitStructure);
  GPIO_StructInit(&GPIO_InitStructure);  
  /****** Configure temperature reading ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = TEMP_FDBK_CHANNEL_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(TEMP_FDBK_CHANNEL_GPIO_PORT, &GPIO_InitStructure);
  /****** Configure bus voltage reading ADC channel GPIO as analog input ****/
  GPIO_InitStructure.GPIO_Pin = BUS_VOLT_FDBK_CHANNEL_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(BUS_VOLT_FDBK_CHANNEL_GPIO_PORT, &GPIO_InitStructure);
    
  /* TIM1 Peripheral Configuration -------------------------------------------*/
  /* TIM1 Registers reset */
  TIM_DeInit(TIM1);
  TIM_TimeBaseStructInit(&TIM1_TimeBaseStructure);
  /* Time Base configuration */
  TIM1_TimeBaseStructure.TIM_Prescaler = PWM_PRSC;                       // 预分频(0 表示不分频)
  TIM1_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1; // 中央对齐模式1(三相互补 PWM)
  TIM1_TimeBaseStructure.TIM_Period = PWM_PERIOD;                        // 自动重装值, 决定 PWM 频率
  TIM1_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;               // 时钟分频(用于数字滤波)
  TIM1_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;               // 重复计数: 决定更新事件频率
  TIM_TimeBaseInit(TIM1, &TIM1_TimeBaseStructure);

  TIM_OCStructInit(&TIM1_OCInitStructure);
  /* Channel 1, 2,3 and 4 Configuration in PWM mode */
  TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   // PWM 模式1(计数值<CCRx 时输出有效电平) 
  TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;                  
  TIM1_OCInitStructure.TIM_Pulse = 0x505; //dummy value
  TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         
  TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM1_OCInitStructure.TIM_OCNIdleState = LOW_SIDE_POLARITY;          
  
  TIM_OC1Init(TIM1, &TIM1_OCInitStructure); 

  TIM1_OCInitStructure.TIM_Pulse = 0x505; //dummy value
  TIM_OC2Init(TIM1, &TIM1_OCInitStructure);

  TIM1_OCInitStructure.TIM_Pulse = 0x505; //dummy value
  TIM_OC3Init(TIM1, &TIM1_OCInitStructure);
  
  /*Timer1 alternate function full remapping*/  
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);
  
  GPIO_StructInit(&GPIO_InitStructure);  

  /* GPIOE Configuration: Channel 1, 1N, 2, 2N, 3 and 3N Output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
  
  /* Lock GPIOE Pin9 to Pin 13 */
  GPIO_PinLockConfig(GPIOE, GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13);
  GPIO_StructInit(&GPIO_InitStructure);
  
  /* GPIOE Configuration: BKIN pin */   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Automatic Output enable, Break, dead time and lock configuration*/
  TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;              // 运行模式下 OSSR, 空闲时互补输出保活
  TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;              // 空闲模式下 OSSI, 输出保持
  TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;                   // 锁定级别1(保护 BDTR 部分位)
  TIM1_BDTRInitStructure.TIM_DeadTime = DEADTIME;                          // 死区时间(计数单位), 防止上下桥臂直通
  TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Enable;                     // 使能刹车输入(过流保护)
  TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;        // 刹车输入低电平有效
  TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable; // 关闭自动恢复输出

  TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);

  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);   // TRGO 选为 Update(下溢), 作为 ADC 注入转换触发源
  
  TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
  TIM_ITConfig(TIM1, TIM_IT_Break, ENABLE);
  
  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);
  
  /* ADC1 registers reset ----------------------------------------------------*/
  ADC_DeInit(ADC1);
  /* ADC2 registers reset ----------------------------------------------------*/
  ADC_DeInit(ADC2);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Mode = ADC_Mode_InjecSimult;              // 注入同步: ADC1/ADC2 由同一次触发同时开始注入转换
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;                    // 扫描模式(注入序列含多个通道)
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;             // 单次转换(由触发启动)
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // 规则组不使用外部触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;           // 数据左对齐(高 12 位有效)
  ADC_InitStructure.ADC_NbrOfChannel = 1;                         // 规则通道数 1
  ADC_Init(ADC1, &ADC_InitStructure);
   
  /* ADC2 Configuration ------------------------------------------------------*/
  ADC_StructInit(&ADC_InitStructure);  
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC2, &ADC_InitStructure);
  
  // Start calibration of ADC1
  ADC_StartCalibration(ADC1);
  // Start calibration of ADC2
  ADC_StartCalibration(ADC2);
  
  // Wait for the end of ADCs calibration 
  while (ADC_GetCalibrationStatus(ADC1) & ADC_GetCalibrationStatus(ADC2))
  {
  }
  ADC_InjectedSequencerLengthConfig(ADC1,2);   // ADC1 注入序列长度 = 2 (A 相电流 + 母线电压)
  SVPWM_IcsCurrentReadingCalibration();        // 采集 A/B 相零电流偏置并切换到正常注入配置
    
  /* ADC2 Injected conversions configuration */ 
  ADC_InjectedSequencerLengthConfig(ADC2,2);   // ADC2 注入序列长度 = 2
  ADC_InjectedChannelConfig(ADC2, PHASE_B_ADC_CHANNEL, 1, 
                                                      SAMPLING_TIME_CK);   // 序列1: B 相电流
  ADC_InjectedChannelConfig(ADC2, TEMP_FDBK_CHANNEL, 2,
                                                      SAMPLING_TIME_CK);   // 序列2: 温度
  
  ADC_ExternalTrigInjectedConvCmd(ADC2,ENABLE);   // 允许 ADC2 注入组被外部触发(TRGO)
  
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  NVIC_StructInit(&NVIC_InitStructure);
  /* Enable the ADC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQChannel;   // ADC1/ADC2 共用中断(处理注入转换结束)
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ADC_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
    
  /* Enable the TIM1 BRK Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BRK_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = BRK_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

} 

/*******************************************************************************
* Function Name  : SVPWM_IcsCurrentReadingCalibration
* Description    : Store zero current converted values for current reading 
                   network offset compensation in case of Ics 
* 功能说明(中文) : 采集 A/B 相电流通道在「零电流」下的 ADC 值作为偏置(offset)。
*                  上电/初始化时电机尚未通电, 此时测得的即为零电流基准。
*                  连续转换 NB_CONVERSIONS 次并累加, 结果存放在 hPhaseAOffset/hPhaseBOffset。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 校准期间改用软件触发(ADC_ExternalTrigInjecConv_None);
*                  每次结果右移 ADC_RIGHT_ALIGNMENT(3) 位后累加, 累加 16 次
*                  约等于 2×平均零电流码, 与读数公式 (ADC值<<1)-偏置 量纲一致;
*                  结束时调用 SVPWM_IcsInjectedConvConfig() 切换回正常采样配置。
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

void SVPWM_IcsCurrentReadingCalibration(void)
{
 static u8 bIndex;   // 循环计数(静态, 只在本函数使用)
   
  /* ADC1 Injected group of conversions end interrupt disabling */
  ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);   // 校准期间关闭注入转换结束中断, 避免误触发
  
  hPhaseAOffset=0;   // A 相偏置累加器清零
  hPhaseBOffset=0;   // B 相偏置累加器清零
   
  /* ADC1 Injected conversions trigger is given by software and enabled */ 
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);   // 改为软件触发
  ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE); 
  
  /* ADC1 Injected conversions configuration */   
  ADC_InjectedChannelConfig(ADC1, PHASE_A_ADC_CHANNEL,1,SAMPLING_TIME_CK);   // 序列1: A 相电流
  ADC_InjectedChannelConfig(ADC1, PHASE_B_ADC_CHANNEL,2,SAMPLING_TIME_CK);   // 序列2: B 相电流
   
  /* Clear the ADC1 JEOC pending flag */
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);   
  ADC_SoftwareStartInjectedConvCmd(ADC1,ENABLE);   // 软件启动一次注入转换
  
  /* ADC Channel used for current reading are read 
     in order to get zero currents ADC values*/ 
  for(bIndex=NB_CONVERSIONS; bIndex !=0; bIndex--)
  {
    while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC)) { }   // 等待本次注入转换完成(JEOC 置位)
  
    hPhaseAOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1)
                                                         >>ADC_RIGHT_ALIGNMENT);   // 累加 A 相零电流码
    hPhaseBOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2)
                                                         >>ADC_RIGHT_ALIGNMENT);   // 累加 B 相零电流码
    /* Clear the ADC1 JEOC pending flag */
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC); 	
    ADC_SoftwareStartInjectedConvCmd(ADC1,ENABLE);   // 软件启动下一次注入转换
  }
  
  SVPWM_IcsInjectedConvConfig();  
}


/*******************************************************************************
* Function Name  : SVPWM_IcsInjectedConvConfig
* Description    : This function configure ADC1 for ICS current 
*                  reading and temperature and voltage feedbcak after a 
*                  calibration of the utilized ADC Channels for current reading
* 功能说明(中文) : 校准结束后把 ADC1 注入通道恢复为正常工作配置:
*                  序列1 = A 相电流, 序列2 = 母线电压; 触发源改为 TIM1 TRGO(更新事件),
*                  并打开 JEOC 中断以便转换结束时读取结果。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 与 ADC2(序列1 = B 相, 序列2 = 温度)配合, 每周期同时采样;
*                  母线电压结果在 SVPWMEOCEvent() 中读取。
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_IcsInjectedConvConfig(void)
{
  /* ADC1 Injected conversions configuration */ 
  ADC_InjectedChannelConfig(ADC1, PHASE_A_ADC_CHANNEL, 1, 
                                                      SAMPLING_TIME_CK);   // 序列1: A 相电流
  ADC_InjectedChannelConfig(ADC1, BUS_VOLT_FDBK_CHANNEL, 
                                                   2, SAMPLING_TIME_CK);   // 序列2: 母线电压
  
  /* ADC1 Injected conversions trigger is TIM1 TRGO */ 
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);   // 触发源 = TIM1 TRGO(更新事件)
  
  /* ADC1 Injected group of conversions end interrupt enabling */
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);   // 使能注入转换结束中断
}

/*******************************************************************************
* Function Name  : SVPWM_IcsPhaseCurrentValues
* Description    : This function computes current values of Phase A and Phase B 
*                 in q1.15 format starting from values acquired from the A/D 
*                 Converter peripheral.
* 功能说明(中文) : 读取 ADC1(注入寄存器 JDR1=A 相)与 ADC2(JDR1=B 相)结果, 减去零电流
*                  偏置并做饱和限幅, 得到 q1.15 定标的相电流 Ia、Ib(每个电流环周期调用)。
* 参数(中文)     : 无
* 返回(中文)     : Curr_Components: qI_Component1=Ia(q1.15), qI_Component2=Ib(q1.15);
*                  范围 [-32768, 32767], 对应约 [-1, 1] 标幺值。
* 备注(中文)     : 换算公式: 电流码 = (ADC原始值<<1) - 零电流偏置; "<<1" 把 ADC 值放大
*                  到与偏置同量纲; 实际安培值 = 电流码/32767 × 通道满量程。
* Input          : None
* Output         : Stat_Curr_a_b
* Return         : None
*******************************************************************************/
Curr_Components SVPWM_IcsGetPhaseCurrentValues(void)
{
  Curr_Components Local_Stator_Currents;
  s32 wAux;

      
 // Ia = (hPhaseAOffset)-(PHASE_A_ADC_CHANNEL vale)  
  wAux = ((ADC1->JDR1)<<1)-(s32)(hPhaseAOffset);          // 读 A 相注入结果(<<1 放大), 减去零电流偏置
 //Saturation of Ia 
  if (wAux < S16_MIN)
  {
    Local_Stator_Currents.qI_Component1= S16_MIN;
  }  
  else  if (wAux > S16_MAX)
        { 
          Local_Stator_Currents.qI_Component1= S16_MAX;
        }
        else
        {
          Local_Stator_Currents.qI_Component1= wAux;
        }
                     
 // Ib = (hPhaseBOffset)-(PHASE_B_ADC_CHANNEL value)
  wAux = ((ADC2->JDR1)<<1)-(s32)(hPhaseBOffset);   // 读 B 相注入结果(<<1 放大), 减去零电流偏置
 // Saturation of Ib
  if (wAux < S16_MIN)
  {
    Local_Stator_Currents.qI_Component2= S16_MIN;
  }  
  else  if (wAux > S16_MAX)
        { 
          Local_Stator_Currents.qI_Component2= S16_MAX;
        }
        else
        {
          Local_Stator_Currents.qI_Component2= wAux;
        }
  
  return(Local_Stator_Currents);   // 返回 Ia(分量1)/Ib(分量2), 均为 q1.15
}

/*******************************************************************************
* Function Name  : SVPWM_IcsCalcDutyCycles
* Description    : Computes duty cycle values corresponding to the input value
		   and configures 
* 功能说明(中文) : SVPWM 核心: 由 α/β 电压指令做扇区判断并计算三相占空比, 直接写入
*                  TIM1 的 CCR1~CCR3(每个电流环周期调用一次)。
* 参数(中文)     : Stat_Volt_Input - α/β 电压分量 Volt_Components(q1.15),
*                  qV_Component1=Vα, qV_Component2=Vβ, 范围各 [-32768, 32767]。
* 返回(中文)     : 无
* 备注(中文)     : wX/wY/wZ 为扇区判据; T/8 是中心对齐下的 50% 占空比基准偏移;
*                  结果除以 131072(=2^17)完成定标; ICS 采样无需单电阻那样的移相修正。
* Input          : Stat_Volt_alfa_beta
* Output         : None
* Return         : None
*******************************************************************************/

void SVPWM_IcsCalcDutyCycles (Volt_Components Stat_Volt_Input)
{
   u8 bSector;                                                    // 当前电压矢量所属扇区(1~6)
   s32 wX, wY, wZ, wUAlpha, wUBeta;                               // 扇区判定与占空比计算中间变量(32 位防溢出)
   u16  hTimePhA=0, hTimePhB=0, hTimePhC=0;                       // A/B/C 三相占空比比较值(单位: TIM1 计数)
    
   wUAlpha = Stat_Volt_Input.qV_Component1 * T_SQRT3 ;            // wUAlpha = Vα·T·√3
   wUBeta = -(Stat_Volt_Input.qV_Component2 * T);                 // wUBeta = -Vβ·T

   wX = wUBeta;                                                   // 三个中间量 wX/wY/wZ
   wY = (wUBeta + wUAlpha)/2;                                     // 由 wUAlpha/wUBeta 线性组合得到
   wZ = (wUBeta - wUAlpha)/2;                                     // 用于判断矢量落在哪个 60° 扇区

  // Sector calculation from wX, wY, wZ
   if (wY<0)
   {
      if (wZ<0)
      {
        bSector = SECTOR_5;
      }
      else // wZ >= 0
        if (wX<=0)
        {
          bSector = SECTOR_4;
        }
        else // wX > 0
        {
          bSector = SECTOR_3;
        }
   }
   else // wY > 0
   {
     if (wZ>=0)
     {
       bSector = SECTOR_2;
     }
     else // wZ < 0
       if (wX<=0)
       {  
         bSector = SECTOR_6;
       }
       else // wX > 0
       {
         bSector = SECTOR_1;
       }
    }
   
   /* Duty cycles computation */
  
  switch(bSector)
  {  
    case SECTOR_1:
    case SECTOR_4:
                hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);   // A 相占空比: 50% 基准 + 相电压分量(÷2^17 定标)
		hTimePhB = hTimePhA + wZ/131072;                    // B 相相对 A 相的偏移
		hTimePhC = hTimePhB - wX/131072;                    // C 相相对 B 相的偏移
                break;
    case SECTOR_2:
    case SECTOR_5:  
                hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);   // 扇区 2/5 的 A 相占空比
        	hTimePhB = hTimePhA + wZ/131072;                    // B 相
		hTimePhC = hTimePhA - wY/131072;                    // C 相
                break;

    case SECTOR_3:
    case SECTOR_6:
                hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);   // 扇区 3/6 的 A 相占空比
		hTimePhC = hTimePhA - wY/131072;                    // C 相
		hTimePhB = hTimePhC + wX/131072;                    // B 相
                break;
    default:
		break;
   }
  
  /* Load compare registers values */
   
  TIM1->CCR1 = hTimePhA;   // 写入 A 相占空比比较值
  TIM1->CCR2 = hTimePhB;   // 写入 B 相占空比比较值
  TIM1->CCR3 = hTimePhC;   // 写入 C 相占空比比较值
}

/*******************************************************************************
* Function Name  : SVPWMEOCEvent
* Description    :  Routine to be performed inside the end of conversion ISR
* 功能说明(中文) : ADC 注入转换结束(JEOC)中断内调用, 读取 ADC2 的温度与 ADC1 的母线电压
*                  注入通道(序列2)结果, 更新全局变量 h_ADCTemp / h_ADCBusvolt。
* 参数(中文)     : 无
* 返回(中文)     : 恒返回 1(u8), 供上层判断 EOC 事件已处理。
* 备注(中文)     : 更新全局变量 h_ADCTemp / h_ADCBusvolt, 供过温/过压等保护逻辑使用。
* Input           : None
* Output          : None
* Return          : None
*******************************************************************************/
u8 SVPWMEOCEvent(void)
{
  // Store the Bus Voltage and temperature sampled values
  h_ADCTemp = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_2);      // 读取温度(ADC2 注入序列2)
  h_ADCBusvolt = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2);   // 读取母线电压(ADC1 注入序列2)
  return ((u8)(1));   // 固定返回 1
}
#endif //ICS_SENSORS
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/  
