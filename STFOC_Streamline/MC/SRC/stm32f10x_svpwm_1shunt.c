/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : STM32x_svpwm_1shunt.c
* Author             : IMS Systems Lab
* Date First Issued  : Mar/08
* Description        : 1 shunt resistor current reading module
********************************************************************************
* History:
* 29/05/08 v2.0
* 02/07/08 v2.0.1
* 03/07/08 v2.0.2
* 11/07/08 v2.0.3
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
*   本文件实现「单电阻(1-shunt)电流采样 + SVPWM(空间矢量脉宽调制)」控制模块，
*   仅在 SINGLE_SHUNT 宏被定义时编译。
*   单电阻采样的基本原理: 三相逆变器只在直流母线回路(下桥臂汇流到地)串一个采样
*   电阻。母线电流并非总是等于某一相电流，只有在某个有效矢量(某相下桥臂单独导通)
*   作用期间，母线电流才等于对应相电流。因此需要在一个 PWM 周期内的两个不同时刻
*   分别采样: 落在某个基本矢量区间时读到的母线电流即为对应相电流，从而重构出
*   两相电流(第三相由 ia+ib+ic=0 得到)。
*   由于矢量作用时间很短，当占空比接近 0% 或 100%(即进入"不可观测区/边界区")时，
*   有效矢量持续时间不足以完成 ADC 采样。此时本模块通过对某相占空比做移相(畸变,
*   INVERT_x)、或按 BOUNDARY_1/2/3 边界策略修正矢量，从而制造出足够的可观测窗口。
*   TIM1 关键配置:
*     - 中央对齐模式3, 三相互补 PWM, 带死区(DEADTIME);
*     - CH1/CH2/CH3 在 PWM 与 Toggle(翻转)之间切换(配合 DMA)实现移相;
*     - CH4 在 OC 比较模式下产生两次 ADC 触发(CC4 事件), 触发 ADC1 注入转换;
*     - DMAR + DMA 突发(burst)在更新事件时一次性写入 CCR1~CCR4。
*******************************************************************************/

#include "STM32F10x_MCconf.h"

#ifdef SINGLE_SHUNT

/* Includes-------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_svpwm_1shunt.h"
#include "MC_Globals.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define NB_CONVERSIONS 16   // 校准阶段连续采样次数(取平均以降低偏置噪声)

#define SQRT_3		1.732051                          // √3 的浮点常量
#define T		(PWM_PERIOD * 4)                  // 时间基准 T = PWM 周期 × 4(占空比定标用, 单位: 计数)
#define T_SQRT3         (u16)(T * SQRT_3)             // T×√3, 预计算避免运行时乘法

#define SECTOR_1	(u32)1   // 扇区 1
#define SECTOR_2	(u32)2   // 扇区 2
#define SECTOR_3	(u32)3   // 扇区 3
#define SECTOR_4	(u32)4   // 扇区 4
#define SECTOR_5	(u32)5   // 扇区 5
#define SECTOR_6	(u32)6   // 扇区 6

/* 定子电压矢量所处区域(用于判断采样窗口是否足够, 决定是否需要移相) */
#define REGULAR         ((u8)0)   // 规则区: 两个可观测窗口都足够, 无需移相
#define BOUNDARY_1      ((u8)1)  // Two small, one big    // 边界1: 两个小占空比+一个大占空比
#define BOUNDARY_2      ((u8)2)  // Two big, one small    // 边界2: 两个大占空比+一个小占空比
#define BOUNDARY_3      ((u8)3)  // Three equal           // 边界3: 三者接近相等(窗口都不足)

#define PHASE_B_ADC_CHANNEL     ADC_Channel_12   // 单电阻采样所用的 ADC 通道(物理上接入采样电阻两端)

#define ADC_PRE_EMPTION_PRIORITY 1   // ADC1_2 中断抢占优先级
#define ADC_SUB_PRIORITY 0           // ADC1_2 中断子优先级

#define BRK_PRE_EMPTION_PRIORITY 0   // TIM1 刹车(Break)中断抢占优先级(最高)
#define BRK_SUB_PRIORITY 0           // TIM1 刹车中断子优先级

#define TIM1_UP_PRE_EMPTION_PRIORITY 0   // TIM1 更新(Update)中断抢占优先级(最高)
#define TIM1_UP_SUB_PRIORITY 0           // TIM1 更新中断子优先级

#define LOW_SIDE_POLARITY  TIM_OCIdleState_Reset   // 空闲/刹车时下桥臂输出的电平(复位=低电平关断)

// Direct address of the registers used by DMA
#define TIM1_CCR1_Address   0x40012C34   // TIM1_CCR1 寄存器地址(DMA 外设地址)
#define TIM1_CCR2_Address   0x40012C38   // TIM1_CCR2 寄存器地址
#define TIM1_CCR3_Address   0x40012C3C   // TIM1_CCR3 寄存器地址
#define TIM1_CCR4_Address   0x40012C40   // TIM1_CCR4 寄存器地址(采样触发比较寄存器)
#define TIM1_DMAR_Address   0x40012C4C   // TIM1_DMAR 寄存器地址(DMA 突发传输目标)

#define CCMR1_OC1PE_BB    0x4225830C   // CCMR1 中 OC1PE 位的位带别名地址(使能/关闭 CH1 预装载)
#define CCMR1_OC2PE_BB    0x4225832C   // CCMR1 中 OC2PE 位的位带别名地址
#define CCMR2_OC3PE_BB    0x4225838C   // CCMR2 中 OC3PE 位的位带别名地址

/* CCMR 中通道输出模式配置位域: OCxM 选择 PWM1(0b110=0x60)或 Toggle(0b011=0x30) */
#define CH1NORMAL 0x0060   // CH1 配置为 PWM 模式1(OC1M=0b110)
#define CH2NORMAL 0x6000   // CH2 配置为 PWM 模式1(OC2M=0b110)
#define CH3NORMAL 0x0060   // CH3 配置为 PWM 模式1(OC3M=0b110)
#define CH4NORMAL 0x7000   // CH4 配置为 PWM 模式2(OC4M=0b111, 用于比较触发)
#define CH1TOGGLE 0x0030   // CH1 配置为 Toggle 翻转模式(OC1M=0b011, 移相用)
#define CH2TOGGLE 0x3000   // CH2 配置为 Toggle 翻转模式
#define CH3TOGGLE 0x0030   // CH3 配置为 Toggle 翻转模式
#define CH4TOGGLE 0x3000   // CH4 配置为 Toggle 翻转模式(OC4M=0b011)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u8  bSector;                  // 当前电压矢量所属扇区(1~6), 由 SVPWM_1ShuntCalcDutyCycles() 计算
u16 hPhaseOffset=0;           // 单电阻零电流时 ADC 值的累加平均(用于电流读数零点补偿)

u8 bInverted_pwm=INVERT_NONE,bInverted_pwm_new=INVERT_NONE;   // 上一周期/本周期的移相相选择(INVERT_x)
u8  bStatorFluxPos, bStatorFluxPosOld;                        // 本周期/上一周期定子矢量区域(REGULAR/BOUNDARY_x)
DUTYVALUESTYPE dvDutyValues;                                  // 三相占空比与两个采样触发点(单位: TIM1 计数)
CURRENTSAMPLEDTYPE csCurrentSampled;                         // 两个采样点分别对应哪一相/极性
u8 bError=0;                                                  // 采样错误标志(窗口不足时置位)

s16 hCurrAOld,hCurrBOld,hCurrCOld;   // 上一周期三相电流(q1.15), 用于边界区补值
s16 hDeltaA,hDeltaB,hDeltaC;         // 边界2 电流畸变补偿量(当前补偿用, CURRENT_COMPENSATION)
u8 bReadDelta;                       // 是否本周期进入边界2以计算补偿量

u16 hPreloadCCMR1Disable;   // CCMR1 的"冻结/无预装载"基准值(移相切换时使用)
u16 hPreloadCCMR1Set;       // CCMR1 的"目标模式"值(PWM 或 Toggle)
u16 hPreloadCCMR2Disable;   // CCMR2 的"冻结/无预装载"基准值
u16 hPreloadCCMR2Set;       // CCMR2 的"目标模式"值
   
u16 hCCDmaBuffCh1[4];   // CH1(CCR1) 的 DMA 突发缓冲: 4 个值在一个周期内依次写入
u16 hCCDmaBuffCh2[4];   // CH2(CCR2) 的 DMA 突发缓冲
u16 hCCDmaBuffCh3[4];   // CH3(CCR3) 的 DMA 突发缓冲
u16 hCCDmaBuffCh4[4];   // CH4(CCR4) 的 DMA 突发缓冲(存放两个采样触发时刻)

u16 hCCRBuff[4];   // 更新事件时经 DMAR 突发写入 CCR1~CCR4 的值

u8 bStBd3 = 0;     // 边界3 下在 A/B 两相之间交替移相的标志
u8 bDistEnab = 0;  // 是否启用单电阻移相(启动电流采样时置1, 停止时清0)

/* Private function prototypes -----------------------------------------------*/

void SVPWM_InjectedConvConfig(void);   // 校准后配置 ADC1 单电阻采样与 ADC2 温度/母线电压采样, 并设置 DMA/变量初值

/*******************************************************************************
* Function Name  : SVPWM_1ShuntInit
* Description    : It initializes PWM and ADC peripherals
* 功能说明(中文) : 上电/启动时调用一次, 初始化单电阻采样所依赖的全部外设:
*                  TIM1 三相互补 PWM(带死区/刹车)、ADC1/ADC2 注入转换、
*                  4 个 DMA 通道(CH2/3/6 用于 CH1/2/3 的突发写, CH4 用于采样触发,
*                  CH5 用于更新事件突发写 CCR1~CCR4)以及相关中断。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 依赖 MC_pwm_1shunt_prm.h 的参数(PWM_PERIOD/DEADTIME/采样时间等);
*                  内部调用 SVPWM_1ShuntCurrentReadingCalibration() 做零电流偏置校准;
*                  TIM1 采用中央对齐模式3, ADCCLK = PCLK2/6 = 12MHz。
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_1ShuntInit(void)
{ 
  ADC_InitTypeDef ADC_InitStructure;
  TIM_TimeBaseInitTypeDef TIM1_TimeBaseStructure;
  TIM_OCInitTypeDef TIM1_OCInitStructure;
  TIM_BDTRInitTypeDef TIM1_BDTRInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  /* ADC1, ADC2, DMA, GPIO, TIM1 clocks enabling -----------------------------*/
  
  /* ADCCLK = PCLK2/6 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);

  /* Enable DMA clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  
  /* Enable GPIOA, GPIOC, GPIOE, AFIO clocks */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOE, ENABLE);
  /* Enable ADC1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

  /* Enable ADC2 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); 
   
  /* Enable TIM1 clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
     
  /* ADC1, ADC2, PWM pins configurations -------------------------------------*/
  GPIO_StructInit(&GPIO_InitStructure);
  /****** Configure PC.00,01,2,3,4 (ADC Channels [10..14]) as analog input ****/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
     
  GPIO_StructInit(&GPIO_InitStructure);
  /****** Configure PA.03 (ADC Channels [3]) as analog input ******/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  // After reset value of DMA buffers
  hCCDmaBuffCh1[0] = PWM_PERIOD-HTMIN;
  hCCDmaBuffCh1[1] = PWM_PERIOD-HTMIN;
  hCCDmaBuffCh1[2] = PWM_PERIOD >> 1;
  hCCDmaBuffCh1[3] = PWM_PERIOD >> 1;
  ;
  hCCDmaBuffCh2[0] = PWM_PERIOD-HTMIN;
  hCCDmaBuffCh2[1] = PWM_PERIOD-HTMIN;
  hCCDmaBuffCh2[2] = PWM_PERIOD >> 1;
  hCCDmaBuffCh2[3] = PWM_PERIOD >> 1;
  
  hCCDmaBuffCh3[0] = PWM_PERIOD-HTMIN;
  hCCDmaBuffCh3[1] = PWM_PERIOD-HTMIN;
  hCCDmaBuffCh3[2] = PWM_PERIOD >> 1;
  hCCDmaBuffCh3[3] = PWM_PERIOD >> 1;
  
  // Default Update DMA buffer Ch 1,2,3,4 after reset
  hCCRBuff[0] = PWM_PERIOD >> 1;
  hCCRBuff[1] = PWM_PERIOD >> 1;
  hCCRBuff[2] = PWM_PERIOD >> 1;
  hCCRBuff[3] = (PWM_PERIOD >> 1) - TBEFORE;
  
  // After reset value of dvDutyValues
  dvDutyValues.hTimePhA = PWM_PERIOD >> 1;
  dvDutyValues.hTimePhB = PWM_PERIOD >> 1;
  dvDutyValues.hTimePhC = PWM_PERIOD >> 1;
  
  /* TIM1 Channel 1 toggle mode */
  /* DMA Channel2 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel2);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)TIM1_CCR1_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(hCCDmaBuffCh1);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel2, &DMA_InitStructure);
  /* Enable DMA Channel2 */
  //DMA_Cmd(DMA_Channel2, ENABLE);
  DMA_Cmd(DMA1_Channel2, DISABLE);
  
  /* TIM1 Channel 2 toggle mode */
  /* DMA Channel3 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel3);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)TIM1_CCR2_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(hCCDmaBuffCh2);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel3, &DMA_InitStructure);
  /* Enable DMA Channel3 */
  //DMA_Cmd(DMA_Channel3, ENABLE);
  DMA_Cmd(DMA1_Channel3, DISABLE);

  /* TIM1 Channel 3 toggle mode */
  /* DMA Channel6 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel6);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)TIM1_CCR3_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(hCCDmaBuffCh3);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel6, &DMA_InitStructure);
  /* Enable DMA Channel6 */
  DMA_Cmd(DMA1_Channel6, DISABLE);
  
  /* TIM1 Channel 4 PWM2 or toggle mode */
  /* DMA channel4 configuration */
  DMA_DeInit(DMA1_Channel4);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)TIM1_CCR4_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(hCCDmaBuffCh4);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  
  DMA_Cmd(DMA1_Channel4, ENABLE);
  
  /* DMA TIM1 update configuration */
  DMA_DeInit(DMA1_Channel5);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)TIM1_DMAR_Address; 
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)(hCCRBuff);
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel5, &DMA_InitStructure);
  
  DMA_Cmd(DMA1_Channel5, ENABLE);
   
  /* TIM1 Peripheral Configuration -------------------------------------------*/
  /* TIM1 Registers reset */
  TIM_DeInit(TIM1);
  TIM_TimeBaseStructInit(&TIM1_TimeBaseStructure);
  /* Time Base configuration */
  TIM1_TimeBaseStructure.TIM_Prescaler = 0x0;                              // 预分频(0 表示不分频)
  TIM1_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned3; // 中央对齐模式3(单电阻方案采用)
  TIM1_TimeBaseStructure.TIM_Period = PWM_PERIOD;                          // 自动重装值, 决定 PWM 频率
  TIM1_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;                 // 时钟分频(用于数字滤波)
  
  // Initial condition is REP=0 to set the UPDATE only on the underflow
  TIM1_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;   // 重复计数: 决定更新事件频率(下溢触发)
  TIM_TimeBaseInit(TIM1, &TIM1_TimeBaseStructure);
  
  TIM_OCStructInit(&TIM1_OCInitStructure);
  /* Channel 1, 2,3 in PWM mode */
  TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
  TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;                  
  TIM1_OCInitStructure.TIM_Pulse = PWM_PERIOD >> 1; //dummy value
  TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         
  TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM1_OCInitStructure.TIM_OCNIdleState = LOW_SIDE_POLARITY;          
  
  TIM_OC1Init(TIM1, &TIM1_OCInitStructure); 
  TIM_OC3Init(TIM1, &TIM1_OCInitStructure);
  TIM_OC2Init(TIM1, &TIM1_OCInitStructure);

  /*Timer1 alternate function full remapping*/
  GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE); 
  
  GPIO_StructInit(&GPIO_InitStructure);
  /* GPIOE Configuration: Channel 1, 1N, 2, 2N, 3, 3N and 4 Output */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                                GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
  
  /* Lock GPIOE Pin9 and Pin11 Pin 13 (High sides) */
  GPIO_PinLockConfig(GPIOE, GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13);

  GPIO_StructInit(&GPIO_InitStructure);
  /* GPIOE Configuration: BKIN pin */   
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  
  TIM_OCStructInit(&TIM1_OCInitStructure);
  /* Channel 4 Configuration in OC */
  TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  
  TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
  TIM1_OCInitStructure.TIM_Pulse = PWM_PERIOD - TMIN - TBEFORE; 
  
  TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
  TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;         
  TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM1_OCInitStructure.TIM_OCNIdleState = LOW_SIDE_POLARITY;            
  
  TIM_OC4Init(TIM1, &TIM1_OCInitStructure);
  
  /* Enables the TIM1 Preload on CC1 Register */
  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Disable);
  /* Enables the TIM1 Preload on CC2 Register */
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Disable);
  /* Enables the TIM1 Preload on CC3 Register */
  TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Disable);
  /* Enables the TIM1 Preload on CC4 Register */
  TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Disable);
  
  /* Automatic Output enable, Break, dead time and lock configuration*/
  TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
  TIM1_BDTRInitStructure.TIM_DeadTime = DEADTIME;   // 死区时间(计数单位), 防止上下桥臂直通
  TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
  TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;
  TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable;

  TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);

  TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);   // TRGO = 更新事件(下溢), 可作为 ADC 触发源
  
  // Clear Break Flag and enable interrupt
  TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
  TIM_ITConfig(TIM1, TIM_IT_Break,ENABLE);
  
  /* TIM1 counter enable */
  TIM_Cmd(TIM1, ENABLE);
  
  // Disnable update interrupt
  TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);
  
  // Resynch to have the Update evend during Undeflow
  TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
  
  // Enable DMA event
  TIM_DMACmd(TIM1, TIM_DMA_CC1, DISABLE);
  TIM_DMACmd(TIM1, TIM_DMA_CC2, DISABLE);
  TIM_DMACmd(TIM1, TIM_DMA_CC3, DISABLE);
  TIM_DMACmd(TIM1, TIM_DMA_Update,ENABLE);
  
  TIM_DMAConfig(TIM1, TIM_DMABase_CCR1, TIM_DMABurstLength_4Bytes);   // DMA 突发: 从 CCR1 起一次写 4 个寄存器
  
  // Sets the disable preload vars for CCMR
  hPreloadCCMR1Disable = TIM1->CCMR1 & 0x8F8F;
  hPreloadCCMR2Disable = TIM1->CCMR2 & 0x8F8F;
   
  /* ADC1 registers reset ----------------------------------------------------*/
  ADC_DeInit(ADC1);
  /* ADC1 registers reset ----------------------------------------------------*/
  ADC_DeInit(ADC2);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_StructInit(&ADC_InitStructure);
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* ADC2 Configuration ------------------------------------------------------*/
  ADC_StructInit(&ADC_InitStructure);  
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC2, &ADC_InitStructure);
  
  ADC_InjectedDiscModeCmd(ADC1,ENABLE);
  ADC_InjectedDiscModeCmd(ADC2,ENABLE);
  
  // Start calibration of ADC1
  ADC_StartCalibration(ADC1);
  // Start calibration of ADC2
  ADC_StartCalibration(ADC2);
  
  // Wait for the end of ADCs calibration 
  while (ADC_GetCalibrationStatus(ADC1) & ADC_GetCalibrationStatus(ADC2))
  {
  }
  
  SVPWM_1ShuntCurrentReadingCalibration();
    
  /* Configure one bit for preemption priority */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  NVIC_StructInit(&NVIC_InitStructure);
  /* Enable the ADC Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ADC_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  /* Enable the Update Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM1_UP_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIM1_UP_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
    
  /* Enable the TIM1 BRK Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BRK_PRE_EMPTION_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = BRK_SUB_PRIORITY;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  // Default value of DutyValues
  dvDutyValues.hTimeSmp1 = (PWM_PERIOD >> 1) - TBEFORE;
  dvDutyValues.hTimeSmp2 = (PWM_PERIOD >> 1) + TAFTER;
} 

/*******************************************************************************
* Function Name  : SVPWM_1ShuntCurrentReadingCalibration
* Description    : Store zero current converted values for current reading 
                   network offset compensation in case of 1 shunt resistors 
* 功能说明(中文) : 采集采样电阻通道在「零电流」下的 ADC 值作为偏置(offset)。
*                  上电/初始化时电机未通电, 此时测得即为零电流基准。
*                  连续转换 NB_CONVERSIONS 次并累加, 结果存入 hPhaseOffset,
*                  用于后续把 ADC 原始值换算为真实电流时扣除零点。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 校准期间用软件触发(ADC_ExternalTrigInjecConv_T1_TRGO 先设为
*                  TRGO 但注入序列设置为同一通道两次); 每次结果右移 3 位后累加;
*                  结束时调用 SVPWM_InjectedConvConfig() 切换到正常工作配置。
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_1ShuntCurrentReadingCalibration(void)
{
  static u16 bIndex;   // 循环计数(静态, 只在本函数使用)
  
  /* ADC1 Injected group of conversions end interrupt disabling */
  ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);
  
  hPhaseOffset=0;
  
  /* ADC1 Injected conversions trigger is given by software and enabled */ 
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);  
  ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE); 
  
  /* ADC1 Injected conversions configuration */ 
  ADC_InjectedSequencerLengthConfig(ADC1,2);
  ADC_InjectedChannelConfig(ADC1, PHASE_B_ADC_CHANNEL, 1,SAMPLING_TIME_CK);
  ADC_InjectedChannelConfig(ADC1, PHASE_B_ADC_CHANNEL, 2,SAMPLING_TIME_CK);

  /* Clear the ADC1 JEOC pending flag */
  ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);  
    
  /* ADC Channel used for current reading are read 
     in order to get zero currents ADC values*/ 
  for(bIndex=0; bIndex <NB_CONVERSIONS; bIndex++)
  {
    while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC)) { }
    
    hPhaseOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1)>>3);   // 右移3位后累加零电流码
            
    /* Clear the ADC1 JEOC pending flag */
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);    
  }
  
  SVPWM_InjectedConvConfig();  
}

/*******************************************************************************
* Function Name  : SVPWM_InjectedConvConfig
* Description    : This function configure ADC1 for 1 shunt current 
*                  reading and ADC2  temperature and voltage feedbcak after a 
*                  calibration, it also setup the DMA and the default value of the 
*                 variables after the start command
* 功能说明(中文) : 校准结束后对 ADC 做正常工作配置并复位相关状态:
*                  - ADC1 注入序列 = 采样电阻电流通道 ×2(两次采样), ADC2 = 温度 + 母线电压;
*                  - ADC2 使能模拟看门狗做母线过压保护, 并打开 ADC2 的 AWD 中断、
*                    ADC1 的 JEOC 中断;
*                  - 设置两个采样触发点初值, 复位 CCR、DMA 缓冲、占空比与电流历史值;
*                  - 使能 TIM1 的 CC4 DMA 事件(采样触发)。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 启动电机后由上层调用, 之后每周期由中断驱动电流采样;
*                  母线过压阈值取 OVERVOLTAGE_THRESHOLD(右移 3 位后写入看门狗)。
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_InjectedConvConfig(void)
{  
  /* ADC2 Injected conversions configuration */ 
  ADC_InjectedSequencerLengthConfig(ADC2,2);                                     // ADC2 注入序列长度 = 2
  ADC_InjectedChannelConfig(ADC2, TEMP_FDBK_CHANNEL, 1,SAMPLING_TIME_CK);        // 序列1: 温度
  ADC_InjectedChannelConfig(ADC2, BUS_VOLT_FDBK_CHANNEL, 2,SAMPLING_TIME_CK);    // 序列2: 母线电压
  
  /* ADC2 Injected conversions trigger is TIM1 TRGO */ 
  ADC_ExternalTrigInjectedConvConfig(ADC2, ADC_ExternalTrigInjecConv_T1_TRGO);   // ADC2 由 TIM1 TRGO(下溢)触发
  ADC_ExternalTrigInjectedConvCmd(ADC2,ENABLE);
  
  /* Bus voltage protection initialization*/                            
  ADC_AnalogWatchdogCmd(ADC2,ADC_AnalogWatchdog_SingleInjecEnable);   // 使能模拟看门狗(仅监视单个注入通道)
  ADC_AnalogWatchdogSingleChannelConfig(ADC2,BUS_VOLT_FDBK_CHANNEL);  // 监视通道 = 母线电压
  ADC_AnalogWatchdogThresholdsConfig(ADC2,OVERVOLTAGE_THRESHOLD>>3,0x00);   // 过压阈值(高阈值, 右移3位对齐)
  
  /* ADC1 Injected group of conversions end and Analog Watchdog interrupts
                                                                     enabling */
  ADC_ITConfig(ADC2, ADC_IT_AWD, ENABLE);
  ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
  

  // Default value of DutyValues
  dvDutyValues.hTimeSmp1 = (PWM_PERIOD >> 1) - TBEFORE;
  dvDutyValues.hTimeSmp2 = (PWM_PERIOD >> 1) + TAFTER;
  
  // Default value of sampling point
  hCCDmaBuffCh4[0] = dvDutyValues.hTimeSmp2; // Second point 
  hCCDmaBuffCh4[1] = dvDutyValues.hTimeSmp2;
  hCCDmaBuffCh4[2] = dvDutyValues.hTimeSmp1; // First point
  hCCDmaBuffCh4[3] = dvDutyValues.hTimeSmp1;

  // Set TIM1 CCx start value
  TIM1->CCR1 = PWM_PERIOD >> 1;
  TIM1->CCR2 = PWM_PERIOD >> 1;
  TIM1->CCR3 = PWM_PERIOD >> 1;
  TIM1->CCR4 = (PWM_PERIOD >> 1) - TBEFORE;
  
  // Default Update DMA buffer Ch 1,2,3,4 after reset
  hCCRBuff[0] = PWM_PERIOD >> 1;
  hCCRBuff[1] = PWM_PERIOD >> 1;
  hCCRBuff[2] = PWM_PERIOD >> 1;
  hCCRBuff[3] = (PWM_PERIOD >> 1) - TBEFORE;
  
  TIM_DMACmd(TIM1, TIM_DMA_CC4, ENABLE);   // 使能 TIM1 的 CC4 DMA 事件(用于刷新采样触发点)
  
  // After start value of DMA buffers
  hCCDmaBuffCh1[0] = PWM_PERIOD-HTMIN;
  hCCDmaBuffCh1[1] = PWM_PERIOD-HTMIN;
  hCCDmaBuffCh1[2] = PWM_PERIOD >> 1;
  hCCDmaBuffCh1[3] = PWM_PERIOD >> 1;
  
  hCCDmaBuffCh2[0] = PWM_PERIOD-HTMIN;
  hCCDmaBuffCh2[1] = PWM_PERIOD-HTMIN;
  hCCDmaBuffCh2[2] = PWM_PERIOD >> 1;
  hCCDmaBuffCh2[3] = PWM_PERIOD >> 1;
  
  hCCDmaBuffCh3[0] = PWM_PERIOD-HTMIN;
  hCCDmaBuffCh3[1] = PWM_PERIOD-HTMIN;
  hCCDmaBuffCh3[2] = PWM_PERIOD >> 1;
  hCCDmaBuffCh3[3] = PWM_PERIOD >> 1;
  
  // After start value of dvDutyValues
  dvDutyValues.hTimePhA = PWM_PERIOD >> 1;
  dvDutyValues.hTimePhB = PWM_PERIOD >> 1;
  dvDutyValues.hTimePhC = PWM_PERIOD >> 1;
  
  // Set the default previous value of Phase A,B,C current
  hCurrAOld=0;
  hCurrBOld=0;
  hCurrCOld=0;
  
  hDeltaA = 0;
  hDeltaB = 0;
  hDeltaC = 0;
  bReadDelta = 0;
  bStatorFluxPosOld = REGULAR;
  bStatorFluxPos = REGULAR;
}

/*******************************************************************************
* Function Name  : SVPWM_1ShuntCalcDutyCycles
* Description    :  Implementation of the single shunt algorithm to setup the 
TIM1 register and DMA buffers values for the next PWM period.
* 功能说明(中文) : 单电阻算法的总入口(每个电流环周期调用一次)。依次完成:
*                  1) 由 α/β 电压指令计算扇区(1~6);
*                  2) 计算三相占空比 hTimePhA/B/C;
*                  3) 判断定子矢量区域(REGULAR/BOUNDARY_1/2/3)并在必要时对某相移相
*                     (INVERT_x), 以保证有一个足够长的可观测窗口;
*                  4) 计算两个 ADC 采样触发点 hTimeSmp1/hTimeSmp2;
*                  5) 确定两个采样点各测哪一相(csCurrentSampled)并设置 CCMR 预装载值;
*                  6) 把占空比与采样点写入 DMA 缓冲, 供更新事件突发写入定时器。
* 参数(中文)     : Stat_Volt_Input - α/β 电压分量 Volt_Components(q1.15),
*                  qV_Component1=Vα, qV_Component2=Vβ, 范围各 [-32768, 32767]。
* 返回(中文)     : 无
* 备注(中文)     : 移相会修改 dvDutyValues 与 bStatorFluxPos, 影响本周期 PWM 波形;
*                  仅在 bDistEnab==1(启动采样)时执行移相逻辑, 否则退化为常规 SVPWM。
* Input          : Stat_Volt_alfa_beta
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_1ShuntCalcDutyCycles (Volt_Components Stat_Volt_Input)
{
    s32 wX, wY, wZ, wUAlpha, wUBeta;   // 扇区判定与占空比计算中间变量(32 位防溢出)
    s16 hDeltaDuty[2];                 // 排序后相邻两相占空比之差(用于判断采样窗口大小)
    u16 hDutyV[4]; // the 4th element is the swap tmp   // 三相占空比排序数组, 第4个元素作交换临时变量
    
/*******************************************************************************
* Function Name  : SVPWM_1ShuntGetDuty
* 功能说明(中文) : (本工程已内联到 SVPWM_1ShuntCalcDutyCycles 中, 原函数调用被注释)
*                  由 α/β 电压指令按 SVPWM 计算三相占空比。
* 参数(中文)     : Stat_Volt_alfa_beta - α/β 电压分量(q1.15)。
* 返回(中文)     : 无(结果写入全局 dvDutyValues)。
* 备注(中文)     : 与文件内实际使用的扇区判断/占空比计算逻辑等价。
* Description    : Computes the three duty cycle values corresponding to the input value
                        using space vector modulation techinque
* Input          : Stat_Volt_alfa_beta
* Output         : None
* Return         : None
*******************************************************************************/
    //SVPWM_1ShuntGetDuty(Stat_Volt_Input);
    wUAlpha = Stat_Volt_Input.qV_Component1 * T_SQRT3 ;   // wUAlpha = Vα·T·√3
    wUBeta = -(Stat_Volt_Input.qV_Component2 * T);        // wUBeta = -Vβ·T
  
    wX = wUBeta;                       // 三个中间量, 用于扇区判定
    wY = (wUBeta + wUAlpha)/2;         // wY 为线性组合
    wZ = (wUBeta - wUAlpha)/2;         // wZ 为线性组合
     
    // Sector calculation from wX, wY, wZ      // 根据 wX/wY/wZ 的符号确定 6 个 60° 扇区之一
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
    // 按扇区计算三相占空比比较值: T/8 为 50% 基准(中心对齐), 各相差异项除以 131072(2^17)定标
    switch(bSector)
    {  
      case SECTOR_1:
          dvDutyValues.hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);
          dvDutyValues.hTimePhB = dvDutyValues.hTimePhA + wZ/131072;
          dvDutyValues.hTimePhC = dvDutyValues.hTimePhB - wX/131072;
                  break;
      case SECTOR_2:
          dvDutyValues.hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);
          dvDutyValues.hTimePhB = dvDutyValues.hTimePhA + wZ/131072;
          dvDutyValues.hTimePhC = dvDutyValues.hTimePhA - wY/131072;
          break;
      case SECTOR_3:
          dvDutyValues.hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);
          dvDutyValues.hTimePhC = dvDutyValues.hTimePhA - wY/131072;
          dvDutyValues.hTimePhB = dvDutyValues.hTimePhC + wX/131072;
          break;
      case SECTOR_4:
          dvDutyValues.hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);
          dvDutyValues.hTimePhB = dvDutyValues.hTimePhA + wZ/131072;
          dvDutyValues.hTimePhC = dvDutyValues.hTimePhB - wX/131072;
          break;  
      case SECTOR_5:
          dvDutyValues.hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);
          dvDutyValues.hTimePhB = dvDutyValues.hTimePhA + wZ/131072;
          dvDutyValues.hTimePhC = dvDutyValues.hTimePhA - wY/131072;
              break;
      case SECTOR_6:
          dvDutyValues.hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);
          dvDutyValues.hTimePhC = dvDutyValues.hTimePhA - wY/131072;
          dvDutyValues.hTimePhB = dvDutyValues.hTimePhC + wX/131072;
          break;
      default:
          break;
    }
    
    if (bDistEnab == 1)   // 仅在启用单电阻移相时执行后续的边界判断/移相/采样点计算
    {
      bStatorFluxPosOld = bStatorFluxPos;   // 保存上一周期区域(供边界切换判断)
    
/*******************************************************************************
* Function Name  : SVPWM_1GetStatorFluxPos
* 功能说明(中文) : (已内联) 对三相占空比排序, 由相邻差值判断定子矢量区域:
*                  REGULAR(窗口足够) / BOUNDARY_1(两小一大) / BOUNDARY_2(两大一小) /
*                  BOUNDARY_3(三者相等, 无足够窗口)。判断阈值 TMIN 为最小可观测时间。
* 参数(中文)     : 无(使用 dvDutyValues 中的三相占空比)。
* 返回(中文)     : 区域代码, 写入全局 bStatorFluxPos。
* 备注(中文)     : 差值计算方法见下方排序与 hDeltaDuty 计算。
* Description    :  Compute the stator vector position 
                        REGULAR if stator vector lies in regular region
                        BOUNDARY_1 if stator vector lies in boundary region 1 (two small, one big)
                        BOUNDARY_2 if stator vector lies in boundary region 2 (two big, one small)
                        BOUNDARY_3 if stator vector lies in boundary region 3 (three equal)
* Input           : None
* Output         : None
* Return         : None
*******************************************************************************/
      //bStatorFluxPos = SVPWM_1GetStatorFluxPos(); 
      hDutyV[0] = dvDutyValues.hTimePhA;
      hDutyV[1] = dvDutyValues.hTimePhB;
      hDutyV[2] = dvDutyValues.hTimePhC;
      
      // Sort ascendant
      if (hDutyV[0] > hDutyV[1])
      {
        // Swap [0] [1]
        hDutyV[3] = hDutyV[0];
        hDutyV[0] = hDutyV[1];
        hDutyV[1] = hDutyV[3];
      }
      if (hDutyV[0] > hDutyV[2])
      {
        // Swap [0] [2]
        hDutyV[3] = hDutyV[0];
        hDutyV[0] = hDutyV[2];
        hDutyV[2] = hDutyV[3];
      }
      if (hDutyV[1] > hDutyV[2])
      {
        // Swap [1] [2]
        hDutyV[3] = hDutyV[1];
        hDutyV[1] = hDutyV[2];
        hDutyV[2] = hDutyV[3];
      }
      
      // Compute delta duty
      hDeltaDuty[0] = (s16)(hDutyV[1]) - (s16)(hDutyV[0]);
      hDeltaDuty[1] = (s16)(hDutyV[2]) - (s16)(hDutyV[1]);
      
      // Check region
      // 用最小可观测时间 TMIN 判断上下两个窗口是否足够, 从而确定矢量区域
      if ((hDeltaDuty[1]>TMIN) && (hDeltaDuty[0]<=TMIN))
        bStatorFluxPos = BOUNDARY_2;                        // 下窗口不足(两大一小)
      else if ((hDeltaDuty[1]<=TMIN) && (hDeltaDuty[0]>TMIN))
        bStatorFluxPos = BOUNDARY_1;                        // 上窗口不足(两小一大)
      else if ((hDeltaDuty[1]>TMIN) && (hDeltaDuty[0]>TMIN))
        bStatorFluxPos = REGULAR;                           // 两个窗口都足够
      else
        bStatorFluxPos = BOUNDARY_3;                        // 两个窗口都不足(三者接近相等)
    
/*******************************************************************************
* Function Name  : SVPWM_1PWMDutyAdj
* 功能说明(中文) : (已内联) 根据定子矢量区域决定对哪一相移相(畸变)以拓宽采样窗口,
*                  并把该相占空比下移 HTMIN; 结果记入 bInverted_pwm_new(INVERT_A/B/C/NONE)。
* 参数(中文)     : 无(依据 bStatorFluxPos 与 bSector)。
* 返回(中文)     : 无(修改 dvDutyValues 与 bInverted_pwm_new)。
* 备注(中文)     : 边界3 下在 A/B 相之间交替移相(bStBd3)。
* Description    :  Compute the PWM channel that must be distorted and updates
                    the value od duty cycle registers
                                
* Input           : None
* Output         : None
* Return         : None
*******************************************************************************/    
      //SVPWM_1PWMDutyAdj();
      if (bStatorFluxPos == REGULAR)
      {
              bInverted_pwm_new = INVERT_NONE;
      }
      else if (bStatorFluxPos == BOUNDARY_1) // Adjust the lower
      {
        switch (bSector)
        {
                case SECTOR_5:
                case SECTOR_6:
                        bInverted_pwm_new = INVERT_A;
                        dvDutyValues.hTimePhA -=HTMIN;
                        break;
                case SECTOR_2:
                case SECTOR_1:
                        bInverted_pwm_new = INVERT_B;
                        dvDutyValues.hTimePhB -=HTMIN;
                        break;
                case SECTOR_4:
                case SECTOR_3:
                        bInverted_pwm_new = INVERT_C;
                        dvDutyValues.hTimePhC -=HTMIN;
                        break;
        }
      }
      else if (bStatorFluxPos == BOUNDARY_2) // Adjust the middler
      {
        switch (bSector)
        {
                case SECTOR_4:
                case SECTOR_5: // Inverto sempre B
                        bInverted_pwm_new = INVERT_B;
                        dvDutyValues.hTimePhB -=HTMIN;
                        break;
                case SECTOR_2:
                case SECTOR_3: // Inverto sempre A
                        bInverted_pwm_new = INVERT_A;
                        dvDutyValues.hTimePhA -=HTMIN;
                        break;
                case SECTOR_6:
                case SECTOR_1: // Inverto sempre C
                        bInverted_pwm_new = INVERT_C;
                        dvDutyValues.hTimePhC -=HTMIN;
                        break;
        }
      }
      else if (bStatorFluxPos == BOUNDARY_3)
      {
        if (bStBd3 == 0)
        {
          bInverted_pwm_new = INVERT_A;
          dvDutyValues.hTimePhA -=HTMIN;
          bStBd3 = 1;
        } 
        else
        {
          bInverted_pwm_new = INVERT_B;
          dvDutyValues.hTimePhB -=HTMIN;
          bStBd3 = 0;
        }
      }
      
      if (bInverted_pwm_new != INVERT_NONE)
      {
        // Check for negative values of duty register
        if (dvDutyValues.hTimePhA > 0xEFFF)
          dvDutyValues.hTimePhA = DMABURSTMIN_A;
        if (dvDutyValues.hTimePhB > 0xEFFF)
          dvDutyValues.hTimePhB = DMABURSTMIN_B;
        if (dvDutyValues.hTimePhC > 0xEFFF)
          dvDutyValues.hTimePhC = DMABURSTMIN_C;
        
        // Duty adjust to avoid commutation inside Update Handler
        if ((dvDutyValues.hTimePhA > MINTIMCNTUPHAND) && (dvDutyValues.hTimePhA < MIDTIMCNTUPHAND))
            dvDutyValues.hTimePhA = MINTIMCNTUPHAND;
        if ((dvDutyValues.hTimePhA >= MIDTIMCNTUPHAND) && (dvDutyValues.hTimePhA < MAXTIMCNTUPHAND))
            dvDutyValues.hTimePhA = MAXTIMCNTUPHAND;     
        if ((dvDutyValues.hTimePhB > MINTIMCNTUPHAND) && (dvDutyValues.hTimePhB < MIDTIMCNTUPHAND))
            dvDutyValues.hTimePhB = MINTIMCNTUPHAND;
        if ((dvDutyValues.hTimePhB >= MIDTIMCNTUPHAND) && (dvDutyValues.hTimePhB < MAXTIMCNTUPHAND))
            dvDutyValues.hTimePhB = MAXTIMCNTUPHAND;
        if ((dvDutyValues.hTimePhC > MINTIMCNTUPHAND) && (dvDutyValues.hTimePhC < MIDTIMCNTUPHAND))
            dvDutyValues.hTimePhC = MINTIMCNTUPHAND;
        if ((dvDutyValues.hTimePhC >= MIDTIMCNTUPHAND) && (dvDutyValues.hTimePhC < MAXTIMCNTUPHAND))
            dvDutyValues.hTimePhC = MAXTIMCNTUPHAND;
      }
    
    /*******************************************************************************
* Function Name  : SVPWM_1PWMSetSamplingPoints
* 功能说明(中文) : (已内联) 依据区域与排序后的占空比计算两个 ADC 采样触发点
*                  hTimeSmp1/hTimeSmp2, 使 ADC 在有效矢量稳定(避开死区/振铃)时采样。
*                  窗口足够时取两相占空比中点; 不足时退化为紧贴换相点前 TBEFORE。
* 参数(中文)     : 无(使用 hDutyV 与 bStatorFluxPos)。
* 返回(中文)     : 无(写入 dvDutyValues.hTimeSmp1/hTimeSmp2)。
* 备注(中文)     : BOUNDARY_1/2/3 时第二个采样点取 PWM_PERIOD - HTMIN + TSAMPLE。
* Description    :  Compute the sampling point and the related phase sampled 	
* Input           : None
* Output         : None
* Return         : None
*******************************************************************************/
      //SVPWM_1PWMSetSamplingPoints();
      // Reset error state & sampling before
      bError = 0;
      
      if (bStatorFluxPos == REGULAR) // Regual zone
      {
        // First point
        if ((hDutyV[1] - hDutyV[0] - TDEAD)> MAX_TRTS)
        {
          dvDutyValues.hTimeSmp1 = (hDutyV[0] + hDutyV[1] + TDEAD) >> 1;
        }
        else
        {
          dvDutyValues.hTimeSmp1 = hDutyV[1] - TBEFORE;
        }
        // Second point
        if ((hDutyV[2] - hDutyV[1] - TDEAD)> MAX_TRTS)
        {
          dvDutyValues.hTimeSmp2 = (hDutyV[1] + hDutyV[2] + TDEAD) >> 1;
        }
        else
        {
          dvDutyValues.hTimeSmp2 = hDutyV[2] - TBEFORE;
        }
      }
      else 
      {
        // Adjust hDuty
        hDutyV[0] = dvDutyValues.hTimePhA;
        hDutyV[1] = dvDutyValues.hTimePhB;
        hDutyV[2] = dvDutyValues.hTimePhC;
        
        // Sort ascendant
        if (hDutyV[0] > hDutyV[1])
        {
          // Swap [0] [1]
          hDutyV[3] = hDutyV[0];
          hDutyV[0] = hDutyV[1];
          hDutyV[1] = hDutyV[3];
        }
        if (hDutyV[0] > hDutyV[2])
        {
          // Swap [0] [2]
          hDutyV[3] = hDutyV[0];
          hDutyV[0] = hDutyV[2];
          hDutyV[2] = hDutyV[3];
        }
        if (hDutyV[1] > hDutyV[2])
        {
          // Swap [1] [2]
          hDutyV[3] = hDutyV[1];
          hDutyV[1] = hDutyV[2];
          hDutyV[2] = hDutyV[3];
        }
      }
      
      if (bStatorFluxPos == BOUNDARY_1) // Two small, one big
      {   
        // Check after the distortion for sampling space
        if ((hDutyV[1] - hDutyV[0])< TMIN)
        {
          // After the distortion the first sampling point can't be performed
          // It is necessary to swtch to Boudary 3
          
          // Restore the distorted duty
          if (bInverted_pwm_new == INVERT_A);
            dvDutyValues.hTimePhA +=HTMIN;
          if (bInverted_pwm_new == INVERT_B);
            dvDutyValues.hTimePhB +=HTMIN;
          if (bInverted_pwm_new == INVERT_C);
            dvDutyValues.hTimePhC +=HTMIN;
          
          // Switch to Boudary 3
          bStatorFluxPos = BOUNDARY_3;        
          if (bStBd3 == 0)
          {
            bInverted_pwm_new = INVERT_A;
            dvDutyValues.hTimePhA -=HTMIN;
            bStBd3 = 1;
          } 
          else
          {
            bInverted_pwm_new = INVERT_B;
            dvDutyValues.hTimePhB -=HTMIN;
            bStBd3 = 0;
          }
        }
        
        // First point
        if ((hDutyV[1] - hDutyV[0] - TDEAD)> MAX_TRTS)
        {
          dvDutyValues.hTimeSmp1 = (hDutyV[0] + hDutyV[1] + TDEAD) >> 1;
        }
        else
        {
          dvDutyValues.hTimeSmp1 = hDutyV[1] - TBEFORE;
        }
        // Second point
        dvDutyValues.hTimeSmp2 = PWM_PERIOD - HTMIN + TSAMPLE;
      }
      
      if (bStatorFluxPos == BOUNDARY_2) // Two big, one small
      {
        // First point
        if ((hDutyV[2] - hDutyV[1] - TDEAD)>= MAX_TRTS)
        {
          dvDutyValues.hTimeSmp1 = (hDutyV[1] + hDutyV[2] + TDEAD) >> 1;
        }
        else
        {
          dvDutyValues.hTimeSmp1 = hDutyV[2] - TBEFORE;
        }
        // Second point
        dvDutyValues.hTimeSmp2 = PWM_PERIOD - HTMIN + TSAMPLE;
      }
      
      if (bStatorFluxPos == BOUNDARY_3) // 
      {
        // First point
        dvDutyValues.hTimeSmp1 = hDutyV[0]-TBEFORE; // Dummy trigger
        // Second point
        dvDutyValues.hTimeSmp2 = PWM_PERIOD - HTMIN + TSAMPLE;
      }
    }
    else
    {
      bInverted_pwm_new = INVERT_NONE;
      bStatorFluxPos = REGULAR;
    }
        
    // Update DMA buffer Ch 1,2,3,4 (These value are required before update event)
    // This buffer is updated using DMA burst
    hCCRBuff[0] = dvDutyValues.hTimePhA;
    hCCRBuff[1] = dvDutyValues.hTimePhB;
    hCCRBuff[2] = dvDutyValues.hTimePhC;
    hCCRBuff[3] = dvDutyValues.hTimeSmp1;
    
/*******************************************************************************
* Function Name  : SVPWM_1ShuntNoPreloadAdj
* 功能说明(中文) : (已内联) 根据本周期移相相 bInverted_pwm_new, 计算 CH1~CH4 的 CCMR
*                  预装载值 hPreloadCCMR1Set/hPreloadCCMR2Set(切换某通道为 Toggle 模式),
*                  供更新中断在临界区切换通道输出模式。
* 参数(中文)     : 无(依据 bInverted_pwm_new)。
* 返回(中文)     : 无(写入 hPreloadCCMR1Set/hPreloadCCMR2Set)。
* 备注(中文)     : 与 SVPWMUpdateEvent 中的切换逻辑配套。
* Description    :  Set the preload variables for PWM mode Ch 1,2,3,4	
* Input           : None
* Output         : None
* Return         : None
*******************************************************************************/
    //SVPWM_1ShuntNoPreloadAdj();
    // Set the preload vars for PWM mode Ch 1,2,3,4 (these value are required 
    // inside update event handler
    switch (bInverted_pwm_new)
    {
    case INVERT_A:
      // Preloads for CCMR
      hPreloadCCMR1Set = hPreloadCCMR1Disable | CH1TOGGLE | CH2NORMAL;
      hPreloadCCMR2Set = hPreloadCCMR2Disable | CH3NORMAL | CH4TOGGLE;
      break;
    case INVERT_B:
      // Preloads for CCMR
      hPreloadCCMR1Set = hPreloadCCMR1Disable | CH1NORMAL | CH2TOGGLE;
      hPreloadCCMR2Set = hPreloadCCMR2Disable | CH3NORMAL | CH4TOGGLE;
      break;
    case INVERT_C:
      // Preloads for CCMR
      hPreloadCCMR1Set = hPreloadCCMR1Disable | CH1NORMAL | CH2NORMAL;
      hPreloadCCMR2Set = hPreloadCCMR2Disable | CH3TOGGLE | CH4TOGGLE;
      break;
    default:
      // Preloads for CCMR
      hPreloadCCMR1Set = hPreloadCCMR1Disable | CH1NORMAL | CH2NORMAL;
      hPreloadCCMR2Set = hPreloadCCMR2Disable | CH3NORMAL | CH4NORMAL;
      break;
    }
    
    // Limit for update event
    
    // The following instruction can be executed after Update handler
    // before the get phase current (Second EOC)
    
    // Set the current sampled
     if (bStatorFluxPos == REGULAR) // Regual zone
    {  
      switch (bSector)
      {
      case SECTOR_1: // Fisrt after C, Second after B
          csCurrentSampled.sampCur1 = SAMP_NIC;
          csCurrentSampled.sampCur2 = SAMP_IA;
          break;
      case SECTOR_2: // Fisrt after C, Second after A 
          csCurrentSampled.sampCur1 = SAMP_NIC;
          csCurrentSampled.sampCur2 = SAMP_IB;
          break;
      case SECTOR_3: // Fisrt after A, Second after C
          csCurrentSampled.sampCur1 = SAMP_NIA;
          csCurrentSampled.sampCur2 = SAMP_IB;
          break;
      case SECTOR_4: // Fisrt after A, Second after B
          csCurrentSampled.sampCur1 = SAMP_NIA;
          csCurrentSampled.sampCur2 = SAMP_IC;
          break;
      case SECTOR_5: // Fisrt after B, Second after A
          csCurrentSampled.sampCur1 = SAMP_NIB;
          csCurrentSampled.sampCur2 = SAMP_IC;
          break;
      case SECTOR_6: // Fisrt after B, Second after C
          csCurrentSampled.sampCur1 = SAMP_NIB;
          csCurrentSampled.sampCur2 = SAMP_IA;
          break;
      }
    }
    
    if (bStatorFluxPos == BOUNDARY_1) // Two small, one big
    {
      switch (bSector)
      {
      case SECTOR_1:    // Phase B is adjusted
      case SECTOR_2:    
          csCurrentSampled.sampCur1 = SAMP_NIC;
          csCurrentSampled.sampCur2 = SAMP_IB;
          break;

      case SECTOR_3:    // Phase C is adjusted 
      case SECTOR_4:    
          csCurrentSampled.sampCur1 = SAMP_NIA;
          csCurrentSampled.sampCur2 = SAMP_IC;
          break;

      case SECTOR_5:   // Phase A is adjusted 
      case SECTOR_6:    
          csCurrentSampled.sampCur1 = SAMP_NIB;
          csCurrentSampled.sampCur2 = SAMP_IA;
          break;
      }
    }
    
    if (bStatorFluxPos == BOUNDARY_2) // Two big, one small
    {
      switch (bSector)
      {
      case SECTOR_2: // Phase A is adjusted
      case SECTOR_3:
          csCurrentSampled.sampCur1 = SAMP_IB;
          csCurrentSampled.sampCur2 = SAMP_IA;
          break;     
      case SECTOR_4: // Phase B is adjusted
      case SECTOR_5:
          csCurrentSampled.sampCur1 = SAMP_IC;
          csCurrentSampled.sampCur2 = SAMP_IB;
          break;  
      case SECTOR_6: // Phase C is adjusted
      case SECTOR_1:
          csCurrentSampled.sampCur1 = SAMP_IA;
          csCurrentSampled.sampCur2 = SAMP_IC;
          break;    
      }
    }
    
    if (bStatorFluxPos == BOUNDARY_3)  
    {
      if (bInverted_pwm_new == INVERT_A)
      {
        csCurrentSampled.sampCur1 = SAMP_OLDB;
        csCurrentSampled.sampCur2 = SAMP_IA;
      }
      if (bInverted_pwm_new == INVERT_B)
      {
        csCurrentSampled.sampCur1 = SAMP_OLDA;
        csCurrentSampled.sampCur2 = SAMP_IB;
      }
    }
    
	#ifdef CURRENT_COMPENSATION
	    // Check for distortion compensation entering Boudary2
	    if ((bStatorFluxPosOld == REGULAR) && (bStatorFluxPos == BOUNDARY_2) && (State == RUN))
	    {
	      bReadDelta = 1;
	    }
	    else
	    {
	      bReadDelta = 0;
	    }
	#endif
    
    // Deleting Delta if Stator Pos is no more BOUDARY_2
    if ((bStatorFluxPosOld == BOUNDARY_2) && (bStatorFluxPos != bStatorFluxPosOld) 
        && (State == RUN))
    {
      hDeltaA = 0;
      hDeltaB = 0;
      hDeltaC = 0;
    }
    
    // Limit for the Get Phase current (Second EOC Handler)
}

/*******************************************************************************
* Function Name  : SVPWM_1ShuntGetPhaseCurrentValues
* 功能说明(中文) : 由两个采样点的 ADC 结果重构三相电流 Ia/Ib/Ic(q1.15)。
*                  依据 csCurrentSampled 记录的各点对应相与极性: 减去零电流偏置、
*                  按需取反、饱和限幅, 再对未直接测到的相用 ia+ib+ic=0 推算。
*                  最后输出返回 (Ia, Ib) 两分量, 并把三相值保存供下周期使用。
* 参数(中文)     : 无
* 返回(中文)     : Curr_Components: qI_Component1=Ia(q1.15), qI_Component2=Ib(q1.15)。
* 备注(中文)     : 读取 ADC1 的 JDR2(第一点)与 JDR1(第二点); 换算含零点补偿
*                  (结果 = (ADC值<<1) - hPhaseOffset); 边界3 时用 SAMP_OLDA/OLDB 取旧值;
*                  CURRENT_COMPENSATION 使能时对边界2 的畸变电流做补偿并更新历史值。
* Description    : This function computes current values of Phase A and Phase B 
*                 in q1.15 format starting from values acquired from the A/D 
*                 Converter peripheral.
* Input          : None
* Output         : Stat_Curr_a_b
* Return         : None
*******************************************************************************/
Curr_Components SVPWM_1ShuntGetPhaseCurrentValues(void)
{
    Curr_Components Local_Stator_Currents;
    s32 wAux;
    s16 hCurrA = 0, hCurrB = 0, hCurrC = 0;
    u8 bCurrASamp = 0, bCurrBSamp = 0, bCurrCSamp = 0;

    
    if (csCurrentSampled.sampCur1 == SAMP_OLDA)
    {
      hCurrA = hCurrAOld;
      bCurrASamp = 1;
    }
    
    if (csCurrentSampled.sampCur1 == SAMP_OLDB)
    {
      hCurrB = hCurrBOld;
      bCurrBSamp = 1;
    }
    
    // First sampling point
    wAux =  (s32)(ADC1->JDR2 << 1) - (s32)(hPhaseOffset);   // 第一采样点: 读 JDR2, <<1 放大后减零点偏置
    
    switch (csCurrentSampled.sampCur1)
    {
    case SAMP_IA:
    case SAMP_IB:
    case SAMP_IC:
            break;
    case SAMP_NIA:
    case SAMP_NIB:
    case SAMP_NIC:
            wAux = -wAux; 
            break;
    default:
            wAux = 0;
    }
    
    // Check saturation
    if (wAux < S16_MIN)
    {
            wAux = S16_MIN;
    }  
    else  if (wAux > S16_MAX)
    { 
            wAux = S16_MAX;
    }
    else
    {
            wAux = (s16)(wAux);
    }
    
    switch (csCurrentSampled.sampCur1)
    {
    case SAMP_IA:
    case SAMP_NIA:
            hCurrA = (s16)(wAux);
            bCurrASamp = 1;
            break;
    case SAMP_IB:
    case SAMP_NIB:
            hCurrB = (s16)(wAux);
            bCurrBSamp = 1;
            break;
    case SAMP_IC:
    case SAMP_NIC:
            hCurrC = (s16)(wAux);
            bCurrCSamp = 1;
            break;
    }
    
    // Second sampling point
    wAux = (s32)(ADC1->JDR1 << 1) - (s32)(hPhaseOffset);   // 第二采样点: 读 JDR1, 同样做零点补偿
    
    switch (csCurrentSampled.sampCur2)
    {
    case SAMP_IA:
    case SAMP_IB:
    case SAMP_IC:
            break;
    case SAMP_NIA:
    case SAMP_NIB:
    case SAMP_NIC:
            wAux = -wAux; 
            break;
    default:
            wAux = 0;
    }
    
    // Check saturation
    if (wAux < S16_MIN)
    {
            wAux = S16_MIN;
    }  
    else  if (wAux > S16_MAX)
    { 
            wAux = S16_MAX;
    }
    else
    {
            wAux = (s16)(wAux);
    }
    
    switch (csCurrentSampled.sampCur2)
    {
    case SAMP_IA:
    case SAMP_NIA:
            hCurrA = (s16)(wAux);
            bCurrASamp = 1;
            break;
    case SAMP_IB:
    case SAMP_NIB:
            hCurrB = (s16)(wAux);
            bCurrBSamp = 1;
            break;
    case SAMP_IC:
    case SAMP_NIC:
            hCurrC = (s16)(wAux);
            bCurrCSamp = 1;
            break;
    }
    
    // Computation of the third value            // 未直接测到的相, 用三相电流和为零推算(ia+ib+ic=0)
    if (bCurrASamp == 0)
            hCurrA = -hCurrB -hCurrC;
    if (bCurrBSamp == 0)
            hCurrB = -hCurrA -hCurrC;
    if (bCurrCSamp == 0)
            hCurrC = -hCurrA -hCurrB;
    
    // hCurrA, hCurrB, hCurrC values are the sampled values
    
	#ifdef CURRENT_COMPENSATION
	    if (bReadDelta == 1)  
	    {
	      hDeltaA = hCurrAOld - hCurrA;
	      hDeltaB = hCurrBOld - hCurrB;
	      hDeltaC = hCurrCOld - hCurrC;
	    }
      
	    if (bStatorFluxPos == BOUNDARY_2)
	    {
	      hCurrA += hDeltaA;
	      hCurrB += hDeltaB;
	      hCurrC += hDeltaC;
	    }
	#endif
    
    hCurrAOld = hCurrA;
    hCurrBOld = hCurrB;
    hCurrCOld = hCurrC;

    Local_Stator_Currents.qI_Component1 = hCurrA;
    Local_Stator_Currents.qI_Component2 = hCurrB;
    
    return(Local_Stator_Currents); 
}

/*******************************************************************************
* Function Name  : SVPWM_1ShuntAdvCurrentReading
* 功能说明(中文) : 使能或关闭单电阻电流采样流程。
*                  ENABLE : 使能 TIM1 更新中断并置 bDistEnab=1(开始移相采样);
*                  DISABLE: 关闭更新中断, 恢复 ADC 触发与 CCMR/DMA 到常规状态,
*                           三相占空比置 50%, 并置 bDistEnab=0(停止移相)。
* 参数(中文)     : cmd - ENABLE(启动采样)/DISABLE(停止采样), 类型 FunctionalState。
* 返回(中文)     : 无
* 备注(中文)     : 由启停电机流程调用, 直接改写 TIM1 CCMR1/CCMR2、CCR1~CCR3 与 DMA 使能。
* Description    :  It is used to enable or disable the current reading.
* Input          : cmd (ENABLE or DISABLE)
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWM_1ShuntAdvCurrentReading(FunctionalState cmd)
{
  if (cmd == ENABLE)
  {
    // Enable UPDATE ISR
    // Clear Update Flag
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    
    // Distortion for single shunt enabling
    bDistEnab = 1;
  }
  else
  {
    // Disable UPDATE ISR
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);

    // Sync ADC trigger with Update
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);

	// Enabling the Injectec conversion for ADC1
  	ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE);
    
    // Clear pending bit and Enable the EOC ISR
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);
    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    
    // Distortion for single shunt disabling
    bDistEnab = 0;
    
    // Disabling the last setting of PWM Mode and Duty Register
    hPreloadCCMR1Set = hPreloadCCMR1Disable | CH1NORMAL | CH2NORMAL;
    hPreloadCCMR2Set = hPreloadCCMR2Disable | CH3NORMAL | CH4NORMAL;
    TIM1->CCMR1 = hPreloadCCMR1Set; // Switch to Normal 
    TIM1->CCMR2 = hPreloadCCMR2Set; // Switch to Normal
    
    // Disabling all DMA previous setting
    TIM_DMACmd(TIM1, TIM_DMA_CC1, DISABLE);
    TIM_DMACmd(TIM1, TIM_DMA_CC2, DISABLE);
    TIM_DMACmd(TIM1, TIM_DMA_CC3, DISABLE);
    
    // Set all duty to 50%
    TIM1->CCR1 = PWM_PERIOD >> 1;
    TIM1->CCR2 = PWM_PERIOD >> 1;
    TIM1->CCR3 = PWM_PERIOD >> 1;    
  }
}

/*******************************************************************************
* Function Name  : SVPWMEOCEvent
* 功能说明(中文) : ADC 注入转换结束(JEOC)中断内调用。读取 ADC2 的温度与母线电压注入
*                  通道结果存入全局变量; 若处于单电阻采样(bDistEnab==1), 则关闭 ADC1 的
*                  外部触发注入转换, 以便下一周期由 TIM1 CC4 重新触发。
* 参数(中文)     : 无
* 返回(中文)     : 恒返回 1(u8)。
* 备注(中文)     : 更新 h_ADCTemp / h_ADCBusvolt; 依赖 bDistEnab 状态。
* Description    :  Routine to be performed inside the end of conversion ISR
			store the first sampled value and compute the bus voltage and temperature
			sensor sampling  and disable the ext. adc triggering.
* Input           : None
* Output         : Return false after first EOC, return true after second EOC
* Return         : None
*******************************************************************************/
u8 SVPWMEOCEvent(void)
{
  if (bDistEnab == 1)
  {
    // Diabling the Injectec conversion for ADC1
    ADC_ExternalTrigInjectedConvCmd(ADC1,DISABLE);
  }
  
  // Store the Bus Voltage and temperature sampled values
  h_ADCTemp = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_1);      // 读取温度(ADC2 注入序列1)
  h_ADCBusvolt = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_2);   // 读取母线电压(ADC2 注入序列2)
  
  return ((u8)(1));
}

/*******************************************************************************
* Function Name  : SVPWMUpdateEvent
* 功能说明(中文) : TIM1 更新事件(下溢)中断内调用的核心处理。职责:
*                  - 当移相相发生变化时, 在临界区切换对应通道的 CCMR 模式(PWM<->Toggle),
*                    并调整 DMA 使能与缓冲长度;
*                  - 若移相相占空比过小(< DMABURSTMIN_x), 强制低/翻转并复位状态;
*                  - 刷新 CH1~CH4 的 DMA 缓冲(下一周期的占空比与两个采样触发点);
*                  - 保存当前移相状态, 重新把 ADC1 触发源设为 TIM1 CC4 并使能注入转换。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 直接操作 TIM1 寄存器与 DMA 通道寄存器; 与 SVPWM_1ShuntCalcDutyCycles()
*                  配合完成单电阻采样时序; 由 TIM1_UP 中断服务程序周期性调用。
* Description    :  Routine to be performed inside the update event ISR. 
                    It is used to set the PWM output mode of the four channels 
                    (Toggle or PWM), enable or disable the DMA event for each 
                    channel, update the DMA buffers, update the DMA lenght and 
                    finally it re-enables the external ADC triggering.	
* Input           : None
* Output         : None
* Return         : None
*******************************************************************************/
void SVPWMUpdateEvent(void)
{
  if (bInverted_pwm_new != bInverted_pwm)  
  {
    // Critical point start
    
    // Update CCMR (OC Mode)
    TIM1->CCMR1 = hPreloadCCMR1Disable; // Switch to Frozen
    TIM1->CCMR1 = hPreloadCCMR1Set; // Switch to Normal or Toggle
    
    TIM1->CCMR2 = hPreloadCCMR2Disable; // Switch to Frozen
    TIM1->CCMR2 = hPreloadCCMR2Set; // Switch to Normal or Toggle
    
    // Disable DMA (in this period is not inverted)
    switch (bInverted_pwm)
    {
    case INVERT_A:
      //TIM1_DMACmd(TIM1_DMA_CC1, DISABLE);
      TIM1->DIER &= ~TIM_DMA_CC1;
      DMA1_Channel2->CCR &= CCR_ENABLE_Reset;
      TIM1->CCR1 = hCCRBuff[0];
      break;
    case INVERT_B:
      //TIM1_DMACmd(TIM1_DMA_CC2, DISABLE);
      TIM1->DIER &= ~TIM_DMA_CC2;
      DMA1_Channel3->CCR &= CCR_ENABLE_Reset;
      TIM1->CCR2 = hCCRBuff[1];
      break;
    case INVERT_C:
      //TIM1_DMACmd(TIM1_DMA_CC3, DISABLE);
      TIM1->DIER &= ~TIM_DMA_CC3;
      DMA1_Channel6->CCR &= CCR_ENABLE_Reset;
      TIM1->CCR3 = hCCRBuff[2];
      break;
    default:
      break;
    }
    
    // Enable DMA (in this period channel is toggled)
    switch (bInverted_pwm_new)
    {
    case INVERT_A:
      DMA1_Channel2->CCR &= CCR_ENABLE_Reset;
      DMA1_Channel2->CNDTR = 4;
      DMA1_Channel2->CCR |= CCR_ENABLE_Set;
      
      //TIM1_DMACmd(TIM1_DMA_CC1, ENABLE); 
      TIM1->DIER |= TIM_DMA_CC1;  
      
      if (bInverted_pwm_new == INVERT_A)
      {
        if (hCCRBuff[0] <= MINTIMCNTUPHAND)
        {
          TIM1->CCMR1 = TIM1->CCMR1 & 0xFF8F; // Switch to Frozen Ch1
          TIM1->CCMR1 = TIM1->CCMR1 | 0x0040; // Force Low Ch1
          TIM1->CCMR1 = TIM1->CCMR1 & 0xFF8F; // Switch to Frozen Ch1
          TIM1->CCMR1 = TIM1->CCMR1 | 0x0030; // Switch to Toggle Ch1
          
          //TIM1_GenerateEvent(TIM1_EventSource_CC1);
          TIM1->EGR |= TIM_EventSource_CC1;
        }
      }
      break;
      
    case INVERT_B:
      DMA1_Channel3->CCR &= CCR_ENABLE_Reset;
      DMA1_Channel3->CNDTR = 4;
      DMA1_Channel3->CCR |= CCR_ENABLE_Set;
      
      //TIM1_DMACmd(TIM1_DMA_CC2, ENABLE); 
      TIM1->DIER |= TIM_DMA_CC2;
      
      if (bInverted_pwm_new == INVERT_B)
      {
        if (hCCRBuff[1] <= MINTIMCNTUPHAND)
        {
          TIM1->CCMR1 = TIM1->CCMR1 & 0x8FFF; // Switch to Frozen Ch2
          TIM1->CCMR1 = TIM1->CCMR1 | 0x4000; // Force Low Ch2
          TIM1->CCMR1 = TIM1->CCMR1 & 0x8FFF; // Switch to Frozen Ch2
          TIM1->CCMR1 = TIM1->CCMR1 | 0x3000; // Switch to Toggle Ch2
          
          //TIM1_GenerateEvent(TIM1_EventSource_CC2);
          TIM1->EGR |= TIM_EventSource_CC2;
        }
      }
      break;
      
    case INVERT_C:
      DMA1_Channel6->CCR &= CCR_ENABLE_Reset;
      DMA1_Channel6->CNDTR = 4;
      DMA1_Channel6->CCR |= CCR_ENABLE_Set;
      
      //TIM1_DMACmd(TIM1_DMA_CC3, ENABLE);
      TIM1->DIER |= TIM_DMA_CC3;
      
      if (bInverted_pwm_new == INVERT_C)
      {
        if (hCCRBuff[2] <= MINTIMCNTUPHAND)
        {
          TIM1->CCMR2 = TIM1->CCMR2 & 0xFF8F; // Switch to Frozen Ch3
          TIM1->CCMR2 = TIM1->CCMR2 | 0x0040; // Force Low Ch3
          TIM1->CCMR2 = TIM1->CCMR2 & 0xFF8F; // Switch to Frozen Ch3
          TIM1->CCMR2 = TIM1->CCMR2 | 0x0030; // Switch to Toggle Ch3
          
          //TIM1_GenerateEvent(TIM1_EventSource_CC3);
          TIM1->EGR |= TIM_EventSource_CC3;
        }
      }
      break;
      
    default:
      break;
    }
    
    // Critical point stop
    
    // Adjust the DMA lenght for Ch4
    //DMA_Cmd(DMA_Channel4, DISABLE);
    DMA1_Channel4->CCR &= CCR_ENABLE_Reset;
    if (bInverted_pwm_new == INVERT_NONE)
    { 
      // Length 2
      DMA1_Channel4->CNDTR = 2;
    }
    else
    {
      // Length 4
      DMA1_Channel4->CNDTR = 4;
    }
    //DMA_Cmd(DMA_Channel4, ENABLE);
    DMA1_Channel4->CCR |= CCR_ENABLE_Set;
  }
  
  switch (bInverted_pwm_new)
  {
  case INVERT_A: 
    if (hCCRBuff[0] <= DMABURSTMIN_A)
    {
      // Reset the status
      TIM1->CCMR1 = TIM1->CCMR1 & 0xFF8F; // Switch to Frozen Ch1
      TIM1->CCMR1 = TIM1->CCMR1 | 0x0040; // Force Low Ch1
      TIM1->CCMR1 = TIM1->CCMR1 & 0xFF8F; // Switch to Frozen Ch1
      TIM1->CCMR1 = TIM1->CCMR1 | 0x0030; // Switch to Toggle Ch1
      
      //TIM1_DMACmd(TIM1_DMA_CC1, ENABLE); 
      TIM1->DIER |= TIM_DMA_CC1;
      DMA1_Channel2->CCR &= CCR_ENABLE_Reset;
      DMA1_Channel2->CNDTR = 4;
      DMA1_Channel2->CCR |= CCR_ENABLE_Set;

      TIM_GenerateEvent(TIM1, TIM_EventSource_CC1);
      
      if (dvDutyValues.hTimePhA < DMABURSTMIN_A)
        dvDutyValues.hTimePhA = DMABURSTMIN_A;
    }
    break;  
      
  case INVERT_B:
    if (hCCRBuff[1] <= DMABURSTMIN_B)
    {
      // Reset the status
      TIM1->CCMR1 = TIM1->CCMR1 & 0x8FFF; // Switch to Frozen Ch2
      TIM1->CCMR1 = TIM1->CCMR1 | 0x4000; // Force Low Ch2
      TIM1->CCMR1 = TIM1->CCMR1 & 0x8FFF; // Switch to Frozen Ch2
      TIM1->CCMR1 = TIM1->CCMR1 | 0x3000; // Switch to Toggle Ch2
      
      //TIM1_DMACmd(TIM1_DMA_CC2, ENABLE); 
      TIM1->DIER |= TIM_DMA_CC2;
      DMA1_Channel3->CCR &= CCR_ENABLE_Reset;
      DMA1_Channel3->CNDTR = 4;
      DMA1_Channel3->CCR |= CCR_ENABLE_Set;

      TIM_GenerateEvent(TIM1, TIM_EventSource_CC2);
      
      if (dvDutyValues.hTimePhB < DMABURSTMIN_B)
        dvDutyValues.hTimePhB = DMABURSTMIN_B;
    }
    break;
  case INVERT_C: 
    if (hCCRBuff[2] <= DMABURSTMIN_C)
    {
      // Reset the status
      TIM1->CCMR2 = TIM1->CCMR2 & 0xFF8F; // Switch to Frozen Ch3
      TIM1->CCMR2 = TIM1->CCMR2 | 0x0040; // Force Low Ch3
      TIM1->CCMR2 = TIM1->CCMR2 & 0xFF8F; // Switch to Frozen Ch3
      TIM1->CCMR2 = TIM1->CCMR2 | 0x0030; // Switch to Toggle Ch3
      
      //TIM1_DMACmd(TIM1_DMA_CC3, ENABLE); 
      TIM1->DIER |= TIM_DMA_CC3;
      DMA1_Channel6->CCR &= CCR_ENABLE_Reset;
      DMA1_Channel6->CNDTR = 4;
      DMA1_Channel6->CCR |= CCR_ENABLE_Set;

      TIM_GenerateEvent(TIM1, TIM_EventSource_CC3);
      
      if (dvDutyValues.hTimePhC < DMABURSTMIN_C)
        dvDutyValues.hTimePhC = DMABURSTMIN_C;
    }
    break;
  }

  // Update remaining DMA buffer
  hCCDmaBuffCh4[0] = dvDutyValues.hTimeSmp2; // Second point 
  hCCDmaBuffCh4[1] = dvDutyValues.hTimeSmp2;
  hCCDmaBuffCh4[2] = dvDutyValues.hTimeSmp1; // First point
  hCCDmaBuffCh4[3] = dvDutyValues.hTimeSmp1;
  
  hCCDmaBuffCh1[2] = dvDutyValues.hTimePhA;
  hCCDmaBuffCh1[3] = dvDutyValues.hTimePhA;
  
  hCCDmaBuffCh2[2] = dvDutyValues.hTimePhB;
  hCCDmaBuffCh2[3] = dvDutyValues.hTimePhB;
  
  hCCDmaBuffCh3[2] = dvDutyValues.hTimePhC;
  hCCDmaBuffCh3[3] = dvDutyValues.hTimePhC;
  
  bInverted_pwm = bInverted_pwm_new;   // 记住本周期采用的移相状态
  
  /* ADC1 Injected conversions trigger is TIM1 TRGO */ 
  ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);   // 触发源改为 CH4 比较事件(采样时刻)
  
  // Enabling the Injectec conversion for ADC1
  ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE);   // 使能 ADC1 注入组外部触发
}


#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/  
