/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : STM32x_svpwm_3shunt.c
* Author             : IMS Systems Lab
* Date First Issued  : 28/11/07
* Description        : 3 shunt resistors current reading module
********************************************************************************
* History:
* 28/11/07 v1.0
* 29/05/08 v2.0
* 03/07/08 v2.0.1
* 09/07/08 v2.0.2
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
* 文件说明(中文) : 三电阻(3-Shunt)电流采样 SVPWM(空间矢量脉宽调制)模块实现。
*
*  一、在 FOC(磁场定向控制)中的角色
*    本模块位于功率级与电流采样层，向上为电流环/电机控制层服务，向下直接驱动
*    TIM1 三相 PWM 与 ADC1/ADC2。电流环输出 α-β 定子电压指令后调用
*    SVPWM_3ShuntCalcDutyCycles()，本模块据其计算六路 PWM 的三相占空比(CCR1/2/3)
*    并生成 CC4 触发信号；随后在三相下桥臂导通窗口内触发 ADC 注入组采样，
*    SVPWM_3ShuntGetPhaseCurrentValues() 把采样值换算为两相定子电流(缺一相时由
*    Ia+Ib+Ic=0 重构)，供 Clarke/Park 变换使用。
*
*  二、三相逆变桥与 SVPWM 原理
*    三相逆变桥由 6 个开关管(MOSFET/IGBT)组成，分为 A/B/C 三个桥臂，每个桥臂
*    有上管(接直流母线正)与下管(接直流母线负)。同一桥臂上下管不能同时导通。
*    用 1 表示某相上管导通(该相接母线正)、0 表示下管导通，则三相桥共有 2^3=8
*    种开关状态；其中 000 与 111 为零矢量，其余 6 种为基本电压矢量(V1~V6)，
*    在 α-β 平面上间隔 60° 构成正六边形；任意期望电压矢量可由其所在扇区的两个
*    相邻基本矢量与零矢量按作用时间合成。
*
*  三、扇区(Sector)判断
*    由 α-β 电压指令算出三个辅助量 wX/wY/wZ，再依据它们的正负号(等价于依据
*    Vα/Vβ 的符号与幅值比较)确定当前指令矢量落在 1~6 号扇区中的哪一个。
*
*  四、T1/T2 与占空比
*    每个 PWM 周期内相邻基本矢量的作用时间 T1、T2 由 wX/wY/wZ 线性组合得到，
*    经定点定标(除以 2^17)换算为 TIM1 计数值，再叠加中点偏移(T/8 = PWM_PERIOD/2)
*    得到三相占空比 hTimePhA/B/C，写入 TIM1->CCR1/2/3。
*******************************************************************************/
#include "STM32F10x_MCconf.h"

#ifdef THREE_SHUNT

/* Includes-------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_svpwm_3shunt.h"
#include "MC_Globals.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* NB_CONVERSIONS: 零电流校准阶段每个电流通道的采样次数(共 16 次，累加后取平均)。 */
#define NB_CONVERSIONS 16

/* SQRT_3: 常数 √3≈1.732051，SVPWM 计算中相邻基本矢量作用时间需要 √3 系数。 */
#define SQRT_3		1.732051
/* T: 一个 PWM 周期对应的计数值 = PWM_PERIOD*4(中心对齐上下计数，且占空比标幺
   系数为 2)，单位 TIM1 计数；T/8 正好等于 PWM_PERIOD/2，即 50% 占空比中点。 */
#define T		    (PWM_PERIOD * 4)
/* T_SQRT3: √3*T 的整数近似，用于把 α 轴分量 wUAlpha 换算到与 wUBeta 同一量纲。 */
#define T_SQRT3     (u16)(T * SQRT_3)

/* SECTOR_1..6: 指令电压矢量所在扇区编号，1~6 依次对应 0°~60°、60°~120°… 的六边形扇区。 */
#define SECTOR_1	(u32)1
#define SECTOR_2	(u32)2
#define SECTOR_3	(u32)3
#define SECTOR_4	(u32)4
#define SECTOR_5	(u32)5
#define SECTOR_6	(u32)6

/* 三相电流采样通道号(ADC1_IN11/12/13)，用于构造 ADC 注入序列寄存器 JSQR。 */
#define PHASE_A_ADC_CHANNEL     ADC_Channel_11
#define PHASE_B_ADC_CHANNEL     ADC_Channel_12
#define PHASE_C_ADC_CHANNEL     ADC_Channel_13

// Setting for sampling of VBUS and Temp after currents sampling
/* 电流采样通道号左移 10 位后与序列长度位拼装成 ADC1 的 JSQR 值(通道号位于
   JSQR 的第 10~14 位)，故移位量为 10。 */
#define PHASE_A_MSK       (u32)((u32)(PHASE_A_ADC_CHANNEL) << 10)
#define PHASE_B_MSK       (u32)((u32)(PHASE_B_ADC_CHANNEL) << 10)
#define PHASE_C_MSK       (u32)((u32)(PHASE_C_ADC_CHANNEL) << 10)

// Settings for current sampling only
/* 下面被注释掉的一组是"仅做电流采样"的另一种写法：通道号左移 15 位(此时 JSQR
   按 5 位一个字段排布)。当前未启用。 */
/*#define PHASE_A_MSK       (u32)((u32)(PHASE_A_ADC_CHANNEL) << 15)
#define PHASE_B_MSK       (u32)((u32)(PHASE_B_ADC_CHANNEL) << 15)
#define PHASE_C_MSK       (u32)((u32)(PHASE_C_ADC_CHANNEL) << 15)*/

// Setting for sampling of VBUS and Temp after currents sampling
/* 温度反馈/母线电压反馈通道号左移 15 位，作为 ADC 注入序列第 2 个转换的通道字段。 */
#define TEMP_FDBK_MSK     (u32)((u32)(TEMP_FDBK_CHANNEL) <<15)
#define BUS_VOLT_FDBK_MSK (u32)((u32)(BUS_VOLT_FDBK_CHANNEL) <<15)

// Settings for current sampling only
/* 仅做电流采样时温度/母线电压通道掩码置 0(不使用第 2 个转换)。当前未启用。 */
//#define TEMP_FDBK_MSK     (u32)(0)
//#define BUS_VOLT_FDBK_MSK (u32)(0)

// Setting for sampling of VBUS and Temp after currents sampling
/* SEQUENCE_LENGHT: ADC 注入序列长度字段(JSQR 第 20 位，编码"2 个转换"，
   即电流 + 母线电压/温度)，当前启用。 */
#define SEQUENCE_LENGHT    0x00100000

// Settings for current sampling only
/* 仅做电流采样时序列长度为 0(编码"1 个转换")。当前未启用。 */
//#define SEQUENCE_LENGHT    0x00000000

/* NVIC 优先级：ADC1_2 中断与 TIM1 更新中断抢占优先级 1、子优先级 0；
   刹车中断抢占优先级 0(最高)、子优先级 0。 */
#define ADC_PRE_EMPTION_PRIORITY 1
#define ADC_SUB_PRIORITY 0

#define BRK_PRE_EMPTION_PRIORITY 0
#define BRK_SUB_PRIORITY 0

#define TIM1_UP_PRE_EMPTION_PRIORITY 1
#define TIM1_UP_SUB_PRIORITY 0

/* LOW_SIDE_POLARITY: 空闲(Idle)状态时下桥臂输出的电平，Reset=低电平有效，
   保证刹车/空闲时下管保持关断(安全状态)。 */
#define LOW_SIDE_POLARITY  TIM_OCIdleState_Reset

/* PWM4_MODE: PWM4(CC4)通道作为 ADC 触发用时的两种极性配置。
   PWM2_MODE=0 表示按 PWM2 模式(下降沿触发)，PWM1_MODE=1 表示按 PWM1 模式。 */
#define PWM2_MODE 0
#define PWM1_MODE 1

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* bSector: 当前 α-β 电压指令所处扇区编号(1~6)，由 SVPWM_3ShuntCalcDutyCycles()
   计算写入，SVPWM_3ShuntGetPhaseCurrentValues() 读取它决定哪一相电流可测。 */
u8  bSector;  

/* hPhaseAOffset/B/C: 三个电流通道的零电流(偏置)ADC 值，由校准例程求得，
   单位 ADC 原始码(12 位，左对齐)。实测电流 = (偏置 - 当前值)<<1 后再饱和。 */
u16 hPhaseAOffset;
u16 hPhaseBOffset;
u16 hPhaseCOffset;

/* PWM4Direction: CC4 通道极性方向标志(0=PWM2_MODE, 1=PWM1_MODE)，决定 ADC 触发
   边沿方向，默认 PWM2_MODE。 */
u8 PWM4Direction=PWM2_MODE;

/* Private function prototypes -----------------------------------------------*/

/* SVPWM_InjectedConvConfig: 配置 ADC1 注入组用于电流+母线电压/温度采样并开启
   看门狗(过压保护)，仅在开关 SVPWM_3ShuntCurrentReadingCalibration() 末尾被调用。 */
void SVPWM_InjectedConvConfig(void);

/*******************************************************************************
* Function Name  : SVPWM_3ShuntInit
* Description    : It initializes PWM and ADC peripherals
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************
* 功能说明(中文) : 三电阻 SVPWM 模块的初始化例程，系统上电时调用一次(非周期)。
*                  依次完成：ADC 时钟/GPIO 模拟输入配置 → TIM1 时基与三相互补
*                  PWM 配置(中心对齐、死区、刹车、CC4 触发) → ADC1/ADC2 注入组
*                  配置与校准 → NVIC 中断优先级配置 → 调用零电流偏置校准例程。
*                  初始化后 TIM1 已启动计数，但电流采样中断仍处于关闭状态。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 依赖 RCC 时钟树(ADCCLK=PCLK2/6=12MHz)；PWM_PERIOD/DEADTIME/
*                  REP_RATE/SAMPLING_TIME_CK 等来自 MC_pwm_3shunt_prm.h 与
*                  MC_Control_Param.h；副作用是把 PA3/PC0~PC4 设为模拟输入、
*                  PE8~PE14 设为 TIM1 复用输出并锁定上桥臂引脚，切忌在别处重复
*                  配置这些引脚。函数末尾调用校准例程，要求此时电机已停止、无
*                  电流流过采样电阻。
*******************************************************************************/
void SVPWM_3ShuntInit(void)
{ 
    ADC_InitTypeDef ADC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM1_TimeBaseStructure;
    TIM_OCInitTypeDef TIM1_OCInitStructure;
    TIM_BDTRInitTypeDef TIM1_BDTRInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* ADC1, ADC2, DMA, GPIO, TIM1 clocks enabling -----------------------------*/
    
    /* ADCCLK = PCLK2/6 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);                    // ADCCLK = 72MHz/6 = 12MHz(ADC 上限 14MHz)
    
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
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |GPIO_Pin_4; // PC0~PC4: 温度/三相电流等模拟输入
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    GPIO_StructInit(&GPIO_InitStructure);
    /****** Configure PA.03 (ADC Channels [3]) as analog input ******/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;            // PA3: 母线电压反馈
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* TIM1 Peripheral Configuration -------------------------------------------*/
    /* TIM1 Registers reset */
    TIM_DeInit(TIM1);
    TIM_TimeBaseStructInit(&TIM1_TimeBaseStructure);
    /* Time Base configuration */
    TIM1_TimeBaseStructure.TIM_Prescaler = 0x0;                          // 预分频 PSC=0，计数时钟 72MHz
    TIM1_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1; // 中心对齐模式 1，产生对称 PWM 且下溢/上溢
    TIM1_TimeBaseStructure.TIM_Period = PWM_PERIOD;                      // ARR=PWM 半周期计数
    TIM1_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
    
    // Initial condition is REP=0 to set the UPDATE only on the underflow
    TIM1_TimeBaseStructure.TIM_RepetitionCounter = REP_RATE;
    TIM_TimeBaseInit(TIM1, &TIM1_TimeBaseStructure);
    
    TIM_OCStructInit(&TIM1_OCInitStructure);
    /* Channel 1, 2,3 in PWM mode */
    TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;                   // PWM1 模式：CNT<CCR 时输出有效
    TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;       // 使能上桥臂输出
    TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;     // 使能下桥臂互补输出
    TIM1_OCInitStructure.TIM_Pulse = 0x505; //dummy value                // 初始占空比占位，稍后由 SVPWM 计算更新
    TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;           // 上桥臂高电平有效
    TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         // 下桥臂互补高电平有效
    TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;        // 空闲时上桥臂输出低
    TIM1_OCInitStructure.TIM_OCNIdleState = LOW_SIDE_POLARITY;           // 空闲时下桥臂输出低(安全)
    
    TIM_OC1Init(TIM1, &TIM1_OCInitStructure); 
    TIM_OC2Init(TIM1, &TIM1_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM1_OCInitStructure);
    
    /*Timer1 alternate function full remapping*/  
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);                     // TIM1 通道全重映射到 PE(可用 PE8~PE14 输出六路 PWM)
    
    GPIO_StructInit(&GPIO_InitStructure);
    /* GPIOE Configuration: Channel 1, 1N, 2, 2N, 3, 3N and 4 Output */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | // PE8~PE14: 三相互补 PWM + CC4 触发
        GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                      // 复用推挽输出，50MHz
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE, &GPIO_InitStructure); 
    
    /* Lock GPIOE Pin9 and Pin11 Pin 13 (High sides) */
    GPIO_PinLockConfig(GPIOE, GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13);   // 锁定三个上桥臂引脚配置，防止被误改
    
    GPIO_StructInit(&GPIO_InitStructure);
    /* GPIOE Configuration: BKIN pin */   
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOE, &GPIO_InitStructure);  
    
    TIM_OCStructInit(&TIM1_OCInitStructure);
    /* Channel 4 Configuration in OC */
    TIM1_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;                   // PWM2 模式，CC4 仅作为 ADC 触发源(不驱动桥臂)
    TIM1_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM1_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;    // CC4 无互补输出
    TIM1_OCInitStructure.TIM_Pulse = PWM_PERIOD - 1;                     // 默认触发点靠后，实际由 SVPWM 每周期更新
    
    TIM1_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
    TIM1_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;         
    TIM1_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM1_OCInitStructure.TIM_OCNIdleState = LOW_SIDE_POLARITY;            
    
    TIM_OC4Init(TIM1, &TIM1_OCInitStructure);
    
    /* Enables the TIM1 Preload on CC1 Register */
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    /* Enables the TIM1 Preload on CC2 Register */
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    /* Enables the TIM1 Preload on CC3 Register */
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    /* Enables the TIM1 Preload on CC4 Register */
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    
    /* Automatic Output enable, Break, dead time and lock configuration*/
    TIM1_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;         // 运行模式下关闭输出时的安全状态
    TIM1_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;         // 空闲模式下关闭输出时的安全状态
    TIM1_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1; 
    TIM1_BDTRInitStructure.TIM_DeadTime = DEADTIME;                      // 死区时间(计数)，防止上下桥臂直通
    TIM1_BDTRInitStructure.TIM_Break = TIM_Break_Enable;                 // 使能刹车输入(BKIN)
    TIM1_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_Low;    // 刹车信号低电平有效
    TIM1_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Disable; // 不自动恢复输出，需软件清除
    
    TIM_BDTRConfig(TIM1, &TIM1_BDTRInitStructure);
    
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);                // TRGO 选更新事件，用于触发 ADC
    
    TIM_ClearITPendingBit(TIM1, TIM_IT_Break);
    TIM_ITConfig(TIM1, TIM_IT_Break,ENABLE);
    
    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);
    
    // Resynch to have the Update evend during Undeflow
    TIM_GenerateEvent(TIM1, TIM_EventSource_Update);                     // 软件产生更新事件，重新同步 REP 计数
    
    // Clear Update Flag
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    
    TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);                          // 初始化阶段先关闭更新中断
    
    TIM_ITConfig(TIM1, TIM_IT_CC4,DISABLE);                              // 关闭 CC4 中断(仅用 CC4 事件触发 ADC)
    
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
    ADC_InitStructure.ADC_Mode = ADC_Mode_InjecSimult;                   // ADC1/ADC2 注入组同步模式，两片 ADC 同时转换
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                         // 扫描模式
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;                  // 单次转换(由触发启动)
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  // 规则组不使用外部触发
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;                // 数据左对齐(12 位结果在高位)
    ADC_InitStructure.ADC_NbrOfChannel = 1;                              // 规则组转换通道数
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
    
    SVPWM_3ShuntCurrentReadingCalibration();                             // 电机静止时测量三相零电流偏置
    
    /* ADC2 Injected conversions configuration */ 
    ADC_InjectedSequencerLengthConfig(ADC2,2);                           // ADC2 注入序列 2 个转换：A 相电流 + 温度
    
    ADC_InjectedChannelConfig(ADC2, PHASE_A_ADC_CHANNEL, 1, 
                              SAMPLING_TIME_CK);                          // 第 1 个转换：A 相电流(第 2 通道在每周期由 SVPWM 更新)
    ADC_InjectedChannelConfig(ADC2, TEMP_FDBK_CHANNEL, 2,
                              SAMPLING_TIME_CK);                          // 第 2 个转换：NTC 温度
    
    /* Configure one bit for preemption priority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                      // 2 位抢占 + 2 位子优先级分组
    
    //NVIC_StructInit(&NVIC_InitStructure);
    /* Enable the ADC Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;                    // ADC1/ADC2 注入转换结束中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = ADC_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = ADC_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable the Update Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;                   // TIM1 更新中断(高级电流读取模式使用)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM1_UP_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = TIM1_UP_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable the TIM1 BRK Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_IRQn;                  // TIM1 刹车中断(过流/故障保护)
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = BRK_PRE_EMPTION_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = BRK_SUB_PRIORITY;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    //  NVIC_Init(&NVIC_InitStructure);                                  // 刹车中断 NVIC 使能被注释掉，实际不产生 BRK 中断
} 


/*******************************************************************************
* Function Name  : SVPWM_3ShuntCurrentReadingCalibration
* Description    : Store zero current converted values for current reading 
network offset compensation in case of 3 shunt resistors 
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************
* 功能说明(中文) : 三相电流通道的零电流(偏置)校准例程。在电机静止、无相电流时，
*                  用软件触发 ADC1 注入组对三相电流通道连续采样 16 次，累加每个
*                  通道的 (ADC 值 >> 3) 后再累加 16 次(等价于取平均值并保留 3 位
*                  小数)，得到 hPhaseAOffset/B/C。仅在 SVPWM_3ShuntInit() 中调用
*                  一次，属上电初始化例程。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 依赖 ADC1 已完成校准且电机确实静止；写入全局偏置变量
*                  hPhaseA/B/COffset(单位: ADC 原始码，左对齐)；结束时调用
*                  SVPWM_InjectedConvConfig() 把 ADC1 切回正常运行配置。
*******************************************************************************/

void SVPWM_3ShuntCurrentReadingCalibration(void)
{
    static u16 bIndex;                                    // 采样循环计数(index 0~15)
    u16 adctemp;                                          // 暂存 A 相注入转换结果
    
    /* ADC1 Injected group of conversions end interrupt disabling */
    ADC_ITConfig(ADC1, ADC_IT_JEOC, DISABLE);             // 校准时关闭注入转换结束中断，避免误入 ISR
    
    hPhaseAOffset=0;                                      // 三相偏置累加器清零
    hPhaseBOffset=0;
    hPhaseCOffset=0;
    
    /* ADC1 Injected conversions trigger is given by software and enabled */ 
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_None);  // 注入组改由软件触发
    ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE); 
    
    /* ADC1 Injected conversions configuration */ 
    ADC_InjectedSequencerLengthConfig(ADC1,3);            // 注入序列长度 3：A/B/C 三相电流
    ADC_InjectedChannelConfig(ADC1, PHASE_A_ADC_CHANNEL,1,SAMPLING_TIME_CK);   // 第 1 个: A 相电流
    ADC_InjectedChannelConfig(ADC1, PHASE_B_ADC_CHANNEL,2,SAMPLING_TIME_CK);   // 第 2 个: B 相电流
    ADC_InjectedChannelConfig(ADC1, PHASE_C_ADC_CHANNEL,3,SAMPLING_TIME_CK);   // 第 3 个: C 相电流
    
    /* Clear the ADC1 JEOC pending flag */
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);  
    ADC_SoftwareStartInjectedConvCmd(ADC1,ENABLE);        // 软件启动第一次注入转换
    
    /* ADC Channel used for current reading are read 
    in order to get zero currents ADC values*/ 
    for(bIndex=0; bIndex <16; bIndex++)                   // 共 16 次，累加求和以平滑噪声
    {
        while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_JEOC)) { } // 等待注入组转换结束
        
        adctemp = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_1);
        hPhaseAOffset += (adctemp>>3);                    // 先右移 3 位再累加，避免 16 次累加溢出 u16
        hPhaseBOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2)>>3);
        hPhaseCOffset += (ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_3)>>3);    
        
        /* Clear the ADC1 JEOC pending flag */
        ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);               // 清标志后再次软件触发
        ADC_SoftwareStartInjectedConvCmd(ADC1,ENABLE);
    }
    
    SVPWM_InjectedConvConfig();                           // 恢复 ADC1 为正常运行配置(电流+母线电压+看门狗)
}



/*******************************************************************************
* Function Name  : SVPWM_InjectedConvConfig
* Description    : This function configure ADC1 for 3 shunt current 
*                  reading and temperature and voltage feedbcak after a 
*                  calibration of the three utilized ADC Channels
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************
* 功能说明(中文) : 把 ADC 注入组配置为"正常运行"模式：ADC1 序列为 B 相电流 +
*                  母线电压，ADC2 序列为 A/C 相电流 + 温度(第 1 通道每周期由
*                  SVPWM_3ShuntCalcDutyCycles() 更新)；ADC1 注入触发源选 TIM1
*                  TRGO；同时开启母线过压模拟看门狗(AWD)与 JEOC 中断。仅在
*                  SVPWM_3ShuntCurrentReadingCalibration() 结尾调用一次。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : AWD 单通道监测母线电压通道，阈值 OVERVOLTAGE_THRESHOLD(单位
*                  ADC 码)>>3 与右移 3 为同一标度(偏置累加时用的右移 3 格式)；
*                  使能 ADC_IT_JEOC|ADC_IT_AWD 后，过压/转换结束都会进入 ADC 中断。
*******************************************************************************/
void SVPWM_InjectedConvConfig(void)
{
    /* ADC1 Injected conversions configuration */ 
    ADC_InjectedSequencerLengthConfig(ADC1,2);            // ADC1 注入序列 2 个转换
    ADC_InjectedSequencerLengthConfig(ADC2,2);            // ADC2 注入序列 2 个转换
    
    ADC_InjectedChannelConfig(ADC1, PHASE_B_ADC_CHANNEL, 1, 
                              SAMPLING_TIME_CK);          // ADC1 第 1 个: B 相电流(每周期动态改写)
    ADC_InjectedChannelConfig(ADC1, BUS_VOLT_FDBK_CHANNEL, 
                              2, SAMPLING_TIME_CK);       // ADC1 第 2 个: 母线电压
    
    /* ADC1 Injected conversions trigger is TIM1 TRGO */ 
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);  // 注入组改由 TIM1 TRGO 触发
    
    ADC_ExternalTrigInjectedConvCmd(ADC2,ENABLE);         // 使能 ADC2 注入组的外部触发(与 ADC1 同步)
    
    /* Bus voltage protection initialization*/                            
    ADC_AnalogWatchdogCmd(ADC1,ADC_AnalogWatchdog_SingleInjecEnable);          // 看门狗仅监测注入组单通道
    ADC_AnalogWatchdogSingleChannelConfig(ADC1,BUS_VOLT_FDBK_CHANNEL);         // 监测母线电压通道
    ADC_AnalogWatchdogThresholdsConfig(ADC1, OVERVOLTAGE_THRESHOLD>>3,0x00);   // 上限=过压阈值(右移 3 对齐偏置格式)，下限=0
    
    
    /* ADC1 Injected group of conversions end and Analog Watchdog interrupts
    enabling */
    ADC_ITConfig(ADC1, ADC_IT_JEOC | ADC_IT_AWD, ENABLE); // 使能注入转换结束中断与模拟看门狗中断
}

/*******************************************************************************
* Function Name  : SVPWM_3ShuntGetPhaseCurrentValues
* Description    : This function computes current values of Phase A and Phase B 
*                 in q1.15 format starting from values acquired from the A/D 
*                 Converter peripheral.
* Input          : None
* Output         : Stat_Curr_a_b
* Return         : None
*******************************************************************************
* 功能说明(中文) : 把 ADC 注入寄存器中的原始采样码换算成两相定子电流 A、B(缺一相
*                  时由基尔霍夫定律 Ia+Ib+Ic=0 重构)，结果以 q1.15 定点格式返回
*                  (1.0 对应 32767)。每个电流采样周期(见 SVPWMUpdateEvent/EOC 回调)
*                  调用一次，属周期性执行。
* 参数(中文)     : 无
* 返回(中文)     : Curr_Components 结构，
*                  qI_Component1 = Ia(A 相电流，q1.15, 范围[-32768,32767])，
*                  qI_Component2 = Ib(B 相电流，q1.15, 范围[-32768,32767])。
* 备注(中文)     : 读取全局 bSector 决定哪两相可测、哪一相需重构；读取全局偏置
*                  hPhaseA/B/COffset 与 ADC1/ADC2 的 JDR1(注入数据寄存器 1)；
*                  直接读寄存器，不经过库函数；所有结果都做 ±S16 饱和限幅，
*                  防止大电流或干扰导致数值溢出。
*******************************************************************************/
Curr_Components SVPWM_3ShuntGetPhaseCurrentValues(void)
{
    Curr_Components Local_Stator_Currents;                // 返回用的两相电流结构
    s32 wAux;                                             // 中间计算量(32 位有符号，防溢出)
    
    switch (bSector)                                      // 依据当前扇区选择可测量的相
    {
    case 4:
    case 5: //Current on Phase C not accessible     
        // Ia = (hPhaseAOffset)-(ADC Channel 11 value)    
        wAux = (s32)(hPhaseAOffset)- ((ADC1->JDR1)<<1);   // A 相: 偏置-当前值；JDR1 左移 1 与偏置的(>>3 ×16)定标对齐
        //Saturation of Ia 
        if (wAux < S16_MIN)                               // 负向过载，下限饱和
        {
            Local_Stator_Currents.qI_Component1= S16_MIN;
        }  
        else  if (wAux > S16_MAX)                         // 正向过载，上限饱和
        { 
            Local_Stator_Currents.qI_Component1= S16_MAX;
        }
        else
        {
            Local_Stator_Currents.qI_Component1= wAux;    // 正常范围，直接赋 q1.15 电流值
        }
        
        // Ib = (hPhaseBOffset)-(ADC Channel 12 value)
        wAux = (s32)(hPhaseBOffset)-((ADC2->JDR1)<<1);    // B 相: 偏置-当前值(来自 ADC2 注入寄存器 1)
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
        break;
        
    case 6:
    case 1:  //Current on Phase A not accessible     
        // Ib = (hPhaseBOffset)-(ADC Channel 12 value)
        wAux = (s32)(hPhaseBOffset)-((ADC1->JDR1)<<1);    // B 相可直接测量
        //Saturation of Ib 
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
        // Ia = -Ic -Ib 
        wAux = ((ADC2->JDR1)<<1)-hPhaseCOffset - Local_Stator_Currents.qI_Component2; // C 相测值换算出 Ic，再用 Ia=-Ib-Ic 重构 A 相
        //Saturation of Ia
        if (wAux> S16_MAX)
        {
            Local_Stator_Currents.qI_Component1 = S16_MAX;
        }
        else  if (wAux <S16_MIN)
        {
            Local_Stator_Currents.qI_Component1 = S16_MIN;
        }
        else
        {  
            Local_Stator_Currents.qI_Component1 = wAux;
        }
        break;
        
    case 2:
    case 3:  // Current on Phase B not accessible
        // Ia = (hPhaseAOffset)-(ADC Channel 11 value)     
        wAux = (s32)(hPhaseAOffset)-((ADC1->JDR1)<<1);    // A 相可直接测量
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
        
        // Ib = -Ic-Ia;
        wAux = ((ADC2->JDR1)<<1) - hPhaseCOffset - Local_Stator_Currents.qI_Component1; // 由 Ic 与 Ia 重构 B 相
        // Saturation of Ib
        if (wAux> S16_MAX)
        {
            Local_Stator_Currents.qI_Component2=S16_MAX;
        }
        else  if (wAux <S16_MIN)
        {  
            Local_Stator_Currents.qI_Component2 = S16_MIN;
        }
        else  
        {
            Local_Stator_Currents.qI_Component2 = wAux;
        }                     
        break;
        
    default:                                              // 扇区非法，返回未初始化的结构(不应发生)
        break;
    } 
    
    return(Local_Stator_Currents); 
}

/*******************************************************************************
* Function Name  : SVPWM_3ShuntCalcDutyCycles
* Description    : Computes duty cycle values corresponding to the input value
and configures the AD converter and TIM0 for next period 
current reading conversion synchronization
* Input          : Stat_Volt_alfa_beta
* Output         : None
* Return         : None
*******************************************************************************
* 功能说明(中文) : SVPWM(空间矢量脉宽调制)核心计算例程。输入电流环给出的 α-β
*                  定子电压指令，完成：①由 Vα/Vβ 计算三个辅助量 wX/wY/wZ；
*                  ②判定指令矢量扇区(1~6)；③计算该扇区两个相邻基本矢量的作用
*                  时间并合成三相占空比 hTimePhA/B/C；④计算 ADC 触发时刻 hTimePhD
*                  并配置 ADC 注入采样通道(选择下桥臂导通最长的两相)；⑤把四个
*                  占空比写入 TIM1->CCR1/2/3/4。每个电流环周期调用一次(周期
*                  =(REP_RATE+1)/(2*PWM_FREQ)，随 PWM 中断/更新事件同步)。
* 参数(中文)     : Stat_Volt_Input - α-β 定子电压指令(Volt_Components)，
*                  qV_Component1=Vα、qV_Component2=Vβ，均为 s16 定点(标幺电压)，
*                  取值范围约 [-32768, 32767]，其定标由电压环/限幅环节保证。
* 返回(中文)     : 无
* 备注(中文)     : 写入全局 bSector、PWM4Direction、hTimePhA/B/C/D；直接写 TIM1
*                  的 CCR/CCER/JSQR 寄存器；假设 CCMR/CCR 预装载已使能，新占空比
*                  在下一个更新事件生效；本函数不做输出限幅，饱和由上游完成。
*******************************************************************************/
u16  hTimePhA=0, hTimePhB=0, hTimePhC=0, hTimePhD=0;   // 三相占空比与 ADC 触发比较值，单位 TIM1 计数
void SVPWM_3ShuntCalcDutyCycles (Volt_Components Stat_Volt_Input)
{
    s32 wX, wY, wZ, wUAlpha, wUBeta;                       // wUAlpha/wUBeta: α/β 分量标定值; wX/wY/wZ: 扇区判断与占空比辅助量
    
    u16  hDeltaDuty;                                       // 两相占空比之差，用于判断下桥臂公共导通窗口
    
    wUAlpha = Stat_Volt_Input.qV_Component1 * T_SQRT3 ;    // Vα 乘 √3*T，与 wUBeta 归一到同一量纲
    wUBeta = -(Stat_Volt_Input.qV_Component2 * T);         // Vβ 乘 T 并取负(坐标系方向约定)
    
    wX = wUBeta;                                           // wX/wY/wZ 为基本矢量作用时间 T1/T2 的线性组合量
    wY = (wUBeta + wUAlpha)/2;
    wZ = (wUBeta - wUAlpha)/2;
    
    // Sector calculation from wX, wY, wZ
    /* 扇区判断：依据 wX/wY/wZ 的正负号(等价于 Vα/Vβ 的符号与幅值比较)确定指令
       矢量落在 1~6 号扇区中的哪一个，判断逻辑为三分支比较。 */
    if (wY<0)                                              // wY<0: 矢量在下半平面(扇区 3/4/5)
    {
        if (wZ<0)                                          // wZ<0 且 wY<0 → 扇区 5
        {
            bSector = SECTOR_5;
        }
        else // wZ >= 0
            if (wX<=0)                                     // wZ≥0 且 wX≤0 → 扇区 4
            {
                bSector = SECTOR_4;
            }
            else // wX > 0
            {
                bSector = SECTOR_3;                        // wZ≥0 且 wX>0 → 扇区 3
            }
    }
    else // wY > 0
    {
        if (wZ>=0)                                         // wZ≥0 且 wY>0 → 扇区 2
        {
            bSector = SECTOR_2;
        }
        else // wZ < 0
            if (wX<=0)
            {  
                bSector = SECTOR_6;                        // wZ<0 且 wX≤0 → 扇区 6
            }
            else // wX > 0
            {
                bSector = SECTOR_1;                        // wZ<0 且 wX>0 → 扇区 1
            }
    }
    
    /* Duty cycles computation */
    /* 按下述通用公式计算三相占空比：
       T/8 = PWM_PERIOD/2 为 50% 中点偏移；
       wX/wY/wZ 经 /131072(即 2^17)定点定标后即为相邻基本矢量的作用时间 T1/T2
       对应的 TIM1 计数；三相占空比由 T1/T2 的线性组合得到，保证任一时刻合成
       矢量等于输入指令矢量。 */
    PWM4Direction=PWM2_MODE;                               // 默认 CC4 按 PWM2 模式(下降沿)触发 ADC
    
    switch(bSector)
    {  
    case SECTOR_1:
        /* 扇区 1 (0°~60°)：相邻基本矢量为 V1、V2；本扇区 A 相下桥臂导通窗口最短
           不可测，故 ADC1 采 B 相、ADC2 采 C 相。 */
        hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);   // A 相占空比(中点 + 与 T1/T2 相关的项，/2^17 定点定标)
        hTimePhB = hTimePhA + wZ/131072;                   // B 相占空比(B 较重)
        hTimePhC = hTimePhB - wX/131072;                   // C 相占空比(最轻 → 下桥臂导通最长)
        
        // ADC Syncronization setting value (计算 ADC 触发比较值 hTimePhD 并使采样落在下桥臂导通窗口内)             
        if ((u16)(PWM_PERIOD-hTimePhA) > TW_AFTER)
        {
            hTimePhD = PWM_PERIOD - 1;                                 // 公共导通窗口足够宽，触发点置于周期末端(CC4≈ARR)
        }
        else
        {
            hDeltaDuty = (u16)(hTimePhA - hTimePhB);           // 两相占空比之差(A/B)，近似下桥臂公共导通窗口宽度
            
            // Definition of crossing point (判断触发点是否越过下桥臂公共导通区)
            if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhA)*2) 
            {
                hTimePhD = hTimePhA - TW_BEFORE; // Ts before Phase A (触发点前移到 A 相下桥臂导通窗口起点之前) 
            }
            else
            {
                hTimePhD = hTimePhA + TW_AFTER; // DT + Tn after Phase A (从 A 相导通起点后延 死区+噪声 再触发 ADC)
                
                if (hTimePhD >= PWM_PERIOD)
                {
                    // Trigger of ADC at Falling Edge PWM4
                    // OCR update
                    
                    //Set Polarity of CC4 Low (CC4 极性设为低，配合折返后的触发点)
                    PWM4Direction=PWM1_MODE;                                   // 触发值越过 ARR，切换 CC4 为反向极性(PWM1 模式)
                    
                    hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;                  // 触发点越过 ARR，折返到上半周期(镜像)等效位置
                }
            }
        }
        
        // ADC_InjectedChannelConfig(ADC1, PHASE_B_CHANNEL,1,
        //                                    SAMPLING_TIME_CK);               
        ADC1->JSQR = PHASE_B_MSK + BUS_VOLT_FDBK_MSK + SEQUENCE_LENGHT; // ADC1 注入序列: 1=B 相电流, 2=母线电压
        //ADC_InjectedChannelConfig(ADC2, PHASE_C_CHANNEL,1,
        //                                    SAMPLING_TIME_CK);                     
        ADC2->JSQR = PHASE_C_MSK + TEMP_FDBK_MSK + SEQUENCE_LENGHT;     // ADC2 注入序列: 1=C 相电流, 2=温度                                         
        break;
    case SECTOR_2:
        /* 扇区 2 (60°~120°)：相邻基本矢量为 V2、V3；本扇区 B 相下桥臂导通窗口
           最短不可测，故 ADC1 采 A 相、ADC2 采 C 相。 */
        hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);   // A 相占空比
        hTimePhB = hTimePhA + wZ/131072;                   // B 相占空比
        hTimePhC = hTimePhA - wY/131072;                   // C 相占空比
        
        // ADC Syncronization setting value (计算 ADC 触发比较值 hTimePhD 并使采样落在下桥臂导通窗口内)
        if ((u16)(PWM_PERIOD-hTimePhB) > TW_AFTER)
        {
            hTimePhD = PWM_PERIOD - 1;                                 // 公共导通窗口足够宽，触发点置于周期末端(CC4≈ARR)
        }
        else
        {
            hDeltaDuty = (u16)(hTimePhB - hTimePhA);           // 两相占空比之差(B/A)
            
            // Definition of crossing point (判断触发点是否越过下桥臂公共导通区)
            if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhB)*2) 
            {
                hTimePhD = hTimePhB - TW_BEFORE; // Ts before Phase B (触发点前移到 B 相下桥臂导通窗口起点之前) 
            }
            else
            {
                hTimePhD = hTimePhB + TW_AFTER; // DT + Tn after Phase B (从 B 相导通起点后延 死区+噪声 再触发 ADC)
                
                if (hTimePhD >= PWM_PERIOD)
                {
                    // Trigger of ADC at Falling Edge PWM4
                    // OCR update
                    
                    //Set Polarity of CC4 Low (CC4 极性设为低，配合折返后的触发点)
                    PWM4Direction=PWM1_MODE;                                   // 触发值越过 ARR，切换 CC4 为反向极性(PWM1 模式)
                    
                    hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;                  // 触发点越过 ARR，折返到上半周期(镜像)等效位置
                }
            }
        }
        
        //ADC_InjectedChannelConfig(ADC1, PHASE_A_CHANNEL,1,
        //                                     SAMPLING_TIME_CK);               
        ADC1->JSQR = PHASE_A_MSK + BUS_VOLT_FDBK_MSK + SEQUENCE_LENGHT; // ADC1 注入序列: 1=A 相电流, 2=母线电压                
        //ADC_InjectedChannelConfig(ADC2, 
        //                   PHASE_C_CHANNEL,1,SAMPLING_TIME_CK);                              
        ADC2->JSQR = PHASE_C_MSK + TEMP_FDBK_MSK + SEQUENCE_LENGHT;     // ADC2 注入序列: 1=C 相电流, 2=温度
        break;
        
    case SECTOR_3:
        /* 扇区 3 (120°~180°)：相邻基本矢量为 V3、V4；本扇区 B 相下桥臂导通窗口
           最短不可测，故 ADC1 采 A 相、ADC2 采 C 相。 */
        hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);   // A 相占空比
        hTimePhC = hTimePhA - wY/131072;                   // C 相占空比
        hTimePhB = hTimePhC + wX/131072;                   // B 相占空比
		
        // ADC Syncronization setting value (计算 ADC 触发比较值 hTimePhD 并使采样落在下桥臂导通窗口内)
        if ((u16)(PWM_PERIOD-hTimePhB) > TW_AFTER)
        {
            hTimePhD = PWM_PERIOD - 1;                                 // 公共导通窗口足够宽，触发点置于周期末端(CC4≈ARR)
        }
        else
        {
            hDeltaDuty = (u16)(hTimePhB - hTimePhC);           // 两相占空比之差(B/C)
            
            // Definition of crossing point (判断触发点是否越过下桥臂公共导通区)
            if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhB)*2) 
            {
                hTimePhD = hTimePhB - TW_BEFORE; // Ts before Phase B (触发点前移到 B 相下桥臂导通窗口起点之前) 
            }
            else
            {
                hTimePhD = hTimePhB + TW_AFTER; // DT + Tn after Phase B (从 B 相导通起点后延 死区+噪声 再触发 ADC)
                
                if (hTimePhD >= PWM_PERIOD)
                {
                    // Trigger of ADC at Falling Edge PWM4
                    // OCR update
                    
                    //Set Polarity of CC4 Low (CC4 极性设为低，配合折返后的触发点)
                    PWM4Direction=PWM1_MODE;                                   // 触发值越过 ARR，切换 CC4 为反向极性(PWM1 模式)
                    
                    hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;                  // 触发点越过 ARR，折返到上半周期(镜像)等效位置
                }
            }
        }
        
        //ADC_InjectedChannelConfig(ADC1, PHASE_A_CHANNEL,1,
        //                                    SAMPLING_TIME_CK);               
        ADC1->JSQR = PHASE_A_MSK + BUS_VOLT_FDBK_MSK + SEQUENCE_LENGHT; // ADC1 注入序列: 1=A 相电流, 2=母线电压                
        //ADC_InjectedChannelConfig(ADC2, PHASE_C_CHANNEL,1,
        //                                    SAMPLING_TIME_CK);                                        
        ADC2->JSQR = PHASE_C_MSK + TEMP_FDBK_MSK + SEQUENCE_LENGHT;     // ADC2 注入序列: 1=C 相电流, 2=温度
        break;
        
    case SECTOR_4:
        /* 扇区 4 (180°~240°)：相邻基本矢量为 V4、V5；本扇区 C 相下桥臂导通窗口
           最短不可测，故 ADC1 采 A 相、ADC2 采 B 相。 */
        hTimePhA = (T/8) + ((((T + wX) - wZ)/2)/131072);   // A 相占空比
        hTimePhB = hTimePhA + wZ/131072;                   // B 相占空比
        hTimePhC = hTimePhB - wX/131072;                   // C 相占空比
        
        // ADC Syncronization setting value (计算 ADC 触发比较值 hTimePhD 并使采样落在下桥臂导通窗口内)
        if ((u16)(PWM_PERIOD-hTimePhC) > TW_AFTER)
        {
            hTimePhD = PWM_PERIOD - 1;                                 // 公共导通窗口足够宽，触发点置于周期末端(CC4≈ARR)
        }
        else
        {
            hDeltaDuty = (u16)(hTimePhC - hTimePhB);           // 两相占空比之差(C/B)
            
            // Definition of crossing point (判断触发点是否越过下桥臂公共导通区)
            if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhC)*2)
            {
                hTimePhD = hTimePhC - TW_BEFORE; // Ts before Phase C (触发点前移到 C 相下桥臂导通窗口起点之前) 
            }
            else
            {
                hTimePhD = hTimePhC + TW_AFTER; // DT + Tn after Phase C (从 C 相导通起点后延 死区+噪声 再触发 ADC)
                
                if (hTimePhD >= PWM_PERIOD)
                {
                    // Trigger of ADC at Falling Edge PWM4
                    // OCR update
                    
                    //Set Polarity of CC4 Low (CC4 极性设为低，配合折返后的触发点)
                    PWM4Direction=PWM1_MODE;                                   // 触发值越过 ARR，切换 CC4 为反向极性(PWM1 模式)
                    
                    hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;                  // 触发点越过 ARR，折返到上半周期(镜像)等效位置
                }
            }
        }
        
        //ADC_InjectedChannelConfig(ADC1, PHASE_A_CHANNEL,1,
        //                                    SAMPLING_TIME_CK);             
        ADC1->JSQR = PHASE_A_MSK + BUS_VOLT_FDBK_MSK + SEQUENCE_LENGHT; // ADC1 注入序列: 1=A 相电流, 2=母线电压                
        //ADC_InjectedChannelConfig(ADC2, PHASE_B_CHANNEL,1,
        //                                     SAMPLING_TIME_CK);                                    
        ADC2->JSQR = PHASE_B_MSK + TEMP_FDBK_MSK + SEQUENCE_LENGHT;     // ADC2 注入序列: 1=B 相电流, 2=温度
        break;  
        
    case SECTOR_5:
        /* 扇区 5 (240°~300°)：相邻基本矢量为 V5、V6；本扇区 C 相下桥臂导通窗口
           最短不可测，故 ADC1 采 A 相、ADC2 采 B 相。 */
        hTimePhA = (T/8) + ((((T + wY) - wZ)/2)/131072);   // A 相占空比
        hTimePhB = hTimePhA + wZ/131072;                   // B 相占空比
        hTimePhC = hTimePhA - wY/131072;                   // C 相占空比
        
        // ADC Syncronization setting value (计算 ADC 触发比较值 hTimePhD 并使采样落在下桥臂导通窗口内)
        if ((u16)(PWM_PERIOD-hTimePhC) > TW_AFTER)
        {
            hTimePhD = PWM_PERIOD - 1;                                 // 公共导通窗口足够宽，触发点置于周期末端(CC4≈ARR)
        }
        else
        {
            hDeltaDuty = (u16)(hTimePhC - hTimePhA);           // 两相占空比之差(C/A)
            
            // Definition of crossing point (判断触发点是否越过下桥臂公共导通区)
            if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhC)*2) 
            {
                hTimePhD = hTimePhC - TW_BEFORE; // Ts before Phase C (触发点前移到 C 相下桥臂导通窗口起点之前) 
            }
            else
            {
                hTimePhD = hTimePhC + TW_AFTER; // DT + Tn after Phase C (从 C 相导通起点后延 死区+噪声 再触发 ADC)
                
                if (hTimePhD >= PWM_PERIOD)
                {
                    // Trigger of ADC at Falling Edge PWM4
                    // OCR update
                    
                    //Set Polarity of CC4 Low (CC4 极性设为低，配合折返后的触发点)
                    PWM4Direction=PWM1_MODE;                                   // 触发值越过 ARR，切换 CC4 为反向极性(PWM1 模式)
                    
                    hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;                  // 触发点越过 ARR，折返到上半周期(镜像)等效位置
                }
            }
        }
        
        //ADC_InjectedChannelConfig(ADC1, PHASE_A_CHANNEL,1,
        //                                   SAMPLING_TIME_CK);              
        ADC1->JSQR = PHASE_A_MSK + BUS_VOLT_FDBK_MSK + SEQUENCE_LENGHT; // ADC1 注入序列: 1=A 相电流, 2=母线电压                
        //ADC_InjectedChannelConfig(ADC2, PHASE_B_CHANNEL,1,
        //                                     SAMPLING_TIME_CK);                                      
        ADC2->JSQR = PHASE_B_MSK + TEMP_FDBK_MSK + SEQUENCE_LENGHT;     // ADC2 注入序列: 1=B 相电流, 2=温度
		break;
        
    case SECTOR_6:
        /* 扇区 6 (300°~360°)：相邻基本矢量为 V6、V1；本扇区 A 相下桥臂导通窗口
           最短不可测，故 ADC1 采 B 相、ADC2 采 C 相。 */
        hTimePhA = (T/8) + ((((T - wX) + wY)/2)/131072);   // A 相占空比
        hTimePhC = hTimePhA - wY/131072;                   // C 相占空比
        hTimePhB = hTimePhC + wX/131072;                   // B 相占空比
        
        // ADC Syncronization setting value (计算 ADC 触发比较值 hTimePhD 并使采样落在下桥臂导通窗口内)
        if ((u16)(PWM_PERIOD-hTimePhA) > TW_AFTER)
        {
            hTimePhD = PWM_PERIOD - 1;                                 // 公共导通窗口足够宽，触发点置于周期末端(CC4≈ARR)
        }
        else
        {
            hDeltaDuty = (u16)(hTimePhA - hTimePhC);           // 两相占空比之差(A/C)
            
            // Definition of crossing point (判断触发点是否越过下桥臂公共导通区)
            if (hDeltaDuty > (u16)(PWM_PERIOD-hTimePhA)*2) 
            {
                hTimePhD = hTimePhA - TW_BEFORE; // Ts before Phase A (触发点前移到 A 相下桥臂导通窗口起点之前) 
            }
            else
            {
                hTimePhD = hTimePhA + TW_AFTER; // DT + Tn after Phase A (从 A 相导通起点后延 死区+噪声 再触发 ADC)
                
                if (hTimePhD >= PWM_PERIOD)
                {
                    // Trigger of ADC at Falling Edge PWM4
                    // OCR update
                    
                    //Set Polarity of CC4 Low (CC4 极性设为低，配合折返后的触发点)
                    PWM4Direction=PWM1_MODE;                                   // 触发值越过 ARR，切换 CC4 为反向极性(PWM1 模式)
                    
                    hTimePhD = (2 * PWM_PERIOD) - hTimePhD-1;                  // 触发点越过 ARR，折返到上半周期(镜像)等效位置
                }
            }
        }
        
        //ADC_InjectedChannelConfig(ADC1, PHASE_B_CHANNEL,1,
        //                                    SAMPLING_TIME_CK);     
        ADC1->JSQR = PHASE_B_MSK + BUS_VOLT_FDBK_MSK + SEQUENCE_LENGHT; // ADC1 注入序列: 1=B 相电流, 2=母线电压                
        //ADC_InjectedChannelConfig(ADC2, PHASE_C_CHANNEL,1,
        //                                    SAMPLING_TIME_CK);                               
        ADC2->JSQR = PHASE_C_MSK + TEMP_FDBK_MSK + SEQUENCE_LENGHT;     // ADC2 注入序列: 1=C 相电流, 2=温度
        break;
    default:
		break;
    }
    
    if (PWM4Direction == PWM2_MODE)
    {
        //Set Polarity of CC4 High
        TIM1->CCER &= 0xDFFF;                              // 清 CC4 极性位: 高电平有效(下降沿触发 ADC)
    }
    else
    {
        //Set Polarity of CC4 Low
        TIM1->CCER |= 0x2000;                              // 置 CC4 极性位: 低电平有效(上升沿触发 ADC)
    }
    
    /* Load compare registers values */ 
    TIM1->CCR1 = hTimePhA;                                 // A 相占空比写入比较寄存器(预装载，更新事件生效)
    TIM1->CCR2 = hTimePhB;                                 // B 相占空比
    TIM1->CCR3 = hTimePhC;                                 // C 相占空比
    TIM1->CCR4 = hTimePhD; // To Syncronyze the ADC          // CC4 触发点，用于同步 ADC 采样时刻
}

/*******************************************************************************
* Function Name  : SVPWM_3ShuntAdvCurrentReading
* Description    :  It is used to enable or disable the advanced current reading.
if disabled the current readign will be performed after update event
* Input          : cmd (ENABLE or DISABLE)
* Output         : None
* Return         : None
*******************************************************************************
* 功能说明(中文) : 使能/禁止"高级电流读取"模式。使能时，ADC 注入组触发源改为
*                  TIM1 的 CC4 事件(由 SVPWM_3ShuntCalcDutyCycles() 每周期精确设定
*                  的 hTimePhD 决定采样时刻)，同时打开 TIM1 更新中断以在每个
*                  PWM 周期重新使能 ADC 触发；禁止时，触发源回到 TIM1 更新事件
*                  (TRGO)，采样在更新事件后发生。由电机控制层在启动/切换采样
*                  方式时调用，非周期。
* 参数(中文)     : cmd - ENABLE 使能高级电流读取(CC4 同步)；DISABLE 禁止，
*                  改回更新事件同步。
* 返回(中文)     : 无
* 备注(中文)     : 直接改写 ADC1->CR2 的 EXTTRIG(位 20)与 JSWSTART(位 15,
*                  "ReEnable EXT. ADC Triggering")以及 TIM1 中断使能；CC4 触发
*                  需要 SVPWM 计算已写入 TIM1->CCR4，故必须与 CalcDutyCycles 配合。
*******************************************************************************/
void SVPWM_3ShuntAdvCurrentReading(FunctionalState cmd)
{
    if (cmd == ENABLE)
    {
        // Enable ADC trigger sync with CC4
        //ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_CC4);  
        ADC1->CR2 |= 0x00001000;                           // CR2 位 12: 选择 TIM1_CC4 作为注入组触发源
        
        // Enable UPDATE ISR
        // Clear Update Flag
        TIM_ClearFlag(TIM1, TIM_FLAG_Update);
        TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);         // 每个更新事件进入 ISR 重新使能 ADC 触发
    }
    else
    {
        // Disable UPDATE ISR
        TIM_ITConfig(TIM1, TIM_IT_Update, DISABLE);        // 关闭更新中断
        
        // Sync ADC trigger with Update
        //ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_T1_TRGO);
        ADC1->CR2 &=0xFFFFEFFF;                            // 清 CR2 位 12: 触发源改回 TIM1_TRGO(更新事件)
        
        // ReEnable EXT. ADC Triggering
        ADC1->CR2 |=0x00008000;                            // 置 CR2 位 15: 重新使能 ADC 外部触发
    }
}

/*******************************************************************************
* Function Name  : SVPWMUpdateEvent
* Description    :  Routine to be performed inside the update event ISR  it reenable the ext adc. triggering
It must be assigned to pSVPWM_UpdateEvent pointer.	
* Input           : None
* Output         : None
* Return         : None
*******************************************************************************
* 功能说明(中文) : TIM1 更新(下溢)事件中断回调函数。在高级电流读取模式下，每个
*                  PWM 周期由更新中断调用一次：重新使能 ADC 外部触发(JSWSTART)，
*                  并清除上一次可能残留的注入转换完成标志。需赋值给电机控制层的
*                  pSVPWM_UpdateEvent 函数指针后由 TIM1_UP 中断服务程序调用。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 需先由 SVPWM_3ShuntAdvCurrentReading(ENABLE) 打开 TIM1 更新
*                  中断才会被调用；直接操作 ADC1->CR2。
*******************************************************************************/
void SVPWMUpdateEvent(void)
{
    // ReEnable EXT. ADC Triggering
    ADC1->CR2 |= 0x00008000;                               // 置 CR2 位 15: 重新使能 ADC 外部触发
    
    // Clear unwanted current sampling
    ADC_ClearFlag(ADC1, ADC_FLAG_JEOC);                    // 清除无效的注入转换结束标志
}

/*******************************************************************************
* Function Name  : SVPWMEOCEvent
* Description    :  Routine to be performed inside the end of conversion ISR
It computes the bus voltage and temperature sensor sampling 
and disable the ext. adc triggering.	
* Input           : None
* Output         : None
* Return         : None
*******************************************************************************
* 功能说明(中文) : ADC 注入组转换结束(JEOC)中断回调函数。由 ADC1_2 中断服务程序
*                  在注入组转换完成时调用：读取 ADC2 第 2 通道(NTC 温度)与 ADC1
*                  第 2 通道(母线电压)的转换结果存入全局变量，并在电机处于
*                  START/RUN 状态时关闭 ADC 外部触发，避免下一个周期误触发。
* 参数(中文)     : 无
* 返回(中文)     : 固定返回 1(u8)，表示转换事件已处理。
* 备注(中文)     : 写全局 h_ADCTemp/h_ADCBusvolt(ADC 原始码，供电机控制层换算
*                  母线电压与温度)；需赋值给 pSVPWM_EOCEvent 指针；State 为电机
*                  控制层全局状态机变量。
*******************************************************************************/
u8 SVPWMEOCEvent(void)
{
    // Store the Bus Voltage and temperature sampled values
    h_ADCTemp = ADC_GetInjectedConversionValue(ADC2,ADC_InjectedChannel_2);      // 读取 ADC2 注入通道 2: NTC 温度
    h_ADCBusvolt = ADC_GetInjectedConversionValue(ADC1,ADC_InjectedChannel_2);   // 读取 ADC1 注入通道 2: 母线电压
    
    if ((State == START) || (State == RUN))                // 仅电机启动/运行期间需要关闭触发
    {          
        // Disable EXT. ADC Triggering
        ADC1->CR2 = ADC1->CR2 & 0xFFFF7FFF;                // 清 CR2 位 15: 禁止 ADC 外部触发
    }
    return ((u8)(1));
}

#endif

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/  
