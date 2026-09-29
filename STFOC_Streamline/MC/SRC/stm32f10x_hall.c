/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : stm32f10x_hall.c
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : Module handling speed feedback provided by three Hall 
*                      sensors
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 26/06/08 v2.0.1
* 27/06/08 v2.0.2
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
 *   本模块实现基于「三路霍尔(Hall)传感器」的转子位置/速度反馈，供 FOC
 *   (磁场定向控制，Field Oriented Control)使用。核心要点如下:
 *     - 三个霍尔开关间隔 120 度或 60 度电角度安装，每个电周期产生 6 个
 *       状态(扇区)；通过状态跳变序列可判定转向，并同步更新转子电角度；
 *     - 用定时器的输入捕获(IC)记录相邻跳变的时间间隔(伪周期)，据此计算
 *       转速；低速时定时器会溢出，用溢出计数把捕获值扩展到大于 16 位；
 *     - 动态调整预分频(PSC)，兼顾「低速可测范围」与「高速分辨率」；
 *     - 用软件 FIFO 做滑动平均以平滑转速；
 *     - 连续多次溢出仍无捕获，则判定为超时(霍尔信号丢失或转速急剧下降)。
 *   术语说明:
 *     - 极对数(POLE_PAIR_NUM): 机械转 1 圈对应 POLE_PAIR_NUM 个电周期，
 *       即 电角度 = 机械角度 x 极对数，电频率 = 机械频率 x 极对数；
 *     - 霍尔换相: 依霍尔状态切换逆变桥开关，使定子磁动势跟随转子。
 * ========================================================================== */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_hall.h"
#include "MC_hall_prm.h"
#include "MC_Globals.h"
#include "stm32f10x_MClib.h"
#include "stm32f10x_it.h"
/* Private define ------------------------------------------------------------*/
// 中文: 以下为本模块内部派生常量(多数由 MC_hall_prm.h 的物理参数换算而来)
#define HALL_MAX_SPEED_FDBK (u16)(HALL_MAX_SPEED_FDBK_RPM/6 * POLE_PAIR_NUM)
                            // 中文: 最高可测转速(电频率定标): rpm/6 得 0.1Hz 机械频率, 再乘极对数得电频率
#define HALL_MIN_SPEED_FDBK (u16)(HALL_MIN_SPEED_FDBK_RPM/6* POLE_PAIR_NUM)
                            // 中文: 最低可测转速(电频率定标): 换算方式同上
#define LOW_RES_THRESHOLD   ((u16)0x5500u)// If capture below, ck prsc decreases
                                           // 中文: 低分辨率阈值; 若捕获值低于该值, 则减小预分频以提高精度
// 中文: ROTOR_SPEED_FACTOR = CKTIM*10/3, 是「转速<->捕获周期」换算的比例系数
//       (把以 CKTIM 计数的周期与 0.1Hz 定标的电频率关联起来)
#define	ROTOR_SPEED_FACTOR  ((u32)((CKTIM*10)) / 3)
#define PSEUDO_FREQ_CONV    ((u32)(ROTOR_SPEED_FACTOR / (SAMPLING_FREQ * 10)) * 0x10000uL)
                            // 中文: 伪频率换算常数; 把捕获周期换算为 s16(0x0000..0xFFFF)定标的转子电频率
#define SPEED_OVERFLOW      ((u32)(ROTOR_SPEED_FACTOR / HALL_MAX_SPEED_FDBK))
                            // 中文: 防 u32 除法溢出的周期门限; 捕获周期大于它时正常计算, 否则直接取最大伪转速
#define MAX_PERIOD          ((u32)(ROTOR_SPEED_FACTOR / HALL_MIN_SPEED_FDBK))
                            // 中文: 允许的最大捕获周期; 超过该值判定转速过低, 按 0 处理
#define HALL_COUNTER_RESET  ((u16) 0)
                            // 中文: 定时器计数器的复位值(清零)
#define S16_PHASE_SHIFT     (s16)(HALL_PHASE_SHIFT * 65536/360)
                            // 中文: 相位偏移换算为 s16 电角度定标(360 度 = 65536)
#define S16_120_PHASE_SHIFT (s16)(65536/3)
                            // 中文: 120 度电角度对应的 s16 值(65536/3)
#define S16_60_PHASE_SHIFT  (s16)(65536/6)
                            // 中文: 60 度电角度对应的 s16 值(65536/6)

#define STATE_0 (u8)0          // 中文: 霍尔状态编码 0 (三路霍尔电平的组合)
#define STATE_1 (u8)1          // 中文: 霍尔状态编码 1
#define STATE_2 (u8)2          // 中文: 霍尔状态编码 2
#define STATE_3 (u8)3          // 中文: 霍尔状态编码 3
#define STATE_4 (u8)4          // 中文: 霍尔状态编码 4
#define STATE_5 (u8)5          // 中文: 霍尔状态编码 5
#define STATE_6 (u8)6          // 中文: 霍尔状态编码 6
#define STATE_7 (u8)7          // 中文: 霍尔状态编码 7

#define NEGATIVE          (s8)-1   // 中文: 转向 = 负(反转)
#define POSITIVE          (s8)1    // 中文: 转向 = 正(正转)
#define NEGATIVE_SWAP     (s8)-2   // 中文: 转向 = 负且本次跳变判定为发生了换向(过渡标志)
#define POSITIVE_SWAP     (s8)2    // 中文: 转向 = 正且本次跳变判定为发生了换向(过渡标志)
#define ERROR             (s8)127  // 中文: 霍尔状态非法/无法判定转向的错误标志

#define GPIO_MSK (u8)0x07      // 中文: 3 路霍尔输入对应的 GPIO 位掩码(取低 3 位)
#define ICx_FILTER (u8) 0x0B // 11 <-> 1333 nsec 
                              // 中文: 输入捕获数字滤波参数, 0x0B 对应约 1333ns 的滤波时间(抗毛刺)

#define TIMx_PRE_EMPTION_PRIORITY 2  // 中文: 该定时器中断的抢占优先级
#define TIMx_SUB_PRIORITY 0          // 中文: 该定时器中断的子优先级

/* if (HALL_SENSORS_PLACEMENT == DEGREES_120)
The sequence of the states is {STATE_5,STATE_1,STATE_3,STATE_2,STATE_6,STATE_4}
else if (HALL_SENSORS_PLACEMENT == DEGREES_60)
the sequence is {STATE_1,STATE_3,STATE_7,STATE_6,STATE_4,STATE_0}*/
// 中文: 上述注释说明两种安装方式下, 转子正转时霍尔状态的循环顺序:
//       120 度排布: 5-1-3-2-6-4 ; 60 度排布: 1-3-7-6-4-0。
//       该顺序即「霍尔换相表」的基础, 用于判定转向并把霍尔扇区与电角度对齐。

// Here is practically assigned the timer for Hall handling
// 中文: 依据编译开关, 实际选择承载霍尔处理的定时器为 TIM2/TIM3/TIM4 之一。
#if defined(TIMER2_HANDLES_HALL)
    #define HALL_TIMER TIM2        // 中文: 使用 TIM2 (本工程配置)
#elif defined(TIMER3_HANDLES_HALL)
    #define HALL_TIMER TIM3        // 中文: 使用 TIM3
#else // TIMER4_HANDLES_HALL
    #define HALL_TIMER TIM4        // 中文: 使用 TIM4
#endif

/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
// 中文: SpeedMeas_s —— 单次霍尔捕获的测速记录(软件 FIFO 的单元):
//         hCapture   : 捕获到的计数值(u16); 单位 = 定时器计数(CKTIM/(PSC+1) 时钟)
//         hPrscReg   : 该次捕获对应的预分频寄存器值(u16); 用于还原真实周期(乘以 PSC+1)
//         bDirection : 该次捕获判定的转向(s8; 取 NEGATIVE/POSITIVE/SWAP 等值)
//       PeriodMeas_s —— 换算后的一段时间测量结果:
//         wPeriod    : 伪周期(u32); 单位 = CKTIM 时钟周期数(已按 hPrscReg+1 还原)
//         bDirection : 该周期对应的转向(s8)
typedef struct {
	u16 hCapture;
	u16 hPrscReg;
        s8 bDirection;
	} SpeedMeas_s;

typedef struct {
        u32 wPeriod;
        s8 bDirection;
        } PeriodMeas_s;
/* Private variables ---------------------------------------------------------*/

volatile SpeedMeas_s SensorPeriod[HALL_SPEED_FIFO_SIZE];// Holding the last captures
                                                        // 中文: 保存最近若干次捕获的测速记录(软件 FIFO)
vu8 bSpeedFIFO_Index;   // Index of above array
                        // 中文: 上述 FIFO 的当前写入下标
vu8 bGP1_OVF_Counter;   // Count overflows if prescaler is too low
                        // 中文: 相邻两次捕获之间定时器溢出的次数(预分频过低时用作扩展位)
vu16 hCaptCounter;      // Counts the number of captures interrupts
                        // 中文: 捕获中断累计次数(用于丢弃首次捕获)
volatile PeriodMeas_s PeriodMeas;
                        // 中文: 最近一次换算得到的周期测量结果(保留备用)

volatile bool RatioDec;          // 中文: 上次捕获是否下调过预分频(用于补偿预装载延迟)
volatile bool RatioInc;          // 中文: 上次捕获是否上调过预分频(用于补偿预装载延迟)
volatile bool DoRollingAverage;  // 中文: 是否启用滑动平均(初值化完成后置位)
volatile bool InitRollingAverage;// 中文: 滑动平均初始化请求(下次捕获时用最新值铺满 FIFO)
volatile bool HallTimeOut;       // 中文: 霍尔反馈超时标志(信号丢失/急减速)
static s16 hElectrical_Angle;    // 中文: 最新转子电角度(s16 定标, 360 度 = 65536)
static s16 hRotorFreq_dpp;       // 中文: 每个 PWM 周期的电角度增量(rad/每PWM周期, s16 定标)
#if (defined HALL_SENSORS || defined VIEW_HALL_FEEDBACK)
static s8 bSpeed;                // 中文: 本次捕获判定出的转向(NEGATIVE/POSITIVE/SWAP/ERROR)
#endif
/* Private function prototypes -----------------------------------------------*/
PeriodMeas_s GetLastHallPeriod(void);   // 中文: 取最近一次捕获换算出的伪周期
PeriodMeas_s GetAvrgHallPeriod(void);   // 中文: 取 FIFO 内多次捕获的平均伪周期
void HALL_StartHallFiltering(void);     // 中文: 启动/重置滑动平均(用最新值铺满 FIFO)
u16  HALL_GetCaptCounter(void);         // 中文: 读取自上次清零以来的捕获中断次数
void HALL_ClrCaptCounter(void);         // 中文: 清零捕获中断计数

u8  ReadHallState(void);                // 中文: 读取 3 路霍尔 GPIO 电平, 返回 0..7 状态码
/*******************************************************************************
* Function Name  : Hall_HallTimerInit
* Description    : Initializes the timer handling Hall sensors feedback
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
/* 功能说明(中文) : 初始化承载霍尔信号的定时器与相关 GPIO/NVIC 中断。
 *                 在电机启动初始化阶段调用一次(非周期性)。
 *                 配置定时器为「从模式=复位(Reset)」并由 TI1FP1 触发,
 *                 三路通道异或(XOR)得到霍尔组合信号, 用于捕获跳变时刻。
 * 参数(中文)     : 无
 * 返回(中文)     : 无
 * 备注(中文)     : 依赖 HALL_MAX_RATIO 作为初始预分频; 使能 CC1 捕获中断与
 *                 Update 溢出中断; 定时器由 HALL_TIMER 宏选定(TIM2/3/4)。
 *******************************************************************************/
void HALL_HallTimerInit(void)
{

  TIM_TimeBaseInitTypeDef TIM_HALLTimeBaseInitStructure;
  TIM_ICInitTypeDef       TIM_HALLICInitStructure;
  NVIC_InitTypeDef        NVIC_InitHALLStructure;
  GPIO_InitTypeDef        GPIO_InitStructure;
  
#if defined(TIMER2_HANDLES_HALL)
    /* TIM2 clock source enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    /* Enable GPIOA, clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure PA.00,01 ,02 as Hall sensors input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#elif defined(TIMER3_HANDLES_HALL)
    /* TIM3 clock source enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    /* Enable GPIOA, clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    /* Enable GPIOB, clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    /* Configure PA.06,07  PB.00 as Hall sensors input */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
  #else // TIMER4_HANDLES_HALL
    /* TIM4 clock source enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    /* Enable GPIOB, clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* Configure PB.06,07,08 as Hall sensors input */	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);	
  #endif
      
    // Timer configuration in Clear on capture mode
    // 中文: 定时器配置为「捕获时复位计数(Clear on capture)」模式; 每次捕获后计数器从 0 重新计数, 捕获值即两次跳变的时间间隔。
    TIM_DeInit(HALL_TIMER);
    
    TIM_TimeBaseStructInit(&TIM_HALLTimeBaseInitStructure);
    // Set full 16-bit working range
    // 中文: 自动重装值设为 U16_MAX, 使计数器使用完整 16 位量程。
    TIM_HALLTimeBaseInitStructure.TIM_Period = U16_MAX;
    TIM_HALLTimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(HALL_TIMER,&TIM_HALLTimeBaseInitStructure);
    
    TIM_ICStructInit(&TIM_HALLICInitStructure);
    TIM_HALLICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_HALLICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
    TIM_HALLICInitStructure.TIM_ICFilter = ICx_FILTER;
    
    TIM_ICInit(HALL_TIMER,&TIM_HALLICInitStructure);
    
    // Force the HALL_TIMER prescaler with immediate access (no need of an update event) 
    TIM_PrescalerConfig(HALL_TIMER, (u16) HALL_MAX_RATIO, 
                       TIM_PSCReloadMode_Immediate);
    TIM_InternalClockConfig(HALL_TIMER);
    // 中文: 采用内部时钟; 预分频初值由 HALL_MAX_RATIO 设定。
    
    //Enables the XOR of channel 1, channel2 and channel3
    // 中文: 使能三通道异或(XOR): 三路霍尔合成一个信号, 任一相跳变都触发捕获。
    TIM_SelectHallSensor(HALL_TIMER, ENABLE);
    
    TIM_SelectInputTrigger(HALL_TIMER, TIM_TS_TI1FP1);
    // 中文: 触发源选为 TI1FP1(经滤波的通道 1)。
    TIM_SelectSlaveMode(HALL_TIMER,TIM_SlaveMode_Reset);
    // 中文: 从模式设为 Reset: 触发有效时把计数器清零(配合「捕获清零」实现间隔测量)。
   
    // Source of Update event is only counter overflow/underflow
    // 中文: 仅由计数器上溢/下溢产生更新(Update)事件。
    TIM_UpdateRequestConfig(HALL_TIMER, TIM_UpdateSource_Regular);
    
    /* Enable the HALL_TIMER IRQChannel*/
#if defined(TIMER2_HANDLES_HALL)
    NVIC_InitHALLStructure.NVIC_IRQChannel = TIM2_IRQChannel;
#elif defined(TIMER3_HANDLES_HALL)
    NVIC_InitHALLStructure.NVIC_IRQChannel = TIM3_IRQChannel;
#else // TIMER4_HANDLES_HALL
    NVIC_InitHALLStructure.NVIC_IRQChannel = TIM4_IRQChannel;
#endif
  
    NVIC_InitHALLStructure.NVIC_IRQChannelPreemptionPriority = 
                                                      TIMx_PRE_EMPTION_PRIORITY;
    NVIC_InitHALLStructure.NVIC_IRQChannelSubPriority = TIMx_SUB_PRIORITY;
    NVIC_InitHALLStructure.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_Init(&NVIC_InitHALLStructure);

    // Clear the TIMx's pending flags
    TIM_ClearFlag(HALL_TIMER, TIM_FLAG_Update + TIM_FLAG_CC1 + TIM_FLAG_CC2 + \
                  TIM_FLAG_CC3 + TIM_FLAG_CC4 + TIM_FLAG_Trigger + TIM_FLAG_CC1OF + \
                  TIM_FLAG_CC2OF + TIM_FLAG_CC3OF + TIM_FLAG_CC4OF);
  
    // Selected input capture and Update (overflow) events generate interrupt
    // 中文: 使能 CC1 捕获中断与 Update(溢出)中断; 分别用于测速与超时检测。
    TIM_ITConfig(HALL_TIMER, TIM_IT_CC1, ENABLE);
    TIM_ITConfig(HALL_TIMER, TIM_IT_Update, ENABLE);

    TIM_SetCounter(HALL_TIMER, HALL_COUNTER_RESET);
       TIM_Cmd(HALL_TIMER, ENABLE);
}


/*******************************************************************************
* ROUTINE Name : HALL_InitHallMeasure
*
* Description : Clear software FIFO where are "pushed" latest speed information
*           This function must be called before starting the motor to initialize
*	    the speed measurement process.
*
* Input       : None
* Output      : None
* Return      : None
* Note        : First measurements following this function call will be done
*               without filtering (no rolling average).
*******************************************************************************/
/* 功能说明(中文) : 电机启动测速前清空软件 FIFO 并复位测速状态标志。
 *                 必须在电机启动之前调用一次, 用于初始化测速流程。
 * 参数(中文)     : 无
 * 返回(中文)     : 无
 * 备注(中文)     : 调用后最初若干次测量不做滑动平均(直接反映瞬时值);
 *                 会暂时关闭捕获中断以保证初始化完整, 结束后重新使能。
 *******************************************************************************/
void HALL_InitHallMeasure( void )
{
   // Mask interrupts to insure a clean intialization
   // 中文: 屏蔽捕获中断, 保证 FIFO 初始化过程不被打断。

   TIM_ITConfig(HALL_TIMER, TIM_IT_CC1, DISABLE);
    
   RatioDec = FALSE;
   RatioInc = FALSE;
   DoRollingAverage = FALSE;
   InitRollingAverage = FALSE;
   HallTimeOut = FALSE;

   hCaptCounter = 0;
   bGP1_OVF_Counter = 0;

   for (bSpeedFIFO_Index=0; bSpeedFIFO_Index < HALL_SPEED_FIFO_SIZE; 
                                                             bSpeedFIFO_Index++)
   {
      SensorPeriod[bSpeedFIFO_Index].hCapture = U16_MAX;
      SensorPeriod[bSpeedFIFO_Index].hPrscReg = HALL_MAX_RATIO;
      SensorPeriod[bSpeedFIFO_Index].bDirection = POSITIVE;
   }

   // First measurement will be stored in the 1st array location
   // 中文: 使首次测量写入 FIFO 的第 1 个位置。
   bSpeedFIFO_Index = HALL_SPEED_FIFO_SIZE-1;

   // Re-initialize partly the timer
   // 中文: 部分复位定时器(重设预分频与计数)。
   HALL_TIMER->PSC = HALL_MAX_RATIO;
   
   HALL_ClrCaptCounter();
     
   TIM_SetCounter(HALL_TIMER, HALL_COUNTER_RESET);
   
   TIM_Cmd(HALL_TIMER, ENABLE);

   TIM_ITConfig(HALL_TIMER, TIM_IT_CC1, ENABLE);

}


/*******************************************************************************
* ROUTINE Name : HALL_GetSpeed
*
* Description : This routine returns Rotor frequency with [0.1Hz] definition.
*		Result is given by the following formula:
*		Frotor = K x (Fosc / (Capture x number of overflow)))
*		where K depends on the number of motor poles pairs
*
* Input    : None
* Output   : None
* Returns  : Rotor mechanical frequency, with 0.1Hz resolution.
* Comments : Result is zero if speed is too low (glitches at start for instance)
*           Excessive speed (or high freq glitches will result in a pre-defined
*           value returned.
* Warning : Maximum expectable accuracy depends on CKTIM: 72MHz will give the
* 	    best results.
*******************************************************************************/
/* 功能说明(中文) : 返回转子机械转速, 以 0.1Hz 为分辨率。
 *                 公式 Frotor = K x (Fosc / (Capture x overflow)),
 *                 其中 K 与电机极对数有关。需要以 Hz 表达转速时调用。
 * 参数(中文)     : 无
 * 返回(中文)     : s16 —— 机械转速, 单位 0.1Hz(如返回 5000 表示 500.0Hz)
 * 备注(中文)     : 转速过低(如启动抖动)时返回 0; 转速过高或高频毛刺时返回
 *                 HALL_MAX_SPEED 预定义值。精度取决于 CKTIM, 72MHz 时最佳。
 *******************************************************************************/
s16 HALL_GetSpeed ( void )
{ 
  s32 wAux;
  
  if( hRotorFreq_dpp == HALL_MAX_PSEUDO_SPEED)
  // 中文: 若内部频率为超速哨兵值, 直接返回预定义的最高转速。 
  {
    return (HALL_MAX_SPEED);
  }
  else
  {
    wAux = ((hRotorFreq_dpp* SAMPLING_FREQ * 10)/(65536*POLE_PAIR_NUM));
    // 中文: 由「每 PWM 周期角度增量」换算为 0.1Hz: 乘采样频率与 10, 再除以 (65536*极对数)。
    return (s16)wAux;
  }
}

/*******************************************************************************
* ROUTINE Name : Hall_GetRotorFreq
*
* Description : This routine returns Rotor frequency with an unit that can be
*               directly integrated to get the speed in the field oriented
*               control loop.
*
* Input    : None
* Output   : None
* Returns  : Rotor mechanical frequency with rad/PWM period unit
*             (here 2*PI rad = 0xFFFF).
* Comments : Result is zero if speed is too low (glitches at start for instance)
*           Excessive speed (or high freq glitches will result in a pre-defined
*           value returned.
* Warning : Maximum expectable accuracy depends on CKTIM: 72MHz will give the
* 	    best results.
*******************************************************************************/
/* 功能说明(中文) : 返回转子机械频率, 但单位可直接用于 FOC 控制环积分
 *                 (即「每个 PWM 周期的电角度增量」, s16 定标)。
 *                 每次捕获后在中断中调用, 更新内部变量 hRotorFreq_dpp。
 * 参数(中文)     : 无
 * 返回(中文)     : s16 —— 机械频率, 单位 rad/每 PWM 周期(此处 2*PI 弧度 = 0xFFFF)
 * 备注(中文)     : 转速过低返回 0; 过高返回 HALL_MAX_PSEUDO_SPEED; 超时清 0。
 *                 精度取决于 CKTIM, 72MHz 时最佳。
 *******************************************************************************/
s16 HALL_GetRotorFreq ( void )
{
   PeriodMeas_s PeriodMeasAux;

   if ( DoRollingAverage)
   // 中文: 若已启用滑动平均则取平均周期, 否则取最近一次原始周期。
   {
      PeriodMeasAux = GetAvrgHallPeriod();
   }
   else
   {  // Raw period
      // 中文: 未启用平均, 使用最近一次原始捕获周期。
      PeriodMeasAux = GetLastHallPeriod();
   }

   if (HallTimeOut == TRUE)
   {
      hRotorFreq_dpp = 0;
   }
   else
   {
     if(PeriodMeasAux.bDirection != ERROR)
     // 中文: 仅在未检测到换向/方向错误时才进行频率换算。
     //No errors have been detected during rotor speed information extrapolation          
     {
        if ( HALL_TIMER->PSC >= HALL_MAX_RATIO )/* At start-up or very low freq */
        {                           /* Based on current prescaler value only */
           hRotorFreq_dpp = 0;
        }
        else
        {
           if( PeriodMeasAux.wPeriod > MAX_PERIOD) /* Speed is too low */
           {
              hRotorFreq_dpp = 0;
           }
           else
           {  /*Avoid u32 DIV Overflow*/
              if ( PeriodMeasAux.wPeriod > (u32)SPEED_OVERFLOW )
              {
                if (HALL_GetCaptCounter()<2)// First capture must be discarded
                {
                  hRotorFreq_dpp=0;
                }
                else                  
                {
                   hRotorFreq_dpp = (s16)((u16) (PSEUDO_FREQ_CONV /
                                                          PeriodMeasAux.wPeriod));
                   hRotorFreq_dpp *= PeriodMeasAux.bDirection;               
                }
              }
              else
              {
                hRotorFreq_dpp = HALL_MAX_PSEUDO_SPEED;
              }
           }
        }
     }          
   }

   return (hRotorFreq_dpp);
}


/*******************************************************************************
* ROUTINE Name : HALL_ClrTimeOut
*
* Description     : Clears the flag indicating that that informations are lost,
*                   or speed is decreasing sharply.
* Input           : None
* Output          : Clear HallTimeOut
* Return          : None
*******************************************************************************/
/* 功能说明(中文) : 清除「霍尔信息丢失/转速急剧下降」的超时标志。
 * 参数(中文)     : 无
 * 返回(中文)     : 无
 * 备注(中文)     : 每次成功捕获到霍尔跳变后会被调用, 以解除超时状态。
 *******************************************************************************/
void HALL_ClrTimeOut(void)
{
   HallTimeOut = FALSE;
}


/*******************************************************************************
* ROUTINE Name : HALL_IsTimedOut
*
* Description     : This routine indicates to the upper layer SW that Hall 
*                   sensors information disappeared or timed out.
* Input           : None
* Output          : None
* Return          : boolean, TRUE in case of Time Out
* Note            : The time-out duration depends on timer pre-scaler,
*                   which is variable; the time-out will be higher at low speed.
*******************************************************************************/
/* 功能说明(中文) : 向高层软件报告霍尔传感器信息丢失或已超时。
 * 参数(中文)     : 无
 * 返回(中文)     : bool —— 超时返回 TRUE, 正常返回 FALSE
 * 备注(中文)     : 超时时长取决于定时器预分频(可变), 低速时超时时长更长。
 *******************************************************************************/
bool HALL_IsTimedOut(void)
{
   return(HallTimeOut);
}


/*******************************************************************************
* ROUTINE Name : Hall_GetCaptCounter
*
* Description     : Gives the number of Hall sensors capture interrupts since last call
*                   of the HALL_ClrCaptCounter function.
* Input           : None
* Output          : None
* Return          : u16 integer (Roll-over is prevented in the input capture
*                   routine itself).
*******************************************************************************/
/* 功能说明(中文) : 返回自上次调用 HALL_ClrCaptCounter 以来霍尔捕获中断的次数。
 * 参数(中文)     : 无
 * 返回(中文)     : u16 —— 捕获中断计数(捕获中断内已做防回绕处理)
 * 备注(中文)     : 用于丢弃首次捕获、判断是否已有有效采集。
 *******************************************************************************/
u16 HALL_GetCaptCounter(void)
{
   return(hCaptCounter);
}


/*******************************************************************************
* ROUTINE Name : HALL_ClrCaptCounter
*
* Description     : Clears the variable holding the number of capture events.
* Input           : None
* Output          : hCaptCounter is cleared.
* Return          : None
*******************************************************************************/
/* 功能说明(中文) : 清零保存捕获事件次数的变量。
 * 参数(中文)     : 无
 * 返回(中文)     : 无
 * 备注(中文)     : 将 hCaptCounter 清零。
 *******************************************************************************/
void HALL_ClrCaptCounter(void)
{
   hCaptCounter = 0;
}


/*******************************************************************************
* ROUTINE Name : GetLastHallPeriod
*
* Description     : returns the rotor pseudo-period based on last capture
* Input           : None
* Output          : None
* Return          : rotor pseudo-period, as a number of CKTIM periods
*******************************************************************************/
/* 功能说明(中文) : 取最近一次捕获换算出的伪周期。
 * 参数(中文)     : 无
 * 返回(中文)     : PeriodMeas_s —— 伪周期(单位 CKTIM 时钟)及其对应的转向
 * 备注(中文)     : 读数时先缓存 FIFO 下标, 以防读取过程中发生新捕获而错乱;
 *                 假设两次捕获间隔大于读取这两个值的时间。
 *******************************************************************************/
PeriodMeas_s GetLastHallPeriod(void)
{
      PeriodMeas_s PeriodMeasAux;
      u8 bLastSpeedFIFO_Index;

   // Store current index to prevent errors if Capture occurs during processing
   // 中文: 先缓存当前 FIFO 下标, 防止处理期间发生新捕获造成数据错乱。
   bLastSpeedFIFO_Index = bSpeedFIFO_Index;

   // This is done assuming interval between captures is higher than time
   // to read the two values
   PeriodMeasAux.wPeriod = SensorPeriod[bLastSpeedFIFO_Index].hCapture;
   PeriodMeasAux.wPeriod *= (SensorPeriod[bLastSpeedFIFO_Index].hPrscReg + 1);
   // 中文: 用 (预分频+1) 还原真实周期(单位 CKTIM 时钟)。
   
   PeriodMeasAux.bDirection = SensorPeriod[bLastSpeedFIFO_Index].bDirection;
   return (PeriodMeasAux);
}


/*******************************************************************************
* ROUTINE Name : GetAvrgHallPeriod
*
* Description    : returns the rotor pseudo-period based on 4 last captures
* Input          : None
* Output         : None
* Return         : averaged rotor pseudo-period, as a number of CKTIM periods
* Side effect: the very last period acquired may not be considered for the
* calculation if a capture occurs during averaging.
*******************************************************************************/
/* 功能说明(中文) : 取 FIFO 内最近 HALL_SPEED_FIFO_SIZE 次捕获的平均伪周期。
 * 参数(中文)     : 无
 * 返回(中文)     : PeriodMeas_s —— 平均后的伪周期(单位 CKTIM 时钟)及其转向
 * 备注(中文)     : 计算时逐项短暂关闭捕获中断, 保证读到的预分频与捕获值属于
 *                 同一段周期; 若平均过程中发生捕获, 最新一次可能不计入。
 *******************************************************************************/
PeriodMeas_s GetAvrgHallPeriod(void)
{
    u32 wFreqBuffer, wAvrgBuffer, wIndex;
    PeriodMeas_s PeriodMeasAux;

  wAvrgBuffer = 0;

  for ( wIndex = 0; wIndex < HALL_SPEED_FIFO_SIZE; wIndex++ )
  {
     // Disable capture interrupts to have presc and capture of the same period
     // 中文: 临时关闭捕获中断, 保证预分频值与捕获值取自同一段周期。
     HALL_TIMER->DIER &= ~TIM_IT_CC1; // NB:Std libray not used for perf issues
     // 中文: 直接操作 DIER 寄存器关闭 CC1 中断(为性能未用标准库函数)。
     
     wFreqBuffer = SensorPeriod[wIndex].hCapture;
     // 中文: 取出该槽位的捕获计数值。
     wFreqBuffer *= (SensorPeriod[wIndex].hPrscReg + 1);
     // 中文: 乘以 (预分频+1) 得到以 CKTIM 计数的真实周期。
     
     HALL_TIMER->DIER |= TIM_IT_CC1;   // NB:Std libray not used for perf issue
     wAvrgBuffer += wFreqBuffer;	// Sum the whole periods FIFO
     PeriodMeasAux.bDirection = SensorPeriod[wIndex].bDirection;
  }
  // Round to upper value
  // 中文: 先加 (FIFO/2 - 1) 再整除, 实现「向大取整」的平均。
  wAvrgBuffer = (u32)(wAvrgBuffer + (HALL_SPEED_FIFO_SIZE/2)-1);  
  wAvrgBuffer /= HALL_SPEED_FIFO_SIZE;        // Average value	

  PeriodMeasAux.wPeriod = wAvrgBuffer;
  
  return (PeriodMeasAux);
}


/*******************************************************************************
* ROUTINE Name : HALL_StartHallFiltering
*
* Description : Set the flags to initiate hall speed values smoothing mechanism.
* Input       : None
* Output      : The result of the next capture will be copied in the whole array
*               to have 1st average = last value.
* Return      : None
* Note: The initialization of the FIFO used to do the averaging will be done
*       when the next input capture interrupt will occur.
*******************************************************************************/
/* 功能说明(中文) : 置位标志, 启动霍尔转速滑动平均机制。
 * 参数(中文)     : 无
 * 返回(中文)     : 无
 * 备注(中文)     : 下一次捕获结果会被复制到整个 FIFO, 使首次平均等于最新值;
 *                 FIFO 的实际初始化在下一次输入捕获中断中完成。
 *******************************************************************************/
void HALL_StartHallFiltering( void )
{
   InitRollingAverage = TRUE;
}

/*******************************************************************************
* ROUTINE Name : HALL_ReadHallState
*
* Description : Read the GPIO Input used for Hall sensor IC and return the state  
* Input       : None
* Output      : None
* Return      : STATE_X
*
*******************************************************************************/

/* 功能说明(中文) : 直接读取 3 路霍尔输入 GPIO 的电平, 组合为霍尔状态码。
 * 参数(中文)     : 无
 * 返回(中文)     : u8 —— 霍尔状态 0..7(三位分别对应三个霍尔开关电平)
 * 备注(中文)     : 读取方式取决于承载霍尔的定时器:
 *                 TIM2 用 GPIOA 低 3 位; TIM3 用 PA6/PA7/PB0; TIM4 用 PA6/7/8。
 *******************************************************************************/
u8 ReadHallState(void)
{
  u8 ReadValue;
#if defined(TIMER2_HANDLES_HALL)  
  
  ReadValue = (u8)(GPIO_ReadInputData(GPIOA)) & GPIO_MSK;
  
#elif defined(TIMER3_HANDLES_HALL)
  
  ReadValue = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0)<<2;
  ReadValue |= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)<<1;
  ReadValue |= GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7);

#elif defined(TIMER4_HANDLES_HALL)
  ReadValue = ((u8)(GPIO_ReadInputData(GPIOA))>>6) & GPIO_MSK;
#endif
  
  return(ReadValue);
}

/*******************************************************************************
* ROUTINE Name : HALL_GetElectricalAngle
*
* Description : Export the variable containing the latest angle updated by IC 
*               interrupt
* Input       : None
* Output      : None
* Return      : Electrical angle s16 format
*
*******************************************************************************/
/* 功能说明(中文) : 导出由输入捕获中断更新的最新转子电角度。
 * 参数(中文)     : 无
 * 返回(中文)     : s16 —— 电角度, s16 定标(360 度 = 65536, S16_MAX = 180 度)
 * 备注(中文)     : 每 60 电角度(一次霍尔跳变)被中断同步刷新一次。
 *******************************************************************************/
s16 HALL_GetElectricalAngle(void)
{
  return(hElectrical_Angle);
}

/*******************************************************************************
* ROUTINE Name : HALL_IncElectricalAngle
*
* Description : Increment the variable containing the rotor position information.
*               This function is called at each FOC cycle for integrating 
*               the speed information
* Input       : None
* Output      : None
* Return      : Electrical angle s16 format
*
*******************************************************************************/
/* 功能说明(中文) : 按转速积分更新转子位置(电角度)。每个 FOC 周期调用一次。
 * 参数(中文)     : 无
 * 返回(中文)     : 无
 * 备注(中文)     : 用 hRotorFreq_dpp(每 PWM 周期的角度增量)累加式积分, 在两次
 *                 霍尔跳变之间提供连续的平滑角度; 超速时沿用上一次增量。
 *******************************************************************************/
void HALL_IncElectricalAngle(void)
{ 
  static s16 hPrevRotorFreq;
 
  if (hRotorFreq_dpp != HALL_MAX_PSEUDO_SPEED)
  // 中文: 非超速时用当前增量积分角度; 超速(哨兵值)时沿用上一次增量, 避免角度突变。
  {
    hElectrical_Angle += hRotorFreq_dpp;
    // 中文: 电角度累加「每 PWM 周期的角度增量」, 实现两次霍尔跳变之间的平滑积分
    hPrevRotorFreq = hRotorFreq_dpp;
    // 中文: 记录本次增量, 供超速时备用
  }
  else
  {
    hElectrical_Angle += hPrevRotorFreq;
    // 中文: 超速时沿用上一次的有效增量继续积分
  }
}

/*******************************************************************************
* ROUTINE Name : HALL_Init_Electrical_Angle
*
* Description : Read the logic level of the three Hall sensor and individuates   
*               this way the position of the rotor (+/- 30°). Electrical angle 
*               variable is then initialized
*
* Input       : None
* Output      : None
* Return      : Electrical angle s16 format
*
*******************************************************************************/
/* 功能说明(中文) : 读取三路霍尔逻辑电平, 判定转子所处位置(约 +/-30 度范围),
 *                 据此初始化电角度变量(启动定位用)。
 * 参数(中文)     : 无
 * 返回(中文)     : 无(内部写 hElectrical_Angle)
 * 备注(中文)     : 依 HALL_SENSORS_PLACEMENT(120 度/60 度)选择不同的状态->角度
 *                 映射; 角度值以 S16_PHASE_SHIFT(相位偏移)为基准叠加/减去
 *                 60/120 度(对应 S16_60/S16_120_PHASE_SHIFT)。
 *******************************************************************************/
void HALL_Init_Electrical_Angle(void)
{
#if (HALL_SENSORS_PLACEMENT == DEGREES_120) 
 // 中文: 120 度排布: 按下表把当前霍尔状态映射为转子电角度(以 S16_PHASE_SHIFT 为基准加减 60/120 度)。
 switch(ReadHallState())
 {
  case STATE_5:
    hElectrical_Angle = (s16)(S16_PHASE_SHIFT+S16_60_PHASE_SHIFT/2);
    break;
  case STATE_1:
    hElectrical_Angle =(s16)(S16_PHASE_SHIFT+S16_60_PHASE_SHIFT+
                                                          S16_60_PHASE_SHIFT/2);
    break;
  case STATE_3:
    hElectrical_Angle =(s16)(S16_PHASE_SHIFT+S16_120_PHASE_SHIFT+
                                                          S16_60_PHASE_SHIFT/2);      
    break;
  case STATE_2:
    hElectrical_Angle =(s16)(S16_PHASE_SHIFT-S16_120_PHASE_SHIFT-
                                                          S16_60_PHASE_SHIFT/2);      
    break;
  case STATE_6:
    hElectrical_Angle =(s16)(S16_PHASE_SHIFT-S16_60_PHASE_SHIFT-
                                                          S16_60_PHASE_SHIFT/2);          
    break;
  case STATE_4:
    hElectrical_Angle =(s16)(S16_PHASE_SHIFT-S16_60_PHASE_SHIFT/2);          
    break;    
  default:    
    break;
  }
#elif (HALL_SENSORS_PLACEMENT == DEGREES_60)
 // 中文: 60 度排布: 霍尔状态->电角度映射(与 120 度不同, 使用 STATE_0/STATE_7 等组合)。
 switch(ReadHallState())
 {  
  case STATE_1:
    hElectrical_Angle =(s16)(S16_PHASE_SHIFT+S16_60_PHASE_SHIFT/2);
    break;
  case STATE_3:
    hElectrical_Angle =(s16)(S16_PHASE_SHIFT+S16_60_PHASE_SHIFT+
                                                          S16_60_PHASE_SHIFT/2);
    break;
  case STATE_7:
    hElectrical_Angle =(s16)(S16_PHASE_SHIFT+S16_120_PHASE_SHIFT+
                                                          S16_60_PHASE_SHIFT/2);      
    break;
  case STATE_6:
    hElectrical_Angle =(s16)(S16_PHASE_SHIFT-S16_120_PHASE_SHIFT-
                                                          S16_60_PHASE_SHIFT/2);      
    break;
  case STATE_4:
    hElectrical_Angle =(s16)(S16_PHASE_SHIFT-S16_60_PHASE_SHIFT-
                                                          S16_60_PHASE_SHIFT/2);          
    break;
  case STATE_0:
    hElectrical_Angle =(s16)(S16_PHASE_SHIFT-S16_60_PHASE_SHIFT/2);          
    break;    
  default:    
    break;
  }
#endif
}

/*******************************************************************************
* Function Name  : TIMx_IRQHandler
* Description    : This function handles both the capture event and Update event 
*                  interrupt handling the hall sensors signal period measurement
*                  
*                  - On 'CAPTURE' event case:
*                    The spinning direction is extracted
*                    The electrical angle is updated (synchronized)
*                    If the average is initialized, the last captured measure is
*                    copied into the whole array.
*                    Period captures are managed as following:
*                    If too low, the clock prescaler is decreased for next measure
*                    If too high (ie there was overflows), the result is
*                    re-computed as if there was no overflow and the prescaler is
*                    increased to avoid overflows during the next capture
*                   
*                  - On 'UPDATE' event case:
*                    This function handles the overflow of the timer handling
*                    the hall sensors signal period measurement.
* Input          : 
*                  - On 'CAPTURE' event case:
*                    None
*                   
*                  - On 'UPDATE' event case: 
*                    None
*
* Output         : 
*                  - On 'CAPTURE' event case:
*                   Updates the array holding the 4 latest period measures, reset
*                   the overflow counter and update the clock prescaler to
*                   optimize the accuracy of the measurement.
*                   
*                  - On 'UPDATE' event case:
*                    Updates a Counter of overflows, handled and reset when next
*                    capture occurs. 
*
* Return         : None (Interrupt Service routine)
*******************************************************************************/
#if (defined HALL_SENSORS || defined VIEW_HALL_FEEDBACK)
/* 功能说明(中文) : 霍尔定时器的中断服务程序, 同时处理「捕获事件」与「更新(溢出)事件」。
 *   捕获事件(CC1):
 *     - 读取霍尔状态, 与上次状态比较判定转向(正转/反转/换向);
 *     - 依状态跳变同步刷新电角度 hElectrical_Angle;
 *     - 保存本次捕获值(必要时用溢出次数扩展到 >16 位)与预分频值;
 *     - 依捕获值大小动态增减预分频: 过小->减小预分频提高分辨率, 过大(曾溢出)
 *       ->按无溢出重算并增大预分频以防下次溢出;
 *     - 若请求初始化平均, 则把最新捕获值铺满 FIFO;
 *     - 更新转子频率 hRotorFreq_dpp。
 *   更新(溢出)事件:
 *     - 累加溢出计数; 若连续溢出次数达到 HALL_MAX_OVERFLOWS 仍未捕获, 则
 *       置超时标志 HallTimeOut 并把频率清 0。
 * 参数(中文)     : 无
 * 返回(中文)     : 无(中断服务例程)
 *******************************************************************************/
#if defined(TIMER2_HANDLES_HALL)
void TIM2_IRQHandler(void)
#elif defined(TIMER3_HANDLES_HALL)
void TIM3_IRQHandler(void)
#else // TIMER4_HANDLES_HALL
void TIM4_IRQHandler(void)
#endif

{
  static u8  bHallState; 
  // 中文: bHallState 保存上次霍尔状态(static 跨中断保持); bPrevHallState 为本次进入时的上次状态
  u8 bPrevHallState;

// Check for the source of TIMx int - Capture or Update Event - 
  if ( TIM_GetFlagStatus(HALL_TIMER, TIM_FLAG_Update) == RESET )
  {
    // A capture event generated this interrupt
    // 中文: 该中断由一次捕获事件产生(非溢出), 进入方向判别与电角度更新流程。
    bPrevHallState = bHallState;
    bHallState = ReadHallState();
    // 中文: 读取本次(现)霍尔状态; 与上次状态比较即可判定转向与所处扇区(见下方 switch)。
#if (HALL_SENSORS_PLACEMENT == DEGREES_120)    
    switch(bHallState)
    {
      case STATE_5:
        if (bPrevHallState == STATE_5)
        {
	 //a speed reversal occured 
         if(bSpeed<0)
         {
           bSpeed = POSITIVE_SWAP;
         }
         else
         {
           bSpeed = NEGATIVE_SWAP;
         }
        }
        else   
          if (bPrevHallState == STATE_6)
          {
           bSpeed = POSITIVE;
          }
          else 
            if (bPrevHallState == STATE_3)
            {
              bSpeed = NEGATIVE;
            }
		// Update angle
        if(bSpeed<0)
        {
          hElectrical_Angle = (s16)(S16_PHASE_SHIFT+S16_60_PHASE_SHIFT);
        }
        else if(bSpeed!= ERROR)
        {
          hElectrical_Angle = S16_PHASE_SHIFT;  
        }
        break;
             
    case STATE_3:
        if (bPrevHallState == STATE_3)
        {
		 //a speed reversal occured
         if(bSpeed<0)
         {
           bSpeed = POSITIVE_SWAP;
         }
         else
         {
           bSpeed = NEGATIVE_SWAP;
         }
        }
        else
          if (bPrevHallState == STATE_5)
          {
           bSpeed = POSITIVE;
          }
          else 
            if (bPrevHallState == STATE_6)
            {
              bSpeed = NEGATIVE;
            }
		// Update of the electrical angle
        if(bSpeed<0)
        {
          hElectrical_Angle = (s16)(S16_PHASE_SHIFT+S16_120_PHASE_SHIFT+
                                                            S16_60_PHASE_SHIFT);
        }
        else if(bSpeed!= ERROR)
        {
          hElectrical_Angle =(s16)(S16_PHASE_SHIFT + S16_120_PHASE_SHIFT);
        }
        break;  
      
      case STATE_6: 
        if (bPrevHallState == STATE_6)
        {
         if(bSpeed<0)
         {
           bSpeed = POSITIVE_SWAP;
         }
         else
         {
           bSpeed = NEGATIVE_SWAP;
         }
        }
        
        if (bPrevHallState == STATE_3)
        {
         bSpeed = POSITIVE; 
        }
        else 
          if (bPrevHallState == STATE_5)
          {
            bSpeed = NEGATIVE;
          }  
        if(bSpeed<0)
        {
          hElectrical_Angle =(s16)(S16_PHASE_SHIFT - S16_60_PHASE_SHIFT);  
        }
        else if(bSpeed!= ERROR)
        {
          hElectrical_Angle =(s16)(S16_PHASE_SHIFT - S16_120_PHASE_SHIFT); 
        }
        break;
        
      default:
        bSpeed = ERROR;
        break;
    }
#elif (HALL_SENSORS_PLACEMENT == DEGREES_60)    
    switch(bHallState)
    {
      case STATE_3:
        if (bPrevHallState == STATE_3)
        {
         if(bSpeed<0)
         {
           bSpeed = POSITIVE_SWAP;
         }
         else
         {
           bSpeed = NEGATIVE_SWAP;
         }
        }
        else          
          if (bPrevHallState == STATE_0)
          {
           bSpeed = POSITIVE;
          }
          else 
            if (bPrevHallState == STATE_6)
            {
              bSpeed = NEGATIVE;              
            }
        if(bSpeed<0)
        {
          hElectrical_Angle = (s16)(S16_PHASE_SHIFT+S16_120_PHASE_SHIFT);
        }
        else if(bSpeed!= ERROR)
        {
          hElectrical_Angle = (s16)(S16_PHASE_SHIFT+S16_60_PHASE_SHIFT);
        }
        break;
             
      case STATE_6:
        if (bPrevHallState == STATE_6)
        {
         if(bSpeed<0)
         {
           bSpeed = POSITIVE_SWAP;
         }
         else
         {
           bSpeed = NEGATIVE_SWAP;
         }
        } 
        else
          if (bPrevHallState == STATE_3)
          {
           bSpeed = POSITIVE;           
          }
          else 
            if (bPrevHallState == STATE_0)
            {
              bSpeed = NEGATIVE;
            }
        if(bSpeed<0)
        {
          hElectrical_Angle = (s16)(S16_PHASE_SHIFT-S16_120_PHASE_SHIFT);
        }
        else if(bSpeed!= ERROR)
        {
          hElectrical_Angle =(s16)(S16_PHASE_SHIFT + S16_120_PHASE_SHIFT+
                                                            S16_60_PHASE_SHIFT);
        }
        break;  
      
      case STATE_0:
        if (bPrevHallState == STATE_0)
        {
         if(bSpeed<0)
         {
           bSpeed = POSITIVE_SWAP;
         }
         else
         {
           bSpeed = NEGATIVE_SWAP;
         }
        } 
        else
          if (bPrevHallState == STATE_6)
          {
           bSpeed = POSITIVE;
          }
          else 
            if (bPrevHallState == STATE_3)
            {
              bSpeed = NEGATIVE;
            }
        
        if(bSpeed<0)
        {
          hElectrical_Angle =(s16)(S16_PHASE_SHIFT );  
        }
        else if(bSpeed!= ERROR)
        {
          hElectrical_Angle =(s16)(S16_PHASE_SHIFT - S16_60_PHASE_SHIFT);  
        }                      
        break;
        
      default:
        bSpeed = ERROR;
        break;
    }
#endif
   // A capture event occured, it clears the flag  	
	TIM_ClearFlag(HALL_TIMER, TIM_FLAG_CC1);
   
   // used for discarding first capture
   // 中文: 递增捕获计数; 首次捕获会被丢弃(因预分频尚未稳定)。
   if (hCaptCounter < U16_MAX)
   {
      hCaptCounter++;
   }

   // Compute new array index
   // 中文: 计算 FIFO 下一个写入下标(循环递增, 写满后回绕到 0)。
   if (bSpeedFIFO_Index != HALL_SPEED_FIFO_SIZE-1)
   {
      bSpeedFIFO_Index++;
   }
   else
   {
      bSpeedFIFO_Index = 0;
   }

   //Timeout Flag is cleared when receiving an IC
   // 中文: 收到一次输入捕获即说明霍尔信号仍在, 清除超时标志。
   HALL_ClrTimeOut();
   
   // Store the latest speed acquisition
   // 中文: 保存本次测速结果(捕获值/预分频/转向)到 FIFO; 分「有溢出」「无溢出」两种情形处理。
   if (bGP1_OVF_Counter != 0)	// There was counter overflow before capture
   {
        u32 wCaptBuf;
        u16 hPrscBuf;

      wCaptBuf = (u32)TIM_GetCapture1(HALL_TIMER);        
      
      hPrscBuf = HALL_TIMER->PSC;

      while (bGP1_OVF_Counter != 0)
      // 中文: 每溢出一次, 真实捕获值就应加一个计数周期(0x10000), 从而把 16 位捕获值扩展到大于 16 位。
      {
         wCaptBuf += 0x10000uL;// Compute the real captured value (> 16-bit)
         bGP1_OVF_Counter--;
         // OVF Counter is 8-bit and Capt is 16-bit, thus max CaptBuf is 24-bits
      }
      while(wCaptBuf > U16_MAX)
      // 中文: 若捕获值超出 16 位, 通过「虚拟预分频」折半压缩到 16 位以内。
      {
         wCaptBuf /= 2;		// Make it fit 16-bit using virtual prescaler
         // Reduced resolution not a problem since result just slightly < 16-bit
         hPrscBuf = (hPrscBuf * 2) + 1;
         if (hPrscBuf > U16_MAX/2) // Avoid Prsc overflow
         {
            hPrscBuf = U16_MAX;
            wCaptBuf = U16_MAX;
         }
      }
      SensorPeriod[bSpeedFIFO_Index].hCapture = wCaptBuf;
      SensorPeriod[bSpeedFIFO_Index].hPrscReg = hPrscBuf;
      SensorPeriod[bSpeedFIFO_Index].bDirection = bSpeed;
      if (RatioInc)
      {
         RatioInc = FALSE;	// Previous capture caused overflow
         // 中文: 上次捕获前已上调过预分频(因预装载/更新机制延迟), 本次不再重复调整。
         // Don't change prescaler (delay due to preload/update mechanism)
      }
      else
      {
         if ((HALL_TIMER->PSC) < HALL_MAX_RATIO) // Avoid OVF w/ very low freq
         {
            (HALL_TIMER->PSC)++; // To avoid OVF during speed decrease
            // 中文: 增大预分频, 避免转速下降时定时器溢出; 新值在下次捕获才真正生效。
            RatioInc = TRUE;	  // new prsc value updated at next capture only
         }
      }
   }
   else		// No counter overflow
   {
      u16 hHighSpeedCapture, hClockPrescaler;   

      hHighSpeedCapture = (u32)TIM_GetCapture1(HALL_TIMER);
        
      SensorPeriod[bSpeedFIFO_Index].hCapture = hHighSpeedCapture;
      SensorPeriod[bSpeedFIFO_Index].bDirection = bSpeed;
      // Store prescaler directly or incremented if value changed on last capt
      // 中文: 记录预分频值; 若上次捕获时改过, 则按补偿后的值保存。
      hClockPrescaler = HALL_TIMER->PSC;

      // If prsc preload reduced in last capture, store current register + 1
      if (RatioDec)  // and don't decrease it again
      // 中文: 上次捕获下调过预分频(寄存器预装载延迟), 本次存储时 +1 补偿, 且不再下调。
      {
         SensorPeriod[bSpeedFIFO_Index].hPrscReg = (hClockPrescaler)+1;
         RatioDec = FALSE;
      }
      else  // If prescaler was not modified on previous capture
      {
         if (hHighSpeedCapture >= LOW_RES_THRESHOLD)// If capture range correct
         // 中文: 捕获值足够大, 分辨率满足要求, 直接使用当前预分频。
         {
            SensorPeriod[bSpeedFIFO_Index].hPrscReg = hClockPrescaler;
         }
         else
         {
            if(HALL_TIMER->PSC == 0) // or prescaler cannot be further reduced
            // 中文: 预分频已为 0 无法再减小, 保持现状。
            {
               SensorPeriod[bSpeedFIFO_Index].hPrscReg = hClockPrescaler;
            }
            else  // The prescaler needs to be modified to optimize the accuracy
            {
               SensorPeriod[bSpeedFIFO_Index].hPrscReg = hClockPrescaler;
               (HALL_TIMER->PSC)--;	// Increase accuracy by decreasing prsc
               // Avoid decrementing again in next capt.(register preload delay)
               RatioDec = TRUE;
            }
         }
      }
   }
    
   if (InitRollingAverage)
   {
        // 中文: 平均初始化请求: 用最新一次捕获值铺满整个 FIFO, 使首次平均即为最新值。
        u16 hCaptBuf, hPrscBuf;
        s8 bSpeedAux;
        u32 wIndex;
      // Read last captured value and copy it into the whole array
      // 中文: 读取最新捕获值, 并复制到 FIFO 的每个槽位。
      hCaptBuf = SensorPeriod[bSpeedFIFO_Index].hCapture;
      hPrscBuf = SensorPeriod[bSpeedFIFO_Index].hPrscReg;
      bSpeedAux = SensorPeriod[bSpeedFIFO_Index].bDirection;
      
      for (wIndex = 0; wIndex != HALL_SPEED_FIFO_SIZE-1; wIndex++)
      {
         SensorPeriod[wIndex].hCapture = hCaptBuf;
         SensorPeriod[wIndex].hPrscReg = hPrscBuf;
         SensorPeriod[wIndex].bDirection = bSpeedAux;
      }
      InitRollingAverage = FALSE;
      // Starting from now, the values returned by MTC_GetRotorFreq are averaged
      // 中文: 自此以后, 返回的转速将采用滑动平均结果。
      DoRollingAverage = TRUE;
    }
   
  //Update Rotor Frequency Computation
  // 中文: 依据刚存入的周期数据重新计算转子频率(每个 PWM 周期的角度增量)。
   hRotorFreq_dpp = HALL_GetRotorFreq();
  
  }
  else 
  {
    TIM_ClearFlag(HALL_TIMER, TIM_FLAG_Update);  
  	// an update event occured for this interrupt request generation
    if (bGP1_OVF_Counter < U8_MAX)
    {
       bGP1_OVF_Counter++;
       // 中文: 累加溢出次数(饱和到 U8_MAX, 防止回绕)。
    }
  
    if (bGP1_OVF_Counter >= HALL_MAX_OVERFLOWS)
    {
       // 中文: 连续溢出次数达到阈值仍无捕获, 判定霍尔信号丢失/急减速, 置超时并把频率清 0。
       HallTimeOut = TRUE;
       hRotorFreq_dpp = 0;
    }    
  }
}

#endif // HALL_SENSORS defined

/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
