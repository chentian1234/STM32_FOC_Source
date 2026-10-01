/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_MotorControl_Layer.c
* Author             : IMS Systems Lab  
* Date First Issued  : 21/11/07
* Description        : This file contains the function implementing the motor 
*                      control layer 
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
* 模块说明(中文) : 电机控制层（MCL）实现。负责电机启动/停止过程中的外设与状态初始化、
*                  功率级与母线电压/温度检测、故障置位与清除、PID 积分项复位，以及
*                  （可选）制动电阻控制。它是 main.c 状态机与 FOC 算法之间的
*                  “逻辑外设管理层”。
*                  说明：FOC 的具体控制流程（电流采样→Clarke→Park→PID→反Park→
*                  SVPWM）与状态机状态（INIT/START/RUN/STOP/FAULT 等）在本库的
*                  MC_FOC_Drive.c 与 main.c 中实现；本模块提供其所需的初始化、母线
*                  电压/温度量与保护接口（过压/欠压/过流/过温/反馈丢失）。
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "stm32f10x_type.h"
#include "MC_Globals.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* 过流/紧急保护(BRK, MCES)输入端口。*/
#define BRK_GPIO GPIOE
/* 过流/紧急保护输入引脚：用于判断 OVER_CURRENT 是否可恢复（高电平=正常）。*/
#define BRK_PIN GPIO_Pin_15

/* 故障状态最短保持时间，单位 0.5ms（600×0.5ms=300ms），避免故障状态抖动。*/
#define FAULT_STATE_MIN_PERMANENCY 600 //0.5msec unit

/* 母线电压滑动平均窗口长度（参与平均的采样次数）。*/
#define BUS_AV_ARRAY_SIZE  (u8)32  //number of averaged acquisitions
/* 温度滑动平均窗口长度（参与平均的采样次数）。*/
#define T_AV_ARRAY_SIZE  (u16)2048  //number of averaged acquisitions

/* 母线电压换算系数：把 ADC 计数换算成伏特（3.32/(BUS_ADC_CONV_RATIO)）。*/
#define BUSV_CONVERSION (u16) (3.32/(BUS_ADC_CONV_RATIO)) 
/* 温度换算系数：满量程 32768 计数对应约 195°C（NTC 线性化后的斜率，含 14 偏移）。*/
#define TEMP_CONVERSION (u8)  195

/* 母线电压平均值初始值：取欠压与过压阈值的中值，避免上电误报。*/
#define VOLT_ARRAY_INIT (u16)(UNDERVOLTAGE_THRESHOLD+ OVERVOLTAGE_THRESHOLD)/2
/* 温度平均值初始值（0 计数）。*/
#define TEMP_ARRAY_INIT (u16)0

/* 制动电阻开关控制端口/引脚。*/
#define BRAKE_GPIO_PORT       GPIOD
#define BRAKE_GPIO_PIN        GPIO_Pin_13

/* 过温阈值（ADC 计数值）：由摄氏阈值 NTC_THRESHOLD_C 换算（32768=满量程）。*/
#define NTC_THRESHOLD (u16) ((32768*(NTC_THRESHOLD_C - 14))/TEMP_CONVERSION)
/* 过温迟滞阈值（ADC 计数值）：由 NTC_HYSTERIS_C 得回差，低于此值才能解除过温。*/
#define NTC_HYSTERIS  (u16) ((32768*(NTC_THRESHOLD_C - NTC_HYSTERIS_C - 14))\
                                                               /TEMP_CONVERSION)

/* Private macro -------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void MCL_Reset_PID_IntegralTerms(void);
/* Private variables ---------------------------------------------------------*/

static s16 h_BusV_Average;   /* 母线电压滑动平均值（ADC 计数定标，s16）*/
static u32 w_Temp_Average;   /* 温度滑动平均值（累加型，s32/u32 定标，供温度换算）*/

u16 h_ADCBusvolt;   /* 母线电压 ADC 原始采样值（由 ADC 中断写入）*/
u16 h_ADCTemp;      /* 功率级温度 NTC 的 ADC 原始采样值（由 ADC 中断写入）*/
  
/*******************************************************************************
* 功能说明 : 每次电机启动前执行的控制初始化：复位 PID 积分项、FOC_Init，
*            按位置/电流采样方式初始化相应外设（编码器/Hall/观测器、
*            三/单电阻或 ICS 电流采样），校准并开 PWM 输出，最后等待约 2ms
*            的 50% 占空比以给上桥自举电容充电。
* 参数     : 无。
* 返回     : 无。
* 备注     : 会开启 TIM1 PWM 输出并阻塞等待 TB_StartUp_Timeout 结束；
*            受编译开关 ENCODER/HALL_SENSORS/NO_SPEED_SENSORS 等影响。
*******************************************************************************/
void MCL_Init(void)
{
// reset PID's integral values
    MCL_Reset_PID_IntegralTerms();   // 复位各 PID 积分项
    FOC_Init();                      // 复位 FOC 内部状态
    
#ifdef ENCODER
    ENC_Clear_Speed_Buffer();
   #ifdef OBSERVER_GAIN_TUNING
      STO_Init();
   #endif
#elif defined HALL_SENSORS
    HALL_InitHallMeasure();
    HALL_Init_Electrical_Angle();
   #ifdef OBSERVER_GAIN_TUNING
      STO_Init();
   #endif
#elif defined NO_SPEED_SENSORS
    STO_Init();
   #ifdef VIEW_ENCODER_FEEDBACK
      ENC_Clear_Speed_Buffer();
   #elif defined VIEW_HALL_FEEDBACK
      HALL_InitHallMeasure();
      HALL_Init_Electrical_Angle();
   #endif
#endif    
        
#ifdef THREE_SHUNT                    
    SVPWM_3ShuntCurrentReadingCalibration();
#elif defined ICS_SENSORS
    SVPWM_IcsCurrentReadingCalibration();
#elif defined SINGLE_SHUNT
    SVPWM_1ShuntCurrentReadingCalibration();
#endif  
      
    Stat_Volt_alfa_beta.qV_Component1 = 0;
    Stat_Volt_alfa_beta.qV_Component2 = 0;             
    CALC_SVPWM(Stat_Volt_alfa_beta);
    hTorque_Reference = PID_TORQUE_REFERENCE;   
 
    //It generates for 2 msec a 50% duty cycle on the three phases to load Boot 
    //capacitance of high side drivers
    TB_Set_StartUp_Timeout(4);   // 设定启动自举充电等待时间（约 2ms）
    
    /* Main PWM Output Enable */
    TIM_CtrlPWMOutputs(TIM1,ENABLE);   // 使能主 PWM 输出
  
    while(!TB_StartUp_Timeout_IsElapsed())
    {
    }  
#ifdef THREE_SHUNT    
    // Enable the Adv Current Reading during Run state
    SVPWM_3ShuntAdvCurrentReading(ENABLE);   // 运行态启用三电阻高级电流读取
#endif  
#ifdef SINGLE_SHUNT    
    // Enable the Adv Current Reading during Run state
    SVPWM_1ShuntAdvCurrentReading(ENABLE);
#endif
}


/*******************************************************************************
* 功能说明 : 初始化母线电压与温度滑动平均值数组的初值，避免复位后因初始值
*            异常而误报故障。系统初始化时调用一次。
* 参数     : 无。
* 返回     : 无。
* 备注     : 母线电压初值取欠压/过压阈值中值；温度初值为 0。
*******************************************************************************/
void MCL_Init_Arrays(void)
{   
    w_Temp_Average = TEMP_ARRAY_INIT;
    h_BusV_Average = VOLT_ARRAY_INIT;   
}


/*******************************************************************************
* 功能说明 : 检查功率级工作条件：调用过温检测与母线欠压检测，异常则置对应
*            故障。周期调用（主循环/状态机）。
* 参数     : 无。
* 返回     : 无。
* 备注     : 母线过压由 ADC 模拟看门狗单独处理，不在此函数内。
*******************************************************************************/
void MCL_ChkPowerStage(void) 
{
    // 检查功率级是否过温
    if (MCL_Chk_OverTemp() == TRUE)   // 功率级是否过温？
    {
      MCL_SetFault(OVERHEAT);         // 置过温故障
    }   
    // 检查母线是否欠压
    if (MCL_Chk_BusVolt() == UNDER_VOLT)   // 母线是否欠压？
    {
      MCL_SetFault(UNDER_VOLTAGE);         // 置欠压故障
    }
    // 母线过压由 ADC 模拟看门狗单独检测
}

/*******************************************************************************
* 功能说明 : 置位一个故障：设置故障最短保持延时、关闭 TIM1 PWM 输出、
*            置位故障标志位、把 State 切到 FAULT 并显示故障菜单，
*            同时关闭 Shunt 高级电流读取。
* 参数     : hFault_type - 故障位掩码（OVERHEAT/OVER_CURRENT/OVER_VOLTAGE/
*            UNDER_VOLTAGE/START_UP_FAILURE/SPEED_FEEDBACK）。
* 返回     : 无。
* 备注     : 会直接关断 PWM 输出；故障需经 MCL_ClearFault 清除。
*******************************************************************************/
void MCL_SetFault(u16 hFault_type)
{
  TB_Set_Delay_500us(FAULT_STATE_MIN_PERMANENCY); 
  /* Main PWM Output Enable */
  TIM_CtrlPWMOutputs(TIM1, DISABLE);   // 关闭主 PWM 输出
  wGlobal_Flags |= hFault_type;        // 置位故障标志
  State = FAULT;                       // 进入 FAULT 状态
  bMenu_index = FAULT_MENU;            // 显示故障菜单
  // It is required to disable AdvCurrentReading in IDLE to sample DC 
  // Bus Value
#ifdef THREE_SHUNT
  SVPWM_3ShuntAdvCurrentReading(DISABLE);
#endif
#ifdef SINGLE_SHUNT
  SVPWM_1ShuntAdvCurrentReading(DISABLE);
#endif
}

/*******************************************************************************
* 功能说明 : 检查故障源是否已消失：在故障保持延时到期后，逐一复核过热、过压、
*            欠压、过流、启动失败、反馈丢失各故障；若条件已满足则清除相应标志。
*            当按下 SEL 键且所有故障位均已清零时返回 TRUE，允许恢复运行。
* 参数     : 无。
* 返回     : bool — TRUE 表示所有故障已清除、可以恢复；FALSE 表示仍有故障。
* 备注     : 过流恢复需检测 BRK 引脚为高电平（MCES 正常）。
*******************************************************************************/
bool MCL_ClearFault(void)
{     
  if (TB_Delay_IsElapsed())
  {   
    if ((wGlobal_Flags & OVERHEAT) == OVERHEAT)   
    {               
      if(MCL_Chk_OverTemp()== FALSE)
      {
        wGlobal_Flags &= ~OVERHEAT;
      }     
    }
    
    if ((wGlobal_Flags & OVER_VOLTAGE) == OVER_VOLTAGE)   
    {            
      if(MCL_Chk_BusVolt()== NO_FAULT)
      {
        wGlobal_Flags &= ~OVER_VOLTAGE;
      } 
    }
    
    if ((wGlobal_Flags & UNDER_VOLTAGE) == UNDER_VOLTAGE)   
    {            
      if(MCL_Chk_BusVolt()== NO_FAULT)
      {
        wGlobal_Flags &= ~UNDER_VOLTAGE;
      } 
    }
    
    if ((wGlobal_Flags & OVER_CURRENT) == OVER_CURRENT)
    {
      // 检查紧急制动引脚是否为高电平
              // 在重新启用 PWM 前检查 MCES 引脚是否为低电平
              if (GPIO_ReadInputDataBit(BRK_GPIO, BRK_PIN))
      {            
        wGlobal_Flags &= ~OVER_CURRENT;
      }
    }
  
    if ((wGlobal_Flags & START_UP_FAILURE) == START_UP_FAILURE )
    {
        wGlobal_Flags &= ~START_UP_FAILURE;
    } 
    
    if ((wGlobal_Flags & SPEED_FEEDBACK) == SPEED_FEEDBACK )
    {
        wGlobal_Flags &= ~SPEED_FEEDBACK;
    } 
  }
  
  if (KEYS_ExportbKey() == SEL)
  {
    if ( (wGlobal_Flags & (OVER_CURRENT | OVERHEAT | UNDER_VOLTAGE | 
                       SPEED_FEEDBACK | START_UP_FAILURE | OVER_VOLTAGE)) == 0 )       
    { 
      return(TRUE);
    } 
    else
    {
      return(FALSE);
    }
  }
  else 
  {
    return(FALSE);
  }
}

/*******************************************************************************
* 功能说明 : 更新温度滑动平均并判断功率级是否过温（带迟滞）：温度高于阈值返回
*            TRUE；处于阈值与迟滞阈值之间且此前已过温则保持 TRUE（防止抖动）。
* 参数     : 无。
* 返回     : bool — TRUE 表示过温。
* 备注     : 使用全局 h_ADCTemp 与 static w_Temp_Average。
*******************************************************************************/
bool MCL_Chk_OverTemp(void)
{
  bool bStatus;
   
  w_Temp_Average = ((T_AV_ARRAY_SIZE-1)*w_Temp_Average + h_ADCTemp)
                                                              /T_AV_ARRAY_SIZE;
  
  if (w_Temp_Average >= NTC_THRESHOLD)    
  {
    bStatus = TRUE;
  }
  else if (w_Temp_Average >= (NTC_HYSTERIS) ) 
    {
    if ((wGlobal_Flags & OVERHEAT) == OVERHEAT)
      {
        bStatus = TRUE;       
      }
    else
      {
        bStatus = FALSE;
      }
    }
  else 
    {
      bStatus = FALSE;
    }

  return(bStatus);
}

/*******************************************************************************
* 功能说明 : 用当前 ADC 采样值对母线电压做一阶滑动平均（指数平均），更新
*            全局 h_BusV_Average。
* 参数     : 无。
* 返回     : 无。
* 备注     : 使用 static h_BusV_Average 与全局 h_ADCBusvolt；窗口
*            BUS_AV_ARRAY_SIZE；在 ADC 中断/周期任务中调用。
*******************************************************************************/
void MCL_Calc_BusVolt(void)
{
 h_BusV_Average = ((BUS_AV_ARRAY_SIZE-1)*h_BusV_Average + h_ADCBusvolt)
                                                             /BUS_AV_ARRAY_SIZE;
}

/*******************************************************************************
* 功能说明 : 依据母线电压平均值判断母线状态：高于过压阈值返回 OVER_VOLT，
*            低于欠压阈值返回 UNDER_VOLT，否则返回 NO_FAULT。
* 参数     : 无。
* 返回     : BusV_t — 母线电压状态枚举（NO_FAULT/OVER_VOLT/UNDER_VOLT）。
* 备注     : 使用 static h_BusV_Average。
*******************************************************************************/
BusV_t MCL_Chk_BusVolt(void)
{
  BusV_t baux;
  if (h_BusV_Average > OVERVOLTAGE_THRESHOLD)    
  {
    baux = OVER_VOLT;
  }
  else if (h_BusV_Average < UNDERVOLTAGE_THRESHOLD)    
  {
    baux = UNDER_VOLT;
  }
  else 
  {
    baux = NO_FAULT; 
  }
  return ((BusV_t)baux);
}

/*******************************************************************************
* 功能说明 : 返回母线电压的 s16 内部定标值（ADC 计数），供观测器
*            STO_Calc_Rotor_Angle 等作为母线电压输入使用。
* 参数     : 无。
* 返回     : s16 — 母线电压平均值（内部 ADC 计数定标）。
* 备注     : 直接返回 static h_BusV_Average。
*******************************************************************************/
s16 MCL_Get_BusVolt(void)
{
  return (h_BusV_Average);
}

/*******************************************************************************
* 功能说明 : 把母线电压平均值换算为伏特返回（供显示/上位使用）。
* 参数     : 无。
* 返回     : u16 — 母线电压，单位 V；换算 = h_BusV_Average*BUSV_CONVERSION/32768。
* 备注     : 使用 static h_BusV_Average。
*******************************************************************************/
u16 MCL_Compute_BusVolt(void)
{
  return ((u16)((h_BusV_Average * BUSV_CONVERSION)/32768));
}

/*******************************************************************************
* 功能说明 : 把温度滑动平均值换算为摄氏温度返回（供显示使用）。
* 参数     : 无。
* 返回     : u8 — 温度，单位 °C；换算 = w_Temp_Average*TEMP_CONVERSION/32768 + 14。
* 备注     : 使用 static w_Temp_Average；+14 为换算偏移。
*******************************************************************************/
u8 MCL_Compute_Temp(void)
{
  return ((u8)((w_Temp_Average * TEMP_CONVERSION)/32768+14));
}      

/*******************************************************************************
* 功能说明 : 复位速度、转矩(q 轴电流)、磁链(d 轴电流)三个 PID 调节器的积分项，
*            避免上次运行的积分残留导致启动冲击。
* 参数     : 无。
* 返回     : 无。
* 备注     : 操作全局 PID_Speed_InitStructure/PID_Torque_InitStructure/
*            PID_Flux_InitStructure 的 wIntegral 成员。
*******************************************************************************/
void MCL_Reset_PID_IntegralTerms(void)
{
  PID_Speed_InitStructure.wIntegral=0;
  PID_Torque_InitStructure.wIntegral=0;
  PID_Flux_InitStructure.wIntegral = 0;
}


#ifdef BRAKE_RESISTOR
/*******************************************************************************
* 功能说明 : 初始化制动电阻开关 GPIO（PD13 推挽输出）。仅当定义 BRAKE_RESISTOR 时编译。
* 参数     : 无。
* 返回     : 无。
* 备注     : 会打开 GPIOD 时钟并复位相关引脚。
*******************************************************************************/
void MCL_Brake_Init(void)
{  
  GPIO_InitTypeDef GPIO_InitStructure;

  /* 使能 GPIOD 时钟 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  GPIO_DeInit(BRAKE_GPIO_PORT);
  GPIO_StructInit(&GPIO_InitStructure);
                  
  /* 配置 PD.13 为推挽输出，用于制动功能 */
  GPIO_InitStructure.GPIO_Pin = BRAKE_GPIO_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(BRAKE_GPIO_PORT, &GPIO_InitStructure);     
}

/*******************************************************************************
* 功能说明 : 打开制动电阻（置位 GPIO 引脚）。仅当定义 BRAKE_RESISTOR 时编译。
* 参数     : 无。
* 返回     : 无。
*******************************************************************************/
void MCL_Set_Brake_On(void)
{  
 GPIO_SetBits(BRAKE_GPIO_PORT, BRAKE_GPIO_PIN);
}

/*******************************************************************************
* 功能说明 : 关闭制动电阻（复位 GPIO 引脚）。仅当定义 BRAKE_RESISTOR 时编译。
* 参数     : 无。
* 返回     : 无。
*******************************************************************************/
void MCL_Set_Brake_Off(void)
{  
 GPIO_ResetBits(BRAKE_GPIO_PORT, BRAKE_GPIO_PIN);
}

#endif
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/