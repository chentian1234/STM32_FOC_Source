/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_State_Observer_Interface.c
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This module implements the State Observer of 
*                      the PMSM B-EMF, thus identifying rotor speed and position
*
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 11/07/08 v2.0.1
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
* 模块说明(中文) : 无位置传感器（Sensorless）反电动势状态观测器的接口/启动管理层。
*                  本模块把电机参数与观测器常数下发给底层观测器，实现：
*                  (1) 转子位置与转速估计（由反电动势观测器 + PLL 锁相环解算）；
*                  (2) 电机开环启动状态机（S_INIT 初始化 → ALIGNMENT 对齐/预定位
*                      → RAMP_UP 开环强拖），在强拖过程中在线判定观测器是否收敛，
*                      收敛后把全局 State 切换为 RUN 进入闭环；
*                  (3) 转速可信度与反馈丢失检测。
*                  在 FOC 控制链路中，本模块处于“转子位置/转速估计”环节，为
*                  Park/反Park 变换提供电角度、为速度环提供转速反馈。
*                  术语：
*                   - 反电动势观测器：用定子电压/电流推算反电动势以估计转子位置。
*                   - PLL 锁相环：以估计反电动势的相位误差经 PI(PLL_P/PLL_I) 校正，
*                     输出跟踪转子角度与转速的环路。
*                   - 开环强拖(ramp-up)：无位置传感器时先按固定频率斜坡拖动电机，
*                     待反电动势足够大、观测器收敛后再切闭环。
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"
#include "MC_const.h"

#include "MC_PMSM_motor_param.h"
#include "MC_State_Observer_Interface.h"
#include "MC_State_Observer.h"
#include "MC_State_Observer_param.h"

/* Private typedef -----------------------------------------------------------*/
/* 无传感器启动状态机的状态枚举。
   S_INIT    : 初始化本次启动的斜坡变量（增量、起始电流、方向）；
   ALIGNMENT : 对齐/预定位阶段（可选，视 NO_SPEED_SENSORS_ALIGNMENT 而定）；
   RAMP_UP   : 开环强拖阶段，按规定斜率增大频率与电流，并判定观测器收敛。*/
typedef enum 
{
 S_INIT, ALIGNMENT, RAMP_UP
} Start_upStatus_t;

/* Private define ------------------------------------------------------------*/
/* 收敛判定用转速窗口下限系数：估计转速须≥强拖当前频率×0.8 才认为接近预期。*/
#define LOWER_THRESHOLD_FACTOR    0.8  //percentage of forced speed
/* 收敛判定用转速窗口上限系数：估计转速须≤强拖当前频率×1.0。*/
#define UPPER_THRESHOLD_FACTOR    1    //percentage of forced speed

/* 下限阈值(×10，因转速以 0.1Hz 定标)，用于内部比较。*/
#define LOW_THRESHOLD             (u8) (LOWER_THRESHOLD_FACTOR*10)
/* 上限阈值(×10)，用于内部比较。*/
#define UP_THRESHOLD              (u8) (UPPER_THRESHOLD_FACTOR*10)

/* OBSERVER_GAIN_TUNING 时在线整定用：系数 hC2 = F1*K1/Fs（电流误差反馈项）。*/
#define HC2_INIT                  (s16)((F1*wK1_LO)/SAMPLING_FREQ)
/* OBSERVER_GAIN_TUNING 时在线整定用：系数 hC4 = (K2*Imax/Vmax)*F2/Fs（反电动势误差反馈项）。*/
#define HC4_INIT                  (s16)(((wK2_LO*MAX_CURRENT)/MAX_BEMF_VOLTAGE)\
                                        *F2/SAMPLING_FREQ)
//Do not be modified
/* 对齐阶段持续的 PWM 周期数（由对齐时间 ms 按采样频率换算）。*/
#define SLESS_T_ALIGNMENT_PWM_STEPS       (u32) ((SLESS_T_ALIGNMENT * SAMPLING_FREQ)\
                                                                          /1000) 
/* 对齐角度换算为内部定标：65536 计数 = 360° 电角度。*/
#define SLESS_ALIGNMENT_ANGLE_S16         (s16)((s32)(SLESS_ALIGNMENT_ANGLE)\
                                                                    * 65536/360)
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 观测器收敛判定计数器：连续满足收敛条件时累加，达到 NB_CONSECUTIVE_TESTS 判为收敛。*/
static u16 bConvCounter;
/* 强拖期间注入的定子电流矢量电角度，定标 65536 = 360° 电角度。*/
static s16 hAngle = 0;
/* 强拖已走过的 PWM 周期计数（wTime==0 表示本次启动刚初始化）。*/
static u32 wTime = 0;
/* 强拖当前的频率累计量，定标 65536 = 360° 电角度（即电角度增量/周期的累加）。*/
static s32 wStart_Up_Freq = 0;
/* 强拖当前的电流给定累计量，定标 ×1024。*/
static s32 wStart_Up_I;
/* 每个 PWM 周期的频率增量（带符号，符号决定正反转）。*/
static s16 hFreq_Inc;
/* 每个 PWM 周期的电流增量（带符号），命名为 s32 变量存储 s16 值。*/
static s32 hI_Inc;

/* 当前启动状态机状态，初值为 S_INIT。*/
static Start_upStatus_t  Start_Up_State = S_INIT;

#ifdef OBSERVER_GAIN_TUNING
/* 在线整定用观测器增益 K1（可被上位界面修改）。*/
volatile s32 wK1_LO = K1;
/* 在线整定用观测器增益 K2（可被上位界面修改）。*/
volatile s32 wK2_LO = K2;
/* 在线整定用 PLL 比例增益。*/
volatile s16 hPLL_P_Gain = PLL_KP_GAIN;
/* 在线整定用 PLL 积分增益。*/
volatile s16 hPLL_I_Gain = PLL_KI_GAIN;
#endif

/* Private functions ---------------------------------------------------------*/
/* 判定观测器是否收敛（转速可信、估计值与强拖频率接近且连续满足，详见函数实现）。*/
bool IsObserverConverged(void);
/* 复位无传感器启动状态机及其内部变量。*/
void STO_StartUp_Init(void);

/*******************************************************************************
* Function Name : STO_StateObserverInterface_Init
* Description : It fills and passes to the State Obsever module the data 
*               structure necessary for rotor position observation 
* Input : None
* Output : None
* Return : None
* 功能说明(中文) : 填充观测器常数结构体 StateObserver_Const（C1/C3/C5、F1/F2/F3、
*                  C6、PLL 增益、最大转速 dpp 等），调用 STO_Gains_Init 下发给
*                  底层观测器。每次电机启动初始化时调用一次（MCL_Init 触发）。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 依赖电机参数 RS/LS、SAMPLING_FREQ、POLE_PAIR_NUM、MAX_CURRENT；
*                  当定义 OBSERVER_GAIN_TUNING 时，使用运行时可调增益
*                  (wK1_LO/wK2_LO/PLL 增益)覆盖 C2/C4 与 PLL 系数。
*******************************************************************************/
void STO_StateObserverInterface_Init(void)
{
 StateObserver_Const StateObserver_ConstStruct;
 
 StateObserver_ConstStruct.hC1 = C1;
 StateObserver_ConstStruct.hC3 = C3;
 StateObserver_ConstStruct.hC5 = C5; 
  
   {
    s16 htempk;
    StateObserver_ConstStruct.hF3 = 1;
    htempk = (s16)((100*65536)/(F2*2*PI));
    while (htempk != 0)
    {
      htempk /=2;
      StateObserver_ConstStruct.hF3 *=2;
    }
    StateObserver_ConstStruct.hC6 = (s16)((F2*StateObserver_ConstStruct.hF3*2*
                                                                    PI)/65536);
  }

#ifdef OBSERVER_GAIN_TUNING  
  /* lines below for debug porpose*/
  StateObserver_ConstStruct.hC2 = C2;
  StateObserver_ConstStruct.hC4 = C4;
    
  StateObserver_ConstStruct.hC2 = (s16)((F1*wK1_LO)/SAMPLING_FREQ);
  StateObserver_ConstStruct.hC4 = (s16)((((wK2_LO*MAX_CURRENT)/(MAX_BEMF_VOLTAGE
                                                       ))*F2)/(SAMPLING_FREQ));
  StateObserver_ConstStruct.PLL_P = hPLL_P_Gain;
  StateObserver_ConstStruct.PLL_I = hPLL_I_Gain;
#else
  StateObserver_ConstStruct.hC2 = C2;
  StateObserver_ConstStruct.hC4 = C4;
  StateObserver_ConstStruct.PLL_P = PLL_KP_GAIN;
  StateObserver_ConstStruct.PLL_I = PLL_KI_GAIN;  
#endif    
  StateObserver_ConstStruct.hF1 = F1;
  StateObserver_ConstStruct.hF2 = F2;
  StateObserver_ConstStruct.wMotorMaxSpeed_dpp = MOTOR_MAX_SPEED_DPP;
  StateObserver_ConstStruct.hPercentageFactor = PERCENTAGE_FACTOR;
  
  STO_Gains_Init(&StateObserver_ConstStruct);
}
  
/*******************************************************************************
* Function Name : STO_Check_Speed_Reliability
* Description : Check for the continuity of the speed reliability. If the speed 
*               is continously not reliable, the motor must be stopped 
* Input : None
* Output : None
* Return : boolean value: TRUE if speed is reliable, FALSE otherwise.
* 功能说明(中文) : 对观测器转速可信度做迟滞滤波：若连续 RELIABILITY_HYSTERESYS
*                  次均不可信才返回 FALSE（请求停机）；期间只要出现一次可信即清零
*                  并返回 TRUE。按速度采样周期调用。
* 参数(中文)     : 无。
* 返回(中文)     : bool — TRUE 转速可信、允许继续运行；FALSE 判定反馈丢失。
* 备注(中文)     : 使用函数内 static bCounter 记录连续不可信次数。
*******************************************************************************/
bool STO_Check_Speed_Reliability(void)
{
  static u8 bCounter=0;
  bool baux;
  
  if(STO_IsSpeed_Reliable() == FALSE)   // 本次估计不可信
  {
   bCounter++;                          // 连续不可信计数
   if (bCounter >= RELIABILITY_HYSTERESYS)   // 达到迟滞次数 → 判为反馈丢失
   {
     bCounter = 0;
     baux = FALSE;
   }
   else
   {
     baux = TRUE;
   }
  }
  else
  {
   bCounter = 0;
   baux = TRUE;
  }
  return(baux);
}
           
/*******************************************************************************
* Function Name : IsObserverConverged
* Description : Check for algorithm convergence. The speed reliability and the
*               range of the value of the estimated speed are checked. 
* Input : None
* Output : None
* Return : boolean value: TRUE if algortihm converged, FALSE otherwise.
* 功能说明(中文) : 判定观测器算法是否收敛，作为开环强拖切换到闭环的依据。
*                  需同时满足：转速可信(方差足够小)、估计转速>MINIMUM_SPEED、
*                  且估计机械转速落入强拖当前频率的约 0.94~1.0 倍窗口内，
*                  并连续满足 NB_CONSECUTIVE_TESTS 次。
* 参数(中文)     : 无。
* 返回(中文)     : bool — TRUE 表示已收敛。
* 备注(中文)     : 使用文件级 static 变量 bConvCounter；比较窗口由当前强拖频率
*                  wStart_Up_Freq 换算，转速以 0.1Hz 定标比较（每个 PWM 周期调用）。
*******************************************************************************/
bool IsObserverConverged(void)
{ 
  s16 hEstimatedSpeed;
  s16 hUpperThreshold;
  s16 hLowerThreshold;

  hEstimatedSpeed = STO_Get_Speed_Hz();   // 观测器估计的机械转速（0.1Hz 定标）
  hEstimatedSpeed = (hEstimatedSpeed < 0 ? -hEstimatedSpeed : hEstimatedSpeed);   // 取绝对值
  hUpperThreshold = ((wStart_Up_Freq/65536) * 160)/(POLE_PAIR_NUM * 16);   // 收敛窗口上限≈强拖频率×1.0
  hUpperThreshold = (hUpperThreshold < 0 ? -hUpperThreshold : hUpperThreshold);
  hLowerThreshold = ((wStart_Up_Freq/65536) *150) / (POLE_PAIR_NUM * 16);   // 收敛窗口下限≈强拖频率×0.94
  hLowerThreshold = (hLowerThreshold < 0 ? -hLowerThreshold : hLowerThreshold);
  
  // If the variance of the estimated speed is low enough...
  if(STO_IsSpeed_Reliable() == TRUE)
  { 
    if(hEstimatedSpeed > MINIMUM_SPEED)
    {
      //...and the estimated value is quite close to the expected value... 
      if(hEstimatedSpeed >= hLowerThreshold)
      {
        if(hEstimatedSpeed <= hUpperThreshold)
        {
          bConvCounter++;
          if (bConvCounter >= NB_CONSECUTIVE_TESTS)
          {
            // ...the algorithm converged.
            return(TRUE);
          }
          else
          {
            return(FALSE);
          }            
        }
        else
        { 
          bConvCounter = 0;
          return(FALSE);
        }              
      }
      else
      { 
        bConvCounter = 0;
        return(FALSE);
      } 
    }
    else
    { 
      bConvCounter = 0;
      return(FALSE);
    } 
  }
  else
  { 
    bConvCounter = 0;
    return(FALSE);
  }    
}

/*******************************************************************************
* Function Name : STO_Get_Speed_Hz
* Description : It returns the motor mechanical speed (Hz*10)
* Input : None.
* Output : None.
* Return : hRotor_Speed_Hz.
* 功能说明(中文) : 把观测器估计转速（内部 dpp 定标）换算成机械转速返回，单位 0.1Hz。
* 参数(中文)     : 无。
* 返回(中文)     : s16 — 机械转速，定标 0.1Hz（Hz×10）；
*                  换算 = STO_Get_Speed()*SAMPLING_FREQ*10/(65536*极对数)。
* 备注(中文)     : 速度环反馈与启动判定均使用该量。
*******************************************************************************/
s16 STO_Get_Speed_Hz(void)
{
  return (s16)((STO_Get_Speed()* SAMPLING_FREQ * 10)/(65536*POLE_PAIR_NUM));
}

/*******************************************************************************
* Function Name : STO_Get_Mechanical_Angle
* Description : It returns the rotor position (mechanical angle,s16) 
* Input : None.
* Output : None.
* Return : hRotor_El_Angle/pole pairs number.
* 功能说明(中文) : 把估计的电角度换算为转子机械角度返回。
* 参数(中文)     : 无。
* 返回(中文)     : s16 — 机械角度，定标 65536 = 360° 机械角（= 电角度 / 极对数）。
* 备注(中文)     : 仅用于显示等对机械角度有需求的场合；FOC 使用电角度。
*******************************************************************************/
s16 STO_Get_Mechanical_Angle(void)
{
  return ((s16)(STO_Get_Electrical_Angle()/POLE_PAIR_NUM));
}

/*******************************************************************************
* Function Name : STO_Start_Up
* Description : This function implements the ramp-up by forcing a stator current 
*               with controlled amplitude and frequency. If the observer 
*               algorithm converged, it also assign RUN to State variable  
* Input : details the input parameters.
* Output : details the output parameters.
* Return : details the return value.
* 功能说明(中文) : 无位置传感器启动主状态机，按 PWM 周期在 PWM 中断中调用
*                  （与电流环同频执行）。三个状态：
*                  S_INIT  : 设定本次启动的斜坡增量与初值（依据速度参考方向）；
*                  ALIGNMENT: 可选对齐阶段，注入固定电角度的电流矢量把转子拖动
*                             到已知位置（仅当定义 NO_SPEED_SENSORS_ALIGNMENT）；
*                  RAMP_UP : 开环强拖，按斜率增大频率与电流并做电流环控制，
*                             同时在线判定观测器是否收敛；收敛后置 State=RUN，
*                             从而切换到闭环运行。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 强拖超时（wTime 超过 FREQ_STARTUP_PWM_STEPS）判为
*                  START_UP_FAILURE 并复位状态机；直接写全局量
*                  hTorque_Reference/hFlux_Reference，并调用 CALC_SVPWM 驱动功率级。
*******************************************************************************/
void STO_Start_Up(void)
{
  s16 hAux;
#ifdef NO_SPEED_SENSORS_ALIGNMENT
  static u32 wAlignmentTbase=0;
#endif  
  
  switch(Start_Up_State)   // 启动状态机分支：按当前状态执行对应阶段
  {
  case S_INIT:             // S_INIT：初始化本次启动的斜坡参数
    //Init Ramp-up variables
    if (hSpeed_Reference >= 0)      // 速度参考非负 → 正转
    {
      hFreq_Inc = FREQ_INC;         // 正转：频率增量为正
      hI_Inc = I_INC;               // 正转：电流增量为正
      if (wTime == 0)               // 首次进入（wTime 尚未累加）时装载起始电流
      {
        wStart_Up_I = FIRST_I_STARTUP *1024;   // 起始强拖电流给定（×1024 定标）
      }
    }
    else                            // 速度参考为负 → 反转
    {
      hFreq_Inc = -(s16)FREQ_INC;   // 反转：频率增量为负
      hI_Inc = -(s16)I_INC;  // 反转：电流增量为负
      if (wTime == 0)
      {
        wStart_Up_I = -(s32)FIRST_I_STARTUP *1024;   // 反转起始电流给定（负值）
      }
    }
    Start_Up_State = ALIGNMENT;     // 初始化完成，进入对齐阶段
    break;
    
  case ALIGNMENT:   // ALIGNMENT：可选对齐阶段——注入固定电角度电流矢量把转子拖到已知位置
#ifdef NO_SPEED_SENSORS_ALIGNMENT
    wAlignmentTbase++;
    if(wAlignmentTbase <= SLESS_T_ALIGNMENT_PWM_STEPS)   // 未到对齐时间，逐步建立电流
    {                  
      // 对齐电流（d 轴参考）随对齐时间线性上升，配合固定电角度把转子锁在已知位置
      hFlux_Reference = SLESS_I_ALIGNMENT * wAlignmentTbase / 
                                                    SLESS_T_ALIGNMENT_PWM_STEPS;               
      hTorque_Reference = 0;   // 对齐期间 q 轴(转矩)给定为 0
      
      Stat_Curr_a_b = GET_PHASE_CURRENTS(); 
      Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b); 
      // 采样三相电流 → Clarke(αβ) → Park(dq)：对齐阶段使用固定电角度 SLESS_ALIGNMENT_ANGLE_S16
      Stat_Curr_q_d = Park(Stat_Curr_alfa_beta, SLESS_ALIGNMENT_ANGLE_S16);  
      /*loads the Torque Regulator output reference voltage Vqs*/   
      // q 轴电流环 PID：由转矩电流给定与反馈得到 q 轴电压 Vqs
      Stat_Volt_q_d.qV_Component1 = PID_Regulator(hTorque_Reference, 
                        Stat_Curr_q_d.qI_Component1, &PID_Torque_InitStructure);  
      /*loads the Flux Regulator output reference voltage Vds*/
      Stat_Volt_q_d.qV_Component2 = PID_Regulator(hFlux_Reference, 
                          Stat_Curr_q_d.qI_Component2, &PID_Flux_InitStructure); 

      RevPark_Circle_Limitation();   // 电压矢量圆限幅，防止超出可调制范围

      /*Performs the Reverse Park transformation,
      i.e transforms stator voltages Vqs and Vds into Valpha and Vbeta on a 
      stationary reference frame*/

      Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);   // 反 Park：dq→αβ

      /*Valpha and Vbeta finally drive the power stage*/ 
      CALC_SVPWM(Stat_Volt_alfa_beta);   // SVPWM 生成三相占空比驱动功率级
    }
    else
    {
      wAlignmentTbase = 0;                
      Stat_Volt_q_d.qV_Component1 = Stat_Volt_q_d.qV_Component2 = 0;   // 对齐结束：电压输出清零
      hTorque_Reference = PID_TORQUE_REFERENCE;   // 恢复默认转矩/磁链参考
      hFlux_Reference = PID_FLUX_REFERENCE;
      Start_Up_State = RAMP_UP;                   // 对齐结束，进入开环强拖
      hAngle = SLESS_ALIGNMENT_ANGLE_S16;         // 强拖起始电角度=对齐角度
    }
#else
    Start_Up_State = RAMP_UP;    // 未定义 NO_SPEED_SENSORS_ALIGNMENT：跳过对齐直接进入强拖
#endif    
    break;
    
  case RAMP_UP:   // RAMP_UP：开环强拖阶段（按斜率升频升流并判定观测器收敛）
    wTime ++;   // 强拖周期计数递增  
    if (wTime <= I_STARTUP_PWM_STEPS)   // 电流斜坡阶段：频率与电流同时上升
    {     
      wStart_Up_Freq += hFreq_Inc;   // 频率累计量按增量上升
      wStart_Up_I += hI_Inc;         // 电流累计量按增量上升
    }
    else if (wTime <= FREQ_STARTUP_PWM_STEPS )   // 电流斜坡结束，仅频率继续上升
    {
      wStart_Up_Freq += hFreq_Inc;   // 频率继续上升
    }       
    else
    {
      MCL_SetFault(START_UP_FAILURE);   // 强拖超时：报启动失败故障
      //Re_initialize Start Up
      STO_StartUp_Init();               // 复位启动状态机
    }
    
    //Add angle increment for ramp-up
    hAux = wStart_Up_Freq/65536;   // 当前电频率(Hz) = 频率累计量 / 65536
    hAngle = (s16)(hAngle + (s32)(65536/(SAMPLING_FREQ/hAux)));   // 开环角度积分：按当前频率累加电角度
        
    Stat_Curr_a_b = GET_PHASE_CURRENTS(); 
    Stat_Curr_alfa_beta = Clarke(Stat_Curr_a_b); 
    Stat_Curr_q_d = Park(Stat_Curr_alfa_beta, hAngle);   // Park：用开环角度得到 dq 电流
    
    hAux = wStart_Up_I/1024;   // 电流累计量换算为电流给定（×1024 定标还原）
    hTorque_Reference = hAux;   // 强拖电流作为 q 轴(转矩)电流给定       
    hFlux_Reference = 0;   // 强拖期间 d 轴(磁链)给定为 0（Id=0 控制）
           
    /*loads the Torque Regulator output reference voltage Vqs*/   
    // q 轴电流环 PID：由转矩电流给定与反馈得到 q 轴电压 Vqs
    Stat_Volt_q_d.qV_Component1 = PID_Regulator(hTorque_Reference, 
                        Stat_Curr_q_d.qI_Component1, &PID_Torque_InitStructure);
    /*loads the Flux Regulator output reference voltage Vds*/
    // d 轴电流环 PID：由磁链(d 轴)电流给定与反馈得到 d 轴电压 Vds
    Stat_Volt_q_d.qV_Component2 = PID_Regulator(hFlux_Reference, 
                          Stat_Curr_q_d.qI_Component2, &PID_Flux_InitStructure); 
    
    RevPark_Circle_Limitation();   // 电压矢量圆限幅
  
    /*Performs the Reverse Park transformation,
    i.e transforms stator voltages Vqs and Vds into Valpha and Vbeta on a 
    stationary reference frame*/
    
    Stat_Volt_alfa_beta = Rev_Park(Stat_Volt_q_d);   // 反 Park：dq→αβ
  
    /*Valpha and Vbeta finally drive the power stage*/ 
    CALC_SVPWM(Stat_Volt_alfa_beta);   // SVPWM 生成三相占空比（最终驱动功率级）
    
    STO_Calc_Rotor_Angle(Stat_Volt_alfa_beta,Stat_Curr_alfa_beta,MCL_Get_BusVolt());   // 观测器：用电压/电流/母线电压估算转子角度与转速
   
    if (IsObserverConverged()==TRUE)   // 观测器是否已收敛？
    {      
      PID_Speed_InitStructure.wIntegral = (s32)(hTorque_Reference*256);   // 预置速度环积分，减小切换冲击
      STO_StartUp_Init();  
      State = RUN;                     // 切换到 RUN（闭环）状态
      if ((wGlobal_Flags & SPEED_CONTROL) != SPEED_CONTROL)   // 未启用速度环（转矩模式）时
      {
        hTorque_Reference = PID_TORQUE_REFERENCE;   // 直接给定额定转矩参考
        hFlux_Reference = PID_FLUX_REFERENCE;       // 直接给定额定磁链参考
      }      
    }    
    break;
  default:
    break;
  }    
}

/*******************************************************************************
* Function Name : STO_StartUp_Init
* Description : This private function initializes the sensorless start-up
* Input : details the input parameters.
* Output : details the output parameters.
* Return : details the return value.
* 功能说明(中文) : 复位无传感器启动状态机及其内部变量，使下次启动从 S_INIT 开始。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 在强拖超时、观测器收敛切换闭环等场合调用；清零角度/时间/频率
*                  累计量与收敛计数器。
*******************************************************************************/
void STO_StartUp_Init(void)
{
  //Re_initialize Start Up
  Start_Up_State = S_INIT;   // 状态机回到初始化
  hAngle = 0;                // 开环电角度清零
  wTime = 0;                 // 强拖周期计数清零
  wStart_Up_Freq = 0;        // 频率累计量清零
  bConvCounter = 0;          // 收敛计数清零
}      
      
/*******************************************************************************
* Function Name : STO_Obs_Gains_Update
* Description : This function updates state observer gains after they have been
*               changed by the user interface
* Input : details the input parameters.
* Output : details the output parameters.
* Return : details the return value.
* 功能说明(中文) : 在增益在线整定模式下，把用户界面修改后的 PLL 比例/积分增益与
*                  观测器系数 hC2/hC4 打包下发，更新底层观测器运行增益。
* 参数(中文)     : 无（读取全局可调量 hPLL_P_Gain/hPLL_I_Gain，并用 HC2_INIT/HC4_INIT 重算）。
* 返回(中文)     : 无。
* 备注(中文)     : 仅当定义 OBSERVER_GAIN_TUNING 时编译；其余模式下增益用常量初始化。
*******************************************************************************/
#ifdef OBSERVER_GAIN_TUNING
void STO_Obs_Gains_Update(void)
{
  StateObserver_GainsUpdate STO_GainsUpdateStruct;

  STO_GainsUpdateStruct.PLL_P = hPLL_P_Gain;   // PLL 比例增益
  STO_GainsUpdateStruct.PLL_I = hPLL_I_Gain;   // PLL 积分增益
  STO_GainsUpdateStruct.hC2 = HC2_INIT;        // 观测器系数 C2（电流误差反馈）
  STO_GainsUpdateStruct.hC4 = HC4_INIT;        // 观测器系数 C4（反电动势误差反馈）
  STO_Gains_Update(&STO_GainsUpdateStruct);
}      
#endif      
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
