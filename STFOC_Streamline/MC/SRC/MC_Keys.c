/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Keys.c
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file handles Joystick and button management
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
* 14/07/08 v2.0.1
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
* 中文文件说明(模块作用) :
*   本文件实现"按键/摇杆"人机交互模块, 属于演示板的本地输入驱动层:
*   1) KEYS_Init()      : 把五向摇杆(上/下/左/右/选择)与用户按键所在 GPIO 配置为浮空输入;
*   2) KEYS_Read()      : 轮询读取按键 GPIO, 逐键完成去抖动与"短按/长按保持"识别;
*   3) KEYS_process()   : 按键功能分发, 实现电机启停与转速给定增减, 并联动状态机 State;
*   4) KEYS_ExportbKey(): 导出最近一次按键代码供其它模块查询。
*   扫描方式为"纯轮询"(非中断), 由主循环周期调用, 去抖依赖 TB 时间基准(500us 计数)。
*   本文件还定义并持有全局菜单索引变量 bMenu_index。
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_lib.h"
#include "stm32f10x_MClib.h"
#include "MC_Globals.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* 按键/摇杆的 GPIO 端口与引脚分配(演示板硬件连线), 输入为低电平有效(按下=0)。 */
#define KEY_UP_PORT GPIOD        // 摇杆"上"键所用端口
#define KEY_UP_BIT  GPIO_Pin_8   // 摇杆"上"键所用引脚: PD8

#define KEY_DOWN_PORT GPIOD      // 摇杆"下"键所用端口
#define KEY_DOWN_BIT  GPIO_Pin_14 // 摇杆"下"键所用引脚: PD14

#define KEY_RIGHT_PORT GPIOE     // 摇杆"右"键所用端口
#define KEY_RIGHT_BIT  GPIO_Pin_0 // 摇杆"右"键所用引脚: PE0

#define KEY_LEFT_PORT GPIOE      // 摇杆"左"键所用端口
#define KEY_LEFT_BIT  GPIO_Pin_1  // 摇杆"左"键所用引脚: PE1

#define KEY_SEL_PORT GPIOD       // 摇杆"选择/确认"键所用端口
#define KEY_SEL_BIT  GPIO_Pin_12  // 摇杆"选择/确认"键所用引脚: PD12

#define USER_BUTTON_PORT GPIOB   // 用户按键所用端口
#define USER_BUTTON_BIT  GPIO_Pin_9 // 用户按键所用引脚: PB9(功能同 SEL)

/* 各按键去抖/状态位掩码(用于 bKey_Flag 的按位读写)。 */
#define  SEL_FLAG        (u8)0x02   // SEL 键状态位掩码(bit1), 本工程实际用于 SEL 键去抖
#define  RIGHT_FLAG      (u8)0x04   // RIGHT 键状态位掩码(bit2)
#define  LEFT_FLAG       (u8)0x08   // LEFT 键状态位掩码(bit3)
#define  UP_FLAG         (u8)0x10   // UP 键状态位掩码(bit4)
#define  DOWN_FLAG       (u8)0x20   // DOWN 键状态位掩码(bit5)

//Variable increment and decrement
/* 各可调量的"每次按键步长"(增减量), 供菜单/按键调参使用。 */

#define SPEED_INC_DEC     (u16)10     // 转速给定步长: 10 个 0.1Hz 单位, 即每按一次 1.0 Hz
#define KP_GAIN_INC_DEC   (u16)250    // 速度环比例增益 Kp 调节步长(定点 Q 格式)
#define KI_GAIN_INC_DEC   (u16)25     // 速度环积分增益 Ki 调节步长(定点 Q 格式)
#define KD_GAIN_INC_DEC   (u16)100    // 速度环微分增益 Kd 调节步长(定点 Q 格式)

#ifdef FLUX_WEAKENING
/* 弱磁(Flux Weakening)相关参数调节步长。 */
#define KP_VOLT_INC_DEC   (u8)50      // 弱磁电压环比例增益 Kp 调节步长
#define KI_VOLT_INC_DEC   (u8)10      // 弱磁电压环积分增益 Ki 调节步长
#define VOLT_LIM_INC_DEC  (u8)5       // 弱磁电压上限(以百分数计)调节步长
#endif

#define TORQUE_INC_DEC    (u16)250    // 转矩(q 轴电流)参考给定调节步长, s16 Q15 格式
#define FLUX_INC_DEC      (u16)250    // 磁链(d 轴电流)参考给定调节步长, s16 Q15 格式

#define K1_INC_DEC        (s16)(250)  // 反电动势观测器增益 K1 调节步长(用于观测器整定)
#define K2_INC_DEC        (s16)(5000) // 反电动势观测器增益 K2 调节步长(用于观测器整定)

#define PLL_IN_DEC        (u16)(25)   // 无感 PLL 锁相环增益调节步长

/* Private macro -------------------------------------------------------------*/
u8 KEYS_Read (void);   // 按键读取函数(私有)声明, 返回按键代码
/* Private variables ---------------------------------------------------------*/
static u8 bKey;           // 本模块保存的"最近一次按键代码"(供 KEYS_ExportbKey 导出)
static u8 bPrevious_key;  // 上一次扫描到的按键代码, 用于区分"初次按下"与"长按保持"
static u8 bKey_Flag;      // 按键状态标志位(位掩码, 如 SEL_FLAG), 记录去抖过程状态

#ifdef FLUX_WEAKENING
/* 弱磁控制相关的外部变量(供菜单调参时使用)。 */
extern s16 hFW_P_Gain;   // 弱磁电压环比例增益
extern s16 hFW_I_Gain;   // 弱磁电压环积分增益
extern s16 hFW_V_Ref;    // 弱磁目标电压参考(以百分数计)
#endif

#ifdef OBSERVER_GAIN_TUNING
/* 反电动势观测器/PLL 增益整定用的外部变量(供菜单调参时使用)。 */
extern volatile s32 wK1_LO;         // 观测器增益 K1(低字, 定点)
extern volatile s32 wK2_LO;         // 观测器增益 K2(低字, 定点)
extern volatile s16 hPLL_P_Gain, hPLL_I_Gain;   // 锁相环 P/I 增益
#endif

#ifdef FLUX_TORQUE_PIDs_TUNING
/* 若定义了转矩/磁链 PID 整定, 菜单默认从转矩控制菜单(菜单12)进入。 */
u8 bMenu_index = CONTROL_MODE_MENU_6;   // 菜单索引全局变量, 默认=CONTROL_MODE_MENU_6(转矩控制菜单)
#else
u8 bMenu_index ;                        // 菜单索引全局变量, 不初始化(由上层在 FAULT 清除等时机赋值)
#endif

/*******************************************************************************
* Function Name  : KEYS_Init
* Description    : Init GPIOs for joystick/button management
* Input          : None
* Output         : None
* Return         : None
* 功能说明(中文) : 初始化摇杆五向键(上/下/左/右/选择)与用户按键的 GPIO, 全部配置为
*                  浮空输入(GPIO_Mode_IN_FLOATING)。系统上电时调用一次。
* 参数(中文)     : 无。
* 返回(中文)     : 无。
* 备注(中文)     : 使能 GPIOA/GPIOB/GPIOC/GPIOD/GPIOE 的 APB2 时钟; 引脚分配见文件
*                  顶部 KEY_xxx / USER_BUTTON 宏。按键低电平有效, 无需外部上下拉配置。
*******************************************************************************/
void KEYS_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
    
  /* Enable GPIOA, GPIOB, GPIOC, GPIOE clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | 
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD |
                         RCC_APB2Periph_GPIOE, ENABLE);
 
  GPIO_StructInit(&GPIO_InitStructure);
  
  /* Joystick GPIOs configuration*/
  
  GPIO_InitStructure.GPIO_Pin = KEY_UP_BIT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(KEY_UP_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = KEY_DOWN_BIT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(KEY_DOWN_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = KEY_RIGHT_BIT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(KEY_RIGHT_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = KEY_LEFT_BIT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(KEY_LEFT_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = KEY_SEL_BIT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(KEY_SEL_PORT, &GPIO_InitStructure);
  
  /* User button GPIO configuration */
  
  GPIO_InitStructure.GPIO_Pin = USER_BUTTON_BIT;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(USER_BUTTON_PORT, &GPIO_InitStructure);
}
  


/*******************************************************************************
* Function Name  : KEYS_Read
* Description    : Reads key from demoboard.
* Input          : None
* Output         : None
* Return         : Return RIGHT, LEFT, SEL, UP, DOWN, KEY_HOLD or NOKEY
* 功能说明(中文) : 轮询读取摇杆/按键 GPIO(低电平=按下), 按固定优先级依次判读
*                  RIGHT > LEFT > SEL > 用户按键 > UP > DOWN, 返回本次按键事件。
*                  通过比较 bPrevious_key 区分"初次按下"(返回键名)与"长按保持"
*                  (返回 KEY_HOLD); SEL/用户按键额外用 TB 时间基准做 50ms 去抖。
* 参数(中文)     : 无。
* 返回(中文)     : u8 - 按键代码: RIGHT/LEFT/SEL/UP/DOWN(初次按下),
*                  KEY_HOLD(同一键长按保持)或 NOKEY(无键按下)。
* 备注(中文)     : 依赖静态变量 bPrevious_key(上次按键)与 bKey_Flag(去抖标志);
*                  为纯轮询实现, 需被周期调用才能及时捕捉按键。
*******************************************************************************/
u8 KEYS_Read ( void )
{
  /* "RIGHT" key is pressed */
  if(!GPIO_ReadInputDataBit(KEY_RIGHT_PORT, KEY_RIGHT_BIT))   // 右键是否被按下?(低电平有效)
  {
    if (bPrevious_key == RIGHT)    // 上次也是右键 → 判为长按保持
    {
      return KEY_HOLD;
    }
    else
    {
      bPrevious_key = RIGHT;       // 记录本次按键, 下次再读到即视为长按
      return RIGHT;                // 首次按下 → 返回右键
    }
  }
  /* "LEFT" key is pressed */
  else if(!GPIO_ReadInputDataBit(KEY_LEFT_PORT, KEY_LEFT_BIT))   // 左键是否被按下?
  {
    if (bPrevious_key == LEFT)     // 上次也是左键 → 长按保持
    {
      return KEY_HOLD;
    }
    else
    {
      bPrevious_key = LEFT;        // 记录本次按键
      return LEFT;                 // 首次按下 → 返回左键
    }
  }
  /* "SEL" key is pressed */
   if(!GPIO_ReadInputDataBit(KEY_SEL_PORT, KEY_SEL_BIT))   // SEL(选择/确认)键是否被按下?
  {
    if (bPrevious_key == SEL)    // 上次也是 SEL → 长按保持
    {
      return KEY_HOLD;
    }
    else
    {
      /* 以下为 SEL 键的软件去抖: 借助 TB 时间基准在 50ms 窗口内确认一次有效按键。
         去抖状态用标志位 SEL_FLAG 表示: 0=待确认(已启动延时), 1=已确认。 */
      if ( (TB_DebounceDelay_IsElapsed() == FALSE) && (bKey_Flag & SEL_FLAG == SEL_FLAG) )  // 已确认但延时未到 → 忽略
      {
        return NOKEY;
      }
      else
      {
      if ( (TB_DebounceDelay_IsElapsed() == TRUE) && ( (bKey_Flag & SEL_FLAG) == 0) )   // 延时已到且尚未标记 → 启动本次去抖
      {
        bKey_Flag |= SEL_FLAG;                          // 置位去抖标志, 标记"已检测到按下"
        TB_Set_DebounceDelay_500us(100); // 50 ms debounce     // 重新装载 100*500us = 50ms 去抖延时
      }
      else if ( (TB_DebounceDelay_IsElapsed() == TRUE) && ((bKey_Flag & SEL_FLAG) == SEL_FLAG) )  // 延时已到且已标记 → 确认有效按键
      {
        bKey_Flag &= (u8)(~SEL_FLAG);                   // 清除去抖标志, 准备下一次
        bPrevious_key = SEL;                            // 记录按键, 供长按判读
        return SEL;                                     // 返回 SEL 键(有效按下)
      }
      return NOKEY;                                     // 其它情况一律返回无键
      }
    }
  }
  /* "SEL" key is pressed */
  else if(!GPIO_ReadInputDataBit(USER_BUTTON_PORT, USER_BUTTON_BIT))   // 用户按键(PB9)是否被按下?(功能等同 SEL)
  {
    if (bPrevious_key == SEL)    // 上次也是该键 → 长按保持
    {
      return KEY_HOLD;
    }
    else
    {
      /* 用户按键与 SEL 键共用同一套去抖逻辑与 SEL_FLAG 标志。 */
      if ( (TB_DebounceDelay_IsElapsed() == FALSE) && (bKey_Flag & SEL_FLAG == SEL_FLAG) )  // 已确认但延时未到 → 忽略
      {
        return NOKEY;
      }
      else
      {
      if ( (TB_DebounceDelay_IsElapsed() == TRUE) && ( (bKey_Flag & SEL_FLAG) == 0) )   // 延时已到且尚未标记 → 启动去抖
      {
        bKey_Flag |= SEL_FLAG;                          // 置位去抖标志
        TB_Set_DebounceDelay_500us(100); // 50 ms debounce     // 装载 100*500us = 50ms 去抖延时
      }
      else if ( (TB_DebounceDelay_IsElapsed() == TRUE) && ((bKey_Flag & SEL_FLAG) == SEL_FLAG) )  // 延时已到且已标记 → 确认有效
      {
        bKey_Flag &= (u8)(~SEL_FLAG);                   // 清除去抖标志
        bPrevious_key = SEL;                            // 记录按键(对外统一当作 SEL)
        return SEL;                                     // 返回 SEL 键
      }
      return NOKEY;                                     // 其它情况返回无键
      }
    }
  }
   /* "UP" key is pressed */
  else if(!GPIO_ReadInputDataBit(KEY_UP_PORT, KEY_UP_BIT))   // 上键是否被按下?
  {
    if (bPrevious_key == UP)     // 上次也是上键 → 长按保持
    {
      return KEY_HOLD;
    }
    else
    {
      bPrevious_key = UP;        // 记录本次按键
      return UP;                 // 首次按下 → 返回上键(增加给定值)
    }
  }
  /* "DOWN" key is pressed */
  else if(!GPIO_ReadInputDataBit(KEY_DOWN_PORT, KEY_DOWN_BIT))   // 下键是否被按下?
  {
    if (bPrevious_key == DOWN)   // 上次也是下键 → 长按保持
    {
      return KEY_HOLD;
    }
    else
    {
      bPrevious_key = DOWN;      // 记录本次按键
      return DOWN;               // 首次按下 → 返回下键(减小给定值)
    }
  }
  
  /* No key is pressed */
  else
  {
    bPrevious_key = NOKEY;       // 无键按下: 清空上次按键, 使下次按下重新判为"首次"
    return NOKEY;                // 返回无键
  }
}



/*******************************************************************************
* Function Name  : KEYS_process
* Description    : Process key 
* Input          : Key code
* Output         : None
* Return         : None
* 功能说明(中文) : 按键功能分发函数。先调用 KEYS_Read() 取得本次按键代码, 再据其执行
*                  对应动作: UP/DOWN 增减转速给定 hSpeed_Reference; SEL 用于电机的
*                  启动/停止(驱动全局状态机 State: RUN/START→STOP, IDLE→INIT)。
*                  由主循环周期调用(见 main.c), 是"按键→电机控制"的落地环节。
* 参数(中文)     : 无(按键代码由内部 KEYS_Read 读取)。
* 返回(中文)     : 无。
* 备注(中文)     : 会直接改写全局变量 hSpeed_Reference(0.1Hz 定标)与 State;
*                  速度给定受 ±MOTOR_MAX_SPEED_HZ 限幅; 其余按键(RIGHT/LEFT)在此不处理。
*******************************************************************************/
void KEYS_process(void)
{
    bKey = KEYS_Read();    // read key pushed (if any...)     // 读取本次按键(无键则返回 NOKEY)

    switch(bKey)
    {
    case UP:                                            // 上键: 增大转速给定
        if (hSpeed_Reference <= MOTOR_MAX_SPEED_HZ)     // 未超过正向上限?
        {
            hSpeed_Reference += SPEED_INC_DEC;          // 转速给定 += 步长(0.1Hz 定标)
        }
        break;
        
        case DOWN:                                      // 下键: 减小转速给定
        if (hSpeed_Reference >= -MOTOR_MAX_SPEED_HZ)    // 未超过反向下限?
        {
            hSpeed_Reference -= SPEED_INC_DEC;          // 转速给定 -= 步长(0.1Hz 定标)
        }
        break;
    case SEL:                                           // SEL/确认键: 启停控制
        if (State == RUN)                               // 运行中 → 停机
        {
            State = STOP;               
        }
        else if (State== START)                         // 启动中 → 停机
        {
            State = STOP; 
        }
        else if(State == IDLE)                          // 空闲 → 进入初始化(启动流程)
        {
            State = INIT;
        }  
    
        break;
    default:                                            // 其它按键/长按: 不做处理
        break;
    }
}

/*******************************************************************************
* Function Name  : KEYS_ExportbKey
* Description    : Export bKey variable
* Input          : None
* Output         : None
* Return         : None
* 功能说明(中文) : 对外导出本模块静态变量 bKey(最近一次 KEYS_Read/KEYS_process 读到的
*                  按键代码), 供其他模块查询当前按键状态。
* 参数(中文)     : 无。
* 返回(中文)     : u8 - 最近一次按键代码(见 MC_Keys.h 的 NOKEY/SEL/RIGHT/LEFT/UP/DOWN/KEY_HOLD)。
* 备注(中文)     : 仅返回缓存值, 不触发任何扫描或硬件动作。
*******************************************************************************/
u8 KEYS_ExportbKey(void)
{
  return(bKey);
}
                   
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/

