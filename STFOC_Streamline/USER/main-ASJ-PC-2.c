/*******************************************************************************
* 文件说明(中文):
*   本文件是历史/备份版本的主程序(ASJ-PC 系列的第 2 个版本)。
*   与另一历史版本 main-ASJ-PC.c、以及当前 main.c 的代码结构基本一致，
*   唯一显著差别是: 本版本把主循环中的 MCL_ChkPowerStage() 语句注释掉了
*   (即不做功率级检查)，可用于对比调试。
*   本文件同样定义了 main()，工程中不应与 main.c 同时编译(通常在 IAR 工程中排除)。
*******************************************************************************/

#include "includes.h"


/*******************************************************************************
* 函数名称 : main
* 描述     : 程序入口。完成系统初始化后进入主循环，运行电机控制状态机。
* 参数     : 无
* 返回     : int (实际不会返回，主循环永不退出)
*******************************************************************************/
int main(void)
{
    LedGpioInit();      // 初始化状态指示灯 GPIO
    UART_init();        // 初始化串口(与上位机通信)
    SVPWM_3ShuntInit(); // 初始化三相 PWM 与三电阻电流采样
    ENC_Init();         // 初始化增量式编码器
    
    TB_Init();          // 初始化时间基准(定时/延时)
    // 初始化转矩环、磁链环、速度环三个 PI(D) 调节器
    PID_Init(&PID_Torque_InitStructure, &PID_Flux_InitStructure, &PID_Speed_InitStructure);
    DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);  // 调试时暂停 TIM1 计数
    MCL_Init_Arrays();  // 初始化电机控制层数组与状态量
    
    KEYS_Init();        // 初始化按键
    //State = INIT;     // (调试用)可强制直接进入初始化状态
    while(1)
    { 
        //UartProcess();  // (可选)处理串口命令

        //MCL_ChkPowerStage();    // 本版本注释掉了功率级检查
        //User interface management    
        KEYS_process();         // 扫描处理按键(启停/菜单)
        
        switch (State)          // 主状态机
        {
        case IDLE:    // Idle state   
            break;
            
        case INIT:              // 初始化态: 完成电机控制层初始化后转入启动流程
            MCL_Init();
            TB_Set_StartUp_Timeout(3000);   // 设置启动超时保护
            State = START; 
            break;
            
        case START:             // 启动态: 由时间基准中断完成启动流程
            break;
            
        case RUN:   // motor running       
            if(ENC_ErrorOnFeedback() == TRUE)   // 编码器反馈是否出错?
            {
                MCL_SetFault(SPEED_FEEDBACK);   // 上报速度反馈故障
            }
            break;  
            
        case STOP:    // motor stopped        // 停止态: 关闭 PWM 并把电压指令清零
            TIM_CtrlPWMOutputs(TIM1, DISABLE);      // 关闭 TIM1 PWM 输出
            State = WAIT;
            SVPWM_3ShuntAdvCurrentReading(DISABLE); // 关闭电流采样
            Stat_Volt_alfa_beta.qV_Component1 = Stat_Volt_alfa_beta.qV_Component2 = 0;   // α/β 电压清零
            SVPWM_3ShuntCalcDutyCycles(Stat_Volt_alfa_beta);                                             
            //TB_Set_Delay_500us(2000); // 1 sec delay
            break;
            
        case WAIT:    // wait state        // 等待态: 等电机停转后回空闲
            if (TB_Delay_IsElapsed() == TRUE) 
            {          
                if(ENC_Get_Mechanical_Speed() ==0)             
                {              
                    State = IDLE;              
                }
            }
            break;
            
        case FAULT:                   // 故障态: 等待清除故障
            if (MCL_ClearFault() == TRUE)       // 故障已成功清除?
            {
                if(wGlobal_Flags & SPEED_CONTROL == SPEED_CONTROL)  // 速度控制模式?
                {
                    bMenu_index = CONTROL_MODE_MENU_1;
                }
                else
                {
                    bMenu_index = CONTROL_MODE_MENU_6;
                }
                State = IDLE;
                wGlobal_Flags |= FIRST_START;   // 标记首次启动
            }
            break;
        default:        
            break;
        }
    }
}








#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
  while (1)
  {
  }
}
#endif
