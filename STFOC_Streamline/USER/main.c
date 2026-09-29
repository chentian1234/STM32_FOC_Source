/*******************************************************************************
* 文件说明(中文):
*   本文件是整个电机控制工程的入口。
*   上电后依次完成: 状态指示灯 GPIO、串口、SVPWM 三电阻电流采样、增量式编码器、
*   时间基准、三个 PI(D) 调节器(转矩环/磁链环/速度环)、全局数组、按键的初始化，
*   随后进入无限循环，按状态机 State 依次执行: 功率级检查 -> 按键处理 -> 状态切换。
*   状态机各状态枚举定义见 MC_type.h 中的 SystStatus_t。
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
    LedGpioInit();      // 初始化状态指示灯所用 GPIO
    UART_init();        // 初始化串口(用于上位机通信/打印)
    SVPWM_3ShuntInit(); // 初始化三相 PWM 输出与三电阻电流采样
    ENC_Init();         // 初始化增量式编码器(提供转速/位置反馈)

    TB_Init();          // 初始化时间基准(用于定周期任务调度与延时)
    // 初始化三个 PI(D) 调节器: 转矩环、磁链环、速度环
    PID_Init(&PID_Torque_InitStructure, &PID_Flux_InitStructure, &PID_Speed_InitStructure);
    // 调试时暂停 TIM1 计数，便于单步调试时观察 PWM 相关寄存器
    DBGMCU_Config(DBGMCU_TIM1_STOP, ENABLE);
    MCL_Init_Arrays();  // 初始化电机控制层使用的数组与状态量

    KEYS_Init();        // 初始化按键
    //State = INIT;     // (调试用)可强制直接进入初始化状态
    while(1)
    {
        //UartProcess();  // (可选)处理串口接收到的命令

        MCL_ChkPowerStage();   // 检查功率级状态(母线电压/过流等)，异常时置故障
        // 用户界面管理
        KEYS_process();        // 扫描并处理按键，用于启停与菜单操作

        switch (State)         // 主状态机
        {
        case IDLE:    // 空闲状态: 等待用户启动命令
            break;

        case INIT:    // 初始化状态: 完成电机控制层初始化后转入启动流程
            MCL_Init();
            TB_Set_StartUp_Timeout(3000);  // 设置启动超时保护
            State = START;
            break;

        case START:   // 启动状态: 由时间基准中断完成对齐/开环强拖等启动流程
            break;

        case RUN:   // 运行状态: 电机正常运行，持续监控速度反馈是否异常
            if(ENC_ErrorOnFeedback() == TRUE)   // 编码器反馈出错?
            {
                MCL_SetFault(SPEED_FEEDBACK);   // 上报速度反馈故障
            }
            break;

        case STOP:    // 停止状态: 关闭 PWM 输出并把电压指令清零
            TIM_CtrlPWMOutputs(TIM1, DISABLE);      // 关闭 TIM1 的 PWM 输出
            State = WAIT;
            SVPWM_3ShuntAdvCurrentReading(DISABLE); // 关闭电流采样
            Stat_Volt_alfa_beta.qV_Component1 = Stat_Volt_alfa_beta.qV_Component2 = 0; // α/β 电压指令清零
            SVPWM_3ShuntCalcDutyCycles(Stat_Volt_alfa_beta);  // 以零电压刷新三相占空比
            //TB_Set_Delay_500us(2000); // 1 sec delay
            break;

        case WAIT:    // 等待状态: 等电机完全停转后再回到空闲
            if (TB_Delay_IsElapsed() == TRUE)       // 延时时间到?
            {
                if(ENC_Get_Mechanical_Speed() ==0)  // 机械转速已降为 0?
                {
                    State = IDLE;
                }
            }
            break;

        case FAULT:   // 故障状态: 等待清除故障
            if (MCL_ClearFault() == TRUE)           // 故障已被成功清除?
            {
                if(wGlobal_Flags & SPEED_CONTROL == SPEED_CONTROL)  // 当前处于速度控制模式?
                {
                    bMenu_index = CONTROL_MODE_MENU_1;
                }
                else
                {
                    bMenu_index = CONTROL_MODE_MENU_6;
                }
                State = IDLE;
                wGlobal_Flags |= FIRST_START;       // 标记为首次启动
            }
            break;
        default:
            break;
        }
    }
}








#ifdef  USE_FULL_ASSERT
/*******************************************************************************
* 函数名称 : assert_failed
* 描述     : 断言失败回调(IAR 标准库 assert 宏触发时调用)。
*            工程中无有效错误信息输出，仅在此死循环，方便调试时定位。
* 参数     : file - 出错的文件名;  line - 出错的行号
* 返回     : 无(不返回)
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
  while (1)
  {
  }
}
#endif
