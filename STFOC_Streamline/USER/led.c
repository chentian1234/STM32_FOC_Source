/*******************************************************************************
* 文件说明(中文):
*   本文件实现状态指示灯的 GPIO 初始化。
*   4 个指示灯连接在 GPIOC 的 PC6~PC9，配置为 50MHz 推挽输出，初始状态全部熄灭。
*   在上电初始化阶段由 main() 调用一次。运行过程中各指示灯的含义大致为:
*     LED1(PC6): 在 ADC 注入中断中翻转，作为 FOC 电流环中断的心跳指示(见 stm32f10x_it.c)；
*     其余 LED 由上层程序用于指示运行/故障等状态。
*******************************************************************************/

#include "led.h"





/*******************************************************************************
* 功能说明(中文) : 初始化 4 个状态指示灯所用 GPIO。打开 GPIOC 时钟后，把
*                  PC6/PC7/PC8/PC9 配置为 50MHz 推挽输出，最后全部熄灭。
*                  由 main() 在上电初始化时调用一次。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 依赖 led.h 中的 Led1Port~Led4Port / Led1Pin~Led4Pin 宏定义；
*                  本板 LED 高电平点亮(引脚输出高电平时灯亮)。
*******************************************************************************/
void LedGpioInit( void )
{
    GPIO_InitTypeDef GPIO_InitStructure;    // GPIO 初始化结构体，用于逐个配置引脚

    //enable gpioc clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);   // 使能 GPIOC 的 APB2 时钟，否则无法配置该端口

    GPIO_InitStructure.GPIO_Pin = Led1Pin;          // 选择 LED1 引脚(PC6)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // 输出速度 50MHz(指示灯对速度无严格要求)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // 推挽输出模式
    GPIO_Init(Led1Port , &GPIO_InitStructure);          // 将上述配置写入 GPIOC

    GPIO_InitStructure.GPIO_Pin = Led2Pin;          // 选择 LED2 引脚(PC7)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(Led2Port , &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = Led3Pin;          // 选择 LED3 引脚(PC8)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(Led3Port , &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = Led4Pin;          // 选择 LED4 引脚(PC9)
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(Led4Port , &GPIO_InitStructure);

    Led1Off();      // 上电默认点亮前先全部熄灭，避免出现不确定的亮灭状态
    Led2Off();
    Led3Off();
    Led4Off();
}


