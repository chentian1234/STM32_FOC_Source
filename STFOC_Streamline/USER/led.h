/*******************************************************************************
* 文件说明(中文):
*   本文件是状态指示灯(LED)驱动的头文件。
*   定义了 4 个指示灯所连接的端口(GPIOC)与引脚(PC6/PC7/PC8/PC9)，
*   并提供点亮(On)、熄灭(Off)、翻转(Toggle)三个操作宏，
*   最后对外声明指示灯 GPIO 初始化函数 LedGpioInit()。
*   指示灯用于向使用者指示系统/电机的运行状态(具体指示含义见 led.c 说明)。
*******************************************************************************/

#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"          // 标准外设库总头文件，提供 GPIO/RCC 等寄存器与函数定义

// 4 个指示灯均接在 GPIOC 端口上
#define Led1Port        GPIOC
#define Led2Port        GPIOC
#define Led3Port        GPIOC
#define Led4Port        GPIOC

// 各指示灯对应的引脚号: LED1->PC6、LED2->PC7、LED3->PC8、LED4->PC9
#define Led1Pin         GPIO_Pin_6
#define Led2Pin         GPIO_Pin_7
#define Led3Pin         GPIO_Pin_8
#define Led4Pin         GPIO_Pin_9


// 熄灭指示灯: 向 BRR(复位寄存器)写对应位，把引脚输出拉低(本板 LED 为高电平点亮，故拉低即熄灭)
#define Led1Off()       Led1Port->BRR = Led1Pin
#define Led2Off()       Led2Port->BRR = Led2Pin
#define Led3Off()       Led3Port->BRR = Led3Pin
#define Led4Off()       Led4Port->BRR = Led4Pin

// 点亮指示灯: 向 BSRR(置位寄存器)写对应位，把引脚输出拉高
#define Led1On()        Led1Port->BSRR = Led1Pin
#define Led2On()        Led2Port->BSRR = Led2Pin
#define Led3On()        Led3Port->BSRR = Led3Pin
#define Led4On()        Led4Port->BSRR = Led4Pin

// 翻转指示灯: 对 ODR(输出数据寄存器)对应位异或取反，实现亮灭状态切换(常用于闪烁/心跳指示)
#define Led1Toggle()    Led1Port->ODR ^= Led1Pin
#define Led2Toggle()    Led2Port->ODR ^= Led2Pin
#define Led3Toggle()    Led3Port->ODR ^= Led3Pin
#define Led4Toggle()    Led4Port->ODR ^= Led4Pin

/*******************************************************************************
* 功能说明(中文) : 初始化 4 个指示灯所用 GPIO，配置为推挽输出并默认全部熄灭。
*                  上电初始化阶段(main 开头)调用一次。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 会打开 GPIOC 时钟；必须在其它使用 PC6~PC9 的外设之前调用。
*******************************************************************************/
void LedGpioInit( void );

#endif


