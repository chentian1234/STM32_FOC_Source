/*******************************************************************************
* 文件说明(中文):
*   本文件实现把 C 标准库的 printf 输出重定向到串口 1(USART1) 的功能。
*   原理: 标准库的 printf 最终会调用 putchar() 逐字符输出，本文件实现了自己的
*   putchar()，在其中把字符写入 USART1 并等待发送完成，从而让 printf 的内容
*   从串口 1 输出。用于调试时向 PC 打印信息。
*   注意: 本文件的 Uart1Init() 与 uart.c 的 UART_init() 都操作 USART1，
*   但波特率不同(此处 115200，uart.c 为 9600)，工程中二者不应同时初始化。
*******************************************************************************/

#include "Uart1Printf.h"


/*******************************************************************************
* 功能说明(中文) : 配置串口 1 使用的 GPIO: PA9 复用推挽输出(TX)、PA10 浮空输入(RX)。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 只配置引脚与 GPIOA/AFIO 时钟，不配置也不使能 USART1。
*******************************************************************************/
void Uart1GpioInit( void )
{
    GPIO_InitTypeDef GPIO_InitStructure;        // GPIO 配置结构体

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);   // 使能 GPIOA 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);    // 使能复用功能时钟

    //PA9 as tx mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;               // TX = PA9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       // 输出速度 50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         // 复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //PA10 as rx mode
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;             // RX = PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   // 浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*******************************************************************************
* 功能说明(中文) : 初始化串口 1，用于 printf 调试输出。配置 115200-8-N-1、
*                  无流控、收发均使能，并使能 USART1。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 不配置 NVIC 中断(仅查询方式发送)；波特率 115200 与 uart.c 不同。
*******************************************************************************/
void Uart1Init( void )
{
    USART_InitTypeDef USART_InitStructure;              // USART 基本参数结构体
    USART_ClockInitTypeDef USART_ClockInitStructure;    // USART 同步时钟参数结构体

    Uart1GpioInit();                                    // 先配置 TX/RX 引脚

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  // 使能 USART1 时钟

    USART_InitStructure.USART_BaudRate = 115200;                            // 波特率 115200bps
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;             // 数据位 8 位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                  // 停止位 1 位
    USART_InitStructure.USART_Parity = USART_Parity_No;                     // 无奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;         // 收发均使能

    USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;             // 关闭同步时钟(异步串口)
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;                   // 未用(仅同步模式有效)
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;                 // 未用(仅同步模式有效)
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;         // 未用(仅同步模式有效)




    USART_Init(USART1,&USART_InitStructure);                    // 写入基本参数配置

    USART_ClockInit(USART1, &USART_ClockInitStructure);         // 写入同步时钟配置(本工程为关闭)

    USART_Cmd(USART1,ENABLE);                                   // 使能 USART1
}

/*******************************************************************************
* 功能说明(中文) : 标准库字符输出函数(printf 重定向的关键)。把 1 个字符写入
*                  USART1 的数据寄存器，并等待发送寄存器为空(TXE)后返回。
*                  链接器在启用 printf 时会调用本函数逐字符输出。
* 参数(中文)     : ch - 要输出的字符(取低 8 位)
* 返回(中文)     : ch (回显所写字符，符合 putchar 语义)
* 备注(中文)     : 采用查询(忙等)方式发送，会阻塞到该字节被送入发送寄存器；
*                  使用前须先调用 Uart1Init() 初始化 USART1。
*******************************************************************************/
int putchar(int ch)
{
    USART_SendData(USART1, (uint8_t) ch);                       // 把字符写入 USART1 数据寄存器
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET) // 等待发送数据寄存器空(TSX 置位)
    {}
    return ch;                                                  // 返回所写字符
}



