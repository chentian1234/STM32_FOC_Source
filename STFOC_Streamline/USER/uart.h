/*******************************************************************************
* 文件说明(中文):
*   本文件是串口(USART1)通信模块的头文件。
*   主要定义: 上位机(主机)与本机(从机)的地址、串口工作状态枚举；
*             导出接收缓冲区 RxBuf；声明串口初始化、状态读写、发送数据、
*             获取命令类型等接口函数。
*   串口用于电机控制板与上位机(PC/主控)之间的命令与数据帧交互。
*******************************************************************************/

#ifndef _UART_H_
#define _UART_H_






// 主机(上位机/主控)地址，出现在协议帧的第 1 个字节
#define MASTER_ADDRESS	0xf1
// 本机(从机，即电机控制板)地址，出现在协议帧的第 2 个字节(原文拼写为 SLAVE_ADRESS)
#define SLAVE_ADRESS	0x01


typedef enum{ Idle=0,Sending,Receiveing,ReceiveCmd }UARTSTATUS;    // 串口状态机: Idle=空闲, Sending=正在发送, Receiveing=正在接收, ReceiveCmd=已收到一条完整命令帧
extern uint8_t RxBuf[128];    // 接收缓冲区(全局)，存放校验通过的一帧命令数据，帧格式见 uart.c 中断处理

void UART_init( void );                              // 初始化 USART1(9600-8-N-1)，配置收发 GPIO、中断，并允许发送/接收中断
UARTSTATUS GetUartStatus( void );                    // 读取当前串口状态
void SetUartStatus( UARTSTATUS status );             // 设置串口状态(供上层在别处复位状态)
void SendData( uint8_t *p, uint16_t Length );        // 通过串口发送 Length 字节数据(以中断方式逐字节发出)
uint8_t GetCmdType( void );                          // 获取已接收命令帧中的命令类型字节(帧第 4 字节)



#endif
