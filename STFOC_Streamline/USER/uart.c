/*******************************************************************************
* 文件说明(中文):
*   本文件实现 USART1 串口通信: 初始化、中断收发、状态机。
*   通信采用主从问答式协议，帧格式为:
*     字节0: 主机地址 (MASTER_ADDRESS = 0xF1)
*     字节1: 从机地址 (SLAVE_ADRESS   = 0x01)
*     字节2: 数据长度 N (整帧字节数)
*     字节3: 命令类型 (CmdType，如 0x80 读波形)
*     字节4..N-2: 数据区
*     字节N-1: 校验和 (前面所有字节累加和的低 8 位)
*   接收采用 USART1 中断逐字节驱动: 先校验收发地址，再累计长度与校验和，
*   校验通过后把整帧拷入全局 RxBuf 并置状态为 ReceiveCmd，由上层 UartProcess() 处理。
*   发送采用"发送完成(TC)中断 + 全局指针/长度"方式逐字节发出，期间状态为 Sending。
*******************************************************************************/

#include "includes.h"

uint16_t TxBufLength=0;     // 待发送的剩余字节数，发送中断每发出 1 字节减 1，减到 0 表示发送结束
uint8_t *pTxBuf;            // 发送数据指针，指向当前待发送字节，发送过程中自增
uint8_t RxBuf[128];         // 接收缓冲区(全局)，保存一帧校验通过的完整命令帧
uint8_t RxDatLength;        // 当前接收帧的总长度(取自帧第 3 字节，即下标 2)
uint8_t RxDatAddsum;        // 接收过程中的累加校验和，用于与帧尾校验字节比对

UARTSTATUS UartStatus;      // 串口当前状态(空闲/发送中/接收中/已收到命令)

/*******************************************************************************
* 功能说明(中文) : 配置 USART1 的中断优先级并使能其 NVIC 中断通道。
*                  在 UART_init() 中被调用一次。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 抢占优先级与子优先级均设为 0(最高)，保证串口中断能及时响应。
*******************************************************************************/
void UART_NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;              // NVIC 初始化结构体
  
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;                         // 选择 USART1 中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;                 // 抢占优先级 0(最高)
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                        // 子优先级 0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                           // 使能该中断通道
  NVIC_Init(&NVIC_InitStructure);                                           // 把配置写入 NVIC
}

/*******************************************************************************
* 功能说明(中文) : 初始化 USART1 串口。依次: 使能 GPIOA/AFIO/USART1 时钟，
*                  配置 PA9 为复用推挽输出(TX)、PA10 为浮空输入(RX)，
*                  设置 9600bps、8 位数据、1 位停止、无校验、无流控，收发均使能，
*                  使能发送完成(TC)与接收非空(RXNE)中断，最后配置 NVIC。
*                  在 main() 上电初始化时调用一次。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 本工程串口波特率为 9600(与 Uart1Printf.c 的 115200 不同，
*                  两者虽都用 USART1，但用途不同: 本文件用于与上位机协议交互)。
*******************************************************************************/
void UART_init( void )
{
    GPIO_InitTypeDef GPIO_InitStructure;            // 复用为串口引脚时的 GPIO 配置结构体
    USART_InitTypeDef USART_InitStructure;          // USART 基本参数配置结构体
    USART_ClockInitTypeDef USART_ClockInitStructure;// USART 同步时钟参数(本工程异步模式，仅作占位)
    
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);    // 使能 GPIOA 时钟(PA9/PA10 作为串口引脚)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);     // 使能复用功能(AFIO)时钟

    //PA9 as tx mode
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;               // TX = PA9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;       // 输出速度 50MHz
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;         // 复用推挽输出(串口发送脚)
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    //PA10 as rx mode
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;             // RX = PA10
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   // 浮空输入(串口接收脚)
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    
    
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  // 使能 USART1 外设时钟

    USART_InitStructure.USART_BaudRate = 9600;                                  // 波特率 9600bps
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                 // 数据位 8 位
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                      // 停止位 1 位
    USART_InitStructure.USART_Parity = USART_Parity_No;                         // 无奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // 无硬件流控
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;             // 同时使能接收与发送

    USART_ClockInitStructure.USART_Clock = USART_Clock_Disable;                 // 关闭同步时钟(异步串口)
    USART_ClockInitStructure.USART_CPOL = USART_CPOL_Low;                       // 未用(仅同步模式有效)
    USART_ClockInitStructure.USART_CPHA = USART_CPHA_2Edge;                     // 未用(仅同步模式有效)
    USART_ClockInitStructure.USART_LastBit = USART_LastBit_Disable;             // 未用(仅同步模式有效)
    
    USART_Init(USART1,&USART_InitStructure);                        // 写入基本参数配置
    USART_ClockInit(USART1, &USART_ClockInitStructure);             // 写入同步时钟配置(本工程为关闭)
    USART_Cmd(USART1,ENABLE);                                       // 使能 USART1
    
    
    USART_ITConfig(USART1,USART_IT_TC,ENABLE);                      // 使能"发送完成(TC)"中断，用于逐字节发送
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);                    // 使能"接收数据寄存器非空(RXNE)"中断，用于逐字节接收
    
    UART_NVIC_Configuration();                                      // 配置并使能 USART1 的 NVIC 中断
}


/*******************************************************************************
* 功能说明(中文) : USART1 中断服务函数，处理两类事件。
*                  (1) 发送完成(TC)事件: 从 pTxBuf 继续取出 1 字节写入 DR 发出，
*                      直到 TxBufLength 减为 0，然后把状态置回 Idle。
*                  (2) 接收非空(RXNE)事件: 逐字节接收并按帧格式解析，
*                      依次校验主机地址、从机地址、长度与累加校验和；
*                      校验通过后把整帧拷入 RxBuf 并置状态 ReceiveCmd。
*                  由硬件在每次发送完成/接收到 1 字节时自动调用。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 使用静态变量 i 记录帧内位置、RxBufStatic 暂存当前帧；
*                  接收完成时立即调用 SendData 回发(回显/应答)。
*******************************************************************************/
void USART1_IRQHandler( void )
{
    uint8_t ch;                          // 本次从中断取到的 1 个字节
    static uint8_t i=0;                  // 当前帧内字节序号(下标)，静态保持跨中断有效
    static char RxBufStatic[128];        // 接收暂存缓冲区，逐字节拼装当前帧

    
    
    if( USART_GetITStatus(USART1,USART_IT_TC) != RESET )    // 发生"发送完成(TC)"事件?
    {
        USART_ClearITPendingBit(USART1,USART_IT_TC);        // 清除 TC 中断标志

        if( TxBufLength != 0 )                              // 还有待发送字节?
        {
            TxBufLength--;                                  // 待发送字节数减 1
            
            USART1->DR = *pTxBuf++;                         // 取出当前字节写入数据寄存器并让指针后移
        }
        else
        {
        	UartStatus = Idle;                              // 全部字节已发完，串口状态回到空闲
        }
    }
    if( USART_GetITStatus(USART1,USART_IT_RXNE) != RESET )      // 发生"接收数据非空(RXNE)"事件?
    {
        USART_ClearITPendingBit(USART1,USART_IT_RXNE);          // 清除 RXNE 中断标志

    	UartStatus = Receiveing;                                // 标记串口正在接收
        
        ch = (uint8_t)USART_ReceiveData(USART1);                // 读出 1 个接收字节
    	RxBufStatic[i]=ch;                                      // 存入暂存缓冲区对应位置

        if( i <= 3 )                                    // 帧头区(下标 0~3): 地址/长度/命令字，需要校验
        {
        	if( i == 0 )
        	{
        		if( ch != MASTER_ADDRESS )                  // 第 0 字节应为"主机地址"
        			i=-1;
        		RxDatAddsum = 0;                            // 帧起始，校验和清零
        	}
        	else if( i == 1 )
        	{
        		if( ch != SLAVE_ADRESS )                    // 第 1 字节应为"从机(本机)地址"，否则丢弃本帧
					i=-1;
        	}
        	else if( i == 3 )                               // 第 3 字节为命令类型(此处仅到达该位置，长度已在上一步取得)
			{
        		RxDatLength = RxBufStatic[2];               // 从第 2 字节取出整帧长度 N
			}
        }
        else if( i == RxDatLength-1 )                   // 已到帧尾(最后一个字节)，进入校验收尾
        {
        	if( RxDatAddsum == ch )                     // 累加校验和与帧尾校验字节一致?
        	{
        		for( i=0;i<RxDatLength;i++)             // 校验通过，把暂存帧整帧拷贝到全局接收缓冲区
        		{
        			RxBuf[i] = RxBufStatic[i];          // 逐字节拷贝
        		}
                
                SendData(RxBuf,RxBuf[2]);               // 立即把收到的帧原样回发(应答/回显)

        		UartStatus = ReceiveCmd;                // 标记"已收到完整命令帧"，等待上层 UartProcess() 处理
        	}
        	i=-1;
        }

        RxDatAddsum += ch;                              // 把本字节累加入校验和(含帧头与数据)
        i++;                                            // 帧内下标后移，准备接收下一字节
    }
}


/*******************************************************************************
* 功能说明(中文) : 读取当前串口状态，供主循环 UartProcess() 轮询判断用。
* 参数(中文)     : 无
* 返回(中文)     : 当前 UARTSTATUS 枚举值(Idle/Sending/Receiveing/ReceiveCmd)
* 备注(中文)     : 只读，不改变状态。
*******************************************************************************/
UARTSTATUS GetUartStatus( void )
{
	return UartStatus;      // 返回当前串口状态
}

/*******************************************************************************
* 功能说明(中文) : 设置串口状态，供上层在需要时复位/切换状态。
* 参数(中文)     : status - 目标状态(UARTSTATUS 枚举)
* 返回(中文)     : 无
* 备注(中文)     : 直接写全局 UartStatus；不要在收发中断进行中误置为 Idle。
*******************************************************************************/
void SetUartStatus( UARTSTATUS status )
{
	UartStatus = status;    // 更新串口状态
}


/*******************************************************************************
* 功能说明(中文) : 以中断方式发送一段数据。记录缓冲区指针与长度后先发出首字节，
*                  其余字节由"发送完成(TC)"中断逐字节发出。
* 参数(中文)     : p      - 待发送数据首地址(发送期间须保持有效)
*                  Length - 待发送字节数(应大于 0)
* 返回(中文)     : 无
* 备注(中文)     : 全局状态置为 Sending，发送完毕由中断改回 Idle；
*                  调用前应确保上一次发送已结束(避免覆盖 pTxBuf/TxBufLength)。
*******************************************************************************/
void SendData( uint8_t *p, uint16_t Length )
{
	UartStatus = Sending;       // 标记串口进入发送状态

	pTxBuf = p;                 // 记录发送缓冲区首地址
	TxBufLength = Length;       // 记录待发送字节数

    if( TxBufLength != 0 )      // 长度不为 0 时先发送第一个字节，启动发送中断链
    {
        TxBufLength--;          // 已发出 1 字节，剩余数减 1
        USART1->DR = *pTxBuf++; // 写出首字节并使指针后移(后续字节由 TC 中断接力)
    }
}

/*******************************************************************************
* 功能说明(中文) : 获取最近一帧命令中的"命令类型"字节。
* 参数(中文)     : 无
* 返回(中文)     : RxBuf[3]，即帧中第 4 个字节(命令类型编码，如 0x80 读波形)
* 备注(中文)     : 仅在 UartStatus==ReceiveCmd 之后调用才有意义。
*******************************************************************************/
uint8_t GetCmdType( void )
{
	return RxBuf[3];            // 返回命令类型字节
}
