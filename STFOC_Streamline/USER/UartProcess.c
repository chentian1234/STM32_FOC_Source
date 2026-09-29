/*******************************************************************************
* 文件说明(中文):
*   本文件实现"上位机读取电机运行波形"的功能，是串口应用层协议处理的核心。
*   工作流程:
*     1) uart.c 的接收中断把校验通过的一帧命令放入 RxBuf 并置状态 ReceiveCmd；
*     2) 主循环调用 UartProcess() 轮询状态，解析命令并调用本文件各处理函数；
*     3) 若为"读波形"命令(CmdType_ReadWaveforms)，先按命令中的位掩码(WaveformsTypeFlag)
*        把被选中的通道绑定到 savetab 的 pTab 指针，再经状态机分包(每包最多 32 点)
*        把 savetab 中缓存的波形通过 SendData() 上传给上位机。
*     4) SaveForms() 由定时任务周期性调用，把各通道实时数据(uartdat)采样进 savetab。
*   帧格式遵循 uart.c 中描述的"地址+长度+命令+数据+校验和"格式。
*******************************************************************************/

#include "includes.h"


uint8_t  TxBuf[128];            // 发送缓冲区(本模块用于组织回帧)
uint16_t i;                     // 通用循环变量(全局，允许多处复用)
uint8_t CmdType;                // 当前正在处理的命令类型
uint16_t WaveformsTypeFlag;     // 波形通道选择位掩码: 位 i 置 1 表示需要上传第 i 个通道；最高位(0xFFFF)表示"读全部"


_SaveTab savetab[SaveTabNum];   // 波形保存表数组，每个被选通道对应其中一条记录
uint16_t SavePtr;               // 采样写指针: 指向 savetab 当前要写入的列(0~TabSize)

uint16_t uartdat[16];           // 16 路实时观测数据(各通道的当前采样值)，被 pTab 指向




// 可观测的波形通道总数(对应 uartdat 的 16 个元素)
#define WaveformsTypeCount		(16)
// "读取全部通道"标志: 命令数据高 16 位为 0xFFFF 时表示选中所有通道
#define ReadAllFlag		((uint16_t)0xffff)


// 通道号 -> 实时数据源地址 的映射表: 第 i 项指向 uartdat[i]，供 savetab[i].pTab 绑定
uint16_t *pSaveMemoryAddressTab[WaveformsTypeCount]=
{
	&uartdat[0],
	&uartdat[1],
	&uartdat[2],
	&uartdat[3],
    
	&uartdat[4],
	&uartdat[5],
	&uartdat[6],
	&uartdat[7],
    
    &uartdat[8],
	&uartdat[9],
	&uartdat[10],
	&uartdat[11],
    
    &uartdat[12],
	&uartdat[13],
	&uartdat[14],
	&uartdat[15],
};


/*******************************************************************************
* 功能说明(中文) : 组织并发送一条"设置参数应答"帧(5 字节)。
* 参数(中文)     : 无(使用全局 CmdType 作为命令类型，TxBuf 作为发送缓冲)
* 返回(中文)     : 无
* 备注(中文)     : 帧头互换(从机地址在前)、命令回显、末字节为累加校验和。
*******************************************************************************/
void ReplySettings( void )
{
	TxBuf[0] = SLAVE_ADRESS;        // 本机(从机)地址
	TxBuf[1] = MASTER_ADDRESS;      // 主机(上位机)地址
	TxBuf[2] = 5 + 0;               // 整帧长度 = 5 字节(无附加数据)
	TxBuf[3] = CmdType;             // 回显命令类型
	TxBuf[TxBuf[2]-1] = 0;          // 末字节(校验和)先清零
	for( i=0; i<TxBuf[2]-1; i++ )   // 对前 N-1 个字节求累加和
		TxBuf[TxBuf[2]-1] += TxBuf[i];

	SendData(TxBuf,TxBuf[2]);       // 发送整帧
}



/*******************************************************************************
* 功能说明(中文) : 解析"读波形"命令的数据域，得到需要上传的通道位掩码。
* 参数(中文)     : 无(命令数据来自全局 RxBuf)
* 返回(中文)     : 无
* 备注(中文)     : 帧第 5、6 字节(下标 4、5)为大端表示的 16 位通道掩码，
*                  与 0xFFFF 相与后非 0 表示"读取全部通道"。
*******************************************************************************/
void ReadWaveforms( void )
{
	WaveformsTypeFlag = RxBuf[4] * 256 + RxBuf[5];  // 高字节*256 + 低字节 = 16 位通道选择掩码
}

// 读波形流程状态机枚举
enum
{
	ReadWaveformsStatusIdle=0,          // 空闲: 等待新的读波形命令
	ReadWaveformsStatusReceivedCmd,     // 已收到命令: 正在按掩码绑定通道指针
	ReadWaveformsStatusSavingDat,       // 采样等待: 等待 SaveForms 采满 TabSize 个点
	ReadWaveformsStatusSendingDat,      // 发送数据: 分包把缓存波形上传给上位机

};

uint16_t ReadWaveformsStatus=ReadWaveformsStatusIdle;   // 读波形状态机当前状态，初值为空闲
/*******************************************************************************
* 功能说明(中文) : 读波形流程状态机，由 UartProcess() 在主循环中周期调用。
*                  四个阶段: 空闲->收到命令(绑定通道)->等待采满->分包发送。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 使用静态变量跨调用保存分包发送进度；
*                  只有 WaveformsTypeFlag 仍带 ReadAllFlag 时才执行上传动作，
*                  上传完成后清掉对应通道位，全部发完则把状态复位为空闲。
*******************************************************************************/
void ReadWaveformsProcess( void )
{
	static uint16_t SendPtr=0;              // 发送进度: 当前包在整条波形中的起始列
	static uint16_t SendCount=0;            // 当前正在发送的通道在 savetab 中的下标
	static uint16_t SendWaveformsType=0;    // 当前正在发送的通道对应的位掩码
	uint16_t i;                             // 局部循环变量
	uint16_t WaveformsCount;                // 已绑定的通道计数

	if( ReadWaveformsStatus == ReadWaveformsStatusIdle )     // 空闲态: 判断是否有新命令待处理
	{
		if( (WaveformsTypeFlag & ReadAllFlag) != 0 )         // 若带"读取全部"标志
			ReadWaveformsStatus = ReadWaveformsStatusReceivedCmd;   // 进入"已收到命令"态
		else
			WaveformsTypeFlag &= ~ReadAllFlag;               // 否则清除该标志
	}


	if( ReadWaveformsStatus == ReadWaveformsStatusReceivedCmd )  // 已收到命令: 按掩码绑定通道
	{
		WaveformsCount = 0;                             // 已绑定通道数清零

		for( i=0; i<WaveformsTypeCount && WaveformsCount < SaveTabNum; i++ )   // 遍历 16 个通道
		{
			if( WaveformsTypeFlag & (0x0001 <<i) )      // 该通道被选中?
			{
				savetab[WaveformsCount].pTab = pSaveMemoryAddressTab[i];   // 把保存表数据源指向该通道
				WaveformsCount++;                       // 已绑定通道数加 1
			}
		}

		ReadWaveformsStatus = ReadWaveformsStatusSavingDat;   // 进入"等待采样"态
		SavePtr = 0;                                          // 采样写指针归零
	}

	if( ReadWaveformsStatus == ReadWaveformsStatusSavingDat )     // 等待采样态: 采满一圈才发送
	{
		if( SavePtr >= TabSize )                        // 已采集满 TabSize 个点?
		{
			ReadWaveformsStatus = ReadWaveformsStatusSendingDat;  // 进入"发送数据"态
			SendPtr = 0;                                     // 发送进度归零
			SendCount = 0;                                   // 从第 0 个通道开始
		}
	}
	if( ReadWaveformsStatus == ReadWaveformsStatusSendingDat )    // 发送数据态: 分包上传波形
	{
		if( (WaveformsTypeFlag & ReadAllFlag) != 0 )    // 仍处于"读所有通道"模式?
		{
			for( i=0;i<WaveformsTypeCount;i++)          // 找到当前仍需发送的最低编号通道
			{
				if( WaveformsTypeFlag & (0x0001 << i) )
				{
					SendWaveformsType = 0x0001 << i;    // 记录其位掩码
					break;                              // 找到即退出
				}
			}

			TxBuf[0] = SLAVE_ADRESS;                    // 本机(从机)地址
			TxBuf[1] = MASTER_ADDRESS;                  // 主机地址
			TxBuf[2] = 5 + 4;                           // 帧长初值: 5(固定头) + 4(通道号2+起始点2)
			TxBuf[3] = CmdType;                         // 命令类型回显

			TxBuf[4] = SendWaveformsType>>8;            // 通道掩码高字节
			TxBuf[5] = SendWaveformsType;               // 通道掩码低字节

			TxBuf[6] = SendPtr>>8;                      // 本包起始点高字节
			TxBuf[7] = SendPtr;                         // 本包起始点低字节

			for( i=0; (i<32) && (SendPtr<TabSize); i++ )   // 每包最多打包 32 个 16 位采样点
			{
				TxBuf[i*2 + 8] =savetab[SendCount].Tab[SendPtr] >> 8;   // 采样值高字节
				TxBuf[i*2 + 9] = savetab[SendCount].Tab[SendPtr];       // 采样值低字节
				SendPtr ++;                             // 发送进度 +1
				TxBuf[2] += 2;                          // 每加 1 个 16 位点，帧长 +2
			}
			TxBuf[TxBuf[2]-1] = 0;                      // 末字节(校验和)清零
			for( i=0; i<TxBuf[2]-1; i++ )               // 计算累加校验和
				TxBuf[TxBuf[2]-1] += TxBuf[i];

			SendData(TxBuf,TxBuf[2]);                   // 发送本包

			if( SendPtr >= TabSize )                    // 本通道已发完?
			{
				SendPtr = 0;                            // 进度归零，准备下一个通道
				if( SendCount < SaveTabNum-1 )          // 还有下一个已绑定通道?
					SendCount++;                        // 切到下一通道
				else
					WaveformsTypeFlag = 0;              // 所有通道发完，清除全部选择位
				WaveformsTypeFlag &= ~SendWaveformsType;   // 清除本通道选择位
			}
		}
		else
		{
			ReadWaveformsStatus = ReadWaveformsStatusIdle;   // 无待发通道，回到空闲态
		}
	}
}


/*******************************************************************************
* 功能说明(中文) : 采样一次: 把所有通道的实时数据写入 savetab 的当前列，然后
*                  把采样写指针 SavePtr 后移一位。应由定时/周期任务调用，
*                  采样周期决定波形的时基(每调用一次记一个时间点)。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 当 SavePtr 达到 TabSize 后不再写入，等待读波形流程处理。
*******************************************************************************/
void SaveForms( void )
{
	uint16_t i;                                 // 通道循环变量
	if( SavePtr < TabSize )                     // 缓冲区未写满时才采样
	{
		for( i=0; i<SaveTabNum; i++ )           // 遍历所有保存通道
		{
			savetab[i].Tab[SavePtr] = *savetab[i].pTab;   // 把该通道实时值存入当前列
		}
		SavePtr ++;                             // 采样位置后移
	}
}

/*******************************************************************************
* 功能说明(中文) : 串口应用层处理总入口，在主循环中反复轮询调用。
*                  空闲时推进读波形状态机；收到完整命令帧时解析命令类型并分派，
*                  处理完把串口状态复位为空闲。
* 参数(中文)     : 无
* 返回(中文)     : 无
* 备注(中文)     : 依赖 uart.c 的中断把完整帧放入 RxBuf 并置状态 ReceiveCmd。
*******************************************************************************/
void UartProcess( void )
{
	if( GetUartStatus() == Idle )                       // 串口空闲: 推进读波形状态机
	{
		ReadWaveformsProcess();
	}
	else if( GetUartStatus() == ReceiveCmd )            // 已收到一帧完整命令
	{
		CmdType = GetCmdType();                         // 取命令类型字节

		switch( CmdType )                               // 按命令类型分派处理
		{
		case CmdType_ReadWaveforms:                     // 命令: 读取波形
			ReadWaveforms();                            // 解析通道掩码
			break;

		default:                                        // 其它命令: 暂不处理
			break;
		}


		SetUartStatus(Idle);                            // 命令处理完毕，状态复位为空闲
	}

}

