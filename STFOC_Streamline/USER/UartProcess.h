/*******************************************************************************
* 文件说明(中文):
*   本文件是串口命令处理/波形读取模块的头文件。
*   定义了波形数据的保存表结构 _SaveTab 及保存表数量、每表长度，
*   导出保存表数组 savetab、保存指针 SavePtr、采样数据数组 uartdat，
*   定义命令类型枚举 UART_CMDTYPE，并声明相关处理函数。
*   该模块配合 uart.c 的收发中断，实现"上位机读取电机波形数据"这一功能。
*******************************************************************************/

#ifndef _UARTPROCESS_H_
#define _UARTPROCESS_H_

// 波形通道(保存表)数量: 最多同时保存/上传 16 路数据
#define SaveTabNum		(16)
// 每个通道保存的采样点数(每条波形的长度)
#define TabSize			(500)

// 波形保存表: 一个通道一条记录
typedef struct
{
	uint16_t Tab[TabSize];      // 采样数据缓冲区，保存该通道最近 TabSize 个采样值(16 位无符号)
	uint16_t *pTab;             // 指向实时数据源(即 uartdat 中某一项)，SaveForms() 据此采样
} _SaveTab;

extern _SaveTab savetab[SaveTabNum];    // 全局波形保存表数组，共 SaveTabNum 条

extern uint16_t SavePtr;                // 当前采样写入位置(0~TabSize)，达到 TabSize 表示一圈采满

extern uint16_t uartdat[16];            // 待观测的实时数据数组(16 个通道)，各通道由 pTab 指向此处



typedef enum
{
	CmdType_ReadWaveforms = 0x80,   // 命令: 读取波形数据(上位机请求上传各通道采样波形)
}UART_CMDTYPE;





extern uint16_t ReadAngWaveformsStatus;     // 声明(注: 与 UartProcess.c 中的 ReadWaveformsStatus 命名不一致，实际未使用)

void ReadWaveformsProcess( void );          // 波形读取状态机: 组织/分包上传各通道采样数据
void UartProcess( void );                   // 串口命令总入口: 在主循环中轮询，解析并分派命令
void SaveForms( void );                     // 采样一次: 把各通道实时数据存入保存表当前列

#endif
