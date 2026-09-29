/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Clarke_Park.c 
* Author             : IMS Systems Lab  
* Date First Issued  : 21/11/07
* Description        : This module implements the reference frame transformations
*                      needed for vector control: Clarke, Park and Reverse Park.
*                      It also performs the voltage circle limitation.
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
* 文件说明(中文):
*   本文件实现 FOC 矢量控制的核心坐标变换与电压圆限制：
*   - Clarke()              :三相定子电流 → 两相静止 αβ 坐标系；
*   - Park()                :静止 αβ → 转子磁链同步旋转 dq 坐标系；
*   - Rev_Park()            :旋转 dq → 静止 αβ(供 SVPWM 使用)；
*   - RevPark_Circle_Limitation() :电压矢量圆限制(限制 dq 电压模值)；
*   - Trig_Functions()      :由 s16 电角度查表求 sin/cos。
*   在 FOC 控制角色中:电流环(FOC_Model，每个 PWM 周期一次)先用 Clarke/Park 得到
*   反馈 Iq、Id 并与参考比较，经 PID 输出 dq 电压，再做圆限制、反 Park 得到 αβ
*   电压，最后送入 SVPWM。本文件被 MC_FOC_Drive.c 调用，依赖 MC_const.h 的
*   SIN_COS_TABLE、divSQRT_3，以及 MC_Globals.h 的全局变量。
*******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "MC_Control_Param.h"
#include "MC_Clarke_Park.h"
#include "MC_const.h" 
#include "MC_Globals.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* 电压圆限制查表(circle_limit_table)与 START_INDEX：
   当 dq 参考电压模值超过 MAX_MODULE 时需要按比例缩小电压矢量以保证不过调制。
   为在定点 MCU 上避免开方运算，采用查表法:
   先用 temp=(Vd^2+Vq^2)/(512*32768) 把模值平方映射为 0~127 的索引，
   再减去 START_INDEX 得到表下标；表中存放的是缩放增益(32768 表示 1.0)。
   用该增益分别乘 Vd、Vq，即把电压矢量等比缩放到半径为 MAX_MODULE 的圆上,
   保持矢量方向(角度)不变。START_INDEX 为该调制比下第 1 个表项对应的索引。
   不同调制比定义长度不同的表，故用条件编译区分。 */
#ifdef MAX_MODULATION_77_PER_CENT
#define START_INDEX     37

static const u16 circle_limit_table[91]=   // 调制比 77% 的电压圆限制增益表(91 项，增益 Q1.15)     
{
    32468,32176,31893,31347,31084,30578,30334,29863,29636,29414,28984,28776,28373,
    28178,27987,27616,27436,27086,26916,26585,26425,26267,25959,25809,25518,25375,
    25098,24962,24829,24569,24442,24194,24072,23953,23719,23604,23381,23271,23056,
    22951,22848,22644,22545,22349,22254,22066,21974,21883,21704,21616,21444,21359,
    21275,21111,21030,20872,20793,20640,20564,20490,20343,20270,20128,20058,19920,
    19852,19785,19653,19587,19459,19396,19271,19209,19148,19028,18969,18852,18795,
    18738,18625,18570,18460,18406,18299,18246,18194,18091,18040,17940,17890,17792
};
#endif


#ifdef MAX_MODULATION_79_PER_CENT
#define START_INDEX     39

static const u16 circle_limit_table[89]=   // 调制比 79% 的电压圆限制增益表(89 项)     
{
    32489,32217,31952,31442,31195,30719,30489,30045,29830,29413,29211,28819,28629,
    28442,28080,27904,27562,27396,27072,26914,26607,26457,26165,26022,25882,25608,
    25475,25214,25086,24836,24715,24476,24359,24130,24019,23908,23692,23586,23378,
    23276,23077,22979,22787,22692,22507,22416,22326,22150,22063,21893,21809,21645,
    21564,21405,21327,21173,21097,21022,20875,20802,20659,20589,20450,20382,20247,
    20181,20051,19986,19923,19797,19735,19613,19553,19434,19375,19260,19202,19090,
    19034,18979,18871,18817,18711,18659,18555,18504,18403,18354,18255
};
#endif

#ifdef MAX_MODULATION_81_PER_CENT
#define START_INDEX     41

static const u16 circle_limit_table[87]=   // 调制比 81% 的电压圆限制增益表(87 项)     
{
    32508,32255,32008,31530,31299,30852,30636,30216,30012,29617,29426,29053,28872,
    28520,28349,28015,27853,27536,27382,27081,26934,26647,26507,26234,26101,25840,
    25712,25462,25340,25101,24984,24755,24643,24422,24315,24103,24000,23796,23696,
    23500,23404,23216,23123,22941,22851,22763,22589,22504,22336,22253,22091,22011,
    21854,21776,21624,21549,21401,21329,21186,21115,20976,20908,20773,20706,20575,
    20511,20383,20320,20196,20135,20015,19955,19838,19780,19666,19609,19498,19443,
    19334,19280,19175,19122,19019,18968,18867,18817,18719
};
#endif

#ifdef MAX_MODULATION_83_PER_CENT
#define START_INDEX     44

static const u16 circle_limit_table[84]=   // 调制比 83% 的电压圆限制增益表(84 项)     
{
    32291,32060,31613,31397,30977,30573,30377,29996,29811,29451,29276,28934,28768,
    28444,28286,27978,27827,27533,27390,27110,26973,26705,26574,26318,26069,25948,
    25709,25592,25363,25251,25031,24923,24711,24607,24404,24304,24107,24011,23821,
    23728,23545,23456,23279,23192,23021,22854,22772,22610,22530,22374,22297,22145,
    22070,21922,21850,21707,21636,21497,21429,21294,21227,21096,21032,20904,20778,
    20717,20595,20534,20416,20357,20241,20184,20071,20015,19905,19851,19743,19690,
    19585,19533,19431,19380,19280,19182
};
#endif


#ifdef MAX_MODULATION_85_PER_CENT
#define START_INDEX     46

static const u16 circle_limit_table[82]=   // 调制比 85% 的电压圆限制增益表(82 项)     
{
    32324,32109,31691,31489,31094,30715,30530,30170,29995,29654,29488,29163,29005,
    28696,28397,28250,27965,27825,27552,27418,27157,26903,26779,26535,26416,26182,
    26067,25842,25623,25515,25304,25201,24997,24897,24701,24605,24415,24230,24139,
    23960,23872,23699,23614,23446,23282,23201,23042,22964,22810,22734,22584,22437,
    22365,22223,22152,22014,21945,21811,21678,21613,21484,21421,21296,21234,21112,
    21051,20932,20815,20757,20643,20587,20476,20421,20312,20205,20152,20048,19996
        ,19894,19844,19744,19645
};
#endif



#ifdef MAX_MODULATION_87_PER_CENT
#define START_INDEX     48

static const u16 circle_limit_table[81]=   // 调制比 87% 的电压圆限制增益表(81 项)     
{
    32559,32154,31764,31575,31205,31025,30674,30335,30170,29847,
    29689,29381,29083,28937,28652,28375,28239,27974,27844,27589,
    27342,27220,26983,26866,26637,26414,26305,26090,25984,25777,
    25575,25476,25280,25184,24996,24811,24720,24542,24367,24281,
    24112,24028,23864,23703,23624,23468,23391,23240,23091,23018,
    22874,22803,22662,22524,22456,22322,22191,22126,21997,21934,
    21809,21686,21625,21505,21446,21329,21214,21157,21045,20990,
    20880,20772,20719,20613,20561,20458,20356,20306,20207,20158,
    20109
};
#endif

#ifdef MAX_MODULATION_89_PER_CENT
#define START_INDEX     50

static const u16 circle_limit_table[78]=   // 调制比 89% 的电压圆限制增益表(78 项)     
{
    32574,32197,32014,31656,31309,31141,30811,30491,30335,30030,
    29734,29589,29306,29031,28896,28632,28375,28249,28002,27881,
    27644,27412,27299,27076,26858,26751,26541,26336,26235,26037,
    25844,25748,25561,25378,25288,25110,24936,24851,24682,24517,
    24435,24275,24118,24041,23888,23738,23664,23518,23447,23305,
    23166,23097,22962,22828,22763,22633,22505,22442,22318,22196,
    22135,22016,21898,21840,21726,21613,21557,21447,21338,21284,
    21178,21074,21022,20919,20819,20769,20670,20573
};
#endif

#ifdef MAX_MODULATION_91_PER_CENT
#define START_INDEX     52

static const u16 circle_limit_table[76]=   // 调制比 91% 的电压圆限制增益表(76 项)     
{
    32588,32411,32066,31732,31569,31250,30940,30789,30492,30205,
    29925,29788,29519,29258,29130,28879,28634,28395,28278,28048,
    27823,27713,27497,27285,27181,26977,26777,26581,26485,26296,
    26110,26019,25840,25664,25492,25407,25239,25076,24995,24835,
    24679,24602,24450,24301,24155,24082,23940,23800,23731,23594,
    23460,23328,23263,23135,23008,22946,22822,22701,22641,22522,
    22406,22291,22234,22122,22011,21956,21848,21741,21636,21584,
    21482,21380,21330,21231,21133,21037
};
#endif


#ifdef MAX_MODULATION_92_PER_CENT
#define START_INDEX     54

const u16 circle_limit_table[74]=   // 调制比 92% 的电压圆限制增益表(74 项)     
{
    32424,32091,31929,31611,31302,31002,30855,30568,30289,30017,
    29884,29622,29368,29243,28998,28759,28526,28412,28187,27968,
    27753,27648,27441,27238,27040,26942,26750,26563,26470,26288,
    26110,25935,25849,25679,25513,25350,25269,25111,24955,24803,
    24727,24579,24433,24361,24219,24079,23942,23874,23740,23609,
    23479,23415,23289,23165,23042,22982,22863,22745,22629,22572,
    22459,22347,22292,22183,22075,21970,21917,21813,21711,21610,
    21561,21462,21365,21268
};
#endif


#ifdef MAX_MODULATION_93_PER_CENT
#define START_INDEX     55

const u16 circle_limit_table[73]=   // 调制比 93% 的电压圆限制增益表(73 项)     
{
    32437,32275,31959,31651,31353,31207,30920,30642,30371,30107,
    29977,29723,29476,29234,29116,28883,28655,28433,28324,28110,
    27900,27695,27594,27395,27201,27011,26917,26733,26552,26375,
    26202,26116,25948,25783,25621,25541,25383,25228,25076,25001,
    24854,24708,24565,24495,24356,24219,24084,24018,23887,23758,
    23631,23506,23444,23322,23202,23083,23025,22909,22795,22683,
    22627,22517,22409,22302,22250,22145,22042,21941,21890,21791,
    21693,21596,21500
};
#endif


#ifdef MAX_MODULATION_94_PER_CENT
#define START_INDEX     56

const u16 circle_limit_table[72]=   // 调制比 94% 的电压圆限制增益表(72 项)     
{
    32607,32293,31988,31691,31546,31261,30984,30714,30451,30322,
    30069,29822,29581,29346,29231,29004,28782,28565,28353,28249,
    28044,27843,27647,27455,27360,27174,26991,26812,26724,26550,
    26380,26213,26049,25968,25808,25652,25498,25347,25272,25125,
    24981,24839,24699,24630,24494,24360,24228,24098,24034,23908,
    23783,23660,23600,23480,23361,23245,23131,23074,22962,22851,
    22742,22635,22582,22477,22373,22271,22170,22120,22021,21924,
    21827,21732
};
#endif


#ifdef MAX_MODULATION_95_PER_CENT
#define START_INDEX     57

const u16 circle_limit_table[71]=   // 调制比 95% 的电压圆限制增益表(71 项)     
{
    32613,32310,32016,31872,31589,31314,31046,30784,30529,30404,
    30158,29919,29684,29456,29343,29122,28906,28695,28488,28285,
    28186,27990,27798,27610,27425,27245,27155,26980,26808,26639,
    26473,26392,26230,26072,25917,25764,25614,25540,25394,25250,
    25109,24970,24901,24766,24633,24501,24372,24245,24182,24058,
    23936,23816,23697,23580,23522,23408,23295,23184,23075,23021,
    22913,22808,22703,22600,22499,22449,22349,22251,22154,22059,
    21964
};
#endif


#ifdef MAX_MODULATION_96_PER_CENT
#define START_INDEX     58

const u16 circle_limit_table[70]=   // 调制比 96% 的电压圆限制增益表(70 项，本项目所选)     
{
    32619,32472,32184,31904,31631,31365,31106,30853,30728,30484,
    30246,30013,29785,29563,29345,29238,29028,28822,28620,28423,
    28229,28134,27946,27762,27582,27405,27231,27061,26977,26811,
    26649,26489,26332,26178,26027,25952,25804,25659,25517,25376,
    25238,25103,25035,24903,24772,24644,24518,24393,24270,24210,
    24090,23972,23855,23741,23627,23516,23461,23352,23244,23138,
    23033,22930,22828,22777,22677,22579,22481,22385,22290,22196
};
#endif


#ifdef MAX_MODULATION_97_PER_CENT
#define START_INDEX     60

const u16 circle_limit_table[68]=   // 调制比 97% 的电压圆限制增益表(68 项)     
{
    32483,32206,31936,31672,31415,31289,31041,30799,30563,30331,
    30105,29884,29668,29456,29352,29147,28947,28750,28557,28369,
    28183,28002,27824,27736,27563,27393,27226,27062,26901,26743,
    26588,26435,26360,26211,26065,25921,25780,25641,25504,25369,
    25236,25171,25041,24913,24788,24664,24542,24422,24303,24186,
    24129,24015,23902,23791,23681,23573,23467,23362,23258,23206,
    23105,23004,22905,22808,22711,22616,22521,22429
};
#endif


#ifdef MAX_MODULATION_98_PER_CENT
#define START_INDEX     61

const u16 circle_limit_table[67]=   // 调制比 98% 的电压圆限制增益表(67 项)     
{
    32494,32360,32096,31839,31587,31342,31102,30868,30639,30415,
    30196,29981,29771,29565,29464,29265,29069,28878,28690,28506,
    28325,28148,27974,27803,27635,27470,27309,27229,27071,26916,
    26764,26614,26467,26322,26180,26039,25901,25766,25632,25500,
    25435,25307,25180,25055,24932,24811,24692,24574,24458,24343,
    24230,24119,24009,23901,23848,23741,23637,23533,23431,23331,
    23231,23133,23036,22941,22846,22753,22661
};
#endif


#ifdef MAX_MODULATION_99_PER_CENT
#define START_INDEX     62

const u16 circle_limit_table[66]=   // 调制比 99% 的电压圆限制增益表(66 项)     
{
    32635,32375,32121,31873,31631,31394,31162,30935,30714,30497,
    30284,30076,29872,29672,29574,29380,29190,29003,28820,28641,
    28464,28291,28122,27955,27791,27630,27471,27316,27163,27012,
    26864,26718,26575,26434,26295,26159,26024,25892,25761,25633,
    25569,25444,25320,25198,25078,24959,24842,24727,24613,24501,
    24391,24281,24174,24067,23963,23859,23757,23656,23556,23458,
    23361,23265,23170,23077,22984,22893
};
#endif


#ifdef MAX_MODULATION_100_PER_CENT
#define START_INDEX     63

const u16 circle_limit_table[65]=   // 调制比 100% 的电压圆限制增益表(65 项)     
{
    32767,32390,32146,31907,31673,31444,31220,31001,30787,30577,30371,
    30169,29971,29777,29587,29400,29217,29037,28861,28687,28517,
    28350,28185,28024,27865,27709,27555,27404,27256,27110,26966,
    26824,26685,26548,26413,26280,26149,26019,25892,25767,25643,
    25521,25401,25283,25166,25051,24937,24825,24715,24606,24498,
    24392,24287,24183,24081,23980,23880,23782,23684,23588,23493,
    23400,23307,23215,23125
};
#endif

/* 象限判断掩码：hindex 为 0~255 的正弦表索引(由电角度换算得到)。
   SIN_MASK=0x0300 用于取出索引的高两位(象限编码)，据此判断角度所在象限:
   U0_90=0x0200、U90_180=0x0300、U180_270=0x0000、U270_360=0x0100。
   需注意这里的象限编码与“0°~360° 常规顺序”不同，是按 hindex=(angle+32768)/64
   的取值区间来划分的。 */
#define SIN_MASK 0x0300
#define U0_90     0x0200
#define U90_180   0x0300
#define U180_270  0x0000
#define U270_360  0x0100

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static Trig_Components Vector_Components;   // 保存最近一次 Trig_Functions 得到的 sin/cos，供反 Park 复用
const s16 hSin_Cos_Table[256] = SIN_COS_TABLE;   // 0~90° 的 256 点正弦表(Q1.15)，由 MC_const.h 的 SIN_COS_TABLE 展开

/*******************************************************************************
* Function Name  : Clarke Transformation
* Description    : This function transforms stator currents qIas and  
*                  qIbs (which are directed along axes each displaced by  
*                  120 degrees) into currents qIalpha and qIbeta in a 
*                  stationary qd reference frame.
*                  qIalpha = qIas
*                  qIbeta = -(2*qIbs+qIas)/sqrt(3)
* Input          : Stat_Curr_a_b
* Output         : Stat_Curr_alfa_beta
* Return         : none.
* 功能说明(中文) : Clarke 变换(三相→两相静止 αβ 坐标系)。把沿互差 120° 三个轴的
*                  定子相电流 Ia、Ib 变换到两相正交静止 αβ 坐标系。
*                  在 FOC 电流环(FOC_Model)中每个 PWM 周期调用一次。
* 参数(中文)     : Curr_Input - 输入定子相电流(s16/Q1.15):
*                  qI_Component1=Ia，qI_Component2=Ib(单位:按量程归一化的 A)。
* 返回(中文)     : 两相静止电流(s16/Q1.15):
*                  qI_Component1=Ialpha=Ia；qI_Component2=Ibeta=-(2*Ib+Ia)/sqrt(3)。
* 备注(中文)     : 定点实现，用 divSQRT_3(1/sqrt(3) 的 Q1.15)相乘后右移 15 位
*                  (除以 32768)实现除以 sqrt(3)。该式基于三相无中线 Ic=-(Ia+Ib)。
*******************************************************************************/

Curr_Components Clarke(Curr_Components Curr_Input)
{
    Curr_Components Curr_Output;
    
    s32 qIa_divSQRT3_tmp;
    s32 qIb_divSQRT3_tmp ;
    
    s16 qIa_divSQRT3;
    s16 qIb_divSQRT3 ;
    
    // qIalpha = qIas
    Curr_Output.qI_Component1= Curr_Input.qI_Component1;   // Ialpha = Ia(α 分量直接取 a 相电流)
    
    qIa_divSQRT3_tmp = divSQRT_3 * Curr_Input.qI_Component1;   // Ia * (1/sqrt(3))，Q1.15 定点乘
    qIa_divSQRT3_tmp /=32768;   // 右移 15 位(除以 32768)反定标，得到 Ia/sqrt(3)   
    
    qIb_divSQRT3_tmp = divSQRT_3 * Curr_Input.qI_Component2;   // Ib * (1/sqrt(3))，Q1.15 定点乘
    qIb_divSQRT3_tmp /=32768;   // 除以 32768 反定标，得到 Ib/sqrt(3)
    
    qIa_divSQRT3=((s16)(qIa_divSQRT3_tmp));			// 截断为 s16:Ia/sqrt(3)
    
    qIb_divSQRT3=((s16)(qIb_divSQRT3_tmp));				// 截断为 s16:Ib/sqrt(3)
    
    //qIbeta = -(2*qIbs+qIas)/sqrt(3)
    Curr_Output.qI_Component2=(-(qIa_divSQRT3)-(qIb_divSQRT3)-(qIb_divSQRT3));   // Ibeta = -(Ia+2*Ib)/sqrt(3)
    
    return(Curr_Output); 
}

/*******************************************************************************
* Function Name  : Park Transformation
* Description    : This function transforms stator currents qIalpha and qIbeta,
*                  which belong to a stationary qd reference frame, to a rotor 
*                  flux synchronous reference frame (properly oriented), so as 
*                  to obtain qIq and qIds.
*                  qId=qIalpha_tmp*sin(theta)+qIbeta_tmp*cos(Theta)
*                  qIq=qIalpha_tmp*cos(Theta)-qIbeta_tmp*sin(Theta)                 
* Input          : Stat_Curr_alfa_beta
* Output         : Stat_Curr_q_d.
* Return         : none.
* 功能说明(中文) : Park 变换(两相静止 αβ → 转子磁链同步旋转 dq 坐标系)。
*                  把静止坐标系电流 Ialpha、Ibeta 旋转到与转子磁链对齐的 dq 坐标系。
*                  在 FOC 电流环(FOC_Model)中每个 PWM 周期调用一次。
* 参数(中文)     : Curr_Input - 静止 αβ 电流(s16/Q1.15)：Component1=Ialpha，Component2=Ibeta。
*                  Theta      - 电角度，s16 定标(0x8000 对应 180°，即整周 65536 对应 360°)。
* 返回(中文)     : dq 旋转坐标系电流(s16/Q1.15):
*                  Component1=Iq=Ialpha*cos - Ibeta*sin；Component2=Id=Ialpha*sin + Ibeta*cos。
* 备注(中文)     : 内部先调用 Trig_Functions 查表得到 sin/cos(存放于全局 Vector_Components)，
*                  乘法结果右移 15 位反定标。本实现约定的 θ 相位与常规 Id/Iq 定义相差 90°。
*******************************************************************************/

Curr_Components Park(Curr_Components Curr_Input, s16 Theta)
{
    Curr_Components Curr_Output;
    s32 qId_tmp_1, qId_tmp_2;
    s32 qIq_tmp_1, qIq_tmp_2;     
    s16 qId_1, qId_2;  
    s16 qIq_1, qIq_2;  
    
    Vector_Components = Trig_Functions(Theta);   // 由电角度查表求 sin/cos，存入全局供本函数与反 Park 使用
    
    //No overflow guaranteed
    qIq_tmp_1 = Curr_Input.qI_Component1 * Vector_Components.hCos;  	// Iq 临时项1 = Ialpha*cos(theta)
    qIq_tmp_1 /= 32768;
    
    //No overflow guaranteed
    qIq_tmp_2 = Curr_Input.qI_Component2 *Vector_Components.hSin;  	// Iq 临时项2 = Ibeta*sin(theta)
    qIq_tmp_2 /= 32768;
    
    qIq_1 = ((s16)(qIq_tmp_1));
    qIq_2 = ((s16)(qIq_tmp_2));
    
    //Iq component in Q1.15 Format 
    Curr_Output.qI_Component1 = ((qIq_1)-(qIq_2));	   // Iq = Ialpha*cos(theta) - Ibeta*sin(theta)
    
    //No overflow guaranteed
    qId_tmp_1 = Curr_Input.qI_Component1 * Vector_Components.hSin;   // Id 临时项1 = Ialpha*sin(theta)
    qId_tmp_1 /= 32768;
    
    //No overflow guaranteed
    qId_tmp_2 = Curr_Input.qI_Component2 * Vector_Components.hCos;   // Id 临时项2 = Ibeta*cos(theta)
    qId_tmp_2 /= 32768;
    
    qId_1 = (s16)(qId_tmp_1);		
    
    qId_2 = (s16)(qId_tmp_2);					
    
    //Id component in Q1.15 Format   
    Curr_Output.qI_Component2 = ((qId_1)+(qId_2));	   // Id = Ialpha*sin(theta) + Ibeta*cos(theta)  
    
    return (Curr_Output);
    
}


/*******************************************************************************
* Function Name  : RevPark_Circle_Limitation
* Description    : Check if
*       Stat_Volt_q_d.qV_Component1^2 + Stat_Volt_q_d.qV_Component2^2 <= 32767^2
*                  Apply limitation if previous condition is not met,
*                  by keeping a constant ratio 
*                  Stat_Volt_q_d.qV_Component1/Stat_Volt_q_d.qV_Component2
* Input          : None
* Output         : None
* Return         : None
* 功能说明(中文) : 电压圆限制(Circle Limitation)。检查全局 dq 参考电压
*                  (Stat_Volt_q_d)Vd^2+Vq^2 是否超过 MAX_MODULE^2；
*                  若超过，则按固定比例等比缩小 Vd、Vq，使电压矢量落在半径为
*                  MAX_MODULE 的圆内，方向(角度)保持不变。电流环每周期调用一次。
* 参数(中文)     : 无(直接读写全局 Stat_Volt_q_d)。
* 返回(中文)     : 无(直接原地修改全局 dq 电压)。
* 备注(中文)     : 用查表 circle_limit_table 近似开方/幅值归一化，避免开方运算；
*                  目的是保证 SVPWM 不过调制。副作用:修改全局 Stat_Volt_q_d。
*******************************************************************************/

void RevPark_Circle_Limitation(void)
{
    s32 temp;
    
    temp = Stat_Volt_q_d.qV_Component1 * Stat_Volt_q_d.qV_Component1 
        + Stat_Volt_q_d.qV_Component2 * Stat_Volt_q_d.qV_Component2;  // min value 0, max value 2*32767*32767   // 计算电压矢量模值平方 Vd^2+Vq^2
    
    if ( temp > (u32)(( MAX_MODULE * MAX_MODULE) ) ) // (Vd^2+Vq^2) > MAX_MODULE^2 ?   // 模值超上限才需限幅
    {
        u16 index;
        
        temp /= (u32)(512*32768);  // min value START_INDEX, max value 127   // 模值平方归一化为查表索引(0~127)
        temp -= START_INDEX ;   // min value 0, max value 127 - START_INDEX   // 减去起始索引得到表下标
        index = circle_limit_table[(u8)temp];   // 查表得到缩放增益(Q1.15，32768 表示 1.0)
        
        temp = (s16)Stat_Volt_q_d.qV_Component1 * (u16)(index);    // Vq * 增益
        Stat_Volt_q_d.qV_Component1 = (s16)(temp/32768);  // 反定标:Vq 等比缩小
        
        temp = (s16)Stat_Volt_q_d.qV_Component2 * (u16)(index);    // Vd * 增益
        Stat_Volt_q_d.qV_Component2 = (s16)(temp/32768);  // 反定标:Vd 等比缩小
    }
    
} 

/*******************************************************************************
* Function Name  : Rev_Park Transformation
* Description    : This function transforms stator voltage qVq and qVd, that 
*                  belong to a rotor flux synchronous rotating frame, to a 
*                 stationary reference frame, so as to obtain qValpha and qVbeta
*                  qValfa=qVq*Cos(theta)+qVd*Sin(theta)
*                  qVbeta=-qVq*Sin(theta)+qVd*Cos(theta)                  
* Input          : Stat_Volt_q_d.
* Output         : Stat_Volt_a_b
* Return         : none.
* 功能说明(中文) : 反 Park 变换(同步旋转 dq → 两相静止 αβ 坐标系)。
*                  把 dq 参考电压变换回静止 αβ 电压，供 SVPWM 生成三相占空比。
*                  在电流环(FOC_Model)中紧跟圆限制之后每个 PWM 周期调用一次。
* 参数(中文)     : Volt_Input - dq 电压(s16/Q1.15):Component1=Vq，Component2=Vd。
* 返回(中文)     : αβ 电压(s16/Q1.15):
*                  Component1=Valpha=Vq*cos+Vd*sin；Component2=Vbeta=-Vq*sin+Vd*cos。
* 备注(中文)     : 复用全局 Vector_Components 中上一次 Trig_Functions 得到的 sin/cos
*                  (调用前须先执行 Park/Trig 更新该全局量)。乘法结果右移 15 位反定标。
*******************************************************************************/

Volt_Components Rev_Park(Volt_Components Volt_Input)
{ 	
    s32 qValpha_tmp1,qValpha_tmp2,qVbeta_tmp1,qVbeta_tmp2;
    s16 qValpha_1,qValpha_2,qVbeta_1,qVbeta_2;
    Volt_Components Volt_Output;
    
    //No overflow guaranteed
    qValpha_tmp1 = Volt_Input.qV_Component1 * Vector_Components.hCos;   // Vq*cos(theta)
    qValpha_tmp1 /= 32768;
    
    qValpha_tmp2 = Volt_Input.qV_Component2 * Vector_Components.hSin;   // Vd*sin(theta)
    qValpha_tmp2 /= 32768;
    
    qValpha_1 = (s16)(qValpha_tmp1);		
    qValpha_2 = (s16)(qValpha_tmp2);			
    
    Volt_Output.qV_Component1 = ((qValpha_1)+(qValpha_2));   // Valpha = Vq*cos(theta) + Vd*sin(theta)
    
    
    qVbeta_tmp1 = Volt_Input.qV_Component1 * Vector_Components.hSin;   // Vq*sin(theta)
    qVbeta_tmp1 /= 32768;
    
    qVbeta_tmp2 = Volt_Input.qV_Component2 * Vector_Components.hCos;   // Vd*cos(theta)
    qVbeta_tmp2 /= 32768;
    
    qVbeta_1 = (s16)(qVbeta_tmp1);				
    qVbeta_2 = (s16)(qVbeta_tmp2);
    
    Volt_Output.qV_Component2 = -(qVbeta_1)+(qVbeta_2);   // Vbeta = -Vq*sin(theta) + Vd*cos(theta)
    
    return(Volt_Output);
}
/*******************************************************************************
* Function Name  : Trig_Functions 
* Description    : This function returns Cosine and Sine functions of the input 
*                  angle
* Input          : angle in s16 format
* Output         : Cosine and Sine in s16 format
* Return         : none.
* 功能说明(中文) : 三角函数查表。输入电角度，返回其 sin/cos 值(Q1.15)。
*                  被 Park() 每个 PWM 周期调用(间接也服务反 Park)。
* 参数(中文)     : hAngle - 电角度，s16 定标:0x8000(=32768)对应 180°，整周 65536 对应 360°。
* 返回(中文)     : Trig_Components:hSin=sin(hAngle)、hCos=cos(hAngle)，均为 s16/Q1.15。
* 备注(中文)     : 将角度映射到 0~255 的表内索引，利用 0~90° 正弦表与象限对称性
*                  查得 sin/cos，避免实时计算三角函数。
*******************************************************************************/
Trig_Components Trig_Functions(s16 hAngle)
{
    u16 hindex;
    Trig_Components Local_Components;
    
    /* 10 bit index computation  */  
    hindex = (u16)(hAngle + 32768);   // 把 s16 电角度平移到 0~65535 的无符号范围
    hindex /= 64;      // 除以 64 得到 0~1023 的索引:低 8 位为表内索引，高 2 位为象限
    
    switch (hindex & SIN_MASK) 
    {
    case U0_90:   // 第一象限(0~90°):sin=表[i]，cos=表[255-i]
        Local_Components.hSin = hSin_Cos_Table[(u8)(hindex)];
        Local_Components.hCos = hSin_Cos_Table[(u8)(0xFF-(u8)(hindex))];
        break;
        
    case U90_180:   // 第二象限(90~180°):sin=表[255-i]，cos=-表[i]
        Local_Components.hSin = hSin_Cos_Table[(u8)(0xFF-(u8)(hindex))];
        Local_Components.hCos = -hSin_Cos_Table[(u8)(hindex)];
        break;
        
    case U180_270:   // 第三象限(180~270°):sin=-表[i]，cos=-表[255-i]
        Local_Components.hSin = -hSin_Cos_Table[(u8)(hindex)];
        Local_Components.hCos = -hSin_Cos_Table[(u8)(0xFF-(u8)(hindex))];
        break;
        
    case U270_360:   // 第四象限(270~360°):sin=-表[255-i]，cos=表[i]
        Local_Components.hSin =  -hSin_Cos_Table[(u8)(0xFF-(u8)(hindex))];
        Local_Components.hCos =  hSin_Cos_Table[(u8)(hindex)]; 
        break;
    default:
        break;
    }
    return (Local_Components);
}
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
