
#define 	MAIN_Fosc		25600000UL	//定义主时钟

#include	"AI8051U.h"


/*************	功能说明	**************

请先别修改程序, 直接下载"SPI.hex"测试, 下载时选择主频24MHz或输入25.6MHz. 实测20~30MHz都能正常驱动。

使用SPI-MOSI输出直接驱动WS2812三基色彩灯, DMA传输，不占用CPU时间。本例驱动32个灯，接成环状。
本例使用USART1-SPI模式的P1.5-MOSI输出驱动信号直接驱动WS2812。
由于使用SPI主机模式，则P1.6、P1.7被SPI占用，不能做他用，但可以将P1.7-SCLK设置为高阻作为输入使用。
MISO-P1.6设置为高阻，并允许下拉电阻，这样SPI发送完成后MOSI是低电平。
DMA发送时间：@25.6MHz, DMA发送一个字节耗时67T(2.6172us)，一个灯要发送12个字节耗时31.5us，32K字节xdata最多一次驱动2730个灯.

每个灯3个字节，分别对应绿、红、蓝则，MSB先发.
800KHz码率, 数据0(1/4占空比): H=0.3125us  L=0.9375us, 数据1(3/4占空比): H=0.9375us  L=0.3125us, RESET>=50us.
高电平时间要精确控制在要求的范围内, 低电平时间不需要精确控制, 大于要求的最小值并小于RES的50us即可.

WS2812S的标准时序如下:
TH+TL = 1.25us±150ns, RES>50us
T0H = 0.25us±150ns = 0.10us - 0.40us
T0L = 1.00us±150ns = 0.85us - 1.15us
T1H = 1.00us±150ns = 0.85us - 1.15us
T1L = 0.25us±150ns = 0.10us - 0.40us
两个位数据之间的间隔要小于RES的50us.

SPI方案:
本例使用SPI模式的P1.5-MOSI输出驱动信号直接驱动WS2812。
由于使用SPI主机模式，则P1.6、P1.7被SPI占用，不能做他用，但可以将P1.7-SCLK设置为高阻作为输入使用。

用SPI传输, 速度3.0~3.5MHz，以3.2MHz为最佳, MSB先发, 每个字节高4位和低4位分别对应一个位数据, 1000为数据0, 1110为数据1.
SPI数据位       D7 D6 D5 D4    D3 D2 D1 D0
SPI数据         1  0  0  0     1  1  1  0
               WS2812数据0    WS2812数据1
SPI数据高半字节对应的WS2812数据0-->0x80, 数据1-->0xe0,
SPI数据低半字节对应的WS2812数据0-->0x08, 数据1-->0x0e,
主频25.6MHz, SPI分频8 = 3.2MHz. 最佳.

******************************************/

/*************	本地常量声明	**************/


/*************	本地变量声明	**************/


/*************	本地函数声明	**************/

#define	COLOR	50				//亮度，最大255

#define	LED_NUM	32				//LED灯个数
#define	SPI_NUM	(LED_NUM*12)	//LED灯对应SPI字节数

u8	xdata  led_RGB[LED_NUM][3];	//LED对应的RGB，led_buff[i][0]-->绿，led_buff[i][1]-->红，led_buff[i][0]-->蓝.
u8	xdata  led_SPI[SPI_NUM];	//LED灯对应SPI字节数

bit	B_SPI_DMA_busy;	//SPI-DMA忙标志

/*************  外部函数和变量声明 *****************/
void  	SPI_Config(u8 SPI_io, u8 SPI_speed);	//SPI初始化函数, 参数:  SPI_io: 切换到的IO, 0: 切换到P5.4 P1.3 P1.4 P1.5,  1: 切换到P2.2 P2.3 P2.4 P2.5, 2: 切换到P5.4 P4.0 P4.1 P4.3,  3: 切换到P3.5 P3.4 P3.3 P3.2,
												//						SPI_speed: SPI的速度, 0: fosc/4,  1: fosc/8,  2: fosc/16,  3: fosc/2
void	LoadSPI(void);
void	SPI_DMA_TxTRIG(u8 xdata *TxBuf, u16 num);
void  	delay_ms(u16 ms);






/*************** 主函数 *******************************/

void main(void)
{
	u16	i,k;
	u8	xdata *px;

	EAXFR = 1;	//允许访问扩展寄存器
	WTST  = 0;
	CKCON = 0;
//	MCLKO47_DIV(100);	//主频分频输出，用于测量主频

	P1M0=0; P1M1 = 0;  //这里实际上SPI_Config里面已经配准双向了

	SPI_Config(0, 1);	//(SPI_io, SPI_speed), 参数: 	SPI_io: 切换IO(SS MOSI MISO SCLK), 0: 切换到P1.4 P1.5 P1.6 P1.7,  1: 切换到P2.4 P2.5 P2.6 P2.7, 2: 切换到P4.0 P4.1 P4.2 P4.3,  3: 切换到P3.5 P3.4 P3.3 P3.2,
						//								SPI_speed: SPI的速度, 0: fosc/4,  1: fosc/8,  2: fosc/16,  3: fosc/2
						//这里项目用的P2.5的MOSI，所以应该用SPI_Config(1, 1);

	/** 下面的代码应该挪到SPI_Config里面 */
	P1n_pure_input(Pin6);			// MISO-P1.6设置为高阻，并允许下拉电阻，这样SPI发送完成后MOSI是低电平。
	PullUpDisable(P1PU, Pin6);		// 禁止端口内部上拉电阻   PxPU, 要设置的端口对应位为1
	PullDownEnable(P1PD, Pin6);		// 允许端口内部下拉电阻   PxPD, 要设置的端口对应位为1
	/** 上面的代码应该挪到SPI_Config里面 */
	EA = 1;

	k = 0;		//

	while (1)
	{
		while(B_SPI_DMA_busy) ;		//等待DMA完成

		px = &led_RGB[0][0];	//亮度(颜色)首地址
		for(i=0; i<(LED_NUM*3); i++, px++)	*px = 0;	//清除所有的颜色

		i = k;
		led_RGB[i][1] = COLOR;		//红色
		if(++i >= LED_NUM)	i = 0;	//下一个灯
		led_RGB[i][0] = COLOR;		//绿色
		if(++i >= LED_NUM)	i = 0;	//下一个灯
		led_RGB[i][2] = COLOR;		//蓝色

		LoadSPI();	//将颜色装载到SPI数据
		SPI_DMA_TxTRIG(led_SPI, SPI_NUM);	//u8 xdata *TxBuf, u16 num), 启动SPI DMA, 720字节一共耗时2.08ms @25.6MHz

		if(++k >= LED_NUM)	k = 0;			//顺时针
	//	if(--k >= LED_NUM)	k = LED_NUM-1;	//逆时针
		delay_ms(50);
	}
}

//================ N ms延时函数 ==============
void  delay_ms(u16 ms)
{
	u16 i;
	do
	{
		i = MAIN_Fosc / 6000;
		while(--i)	;
	}while(--ms);
}


//================ 将颜色装载到SPI数据 ==============
void	LoadSPI(void)
{
	u8	xdata *px;
	u16	i,j;
	u8	k;
	u8	dat;

	for(i=0; i<SPI_NUM; i++)	led_SPI[i] = 0;
	px = &led_RGB[0][0];	//首地址
	for(i=0, j=0; i<(LED_NUM*3); i++)
	{
		dat = *px;
		px++;
		for(k=0; k<4; k++)
		{
			if(dat & 0x80)	led_SPI[j]  = 0xE0;	//数据1
			else			led_SPI[j]  = 0x80;	//数据0
			if(dat & 0x40)	led_SPI[j] |= 0x0E;	//数据1
			else			led_SPI[j] |= 0x08;	//数据0
			dat <<= 2;
			j++;
		}
	}
}



//========================================================================
// 函数: void  SPI_Config(u8 io, u8 speed)
// 描述: SPI初始化函数。
// 参数: io: 切换到的IO,            SS  MOSI MISO SCLK
//                       0: 切换到 P1.4 P1.5 P1.6 P1.7
//                       1: 切换到 P2.4 P2.5 P2.6 P2.7
//                       2: 切换到 P4.0 P4.1 P4.2 P4.3
//                       3: 切换到 P3.5 P3.4 P3.3 P3.2
//       SPI_speed: SPI的速度, 0: fosc/4,  1: fosc/8,  2: fosc/16,  3: fosc/2
// 返回: none.
// 版本: VER1.0
// 日期: 2018-6-13
// 备注:
//========================================================================

void  SPI_Config(u8 SPI_io, u8 SPI_speed)
{
	SPCTL = SPI_speed & 3;	//配置SPI 速度, 这条指令放在第一位, Bit7~Bit2清0
	SSIG = 1;	//1: 忽略SS脚，由MSTR位决定主机还是从机		0: SS脚用于决定主机还是从机。
	SPEN = 1;	//1: 允许SPI，								0：禁止SPI，所有SPI管脚均为普通IO
	DORD = 0;	//1：LSB先发，								0：MSB先发
	MSTR = 1;	//1：设为主机								0：设为从机
	CPOL = 0;	//1: 空闲时SCLK为高电平，					0：空闲时SCLK为低电平
	CPHA = 0;	//1: 数据在SCLK前沿驱动,后沿采样.			0: 数据在SCLK前沿采样,后沿驱动.
	P_SW1 = (P_SW1 & ~0x0c) | ((SPI_io<<2) & 0x0c);		//切换IO

	HSCLKDIV   = 1;		//HSCLKDIV主时钟分频
	SPI_CLKDIV = 1;		//SPI_CLKDIV主时钟分频
	if(SPI_io == 0)
	{
		P1n_standard(0xf0);			//切换到 P1.4(SS) P1.5(MOSI) P1.6(MISO) P1.7(SCLK), 设置为准双向口
		PullUpEnable(P1PU, 0xf0);	//设置上拉电阻    允许端口内部上拉电阻   PxPU, 要设置的端口对应位为1
		P1n_push_pull(Pin7+Pin5);	//MOSI SCLK设置为推挽输出
		SlewRateHigh(P1SR, Pin7+Pin5);	//MOSI SCLK端口输出设置为高速模式   PxSR, 要设置的端口对应位为1.    高速模式在3.3V供电时速度可以到13.5MHz(27MHz主频，SPI速度2分频)
	}
	else if(SPI_io == 1)
	{
		P2n_standard(0xf0);			//切换到P2.4(SS) P2.5(MOSI) P2.6(MISO) P2.7(SCLK), 设置为准双向口
		PullUpEnable(P2PU, 0xf0);	//设置上拉电阻    允许端口内部上拉电阻   PxPU, 要设置的端口对应位为1
		P2n_push_pull(Pin7+Pin5);	//MOSI SCLK设置为推挽输出
		SlewRateHigh(P2SR, Pin7+Pin5);	//MOSI SCLK端口输出设置为高速模式   PxSR, 要设置的端口对应位为1.    高速模式在3.3V供电时速度可以到13.5MHz(27MHz主频，SPI速度2分频)
	}
	else if(SPI_io == 2)
	{
		P4n_standard(0x0f);			//切换到P4.0(SS) P4.1(MOSI) P4.2(MISO) P4.3(SCLK), 设置为准双向口
		PullUpEnable(P4PU, 0x0f);	//设置上拉电阻    允许端口内部上拉电阻   PxPU, 要设置的端口对应位为1
		P4n_push_pull(Pin3+Pin1);	//MOSI SCLK设置为推挽输出
		SlewRateHigh(P4SR, Pin3+Pin1);	//MOSI SCLK端口输出设置为高速模式   PxSR, 要设置的端口对应位为1.    高速模式在3.3V供电时速度可以到13.5MHz(27MHz主频，SPI速度2分频)
	}
	else if(SPI_io == 3)
	{
		P3n_standard(0x3C);		//切换到P3.5(SS) P3.4(MOSI) P3.3(MISO) P3.2(SCLK), 设置为准双向口
		PullUpEnable(P3PU, 0x3c);	//设置上拉电阻    允许端口内部上拉电阻   PxPU, 要设置的端口对应位为1
		P3n_push_pull(Pin4+Pin2);	//MOSI SCLK设置为推挽输出
		SlewRateHigh(P3SR, Pin4+Pin2);	//MOSI SCLK端口输出设置为高速模式   PxSR, 要设置的端口对应位为1.    高速模式在3.3V供电时速度可以到13.5MHz(27MHz主频，SPI速度2分频)
	}
}


//DMA_SPI_CR 	SPI_DMA控制寄存器
#define		DMA_ENSPI		(1<<7)	// SPI DMA功能使能控制位，    bit7, 0:禁止SPI DMA功能，  1：允许SPI DMA功能。
#define		SPI_TRIG_M		(1<<6)	// SPI DMA主机模式触发控制位，bit6, 0:写0无效，          1：写1开始SPI DMA主机模式操作。
#define		SPI_TRIG_S		(1<<5)	// SPI DMA从机模式触发控制位，bit5, 0:写0无效，          1：写1开始SPI DMA从机模式操作。
#define		SPI_CLRFIFO			1	// 清除SPI DMA接收FIFO控制位，bit0, 0:写0无效，          1：写1复位FIFO指针。


//DMA_SPI_CFG 	SPI_DMA配置寄存器
#define		DMA_SPIIE	(1<<7)	// SPI DMA中断使能控制位，bit7, 0:禁止SPI DMA中断，     1：允许中断。
#define		SPI_ACT_TX	(1<<6)	// SPI DMA发送数据控制位，bit6, 0:禁止SPI DMA发送数据，主机只发时钟不发数据，从机也不发. 1：允许发送。
#define		SPI_ACT_RX	(1<<5)	// SPI DMA接收数据控制位，bit5, 0:禁止SPI DMA接收数据，主机只发时钟不收数据，从机也不收. 1：允许接收。
#define		DMA_SPIIP	(0<<2)	// SPI DMA中断优先级控制位，bit3~bit2, (最低)0~3(最高).
#define		DMA_SPIPTY		0	// SPI DMA数据总线访问优先级控制位，bit1~bit0, (最低)0~3(最高).

//DMA_SPI_CFG2 	SPI_DMA配置寄存器2
#define		SPI_WRPSS	(0<<2)	// SPI DMA过程中使能SS脚控制位，bit2, 0: SPI DMA传输过程不自动控制SS脚。  1：自动拉低SS脚。
#define		SPI_SSS	    	0	// SPI DMA过程中自动控制SS脚选择位，bit1~bit0, 0: P1.4,  1：P2.4,  2: P4.0,  3:P3.5。

//DMA_SPI_STA 	SPI_DMA状态寄存器
#define		SPI_TXOVW	(1<<2)	// SPI DMA数据覆盖标志位，bit2, 软件清0.
#define		SPI_RXLOSS	(1<<1)	// SPI DMA接收数据丢弃标志位，bit1, 软件清0.
#define		DMA_SPIIF		1	// SPI DMA中断请求标志位，bit0, 软件清0.

//HSSPI_CFG  高速SPI配置寄存器
#define		SS_HOLD		(0<<4)	//高速模式时SS控制信号的HOLD时间， 0~15, 默认3. 在DMA中会增加N个系统时钟，当SPI速度为系统时钟/2时执行DMA，SS_HOLD、SS_SETUP和SS_DACT都必须设置大于2的值.
#define		SS_SETUP		3	//高速模式时SS控制信号的SETUP时间，0~15, 默认3. 在DMA中不影响时间，       当SPI速度为系统时钟/2时执行DMA，SS_HOLD、SS_SETUP和SS_DACT都必须设置大于2的值.

//HSSPI_CFG2  高速SPI配置寄存器2
#define		SPI_IOSW	(1<<6)	//bit6:交换MOSI和MISO脚位，0：不交换，1：交换
#define		HSSPIEN		(1<<5)	//bit5:高速SPI使能位，0：关闭高速模式，1：使能高速模式
#define		FIFOEN		(1<<4)	//bit4:高速SPI的FIFO模式使能位，0：关闭FIFO模式，1：使能FIFO模式，使能FIFO模式在DMA中减少13个系统时间。
#define		SS_DACT			3	//bit3~0:高速模式时SS控制信号的DEACTIVE时间，0~15, 默认3. 当SPI速度为系统时钟/2时执行DMA，SS_HOLD、SS_SETUP和SS_DACT都必须设置大于2的值.

/*****************************************************************************
 * @name       :void	SPI_DMA_TxTRIG(u8 xdata *TxBuf, u16 num)
 * @date       :2024-1-5
 * @function   :Config SPI DMA
 * @parameters :None
 * @retvalue   :None
******************************************************************************/
void	SPI_DMA_TxTRIG(u8 xdata *TxBuf, u16 num)
{
	u16 j;
	HSSPI_CFG  = SS_HOLD | SS_SETUP;	//SS_HOLD会增加N个系统时钟, SS_SETUP没有增加时钟。
	HSSPI_CFG2 = HSSPIEN | FIFOEN | SS_DACT;	//FIFOEN允许FIFO会减小13个时钟, 67T @8分频.

	j = (u16)TxBuf;		//取首地址
	DMA_SPI_TXAH = (u8)(j >> 8);				//发送地址寄存器高字节
	DMA_SPI_TXAL = (u8)j;						//发送地址寄存器低字节
	DMA_SPI_AMTH = (u8)((num-1)/256);		//设置传输总字节数 = n+1
	DMA_SPI_AMT  = (u8)((num-1)%256);		//设置传输总字节数 = n+1
	DMA_SPI_ITVH = 0;					//增加的间隔时间，N+1个系统时钟
	DMA_SPI_ITVL = 0;
	DMA_SPI_STA  = 0x00;
	DMA_SPI_CFG  = DMA_SPIIE | SPI_ACT_TX | DMA_SPIIP | DMA_SPIPTY;
	DMA_SPI_CFG2 = SPI_WRPSS | SPI_SSS;
	DMA_SPI_CR   = DMA_ENSPI | SPI_TRIG_M | SPI_CLRFIFO;
	B_SPI_DMA_busy = 1;		//SPI-DMA忙标志
}


//========================================================================
// 函数: void SPI_DMA_ISR (void) interrupt DMA_SPI_VECTOR
// 描述:  SPI_DMA中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2024-1-5
//========================================================================
void SPI_DMA_ISR (void) interrupt DMA_SPI_VECTOR
{
	DMA_SPI_STA = 0;		//清除中断标志
	B_SPI_DMA_busy = 0;		//SPI-DMA忙标志
}

