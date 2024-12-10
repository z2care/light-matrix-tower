
#define 	MAIN_Fosc		40000000UL	//定义主时钟

#include	"AI8051U.h"


/*************	功能说明	**************

请先别修改程序, 直接下载"PWM.hex"测试, 下载时选择主频40MHz。

使用PWMA-P1.0输出直接驱动WS2812 三基色彩灯, DMA传输，不占用CPU时间。本例驱动32个灯，接成环状。每个灯有24bit数据，需要24字节的占空比来控制。
鉴于PWM的特点，在发数据前先发一个占空比为0的周期，最后也发一个占空比为0的周期(发送完后连续输出低电平)。

每个灯3个字节，分别对应绿、红、蓝则，MSB先发.
800KHz码率, 数据0(1/4占空比): H=0.3125us  L=0.9375us, 数据1(3/4占空比): H=0.9375us  L=0.3125us, RESET>=50us.
高电平时间要精确控制在要求的范围内, 低电平时间不需要精确控制, 大于要求的最小值并小于RES的50us即可.
DMA发送时间：@40MHz, 发送字节=24*24+1=577, DMA发送时间720us，32K xdata最多一次驱动1365个灯.

WS2812S的标准时序如下:
TH+TL = 1.25us±150ns, RES>50us
T0H = 0.25us±150ns = 0.10us - 0.40us
T0L = 1.00us±150ns = 0.85us - 1.15us
T1H = 1.00us±150ns = 0.85us - 1.15us
T1L = 0.25us±150ns = 0.10us - 0.40us
两个位数据之间的间隔要小于RES的50us.

PWMA-P1.0方案:
本例使用PWMA-P1.0输出直接驱动WS2812。
用DMA传输, 一个周期传输一个bit数据, 一个周期为1.25us，数据0的占空比为1/4，数据1的占空比为3/4。
本例使用40MHz时钟，周期50T=1.25us，数据1占空比为37T，数据0占空比为12T。

******************************************/

/*************	本地常量声明	**************/


/*************	本地变量声明	**************/


/*************	本地函数声明	**************/

#define	COLOR	50				//亮度，最大255

#define	LED_NUM	24				//LED灯个数
#define	PWM_NUM	(LED_NUM*24+2)	//LED灯对应PWM字节数, 多发2个周期，第一个占空比为0(避免异步装载DMA丢失第一个数据)，最后一个占空比为0，连续输出0.

u8	xdata  led_RGB[LED_NUM][3];	//LED对应的RGB，led_buff[i][0]-->绿，led_buff[i][1]-->红，led_buff[i][2]-->蓝.
u8	xdata  led_pwm[PWM_NUM];	//LED灯对应PWM字节数

bit	B_PWMAT_DMA_busy;	//PWMAT-DMA忙标志


/*************  外部函数和变量声明 *****************/
void	LoadPWM(void);
void 	PWMA_config(void);
void	PWMAT_DMA_TRIG(u8 xdata *TxBuf, u16 num);
void	delay_ms(u16 ms);



/*************** 主函数 *******************************/

void main(void)
{
	u16	i,k;
	u8	xdata *px;

	EAXFR = 1;	//允许访问扩展寄存器
	WTST  = 0;
	CKCON = 0;

	P1M1 = 0;	P1M0 = 0;
	P3M1 = 0;	P3M0 = 0;

	PWMA_config();
	EA = 1;
//	MCLKO47_DIV(100);	//主频分频输出，用于测量主频

	k = 0;		//
	B_PWMAT_DMA_busy = 0;

	while (1)
	{
		while(B_PWMAT_DMA_busy)	;	//等待DMA完成

		px = &led_RGB[0][0];	//亮度(颜色)首地址
		for(i=0; i<(LED_NUM*3); i++, px++)	*px = 0;	//清除所有的颜色

		i = k;
		led_RGB[i][1] = COLOR;		//红色
		if(++i >= LED_NUM)	i = 0;	//下一个灯
		led_RGB[i][0] = COLOR;		//绿色
		if(++i >= LED_NUM)	i = 0;	//下一个灯
		led_RGB[i][2] = COLOR;		//蓝色

		LoadPWM();			//将颜色装载到PWM数据
		PWMAT_DMA_TRIG(led_pwm, PWM_NUM);	//(u16 xdata *TxBuf, u16 num)	触发SPWMA DMA，num为要传输的字节数

		if(++k >= LED_NUM)	k = 0;			//顺时针
	//	if(--k >= LED_NUM)	k = LED_NUM-1;	//逆时针
		delay_ms(50);
	}
}


//================ 延时函数 ==============
void  delay_ms(u16 ms)
{
	u16 i;
	do
	{
		i = MAIN_Fosc / 6000;
		while(--i)	;
	}while(--ms);
}



//================ 将颜色装载到PWM数据 ==============
void	LoadPWM(void)
{
	u8	xdata *px;
	u16	i,j;
	u8	k;
	u8	dat;

	for(i=0; i<(PWM_NUM); i++)	led_pwm[i] = 0;
	px = &led_RGB[0][0];	//首地址
	for(i=0, j=1; i<(LED_NUM*3); i++)
	{
		dat = *px;
		px++;
		for(k=0; k<8; k++)
		{
			if(dat & 0x80)	led_pwm[j]  = 37;	//数据1
			else			led_pwm[j]  = 12;	//数据0
			dat <<= 1;
			j++;
		}
	}
}


//========================================================================
// 函数: void PWMA_config(void)
// 描述: PWM配置函数。
// 参数: noe.
// 返回: none.
// 版本: V1.0, 2022-3-15
// 备注:
//========================================================================
void PWMA_config(void)
{
	u8	ccer1;         // 捕获/比较使能寄存器1
	u8	ccer2;         // 捕获/比较使能寄存器2
	u8	ps;            // 选择IO, 0:选择P1.0 P1.1, 1:选择P0.0 P0.1, 2:选择P2.0 P2.1,
	u8	eno;           // IO输出允许,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P

	PWMA_ENO    = 0;	// IO输出禁止
	PWMA_IER    = 0;	// 禁止中断
	PWMA_SR1    = 0;	// 清除状态
	PWMA_SR2    = 0;	// 清除状态
	ccer1 = 0;
	ccer2 = 0;
	ps    = 0;
	eno   = 0;

	PWMA_PSCRH = 0;		// 预分频寄存器, 分频 Fck_cnt = Fck_psc/(PSCR[15:0}+1), 边沿对齐PWM频率 = SYSclk/((PSCR+1)*(AAR+1)), 中央对齐PWM频率 = SYSclk/((PSCR+1)*(AAR+1)*2).
	PWMA_PSCRL = 0;
	PWMA_DTR  = 24;			// 死区时间配置, n=0~127: DTR= n T,   0x80 ~(0x80+n), n=0~63: DTR=(64+n)*2T,
							//				0xc0 ~(0xc0+n), n=0~31: DTR=(32+n)*8T,   0xE0 ~(0xE0+n), n=0~31: DTR=(32+n)*16T,
	PWMA_ARRH   = 0;	// 自动重装载寄存器,  控制PWM周期
	PWMA_ARRL   = (50-1);

	PWMA_CCMR1  = 0x60;		// 通道模式配置, PWM模式1
	PWMA_CCR1H  = 0;		// 比较值, 控制占空比(高电平时钟数)
	PWMA_CCR1L  = 0;
	ccer1 |= 0x05;			// 开启比较输出, 高电平有效
	ps    |= 0;				//项目里用PWM3P，这里应该是0x10// 选择IO, 0:选择P1.0 P1.1, 1:选择P0.0 P0.1, 2:选择P2.0 P2.1,
	eno   |= 0x01;			//项目里用的P0.4，这里应该是0x10// IO输出允许,  bit7: ENO4N, bit6: ENO4P, bit5: ENO3N, bit4: ENO3P,  bit3: ENO2N,  bit2: ENO2P,  bit1: ENO1N,  bit0: ENO1P
							//项目中使用的是3P通道，所以应该是0x10
	P1n_push_pull(Pin0);

	PWMA_CCER1  = ccer1;	// 捕获/比较使能寄存器1
	PWMA_CCER2  = ccer2;	// 捕获/比较使能寄存器2
	PWMA_PS     = ps;		// 选择IO
	PWMA_CCMR1  = 0x68;		// 通道模式配置, PWM模式1, 预装载允许

	PWMA_BKR    = 0x80;		// 主输出使能 相当于总开关
	PWMA_CR1    = 0x81;		// 使能计数器, 允许自动重装载寄存器缓冲, 边沿对齐模式, 向上计数,  bit7=1:写自动重装载寄存器缓冲(本周期不会被打扰), =0:直接写自动重装载寄存器本(周期可能会乱掉)
	PWMA_EGR    = 0x01;		//产生一次更新事件, 清除计数器和预分频计数器, 装载预分频寄存器的值
	PWMA_ENO    = eno;		// 允许IO输出
}

//	PWMA_PS   = (0<<6)+(0<<4)+(0<<2)+0;	//选择IO, 4项从高到低(从左到右)对应PWM1 PWM2 PWM3 PWM4, 0:选择P1.x, 1:选择P2.x, 2:选择P6.x,
//  PWMA_PS    PWM4N PWM4P    PWM3N PWM3P    PWM2N PWM2P    PWM1N PWM1P
//    00       P1.7  P1.6     P1.5  P1.4     P1.3  P1.2     P1.1  P1.0
//    01       P0.7  P0.6     P0.5  P0.4     P0.3  P0.2     P0.1  P0.0
//    02       P2.7  P2.6     P2.5  P2.4     P2.3  P2.2     P2.1  P2.0
//    03        --    --       --    --       --    --       --    --


//================================== PWMA DMA =================================================================================
//DMA_PWMAT_CR 	PWMAT_DMA控制寄存器
#define		DMA_ENPWMAT		(1<<7)	// PWMAT_DMA功能使能控制位, bit7, 0:禁止PWMAT DMA功能，  1：允许PWMAT DMA功能。
#define		PWMAT_TRIG		(1<<6)	// PWMAT_DMA触发控制位，    bit6, 0:写0无效，          1：写1开始PWMAT DMA操作。

//DMA_PWMAT_CFG 	PWMAT_DMA配置寄存器
#define		DMA_PWMATIE	(1<<7)	// PWMAT_DMA中断使能控制位，bit7, 0:禁止PWMAT DMA中断，     1：允许中断。
#define		DMA_PWMATIP	(0<<2)	// PWMAT_DMA中断优先级控制位，bit3~bit2, (最低)0~3(最高).
#define		DMA_PWMATPTY	2	// PWMAT_DMA数据总线访问优先级控制位，bit1~bit0, (最低)0~3(最高).

//DMA_PWMAT_STA 	PWMAT_DMA状态寄存器
#define		PWMAT_TXOVW	(1<<2)	// PWMAT DMA数据覆盖标志位，bit2, 软件清0.
#define		DMA_PWMATIF		1	// PWMAT DMA中断请求标志位，bit0, 软件清0.

void	PWMAT_DMA_TRIG(u8 xdata *TxBuf, u16 num)
{
	u16	i;

    PWMA_DBA   = 0x0D;	//DMA传输目标起始地址(offset代码), 0x0d对应的就是PWM1_CCR1H, 请参考“基地址DBA与实际占空比寄存器对应关系表.jpg”
    PWMA_DBL   = 0x00;	//DMA传输通道数为1通道(n+1)		DBL=DMA burst length = N+1
    PWMA_DER   = 0x01;	//DMA传输数据触发条件为更新请求，即每个PWM周期开始后就触发DMA发送一次
    PWMA_DMACR = 0x14;	//Bit4(DSKIP)=1:跳过保留字节, Bit3(DDIR)=0:传输方向XRAM到PWMA，BIt2(DMAEN)=1:允许PWMA_DMA， dsize{1:0]=1:每个通道发送字节数为2字节。

	i = (u16)TxBuf;	//取首地址
	DMA_PWMAT_TXAH = (u8)(i >> 8);		//发送地址寄存器高字节
	DMA_PWMAT_TXAL = (u8)i;				//发送地址寄存器低字节
	DMA_PWMAT_AMTH = (u8)((num-1)/256);	//设置传输总字节数 = n+1
	DMA_PWMAT_AMT  = (u8)((num-1)%256);	//设置传输总字节数 = n+1
//	DMA_PWMA_ITVH  = (u8)(2460/256);	//增加的间隔时间，N+1个系统时钟
//	DMA_PWMA_ITVL  = (u8)(2460%256);
	DMA_PWMAT_STA  = 0x00;
	DMA_PWMAT_CFG  = DMA_PWMATIE | DMA_PWMATIP | DMA_PWMATPTY;
	DMA_PWMAT_CR   = DMA_ENPWMAT | PWMAT_TRIG;
P33 = 1;	//高电平时间用于指示DMA传输总耗时
	B_PWMAT_DMA_busy = 1;	//标志PWMAT-DMA忙，PWMAT DMA中断中清除此标志，使用PWMAT DMA前要确认此标志为0

}

//========================================================================
// 函数: void PWMAT_DMA_ISR (void) interrupt DMA_PWMAT_VECTOR
// 描述:  PWMAT_DMA中断函数.
// 参数: none.
// 返回: none.
// 版本: V1.0, 2024-1-5
//========================================================================
void PWMAT_DMA_ISR (void) interrupt DMA_PWMAT_VECTOR
{
P33 = 0;	//高电平时间用于指示DMA传输总耗时
	B_PWMAT_DMA_busy = 0;	//清除标志PWMA-DMA忙，PWMAT DMA中断中清除此标志，使用PWMAT DMA前要确认此标志为0
//	DMA_PWMAT_CR  = 0;
	DMA_PWMAT_STA = 0;		//清除中断标志
}
