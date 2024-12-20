#include "AI8051U.h"
#include "stdio.h"
#include "intrins.h"

/****************************** 用户定义宏 ***********************************/

#define MAIN_Fosc       24000000UL
#define SysTick         10000       // 次/秒, 系统滴答频率, 在4000~16000之间
#define Timer0_Reload   (65536UL - ((MAIN_Fosc + SysTick/2) / SysTick))     //Timer 0 中断频率

#define Baudrate        115200L
#define TM              (65536 -(MAIN_Fosc/Baudrate/4))
#define PrintUart       1        //1:printf 使用 UART1; 2:printf 使用 UART2


#define	COLOR	50				//亮度，最大255

#define	LED_NUM	32				//LED灯个数
#define	SPI_NUM	(LED_NUM*12)	//LED灯对应SPI字节数

#define IR_CODE_1 10
#define IR_CODE_2 20
#define IR_CODE_3 30
#define IR_CODE_4 40
#define IR_CODE_5 50
#define IR_CODE_6 60

/*************  本地变量声明    **************/

bit B_1ms;          //1ms标志
u8  cnt_1ms;        //1ms基本计时


u8	xdata  led_RGB[LED_NUM][3];	//LED对应的RGB，led_buff[i][0]-->绿，led_buff[i][1]-->红，led_buff[i][0]-->蓝.
u8	xdata  led_SPI[SPI_NUM];	//LED灯对应SPI字节数

bit	B_SPI_DMA_busy;	//SPI-DMA忙标志

/*************  红外接收程序变量声明    **************/

sbit P_IR_RX = P3^5;    //定义红外接收输入IO口//工程里用P5.0，这里应该改为P5^0

u8  IR_SampleCnt;       //采样计数
u8  IR_BitCnt;          //编码位数
u8  IR_UserH;           //用户码(地址)高字节
u8  IR_UserL;           //用户码(地址)低字节
u8  IR_data;            //数据原码
u8  IR_DataShit;        //数据移位

bit P_IR_RX_temp;       //Last sample
bit B_IR_Sync;          //已收到同步标志
bit B_IR_Press;         //红外接收标志
u8  IR_code;            //红外键码
u16 UserCode;           //用户码

/*************  本地函数声明    **************/

void SPI_Config(u8 SPI_io, u8 SPI_speed);
void LoadSPI(void);
void SPI_DMA_TxTRIG(u8 xdata *TxBuf, u16 num);
void delay_ms(u16 ms);
void sample1_run_single();
void sample2_line_by_line_X();
void sample3_line_by_line_Y();
void sample4_layer_by_layer();
void sample5_rainbow();

void sample1_run_single()
{
    u16 j = 0;
    static u16 k = 0;

    u8	xdata *px;
    px = &led_RGB[0][0];	//亮度(颜色)首地址
    for(u16 i=0; i<(LED_NUM*3); i++, px++)	*px = 0;	//清零 TODO: memset instead

    j = k;

    //set RGB=50 50 50
    led_RGB[j][1] = COLOR;		//红色
    if(++j >= LED_NUM)	j = 0;	//下一个灯
    led_RGB[j][0] = COLOR;		//绿色
    if(++j >= LED_NUM)	j = 0;	//下一个灯
    led_RGB[j][2] = COLOR;		//蓝色

    if(++k >= LED_NUM)	k = 0;			//顺时针
//	if(--k >= LED_NUM)	k = LED_NUM-1;	//逆时针

}

/********************* 主函数 *************************/
void main(void)
{
    WTST = 0;  //设置程序指令延时参数，赋值为0可将CPU执行指令的速度设置为最快
    EAXFR = 1; //扩展寄存器(XFR)访问使能
    CKCON = 0; //提高访问XRAM速度

    P0M1 = 0x00;   P0M0 = 0x00;   //设置为准双向口
    P1M1 = 0x00;   P1M0 = 0x00;   //设置为准双向口
    P2M1 = 0x00;   P2M0 = 0x00;   //设置为准双向口
    P3M1 = 0x00;   P3M0 = 0x00;   //设置为准双向口
    P4M1 = 0x00;   P4M0 = 0x00;   //设置为准双向口
    P5M1 = 0x00;   P5M0 = 0x00;   //设置为准双向口
    P6M1 = 0x00;   P6M0 = 0x00;   //设置为准双向口
    P7M1 = 0x00;   P7M0 = 0x00;   //设置为准双向口

    AUXR = 0x80;    //Timer0 set as 1T, 16 bits timer auto-reload, 
    TH0 = (u8)(Timer0_Reload / 256);
    TL0 = (u8)(Timer0_Reload % 256);
    ET0 = 1;        //Timer0 interrupt enable
    TR0 = 1;        //Tiner0 run

    cnt_1ms = SysTick / 1000;

    SPI_Config(1, 1);

    EA = 1;     //打开总中断
    
    while(1)
    {
        if(B_1ms)   //1ms到
        {
            B_1ms = 0;
            
            if(B_IR_Press)      //检测到收到红外键码
            {
                B_IR_Press = 0;
                if(IR_code == IR_CODE_1)
                    sample1_run_single();
                else if(IR_code == IR_CODE_2)
                    sample2_line_by_line_X();
                else if(IR_code == IR_CODE_3)
                    sample3_line_by_line_Y();
                else if(IR_code == IR_CODE_4)
                    sample4_layer_by_layer();
                else if(IR_code == IR_CODE_5)
                    sample5_rainbow();
                else
                    sample1_run_single();
                LoadSPI();	//将颜色装载到SPI数据
                SPI_DMA_TxTRIG(led_SPI, SPI_NUM);	//u8 xdata *TxBuf, u16 num), 启动SPI DMA, 720字节一共耗时2.08ms @25.6MHz
                printf("Read: UserCode=0x%04x,IRCode=%u\r\n",UserCode,IR_code);

                delay_ms(50);
            }
        }
    }
}

//**************************** IR相关函数 ********************************************

#define IR_SAMPLE_TIME      (1000000UL/SysTick)     //查询时间间隔, us, 红外接收要求在60us~250us之间
#if ((IR_SAMPLE_TIME <= 250) && (IR_SAMPLE_TIME >= 60))
    #define D_IR_sample         IR_SAMPLE_TIME      //定义采样时间，在60us~250us之间
#endif

#define D_IR_SYNC_MAX       (15000/D_IR_sample) //SYNC max time
#define D_IR_SYNC_MIN       (9700 /D_IR_sample) //SYNC min time
#define D_IR_SYNC_DIVIDE    (12375/D_IR_sample) //decide data 0 or 1
#define D_IR_DATA_MAX       (3000 /D_IR_sample) //data max time
#define D_IR_DATA_MIN       (600  /D_IR_sample) //data min time
#define D_IR_DATA_DIVIDE    (1687 /D_IR_sample) //decide data 0 or 1
#define D_IR_BIT_NUMBER     32                  //bit number

void IR_RX_NEC(void)
{
    u8  SampleTime;

    IR_SampleCnt++;                         //Sample + 1

    F0 = P_IR_RX_temp;                      //Save Last sample status
    P_IR_RX_temp = P_IR_RX;                 //Read current status
    if(F0 && !P_IR_RX_temp)                 //Pre-sample is high，and current sample is low, so is fall edge
    {
        SampleTime = IR_SampleCnt;          //get the sample time
        IR_SampleCnt = 0;                   //Clear the sample counter

        if(SampleTime > D_IR_SYNC_MAX)     B_IR_Sync = 0;  //large the Maxim SYNC time, then error
        else if(SampleTime >= D_IR_SYNC_MIN)                    //SYNC
        {
            if(SampleTime >= D_IR_SYNC_DIVIDE)
            {
                B_IR_Sync = 1;                  //has received SYNC
                IR_BitCnt = D_IR_BIT_NUMBER;    //Load bit number
            }
        }
        else if(B_IR_Sync)                      //has received SYNC
        {
            if(SampleTime > D_IR_DATA_MAX)      B_IR_Sync=0;    //data samlpe time too large
            else
            {
                IR_DataShit >>= 1;                  //data shift right 1 bit
                if(SampleTime >= D_IR_DATA_DIVIDE)  IR_DataShit |= 0x80;    //devide data 0 or 1
                if(--IR_BitCnt == 0)                //bit number is over?
                {
                    B_IR_Sync = 0;                  //Clear SYNC
                    if(~IR_DataShit == IR_data)     //判断数据正反码
                    {
                        UserCode = ((u16)IR_UserH << 8) + IR_UserL;
                        IR_code      = IR_data;
                        B_IR_Press   = 1;           //数据有效
                    }
                }
                else if((IR_BitCnt & 7)== 0)        //one byte receive
                {
                    IR_UserL = IR_UserH;            //Save the User code high byte
                    IR_UserH = IR_data;             //Save the User code low byte
                    IR_data  = IR_DataShit;         //Save the IR data byte
                }
            }
        }
    }
}

/********************** Timer0中断函数 ************************/
void timer0 (void) interrupt 1
{
    IR_RX_NEC();
    if(--cnt_1ms == 0)
    {
        cnt_1ms = SysTick / 1000;
        B_1ms = 1;      //1ms标志
    }
}

/********************** SPI相关函数 ************************/

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

        /** 下面的代码应该挪到SPI_Config里面 */
        P1n_pure_input(Pin6);			// MISO-P1.6设置为高阻，并允许下拉电阻，这样SPI发送完成后MOSI是低电平。
        PullUpDisable(P1PU, Pin6);		// 禁止端口内部上拉电阻   PxPU, 要设置的端口对应位为1
        PullDownEnable(P1PD, Pin6);		// 允许端口内部下拉电阻   PxPD, 要设置的端口对应位为1
        /** 上面的代码应该挪到SPI_Config里面 */
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

void SPI_DMA_ISR (void) interrupt DMA_SPI_VECTOR
{
	DMA_SPI_STA = 0;		//清除中断标志
	B_SPI_DMA_busy = 0;		//SPI-DMA忙标志
}
