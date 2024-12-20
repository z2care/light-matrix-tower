#include "AI8051U.h"
#include "stdio.h"
#include "intrins.h"

/****************************** �û������ ***********************************/

#define MAIN_Fosc       24000000UL
#define SysTick         10000       // ��/��, ϵͳ�δ�Ƶ��, ��4000~16000֮��
#define Timer0_Reload   (65536UL - ((MAIN_Fosc + SysTick/2) / SysTick))     //Timer 0 �ж�Ƶ��

#define Baudrate        115200L
#define TM              (65536 -(MAIN_Fosc/Baudrate/4))
#define PrintUart       1        //1:printf ʹ�� UART1; 2:printf ʹ�� UART2


#define	COLOR	50				//���ȣ����255

#define	LED_NUM	32				//LED�Ƹ���
#define	SPI_NUM	(LED_NUM*12)	//LED�ƶ�ӦSPI�ֽ���

#define IR_CODE_1 10
#define IR_CODE_2 20
#define IR_CODE_3 30
#define IR_CODE_4 40
#define IR_CODE_5 50
#define IR_CODE_6 60

/*************  ���ر�������    **************/

bit B_1ms;          //1ms��־
u8  cnt_1ms;        //1ms������ʱ


u8	xdata  led_RGB[LED_NUM][3];	//LED��Ӧ��RGB��led_buff[i][0]-->�̣�led_buff[i][1]-->�죬led_buff[i][0]-->��.
u8	xdata  led_SPI[SPI_NUM];	//LED�ƶ�ӦSPI�ֽ���

bit	B_SPI_DMA_busy;	//SPI-DMAæ��־

/*************  ������ճ����������    **************/

sbit P_IR_RX = P3^5;    //��������������IO��//��������P5.0������Ӧ�ø�ΪP5^0

u8  IR_SampleCnt;       //��������
u8  IR_BitCnt;          //����λ��
u8  IR_UserH;           //�û���(��ַ)���ֽ�
u8  IR_UserL;           //�û���(��ַ)���ֽ�
u8  IR_data;            //����ԭ��
u8  IR_DataShit;        //������λ

bit P_IR_RX_temp;       //Last sample
bit B_IR_Sync;          //���յ�ͬ����־
bit B_IR_Press;         //������ձ�־
u8  IR_code;            //�������
u16 UserCode;           //�û���

/*************  ���غ�������    **************/

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
    px = &led_RGB[0][0];	//����(��ɫ)�׵�ַ
    for(u16 i=0; i<(LED_NUM*3); i++, px++)	*px = 0;	//���� TODO: memset instead

    j = k;

    //set RGB=50 50 50
    led_RGB[j][1] = COLOR;		//��ɫ
    if(++j >= LED_NUM)	j = 0;	//��һ����
    led_RGB[j][0] = COLOR;		//��ɫ
    if(++j >= LED_NUM)	j = 0;	//��һ����
    led_RGB[j][2] = COLOR;		//��ɫ

    if(++k >= LED_NUM)	k = 0;			//˳ʱ��
//	if(--k >= LED_NUM)	k = LED_NUM-1;	//��ʱ��

}

/********************* ������ *************************/
void main(void)
{
    WTST = 0;  //���ó���ָ����ʱ��������ֵΪ0�ɽ�CPUִ��ָ����ٶ�����Ϊ���
    EAXFR = 1; //��չ�Ĵ���(XFR)����ʹ��
    CKCON = 0; //��߷���XRAM�ٶ�

    P0M1 = 0x00;   P0M0 = 0x00;   //����Ϊ׼˫���
    P1M1 = 0x00;   P1M0 = 0x00;   //����Ϊ׼˫���
    P2M1 = 0x00;   P2M0 = 0x00;   //����Ϊ׼˫���
    P3M1 = 0x00;   P3M0 = 0x00;   //����Ϊ׼˫���
    P4M1 = 0x00;   P4M0 = 0x00;   //����Ϊ׼˫���
    P5M1 = 0x00;   P5M0 = 0x00;   //����Ϊ׼˫���
    P6M1 = 0x00;   P6M0 = 0x00;   //����Ϊ׼˫���
    P7M1 = 0x00;   P7M0 = 0x00;   //����Ϊ׼˫���

    AUXR = 0x80;    //Timer0 set as 1T, 16 bits timer auto-reload, 
    TH0 = (u8)(Timer0_Reload / 256);
    TL0 = (u8)(Timer0_Reload % 256);
    ET0 = 1;        //Timer0 interrupt enable
    TR0 = 1;        //Tiner0 run

    cnt_1ms = SysTick / 1000;

    SPI_Config(1, 1);

    EA = 1;     //�����ж�
    
    while(1)
    {
        if(B_1ms)   //1ms��
        {
            B_1ms = 0;
            
            if(B_IR_Press)      //��⵽�յ��������
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
                LoadSPI();	//����ɫװ�ص�SPI����
                SPI_DMA_TxTRIG(led_SPI, SPI_NUM);	//u8 xdata *TxBuf, u16 num), ����SPI DMA, 720�ֽ�һ����ʱ2.08ms @25.6MHz
                printf("Read: UserCode=0x%04x,IRCode=%u\r\n",UserCode,IR_code);

                delay_ms(50);
            }
        }
    }
}

//**************************** IR��غ��� ********************************************

#define IR_SAMPLE_TIME      (1000000UL/SysTick)     //��ѯʱ����, us, �������Ҫ����60us~250us֮��
#if ((IR_SAMPLE_TIME <= 250) && (IR_SAMPLE_TIME >= 60))
    #define D_IR_sample         IR_SAMPLE_TIME      //�������ʱ�䣬��60us~250us֮��
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
    if(F0 && !P_IR_RX_temp)                 //Pre-sample is high��and current sample is low, so is fall edge
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
                    if(~IR_DataShit == IR_data)     //�ж�����������
                    {
                        UserCode = ((u16)IR_UserH << 8) + IR_UserL;
                        IR_code      = IR_data;
                        B_IR_Press   = 1;           //������Ч
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

/********************** Timer0�жϺ��� ************************/
void timer0 (void) interrupt 1
{
    IR_RX_NEC();
    if(--cnt_1ms == 0)
    {
        cnt_1ms = SysTick / 1000;
        B_1ms = 1;      //1ms��־
    }
}

/********************** SPI��غ��� ************************/

void	LoadSPI(void)
{
	u8	xdata *px;
	u16	i,j;
	u8	k;
	u8	dat;

	for(i=0; i<SPI_NUM; i++)	led_SPI[i] = 0;
	px = &led_RGB[0][0];	//�׵�ַ
	for(i=0, j=0; i<(LED_NUM*3); i++)
	{
		dat = *px;
		px++;
		for(k=0; k<4; k++)
		{
			if(dat & 0x80)	led_SPI[j]  = 0xE0;	//����1
			else			led_SPI[j]  = 0x80;	//����0
			if(dat & 0x40)	led_SPI[j] |= 0x0E;	//����1
			else			led_SPI[j] |= 0x08;	//����0
			dat <<= 2;
			j++;
		}
	}
}

void  SPI_Config(u8 SPI_io, u8 SPI_speed)
{
	SPCTL = SPI_speed & 3;	//����SPI �ٶ�, ����ָ����ڵ�һλ, Bit7~Bit2��0
	SSIG = 1;	//1: ����SS�ţ���MSTRλ�����������Ǵӻ�		0: SS�����ھ����������Ǵӻ���
	SPEN = 1;	//1: ����SPI��								0����ֹSPI������SPI�ܽž�Ϊ��ͨIO
	DORD = 0;	//1��LSB�ȷ���								0��MSB�ȷ�
	MSTR = 1;	//1����Ϊ����								0����Ϊ�ӻ�
	CPOL = 0;	//1: ����ʱSCLKΪ�ߵ�ƽ��					0������ʱSCLKΪ�͵�ƽ
	CPHA = 0;	//1: ������SCLKǰ������,���ز���.			0: ������SCLKǰ�ز���,��������.
	P_SW1 = (P_SW1 & ~0x0c) | ((SPI_io<<2) & 0x0c);		//�л�IO

	HSCLKDIV   = 1;		//HSCLKDIV��ʱ�ӷ�Ƶ
	SPI_CLKDIV = 1;		//SPI_CLKDIV��ʱ�ӷ�Ƶ
	if(SPI_io == 0)
	{
		P1n_standard(0xf0);			//�л��� P1.4(SS) P1.5(MOSI) P1.6(MISO) P1.7(SCLK), ����Ϊ׼˫���
		PullUpEnable(P1PU, 0xf0);	//������������    ����˿��ڲ���������   PxPU, Ҫ���õĶ˿ڶ�ӦλΪ1
		P1n_push_pull(Pin7+Pin5);	//MOSI SCLK����Ϊ�������
		SlewRateHigh(P1SR, Pin7+Pin5);	//MOSI SCLK�˿��������Ϊ����ģʽ   PxSR, Ҫ���õĶ˿ڶ�ӦλΪ1.    ����ģʽ��3.3V����ʱ�ٶȿ��Ե�13.5MHz(27MHz��Ƶ��SPI�ٶ�2��Ƶ)

        /** ����Ĵ���Ӧ��Ų��SPI_Config���� */
        P1n_pure_input(Pin6);			// MISO-P1.6����Ϊ���裬�������������裬����SPI������ɺ�MOSI�ǵ͵�ƽ��
        PullUpDisable(P1PU, Pin6);		// ��ֹ�˿��ڲ���������   PxPU, Ҫ���õĶ˿ڶ�ӦλΪ1
        PullDownEnable(P1PD, Pin6);		// ����˿��ڲ���������   PxPD, Ҫ���õĶ˿ڶ�ӦλΪ1
        /** ����Ĵ���Ӧ��Ų��SPI_Config���� */
	}
	else if(SPI_io == 1)
	{
		P2n_standard(0xf0);			//�л���P2.4(SS) P2.5(MOSI) P2.6(MISO) P2.7(SCLK), ����Ϊ׼˫���
		PullUpEnable(P2PU, 0xf0);	//������������    ����˿��ڲ���������   PxPU, Ҫ���õĶ˿ڶ�ӦλΪ1
		P2n_push_pull(Pin7+Pin5);	//MOSI SCLK����Ϊ�������
		SlewRateHigh(P2SR, Pin7+Pin5);	//MOSI SCLK�˿��������Ϊ����ģʽ   PxSR, Ҫ���õĶ˿ڶ�ӦλΪ1.    ����ģʽ��3.3V����ʱ�ٶȿ��Ե�13.5MHz(27MHz��Ƶ��SPI�ٶ�2��Ƶ)
	}
	else if(SPI_io == 2)
	{
		P4n_standard(0x0f);			//�л���P4.0(SS) P4.1(MOSI) P4.2(MISO) P4.3(SCLK), ����Ϊ׼˫���
		PullUpEnable(P4PU, 0x0f);	//������������    ����˿��ڲ���������   PxPU, Ҫ���õĶ˿ڶ�ӦλΪ1
		P4n_push_pull(Pin3+Pin1);	//MOSI SCLK����Ϊ�������
		SlewRateHigh(P4SR, Pin3+Pin1);	//MOSI SCLK�˿��������Ϊ����ģʽ   PxSR, Ҫ���õĶ˿ڶ�ӦλΪ1.    ����ģʽ��3.3V����ʱ�ٶȿ��Ե�13.5MHz(27MHz��Ƶ��SPI�ٶ�2��Ƶ)
	}
	else if(SPI_io == 3)
	{
		P3n_standard(0x3C);		//�л���P3.5(SS) P3.4(MOSI) P3.3(MISO) P3.2(SCLK), ����Ϊ׼˫���
		PullUpEnable(P3PU, 0x3c);	//������������    ����˿��ڲ���������   PxPU, Ҫ���õĶ˿ڶ�ӦλΪ1
		P3n_push_pull(Pin4+Pin2);	//MOSI SCLK����Ϊ�������
		SlewRateHigh(P3SR, Pin4+Pin2);	//MOSI SCLK�˿��������Ϊ����ģʽ   PxSR, Ҫ���õĶ˿ڶ�ӦλΪ1.    ����ģʽ��3.3V����ʱ�ٶȿ��Ե�13.5MHz(27MHz��Ƶ��SPI�ٶ�2��Ƶ)
	}
}

//DMA_SPI_CR 	SPI_DMA���ƼĴ���
#define		DMA_ENSPI		(1<<7)	// SPI DMA����ʹ�ܿ���λ��    bit7, 0:��ֹSPI DMA���ܣ�  1������SPI DMA���ܡ�
#define		SPI_TRIG_M		(1<<6)	// SPI DMA����ģʽ��������λ��bit6, 0:д0��Ч��          1��д1��ʼSPI DMA����ģʽ������
#define		SPI_TRIG_S		(1<<5)	// SPI DMA�ӻ�ģʽ��������λ��bit5, 0:д0��Ч��          1��д1��ʼSPI DMA�ӻ�ģʽ������
#define		SPI_CLRFIFO			1	// ���SPI DMA����FIFO����λ��bit0, 0:д0��Ч��          1��д1��λFIFOָ�롣

//DMA_SPI_CFG 	SPI_DMA���üĴ���
#define		DMA_SPIIE	(1<<7)	// SPI DMA�ж�ʹ�ܿ���λ��bit7, 0:��ֹSPI DMA�жϣ�     1�������жϡ�
#define		SPI_ACT_TX	(1<<6)	// SPI DMA�������ݿ���λ��bit6, 0:��ֹSPI DMA�������ݣ�����ֻ��ʱ�Ӳ������ݣ��ӻ�Ҳ����. 1�������͡�
#define		SPI_ACT_RX	(1<<5)	// SPI DMA�������ݿ���λ��bit5, 0:��ֹSPI DMA�������ݣ�����ֻ��ʱ�Ӳ������ݣ��ӻ�Ҳ����. 1��������ա�
#define		DMA_SPIIP	(0<<2)	// SPI DMA�ж����ȼ�����λ��bit3~bit2, (���)0~3(���).
#define		DMA_SPIPTY		0	// SPI DMA�������߷������ȼ�����λ��bit1~bit0, (���)0~3(���).

//DMA_SPI_CFG2 	SPI_DMA���üĴ���2
#define		SPI_WRPSS	(0<<2)	// SPI DMA������ʹ��SS�ſ���λ��bit2, 0: SPI DMA������̲��Զ�����SS�š�  1���Զ�����SS�š�
#define		SPI_SSS	    	0	// SPI DMA�������Զ�����SS��ѡ��λ��bit1~bit0, 0: P1.4,  1��P2.4,  2: P4.0,  3:P3.5��

//DMA_SPI_STA 	SPI_DMA״̬�Ĵ���
#define		SPI_TXOVW	(1<<2)	// SPI DMA���ݸ��Ǳ�־λ��bit2, �����0.
#define		SPI_RXLOSS	(1<<1)	// SPI DMA�������ݶ�����־λ��bit1, �����0.
#define		DMA_SPIIF		1	// SPI DMA�ж������־λ��bit0, �����0.

//HSSPI_CFG  ����SPI���üĴ���
#define		SS_HOLD		(0<<4)	//����ģʽʱSS�����źŵ�HOLDʱ�䣬 0~15, Ĭ��3. ��DMA�л�����N��ϵͳʱ�ӣ���SPI�ٶ�Ϊϵͳʱ��/2ʱִ��DMA��SS_HOLD��SS_SETUP��SS_DACT���������ô���2��ֵ.
#define		SS_SETUP		3	//����ģʽʱSS�����źŵ�SETUPʱ�䣬0~15, Ĭ��3. ��DMA�в�Ӱ��ʱ�䣬       ��SPI�ٶ�Ϊϵͳʱ��/2ʱִ��DMA��SS_HOLD��SS_SETUP��SS_DACT���������ô���2��ֵ.

//HSSPI_CFG2  ����SPI���üĴ���2
#define		SPI_IOSW	(1<<6)	//bit6:����MOSI��MISO��λ��0����������1������
#define		HSSPIEN		(1<<5)	//bit5:����SPIʹ��λ��0���رո���ģʽ��1��ʹ�ܸ���ģʽ
#define		FIFOEN		(1<<4)	//bit4:����SPI��FIFOģʽʹ��λ��0���ر�FIFOģʽ��1��ʹ��FIFOģʽ��ʹ��FIFOģʽ��DMA�м���13��ϵͳʱ�䡣
#define		SS_DACT			3	//bit3~0:����ģʽʱSS�����źŵ�DEACTIVEʱ�䣬0~15, Ĭ��3. ��SPI�ٶ�Ϊϵͳʱ��/2ʱִ��DMA��SS_HOLD��SS_SETUP��SS_DACT���������ô���2��ֵ.


void	SPI_DMA_TxTRIG(u8 xdata *TxBuf, u16 num)
{
	u16 j;
	HSSPI_CFG  = SS_HOLD | SS_SETUP;	//SS_HOLD������N��ϵͳʱ��, SS_SETUPû������ʱ�ӡ�
	HSSPI_CFG2 = HSSPIEN | FIFOEN | SS_DACT;	//FIFOEN����FIFO���С13��ʱ��, 67T @8��Ƶ.

	j = (u16)TxBuf;		//ȡ�׵�ַ
	DMA_SPI_TXAH = (u8)(j >> 8);				//���͵�ַ�Ĵ������ֽ�
	DMA_SPI_TXAL = (u8)j;						//���͵�ַ�Ĵ������ֽ�
	DMA_SPI_AMTH = (u8)((num-1)/256);		//���ô������ֽ��� = n+1
	DMA_SPI_AMT  = (u8)((num-1)%256);		//���ô������ֽ��� = n+1
	DMA_SPI_ITVH = 0;					//���ӵļ��ʱ�䣬N+1��ϵͳʱ��
	DMA_SPI_ITVL = 0;
	DMA_SPI_STA  = 0x00;
	DMA_SPI_CFG  = DMA_SPIIE | SPI_ACT_TX | DMA_SPIIP | DMA_SPIPTY;
	DMA_SPI_CFG2 = SPI_WRPSS | SPI_SSS;
	DMA_SPI_CR   = DMA_ENSPI | SPI_TRIG_M | SPI_CLRFIFO;
	B_SPI_DMA_busy = 1;		//SPI-DMAæ��־
}

void SPI_DMA_ISR (void) interrupt DMA_SPI_VECTOR
{
	DMA_SPI_STA = 0;		//����жϱ�־
	B_SPI_DMA_busy = 0;		//SPI-DMAæ��־
}
