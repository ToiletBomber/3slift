//zjbtest
/*
*****************************����ʹ���Զ�����ģʽ����ʼ��ַ0x22����������3������ֵ1��ʮ���Ƹ�ʽ
*****************************ע�⣬
*****************************��¼����18.4320M��
*****************************�͵�ѹ��ֹeeprom������
*****************************eeprom���ΪFF

ID�洢λ�� 0x20,0x21,0x22
��Ȩ��Чʱ��洢λ��ҳ��0��1��2����ַ0x00(��)��0x01(��),0x02(��)

���濪��ʱʱ��ҳ��4��5��6����ַ0x00(��)��0x01(��),0x02(��),0x03(ʱ),0x04(��),0x05(��)

����ʱ��ͣ���Զ��޸�����ҳ��8����ַ0x00��������10�α���

������Ȩ��ʽ����������0.1���ӵļ�⣬�ж��Ƿ������Ȩ����

2017-5-31

����ǰ�ĳ������޸�����Ϊstc15w4k32s4���°汾������ǰ������޸ļ�¼�������������Ѿ�û������
stc15w4k32s4ʹ��ͷ�ļ���ʽ�������Լ�����

������û��1302ǰ���޸ļ�¼
2017-5-27
	1\ʹ���ⲿ�洢�������һЩ������Ҫ��ʼ��
	2\������û��̫�������E31����
2017-5-25
1\������������ʱ��������
	ֻ����������ʱ�䣬
	����ʱ���÷��Ӽ�,ʹ��2���ֽ�,65535��=1092Сʱ��
	����ʱ�䳬ʱʱ,ֻ��������,�������½�,��ֻ���¿���Դʱ����Ч��

2\ʱ�������ʱ���ó����ַ�޸�,�����ַΪ[0x0010,0x0011],���ʱ��=[0x0010]*256+[0x0011]
��16����,���Դ���0x40ΪһСʱ,0x80,0xc0,0x100......

3\����ÿ��ʹ��1��,ÿ��20����,����Ϊ10����,����10���ӵ��β��ڼ�ʱ

4\���û������Ҫ�������ʱ����Ϊ300Сʱ

2017-05-22
���ڿͻ��ڰ�ȫ������״̬�£�����ʹ�ý�����ʽ�����ϵ�ʱ���½���ť��Ӧ�ð�������ťȻ���ֶ����������������ε��½���Ч

2017-01-04���Խ�����������������200k���ӵ�600k,ͬʱ����迹��330����1k

2015-08-12
����ʹ�÷������ѹ���ͱ����Ḳ���������ϱ���,����Ӱ��ͷ�����,�ĵ�ѹ��������ȫ����ԭ�й�����Ϣ

2015-04-08
�ֳ�����Ƶ������ѹ����,�޸�ԭ��360Ϊ330��

2014/3/19
����һ��ʱ�������,������λ��������,��¼�����������½���ʱ��,�Ա�ֹ���������λ��,�ж�����ʱ�ٳ����µ������͸�˿����Ӱ��,������int GaoDu,����һ������(��Ȩϵ��)GaoDuK

2013-05-03�޸�
1�����ټ�����������
2�����ز��ٲ������췽ʽ
3��

2012-12-3���Խ��
1��������21�Ŵ���Ȼ����ʧ
2������û��ʱ����ʾ����������������������������ʾ��1.2�ݲ��账��

3���趨��360~440��ʵ��360~400
4��ȱ�౨��ѹ��
5������δ����
*/


/*
�����޸ļ�¼
2012-10-25 �ڶ������ݳ������������5�����ݳ���Դ����contr204.c
1���̵������ƹ���ͨ��
2������ʧ�ܣ�����˸������������ѹ����Դ

2012-11-28��¼
1����ѹ����ͨ��
2������ʧ�ܣ���ѹоƬ���ȣ�����ר����������Դ

2012-8-2
���ౣ���ʹ��ౣ��ʱ����250���� ���ӵ�2.5��
�����޸ļ�¼
2012-3-24
���뱸���ű���
���ౣ������������0.6A����1.8A

������ڣ�2009-12-25
��ɵĵ���
1�����Ե�Ĳɼ���˳������
2�����صļ��Ϳ���
3����ȫ���̵����Ŀ���
4�����ϼ̵����Ŀ���
5�����صı궨
6������·�Ŀ���
7��������·�Ŀ���
8���½���·�Ŀ���
9�������Ŀ���
δ��ɵĵ���
1������ļ��Ϳ���
2��ȱ��ļ��Ϳ���
������ڣ�2010-1-11
*/
//���ļ�
#include <intrins.h>
#include <ABSACC.h>
#include <STC15W4K32S4.h>

//#include <STDIO.H>
//�����ĺ���

//����
#define OverLoadLimite	(128*4+6*10)	//��������ֵ 0.2mV<==>25kg<==>6
#define OverLoadLimite1	(OverLoadLimite-1)	

#define Crystal 	18432000
#define BaudRate 	19200
#define InitTime 	200//(us)���256*12/18432000=42.6ms

#define Alarm	 	0x40
#define Error	 	0x20

#define CheckCycle	31//��Լ25ms 800us*31=24.8ms
/*
#define MaxPower	((440-40)/2.5)//��ѹ���ޣ�440V
//#define MaxPower	(30/3.3)//��ѹ���ޣ�������Ӧ��=��ѹ/3����ʵ���ǵ�ѹ/3.3
#define MinPower	((360-40)/2.5)//��ѹ����  360V
#define LosePower	((110-40)/2.5)//��ѹ����  110V
*/
#define MaxPower	(440/3.1)//��ѹ���ޣ�ʵ��420V
//#define MinPower	(360/3.1)//��ѹ����  ʵ��360V
#define MinPower	(330/3.1)//2015-04-08
//#define MinPower	(430/3.1)//2015-08-12//test
#define LosePower	(110/3.1)//��ѹ����  110V

/*ʵ����
380V	80h

388V	87h

400V	7Eh	
360V	6Dh


20121212
��һ�β��������е�һ����������װ���ڶ�����װ���������Ǽ���ó�
50hz
120	26 25 25
300	68 66 5c
320	6f 6d 63
340	77 75 6a
360	7f 7c 70
380	87 84 78
400	8f 8c 7f
420	97 94 86
440	9f 9c 8e
460	a8 a4 95

	60hz		50hz
460	a7 a3 92	a9 a5 96
400	8f 8c 7d	90 8c 80
340	78 75 69	77 75 6a

��һ�β��������е�һ����������װ���ڶ�����װ���������Ǽ���ó�
	60hz		50hz
460	a7 a3 92	a9 a5 96
400	8f 8c 7d	90 8c 80
340	78 75 69	77 75 6a


�õ���ϵʽ������=(��ѹ-40)/2.5

����ÿ�����ݴ����+1��-1�ϸ���
*/


//IOλ����
//˫��
 
//A/D���⼸������ֻ��Ϊ��˵��ioλ�ã�ʵ�ʳ����в�������
sbit	Load	=P1^2;	//���ش�����������
sbit	AD0		=P1^3;	//A��
sbit	AD1		=P1^4;	//B��
sbit	AD2		=P1^5;	//C��
sbit	AD25	=P1^6;	//��������׼��ѹ

//���
sbit	DrvRun	=P3^2;	//���Ͽ���(ȱ�࣬���࣬����)
sbit	DrvSl3	=P3^3;	//��ȫ������
sbit	Ss0	=P0^6;	//���ã���·����չ��
sbit	Ss1	=P0^5;	//���ã���·����չ��

sbit	L_Run	=P5^0;	//��
/*595��165������ʹ�ã��޸������ɾ��
sbit	Pl	=P2^4;
sbit	DatOut	=P2^5;
sbit	Lsetb	=P2^6;
sbit	Lclk	=P2^7;
sbit	Nc2	=P3^2;	//��
sbit	Nc3	=P3^3;	//��
sbit	Nc4	=P3^4;	//��
sbit	Clk	=P3^5;
sbit	Ldat	=P3^7;

//���⹦�ܼĴ���
sfr	P0M0		=0x93;
sfr	P0M1		=0x94;
sfr	P1M0		=0x91;
sfr	P1M1		=0x92;
sfr	P2M0		=0x95;
sfr	P2M1		=0x96;
sfr	P3M0		=0xb1;
sfr	P3M1		=0xb2;

sfr	ISP_DATA	=0xe2;//ISP/IAP�������ݼĴ���
sfr	ISP_ADDRH	=0xe3;//ISP/IAP������ַ�Ĵ����߰�λ
sfr	ISP_ADDRL	=0xe4;//ISP/IAP������ַ�Ĵ����Ͱ�λ
sfr	ISP_CMD		=0xe5;//ISP/IAP��������Ĵ�������������Ĵ�������������Ч
				//=0���������޲���=1=2=3
				//=1����
				//=2��д
				//=3������
sfr	ISP_TRIG	=0xe6;//ISP/IAP����������Ĵ���
				//��ISPEN(ISP_CONTR.7)=1ʱ����д��0x46����д��0xb9��ISP/IAP����Ż���Ч
sfr	ISP_CONTR	=0xe7;////ISP/IAP���ƼĴ���

sfr	IPH		=0xb7;

sfr	AUXR		=0x8e;


sfr	WDT_CONTR		=0xe1;//���Ź�

*/

//����ȫ�ֱ���
unsigned char 	DelayStart;	//������ѹ�����ʱ
unsigned char	Sec5;		//һ��5���ѭ��������2015-08-12

unsigned char xdata	State[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	//�ɼ����״̬�Ĵ���
unsigned char Tcount;		//�ɼ�������

unsigned char 	ErrorState;	//���͵������ֽ�,�ֽڶ����StateErrorCheck()
unsigned char 	CountIndex;	//��ʱ������,��֤20ms����һ��ָ��
unsigned char  	SecCount;	//��ʱ��������1��ļ�ʱ,25ms��һ

unsigned int xdata 	OverLoad=0;
unsigned long xdata	OverLoad1=0;
unsigned int xdata	OverLoadCount=0;

unsigned int xdata	Clock_1s=0;

//λ
bit DrvUpSta;		//?
bit FlagPowerL;		//���Լ���ѹ���� Ƿѹ
bit FlagPowerH;		//���Լ���ѹ���� ��ѹ
bit FlagOverLoad;	//���Լ����ع���
bit FlagErrorPhase;	//���Լ��������
bit FlagLosePhase;	//���Լ��ȱ�����
bit FlagStart;		//���Ա�ʾ����=0��ʾʱ�䣬=1��ʾ״̬

bit FlagRun;					//״̬��ʾ =1����״̬ =0����״̬ ���Ա��������в��Թ��ز�������������ͣ��
bit FlagReadCurrent;			//=1����������

#define LoadZero 	(128*4)		//0����ֵ
//#define LoadMin 	(16*4)		//��������ֵ �൱��20mA*16*4=1.2A
//#define PhaseLoseMax	0x60	//ȱ������е�����׼���൱��20mA*0x60/2=0.9A
//#define PhaseLoseMin	0x40	//ȱ������޵�����׼���൱��20mA*0x40/2=0.6A

#define LoadMin 	64		//��������ֵ �൱��20mA*16*4=1.2A
#define PhaseLoseMax	200	//ȱ������е�����׼���൱��20mA*200/2=2A
#define PhaseLoseMin	180	//ȱ������޵�����׼���൱��20mA*180/2=1.8A

/*
	ԭʼ���ݣ�	ta12-200 5A==>2.5mA
	������̣�	2.5mA*470/5V*1024=240
	�ֱ��ʣ�	5A/240=20mA
*/
unsigned int xdata 	DelaySecCount=0;//��ʱ������
//�������

unsigned int xdata	LoadDot[4]={0,0,0,0};	//��һʱ�̲������ĵ���
//unsigned int 	CurrentMax[3];				//��������
//unsigned int 	CurrentMin[3];				//��������

//0~2��ABC��������ѹ��3��A�ߵ�ѹ
unsigned int xdata	LoadCurrent[4]={0,0,0,0};//��ѹ��ֵ=SumLoadCurrent[3]/SumCount
unsigned long xdata	SumLoadCurrent[4]={0,0,0,0};

unsigned int	SumCount;
unsigned char	PhaseLose;		//ȱ����������������10�����ڲ��ж�

bit				Polarity0;		//a���ߵ�ǰ����
bit				Polarity1;		//b���ߵ�ǰ����
bit				Polarity2;		//c���ߵ�ǰ����
unsigned char	PhaseTimer[3];	//�����ʱ
unsigned char	PhaseError;		//������������������10�����ڲ��ж�
unsigned char 	LoadIndex;		//�豸����������������


/*2014/3/19
�ٶ�����������10���Ӽ���50����
10����=600��=600000����
GaoDuÿ25�������һ��,����ֵ=600000����/25����=24000
��25����,��ģת��ֵ��6(���ֵ�Ƿ�׼ȷδ֪,�ȼٶ�׼ȷ)
50�������12,24000/12=2000,�����GaoDuK��ֵ
*/
unsigned int	GaoDu;//2014/3/19
#define GaoDuK	2000//2014/3/19

//2017-5-25 start
unsigned char 	OldErrorState;	//
bit 			FlagOverTimer;	//�ϵ�ʱ��¼��ʱ״̬
bit				OverTimer;		//��ʱ����������������
unsigned int	OneMin;
unsigned int	OldWorkTime;	//�豸����ʱ��
unsigned int	WorkTime;		//�豸����ʱ��
unsigned int	Pc_WorkTime;	//����ʱ���ַƫ���� 
//#define F_WorkTime	0x2800//stc12c5410ad
#define F_WorkTime	0x0000//stc15w4k32s4����stc15ϵ�У�eeprom����ƫ�ƣ�Ӳ��ƫ�ƣ�stc15w4k32s4ƫ��Ϊ0x8000�����ʹ��movcָ���Ҫ����ƫ������
//#define Max_WorkTime (x*60+y)//xСʱ��y����
#define Max_WorkTime (300*60+0)//���ʱ��300*2=600Сʱ
//#define Max_WorkTime (1*60+0)//�趨1Сʱ
unsigned char	WorkTimeStart;//���ο���ʱ��
//2017-5-25 end

unsigned char RL1,RL2;//��λ��������ʾ����2017-8-22

//ds1302����
//�Ĵ����궨�� 
#define WRITE_SECOND       	0x80 
#define WRITE_MINUTE       	0x82 
#define WRITE_HOUR         	0x84 
#define READ_SECOND        	0x81 
#define READ_MINUTE        	0x83 
#define READ_HOUR          	0x85 
#define WRITE_PROTECT      	0x8E
#define READ_PROTECT      	0x8F
#define WRITE_POWER       	0x90 
#define READ_POWER       	0x91 
#define WRITE_YEAR       	0x8C
#define WRITE_MONTH       	0x88 
#define WRITE_DATE         	0x86 
#define READ_YEAR        	0x8D 
#define READ_MONTH        	0x89 
#define READ_DATE          	0x87

unsigned char xdata DATE[7];	//0��1��2��3ʱ4��5�� 

sbit 	SCLK 	= P2^1;			//DS1302ʱ���ź�
sbit 	DIO		= P2^2;			// DS1302�����ź�
sbit 	CE 		= P2^3;			// DS1302Ƭѡ 

sbit	DA		= P4^3;
sbit	DB		= P4^2;
sbit	DF		= P4^1;
sbit	DC		= P7^3;
sbit	DD		= P7^2;
sbit	DE		= P7^1;
sbit	DG		= P7^0;

sbit	LED1	= P3^6;
sbit	LED2	= P3^5;
sbit	LED3	= P3^4;
sbit	IN3		= P2^4;
sbit 	IN2		= P2^5;
sbit 	IN1		= P2^6;
unsigned char	ShowTimer;
code char Bcd7Seg[64]	=
{
//dp g  f  e  d  c  b  a
    0x3F,// 0>>0
	0x06,// 1>>1
	0x5B,// 2>>2
	0x4F,// 3>>3
	0x66,// 4>>4
	0x6D,// 5>>5
	0x7D,// 6>>6
	0x07,// 7>>7
	0x7F,// 8>>8
	0x6F,// 9>>9
	0x77,// 10>>A
	0x7C,// 11>>b
	0x39,// 12>>C
	0x5E,// 13>>d
	0x79,// 14>>E
	0x71,// 15>>F
	0x76,// 16>>H
	0x74,// 17>>h
	0x38,// 18>>L
	0x54,// 19>>n
	0x63,// 20>>o��
	0x5C,// 21>>o��
	0x73,// 22>>p
	0x67,// 23>>q
	0x50,// 24>>r
	0x3E,// 25>>U
	0x40,// 26>>-
	0x23,// 27>>����
	0x1C,// 28>>�½�
	0x00,// 29>>�ո�
	0x00,// 30>>�ո�
	0x46,// 31>>-1

//a  b  c  d  e  f  g  dp
    0xFC,// 0>>0
	0x60,// 1>>1
	0xDA,// 2>>2
	0xF2,// 3>>3
	0x66,// 4>>4
	0xB6,// 5>>5
	0xBE,// 6>>6
	0xE0,// 7>>7
	0xFE,// 8>>8
	0xF6,// 9>>9
	0xEE,// 10>>A
	0x3E,// 11>>b
	0x9C,// 12>>C
	0x7A,// 13>>d
	0x9E,// 14>>E
	0x8E,// 15>>F
	0x6E,// 16>>H
	0x2E,// 17>>h
	0x1C,// 18>>L
	0x2A,// 19>>n
	0xC6,// 20>>o��
	0x3A,// 21>>o��
	0xCE,// 22>>p
	0xE6,// 23>>q
	0x0A,// 24>>r
	0x7C,// 25>>U
	0x02,// 26>>-
	0xC4,// 27>>����
	0x38,// 28>>�½�

	0x00,// 29>>�ո�
	0x00,// 30>>�ո�
	0x00,// 31>>�ո�
};
void Delay_us(unsigned int index)
{
	while(index--)
	{
		_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();
		_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();
		_nop_();_nop_();_nop_();_nop_();_nop_();_nop_();
	}
}

#define S2RI    0x01
#define S2TI    0x02

void Verify();
void test();

//��ַ�����ݷ����ӳ��� 
void Write1302 ( unsigned char addr,dat )     
{ 
	unsigned char i,temp; 
	
	CE=0;                         //CE����Ϊ�ͣ����ݴ�����ֹ 
	SCLK=0;                    //����ʱ������ 
	Delay_us(4);
	CE = 1;                       //CE����Ϊ�ߣ��߼�������Ч 
	Delay_us(4);
	//���͵�ַ 
	for ( i=8; i>0; i-- ) //ѭ��8����λ 
	{     
	      SCLK = 0; 
	      temp = addr; 
	      DIO = (bit)(temp&0x01);          //ÿ�δ�����ֽ� 
		  Delay_us(1);
	      addr >>= 1;                //����һλ 
	      SCLK = 1; 
	} 
	//�������� 
	for ( i=8; i>0; i-- ) 
	{     
	      SCLK = 0; 
	      temp = dat; 
	      DIO = (bit)(temp&0x01);          
		  Delay_us(1);
	      dat >>= 1;                   
	      SCLK = 1; 
	} 
	CE = 0;         
} 


//���ݶ�ȡ�ӳ��� 
unsigned char Read1302 ( unsigned char Index) 
{ 
	unsigned char i,temp,temp1,addr;

	do
	{
		temp1=temp;
		addr=Index;
		
		CE=0;           
		SCLK=0;
		Delay_us(4);
		CE = 1;  
		Delay_us(4);
       //���͵�ַ 
		for ( i=8; i>0; i-- )                      //ѭ��8����λ 
		{     
		      SCLK = 0; 
			  DIO = (bit)(addr&0x01);          //ÿ�δ�����ֽ�
			  Delay_us(1);
		      addr >>= 1;                              //����һλ
		      SCLK = 1; 
		} 

//��ȡ����

//	DIO =1;

		for ( i=8; i>0; i-- ) 
		{ 
		      SCLK = 1; 
			  temp>>=1;
		      SCLK = 0;
			  Delay_us(1);
			  if(DIO)temp|=0x80;
		}     
		CE=0;

	}while(temp!=temp1);
 
	return (temp); 
}



//��ʼ��DS1302 
void Initial(void)    
{ 
   Write1302 (WRITE_PROTECT,0X00);       	//��ֹд���� 

   Write1302 (WRITE_YEAR,DATE[0]);      		//��λ��ʼ�� 
   Write1302 (WRITE_MONTH,DATE[1]);        	//���ӳ�ʼ�� 
   Write1302 (WRITE_DATE,DATE[2]);      		//��λ��ʼ�� 
   Write1302 (WRITE_HOUR,DATE[3]);          	//Сʱ��ʼ��
   Write1302 (WRITE_MINUTE,DATE[4]);        	//���ӳ�ʼ�� 
   Write1302 (WRITE_SECOND,0x00);      		//��λ��ʼ�� 

   Write1302 (WRITE_POWER,0xAA);          	//2�������ܣ�4ǧŷ
   Write1302 (WRITE_PROTECT,0x80);         	//����д���� 
} 



//����
unsigned int t_int1;
unsigned char t_char;

#define	CMD_IDLE	0	//����ģʽ
#define	CMD_READ	1	//IAP��
#define	CMD_PROGRAM	2	//IAPд
#define	CMD_ERASE	3	//IAP����

#define	ENABLE_IAP	0X81	//SYSCLK<24M

void IapIdle()
{
	IAP_CONTR=0;
	IAP_CMD=0;
	IAP_TRIG=0;
	IAP_ADDRH=0x80;
	IAP_ADDRL=0;
}
void WriteFlash(unsigned int IspAddr,unsigned char IspData)
{
	IAP_CONTR=ENABLE_IAP;
	IAP_CMD=CMD_PROGRAM;
	IAP_ADDRL=IspAddr%0x100;
	IAP_ADDRH=IspAddr/0x100;
	IAP_DATA=0xff-IspData;
	IAP_TRIG=0x5A;
	IAP_TRIG=0xA5;
	_nop_();
	IapIdle();
}
unsigned char ReadFlash(unsigned int IspAddr)
{
	unsigned char i;
	IAP_CONTR=ENABLE_IAP;
	IAP_CMD=CMD_READ;
	IAP_ADDRL=IspAddr%0x100;
	IAP_ADDRH=IspAddr/0x100;
	IAP_TRIG=0x5A;
	IAP_TRIG=0xA5;
	_nop_();
	i=0xff-IAP_DATA;
	IapIdle();

	return i;
}
void EraseFlash(unsigned int IspAddr)
{
	IAP_CONTR=ENABLE_IAP;
	IAP_CMD=CMD_ERASE;
	IAP_ADDRL=IspAddr%0x100;
	IAP_ADDRH=IspAddr/0x100;
	IAP_TRIG=0x5A;
	IAP_TRIG=0xA5;
	_nop_();
	IapIdle();
}

unsigned int crc_cal_value(unsigned char data_value,unsigned int crc_value)
{
	unsigned char i;
	union{
			unsigned char Addr[2];
		    unsigned int crc_value;
		}temp;

	temp.crc_value=crc_value;
	temp.crc_value^=data_value;

	for(i=0;i<8;i++)
	{
		if(temp.crc_value&0x0001)
		{
			temp.crc_value = temp.crc_value>>1;
			temp.Addr[0]^=0xa0;
			temp.Addr[1]^=0x01;
		}
		else temp.crc_value=temp.crc_value>>1;
	}
	return(temp.crc_value);
}

unsigned char xdata LedStr[3];
bit Flag_CheckID;
bit Flag_Timer0;		//���ڿ���ʱ����ʱ�䣬ÿ�ζ�ʱ�ж���1
unsigned char code ID2 _at_ 	0x0020;
unsigned char code ID1 _at_ 	0x0021;
unsigned char code ID0 _at_ 	0x0022;

unsigned char code EndYear _at_ 0x8000;//�����iap��д����ַΪ0x0000
unsigned char code EndMon  _at_ 0x8001;
unsigned char code EndDay  _at_ 0x8002;

unsigned char code EndYear1 _at_ 0x8200;//�����iap��д����ַΪ0x0000
unsigned char code EndMon1  _at_ 0x8201;
unsigned char code EndDay1  _at_ 0x8202;

unsigned char code EndYear2 _at_ 0x8400;//�����iap��д����ַΪ0x0000
unsigned char code EndMon2  _at_ 0x8401;
unsigned char code EndDay2  _at_ 0x8402;

#define TB_NowTime		0xe9
#define TB_EndTime		0xed

unsigned char Sint_value=0;
unsigned char Sint_value1=0;
unsigned char Sint_count=0;

void RstPower()
{
	unsigned char t;
	if(RI)//�ڳ������й�����Ҳ������ȨУ��ʱ��
	{
		RI=0;
		t=SBUF;
		if(t==TB_NowTime)//Уʱͬ��
		{
			if(Sint_value==TB_NowTime)Sint_count++;
			else Sint_count=0;
	 	}
		if(t==TB_EndTime)//��Ȩͬ��
		{
			if(Sint_value==TB_EndTime)Sint_count++;
			else Sint_count=0;
	 	}
		if(Sint_value1==0xEB)
		{
			if(Sint_value<=0x20)
			{
				TI=0;
				SBUF=ReadFlash(Sint_value*256+t);
				while(!TI);
			}
			else if(Sint_value<0x80){;}
			else if(Sint_value<=0x90)
			{
				Sint_value=Sint_value|0x01;//���������д��ǿ�Ƹ�Ϊ��
				TI=0;
				SBUF=Read1302(Sint_value);
				while(!TI){};
			}
		}
		Sint_value1=Sint_value;
		Sint_value=t;
		if(Sint_count>10)IAP_CONTR=0x20;//��������
 	}
}


void CheckID()
{
	unsigned int i,j;
	unsigned char t,t0;
	unsigned int crc_value;
	unsigned char BZ;//=0:���ID��=1:��������ID
	
	union{
			unsigned char Addr[4];
		    unsigned long value;
		}EndDate;
	
	union{				   
			unsigned char Addr[4];
		    unsigned long value;
		}MeID;
	
	union{
			unsigned char Addr[4];
		    unsigned long value;
		}AcceptDat;

	MeID.Addr[0]=0;
	MeID.Addr[1]=ID2;
	MeID.Addr[2]=ID1;
	MeID.Addr[3]=ID0;
	AcceptDat.Addr[0]=0;
	BZ=0;
	Flag_CheckID=1;
	EA=1;
//����Ƿ����µ���Ȩ
//���ʱ���Ƿ����������ʱ���Ƿ�ͣ�ڣ����ͣ�ڱ�E01��ͬʱ��ʾA01������ֹ����
//���ʱ���Ƿ����������ʱ�ӳ�����ã��������E01��ͬʱ��ʾA03������ֹ����
//����Ƿ���Ȩ���ڣ�������ڱ�E01��ͬʱ��ʾA01������ֹ����
//ʱ��У��

	RI=0;Flag_Timer0=0;i=0;

	//�ȴ���Ȩָ������л���û�У�ȷ��û�к�����Ȩ״̬
	while((i<200))//500*200us=100ms,14ms��10%�Ļ������ͨ�ţ�16ms��80%�������ͨ�ţ�18ms��16ms��࣬20ms����100%�����ȡ40ms
	{
		if(Flag_Timer0)	//�������ֻ���ڼ����Ȩʱ��λ,������ʵ�������������ʱ,�ϵ�ʱ����Ƿ�Ҫ��Ȩ
		{
			i++;
			Flag_Timer0=0;
		}
		if(RI)	  		//������ڽ��յ�����
		{
			RI=0;
			t=SBUF;		//���յ������ݴ���t������
			if(t==TB_EndTime) //�յ�ͬ��ͷΪ 0xed
			{
				while(t==TB_EndTime)//����ͬ���룬ͬʱ�ȴ���λ��Ϣ
				{
					TI=0;
					SBUF=0xaa;	  //����0XAA
					while(!RI){;}
					RI=0;
					t0=SBUF;	  //�¸����ݴ���t0������
					t=t0;		  //��t0�е����ݸ�tȷ��t0һֱ�������յ���ֵ
				}

				//��ʼ��������,����CRC����
				crc_value=0xffff;
				crc_value=crc_cal_value(t,crc_value);//�����

				for(i=0;i<3;i++)//������Ȩ����
				{
					while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[3]=t;crc_value=crc_cal_value(t,crc_value);
					while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[2]=t;crc_value=crc_cal_value(t,crc_value);
					while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[1]=t;crc_value=crc_cal_value(t,crc_value);
				}
				EndDate.value=AcceptDat.value;

				for(i=0;i<1000;i++)//����ID
				{
					while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[1]=t;crc_value=crc_cal_value(t,crc_value);
					while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[2]=t;crc_value=crc_cal_value(t,crc_value);
					while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[3]=t;crc_value=crc_cal_value(t,crc_value);
					if(AcceptDat.value==MeID.value)BZ=1;
				}

				//����crc
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[1]=t;crc_value=crc_cal_value(t,crc_value);
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[1]=t;crc_value=crc_cal_value(t,crc_value);

				if(crc_value==0)  //crc�޴���
				{
					if(BZ==1)	 //id������Ȩ��Χ��
					{
						t=0x55;	//��Ϣ�������
					//������Ȩʱ��
						for(i=0;i<3;i++)
						{
							BZ=0;
							while(!BZ)
							{
								EraseFlash(i*512);	//��1,2,3�����洢��Ȩ����ֹʱ��
								WriteFlash(i*512+0,EndDate.Addr[1]);
								WriteFlash(i*512+1,EndDate.Addr[2]);
								WriteFlash(i*512+2,EndDate.Addr[3]);
								BZ=(ReadFlash(i*512+0)==EndDate.Addr[1])&(ReadFlash(i*512+1)==EndDate.Addr[2])&(ReadFlash(i*512+2)==EndDate.Addr[3]);
							}
						}
						
					}
					else t=0xe1;
			   	}
				else t=0xee;

				for(i=0;i<5;i++)
				{
					TI=0;
					SBUF=t;
					while(!TI);
				}

			}
			else if(t==TB_NowTime)
			{
				while(t==TB_NowTime)//����ͬ���룬ͬʱ�ȴ���λ��Ϣ
				{
					TI=0;
					SBUF=0xa5;
					while(!RI){};
					RI=0;
					t0=SBUF;
					t=t0;
				}

				//��ʼ��������,����CRC����
				crc_value = 0xffff;
				crc_value = crc_cal_value(t,crc_value);//�����

				//�������ʱ
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[3]=t;crc_value=crc_cal_value(t,crc_value);
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[2]=t;crc_value=crc_cal_value(t,crc_value);
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[1]=t;crc_value=crc_cal_value(t,crc_value);
				EndDate.value=AcceptDat.value;//���üĴ���
				
				//����������
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[3]=t;crc_value=crc_cal_value(t,crc_value);
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[2]=t;crc_value=crc_cal_value(t,crc_value);
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[1]=t;crc_value=crc_cal_value(t,crc_value);

				//����crc
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;crc_value=crc_cal_value(t,crc_value);
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;crc_value=crc_cal_value(t,crc_value);

				if(crc_value==0)
				{
					//����ʱ��
					BZ=0;
					while(!BZ)
					{
				     	Write1302 (WRITE_PROTECT,0X00);       				//��ֹд���� 
				       	Write1302 (WRITE_SECOND,EndDate.Addr[3]);      		//��λ��ʼ�� 
				       	Write1302 (WRITE_MINUTE,EndDate.Addr[2]);        	//���ӳ�ʼ�� 
				       	Write1302 (WRITE_HOUR,EndDate.Addr[1]);          	//Сʱ��ʼ��
				       	EndDate.value=AcceptDat.value;						//���üĴ���
						Write1302 (WRITE_DATE,EndDate.Addr[3]);      		//��λ��ʼ�� 
				       	Write1302 (WRITE_MONTH,EndDate.Addr[2]);        	//���ӳ�ʼ�� 
				       	Write1302 (WRITE_YEAR,EndDate.Addr[1]);          	//��ʱ��ʼ��
				       	Write1302 (WRITE_POWER,0xAA);          				//2�������ܣ�4ǧŷ
				       	Write1302 (WRITE_PROTECT,0x80);         			//����д���� 
	
						BZ=(Read1302(READ_DATE)==EndDate.Addr[3])&(Read1302(READ_MONTH)==EndDate.Addr[2])&(Read1302(READ_YEAR)==EndDate.Addr[1]);
						//BZ=1;
					}
					EraseFlash(8*512);
					WriteFlash(8*512,0);//ÿ��Уʱ��ʱ���Զ��޸��޴ι�0
					
					IN1=1;
					LED1 =1;
	
					t=0x55;
			   	}
				else 	t=0xee;
				for(i=0;i<5;i++)
				{
					TI=0;
					SBUF=t;
					while(!TI);
				}
			}
		}
	}


//���ʱ���Ƿ����������ʱ���Ƿ�ͣ�ڣ����ͣ�ڱ�E01��ͬʱ��ʾA01������ֹ����
	
	do
	{
		t=Read1302(READ_PROTECT);
	}while(t&0x7f);//ȷ��1302��0���������0x80��0x00
	
	t=Read1302(READ_SECOND);
	if(t&0x80)
	{
		t0=0;
		//�����޸�
		BZ=0;
		i=4;j=5;
		if((ReadFlash(i*512+0)==ReadFlash(j*512+0))&(ReadFlash(i*512+1)==ReadFlash(j*512+1))&(ReadFlash(i*512+2)==ReadFlash(j*512+2))&(ReadFlash(i*512+3)==ReadFlash(j*512+3))&(ReadFlash(i*512+4)==ReadFlash(j*512+4))&(ReadFlash(i*512+5)==ReadFlash(j*512+5))&!BZ)
		{DATE[0]=ReadFlash(i*512+0);DATE[1]=ReadFlash(i*512+1);DATE[2]=ReadFlash(i*512+2);DATE[3]=ReadFlash(i*512+3);DATE[4]=ReadFlash(i*512+4);DATE[5]=ReadFlash(i*512+5);BZ=1;}
		i=4;j=6;
		if((ReadFlash(i*512+0)==ReadFlash(j*512+0))&(ReadFlash(i*512+1)==ReadFlash(j*512+1))&(ReadFlash(i*512+2)==ReadFlash(j*512+2))&(ReadFlash(i*512+3)==ReadFlash(j*512+3))&(ReadFlash(i*512+4)==ReadFlash(j*512+4))&(ReadFlash(i*512+5)==ReadFlash(j*512+5))&!BZ)
		{DATE[0]=ReadFlash(i*512+0);DATE[1]=ReadFlash(i*512+1);DATE[2]=ReadFlash(i*512+2);DATE[3]=ReadFlash(i*512+3);DATE[4]=ReadFlash(i*512+4);DATE[5]=ReadFlash(i*512+5);BZ=1;}
		i=5;j=6;
		if((ReadFlash(i*512+0)==ReadFlash(j*512+0))&(ReadFlash(i*512+1)==ReadFlash(j*512+1))&(ReadFlash(i*512+2)==ReadFlash(j*512+2))&(ReadFlash(i*512+3)==ReadFlash(j*512+3))&(ReadFlash(i*512+4)==ReadFlash(j*512+4))&(ReadFlash(i*512+5)==ReadFlash(j*512+5))&!BZ)
		{DATE[0]=ReadFlash(i*512+0);DATE[1]=ReadFlash(i*512+1);DATE[2]=ReadFlash(i*512+2);DATE[3]=ReadFlash(i*512+3);DATE[4]=ReadFlash(i*512+4);DATE[5]=ReadFlash(i*512+5);BZ=1;}

		i=DATE[0];j=i%16;i=i/16;i=i*10+j;if(i>99)BZ=0;
		i=DATE[1];j=i%16;i=i/16;i=i*10+j;if((i==0)|(i>12))BZ=0;
		i=DATE[2];j=i%16;i=i/16;i=i*10+j;if((i==0)|(i>31))BZ=0;

		i=ReadFlash(8*512+0);
		if(i>10)
		{
			BZ=0;
			IN1=0;
			LED1 =0;
		}//�޸�ʱ������Ϊ10��
		else 
		{
			EraseFlash(8*512);
			WriteFlash(8*512,i+1);
			IN1=1;
			LED1 =1;
		}
		if(BZ)Initial();
		else
		{
			while(1)
			{
				//ErrorState=Error+1;Verify();S2CON&=~S2TI;S2BUF=ErrorState;while(!(S2CON&S2TI)){}
				DrvRun=1;//��ֹ����
				LED1 = 0;
				IN1 =0;
				RstPower();

				//����һ��50000*200us=10�������
				while(!Flag_Timer0){}
				i++;
				Flag_Timer0=0;
				if(i>50000)i=0;

				//20ms��B�巢״̬			
				t=i%100;
				if(t==1){ErrorState=Error+1;Verify();S2CON&=~S2TI;S2BUF=ErrorState;}

				//��ʾ��ַ�͹��ϴ���
				t=i/10000;ShowTimer=7;
				if(t==2){t0=ID2;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
				if(t==3){t0=ID1;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
				if(t==4){t0=ID0;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
				if(t<2){LedStr[0]=10;LedStr[1]=0;LedStr[2]=1;}//=7��ʾbcd7�α��룬=8ֱ����ʾ����
			}
		}
	}

//���ʱ���Ƿ����������ʱ�ӳ�����ã��������E01��ͬʱ��ʾA03������ֹ����
	t=Read1302(READ_POWER);
	if(t!=0xAA)
	{
		t0=0;
		while(1)
		{
			//ErrorState=Error+1;Verify();S2CON&=~S2TI;S2BUF=ErrorState;while(!(S2CON&S2TI)){}
			DrvRun=1;//��ֹ����
			LED1 = 0;
			IN1 =0;
			RstPower();

			//����һ��50000*200us=10�������
			while(!Flag_Timer0){}
			i++;Flag_Timer0=0;
			if(i>50000)i=0;

			//20ms��B�巢״̬
			t=i%100;
			if(t==1){ErrorState=Error+1;Verify();S2CON&=~S2TI;S2BUF=ErrorState;}

			//��ʾ��ַ�͹��ϴ���
			t=i/10000;ShowTimer=7;
			if(t==2){t0=ID2;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
			if(t==3){t0=ID1;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
			if(t==4){t0=ID0;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
			if(t<2){LedStr[0]=10;LedStr[1]=0;LedStr[2]=3;}//=7��ʾbcd7�α��룬=8ֱ����ʾ����
		}
	}

//����Ƿ���Ȩ���ڣ�������ڱ�E01��ͬʱ��ʾA02������ֹ����
	EndDate.Addr[0]=0;
	EndDate.Addr[1]=~EndYear;
	EndDate.Addr[2]=~EndMon;
	EndDate.Addr[3]=~EndDay;

	MeID.Addr[0]=0;
	MeID.Addr[1]=~EndYear1;
	MeID.Addr[2]=~EndMon1;
	MeID.Addr[3]=~EndDay1;
	if(EndDate.value!=MeID.value)
	{
		EndDate=MeID;
		MeID.Addr[0]=0;
		MeID.Addr[1]=~EndYear2;
		MeID.Addr[2]=~EndMon2;
		MeID.Addr[3]=~EndDay2;

		if(EndDate.value!=MeID.value)
		{
			EndDate=MeID;
			MeID.Addr[0]=0;
			MeID.Addr[1]=~EndYear;
			MeID.Addr[2]=~EndMon;
			MeID.Addr[3]=~EndDay;

			if(EndDate.value!=MeID.value)EndDate.value=0;
		}
	}

	AcceptDat.Addr[0]=0;AcceptDat.Addr[1]=Read1302(READ_YEAR);AcceptDat.Addr[2]=Read1302(READ_MONTH);AcceptDat.Addr[3]=Read1302(READ_DATE);
	if(AcceptDat.value>EndDate.value)
	{
		while(1)
		{
			//ErrorState=Error+1;Verify();S2CON&=~S2TI;S2BUF=ErrorState;while(!(S2CON&S2TI)){}
			DrvRun=1;//��ֹ����
			LED1 = 0;
			IN1 =0;
			RstPower();

			//����һ��50000*200us=10�������
			while(!Flag_Timer0){}
			i++;
			Flag_Timer0=0;
			if(i>50000)i=0;

			//20ms��B�巢״̬
			t=i%100;
			if(t==1){ErrorState=Error+1;Verify();S2CON&=~S2TI;S2BUF=ErrorState;}

			//��ʾ��ַ�͹��ϴ���
			t=i/10000;ShowTimer=7;
			if(t==2){t0=ID2;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
			if(t==3){t0=ID1;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
			if(t==4){t0=ID0;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
			if(t<2){LedStr[0]=10;LedStr[1]=0;LedStr[2]=2;}//=7��ʾbcd7�α��룬=8ֱ����ʾ����
		}
	}

//�洢ʱ�ӵ�������ʱ���룬�Ա����ݶ�ʧ�������лظ�ʹ��
	DATE[0]=Read1302(READ_YEAR);
	DATE[1]=Read1302(READ_MONTH);
	DATE[2]=Read1302(READ_DATE);
	DATE[3]=Read1302(READ_HOUR);
	DATE[4]=Read1302(READ_MINUTE);
	DATE[5]=Read1302(READ_SECOND);

	for(i=4;i<6;i++)
	{
		BZ=0;
		while(!BZ)
		{
			EraseFlash(i*512);
			WriteFlash(i*512+0,DATE[0]);
			WriteFlash(i*512+1,DATE[1]);
			WriteFlash(i*512+2,DATE[2]);
			WriteFlash(i*512+3,DATE[3]);
			WriteFlash(i*512+4,DATE[4]);
			WriteFlash(i*512+5,DATE[5]);
			BZ=(ReadFlash(i*512+0)==DATE[0])&(ReadFlash(i*512+1)==DATE[1])&(ReadFlash(i*512+2)==DATE[2])&(ReadFlash(i*512+3)==DATE[3])&(ReadFlash(i*512+4)==DATE[4])&(ReadFlash(i*512+5)==DATE[5]);
		}
	}

	Flag_CheckID=0;
}

unsigned char Line;

void LedWr(unsigned char Index)
{
	DA=Index&0x01;
	DB=Index&0x02;
	DC=Index&0x04;
	DD=Index&0x08;
	DE=Index&0x10;
	DF=Index&0x20;
	DG=Index&0x40;
}

void ShowID()//ShowTimer=0,1��ʾ4��ID2��=2��ʾ2��ID1��=3��ʾ2��ID0��
{
	unsigned int i;
	unsigned char t0,t1,t2;
	switch(ShowTimer)
	{
		case 2:i=ID1;t2=1;break;
		case 3:i=ID0;t2=2;break;
		default:i=ID2;t2=0;
	}

	t0=i%16;i=i/16;
	t1=i;


}

void ShowLoad()//��ʾ����ʵ���������׼���ݵĲ�ֵ������ֵ3λ������ʾ����
{
	unsigned int i;
	unsigned char t0,t1,t2;

	RL1--;
	if(!RL1)
	{
		RL1=100;//100*1.6ms=160ms
		if(OverLoad<=OverLoadLimite1)
		{
			RL2=(RL2*2)%64;
			if(RL2==0)RL2=1;
			t2=RL2;
		}else
		{
			RL2=RL2/2;
			if(RL2==0)RL2=32;
			t2=RL2;
		}
	}

	if(OverLoad<=OverLoadLimite1)
	{
		i=(OverLoadLimite1-OverLoad);
		if(i>199)
		{
			t0=i%10;i=i/10;
			t1=i%10;i=i/10;
		}else
		{
			t0=i%10;i=i/10;
			t1=i%10;i=i/10;
			t2=Bcd7Seg[i%10];
		}
   	}
	else
    {
		i=(OverLoad-OverLoadLimite1);
		if(i>199)
		{
			t0=i%10;i=i/10;
			t1=i%10;i=i/10;
		}else if(i>99)
		{
			t0=i%10;i=i/10;
			t1=i%10;i=i/10;
			t2=0x46;//-1
		}else
		{
			t0=i%10;i=i/10;
			t1=i%10;i=i/10;
			t2=0x40;//����-
		}		
	}

}

void ShowErrorState()
{
	unsigned int i;
	unsigned char t0,t1,t2;

	i=ErrorState%32;
	t0=i%10;i=i/10;
	t1=i%10;
	i=ErrorState/32;
	t2=i%4;

}

void ShowSec()
{
	unsigned char t0,t1,t2;
    //t2=Read1302 (READ_SECOND);
	t0=t2%16;
	t1=t2/16;
}

void ShowBcd()
{;
}

void ShowStr()
{;
}


void LedShow()
{
//ShowTimer==0,1,2,3;������ʾID,��ʾ���̣�3-,D1,D2,D3
//ShowTimer==4;Ȼ����ʾ�������ݣ�Ϊ���ر궨�ṩ����
//ShowTimer==5;���к���ʾ����״̬

	switch(ShowTimer)
	{
		case 3:ShowID();break;
		case 4:ShowLoad();break;
		case 5:ShowErrorState();break;
		case 6:ShowSec();break;
		case 7:ShowBcd();break;
		default:ShowStr();
	}
}

//A/Dת��׼��
void AdReady(unsigned char INDEX)
{
	if(INDEX==0)ADC_CONTR=0xe3;//A�����
	else if(INDEX==1)ADC_CONTR=0xe4;//B�����
	else if(INDEX==2)ADC_CONTR=0xe5;//C�����
	else if(INDEX==3)ADC_CONTR=0xe2;//���ز���
}

void AdStart()
{
	ADC_RES=0;
	ADC_RESL=0;
	ADC_CONTR=ADC_CONTR|0x08;
}

unsigned int GetAdResult()
{
	while(!(ADC_CONTR&0x10)){};
	ADC_CONTR=ADC_CONTR&0xe7;

	return ADC_RES*4+ADC_RESL%4;  //<<2��2�η� ������λ
}

sbit To1	=P5^2;
sbit To2	=P0^4;
sbit To3	=P0^3;
sbit To7	=P2^7;
sbit To8	=P7^4;
sbit To9	=P7^5;
sbit To10	=P7^6;
sbit To11	=P7^7;
sbit To12	=P4^5;
sbit To13	=P4^6;
sbit To14	=P0^0;
sbit To15	=P0^1;
sbit To16	=P0^2;

void StateCheck(void)//״̬���
{
//���ݶ�ȡ
//�����24V��DatOut=0,State[]!=0
//��ȡ˳���е�·�����
	
	if(!To1){if(State[1]<200)State[1]++;}else{if(State[1]>0)State[1]--;}
	if(!To2){if(State[2]<200)State[2]++;}else{if(State[2]>0)State[2]--;}
	if(!To3){if(State[3]<200)State[3]++;}else{if(State[3]>0)State[3]--;}
	if(!To7){if(State[7]<200)State[7]++;}else{if(State[7]>0)State[7]--;}
	if(!To8){if(State[8]<200)State[8]++;}else{if(State[8]>0)State[8]--;}
	if(!To9){if(State[9]<200)State[9]++;}else{if(State[9]>0)State[9]--;}
	if(!To10){if(State[10]<200)State[10]++;}else{if(State[10]>0)State[10]--;} //����
	if(!To11){if(State[11]<200)State[11]++;}else{if(State[11]>0)State[11]--;}
	if(!To12){if(State[12]<200)State[12]++;}else{if(State[12]>0)State[12]--;}
	if(!To13){if(State[13]<200)State[13]++;}else{if(State[13]>0)State[13]--;}
	if(!To14){if(State[14]<200)State[14]++;}else{if(State[14]>0)State[14]--;} //�½�
	if(!To15){if(State[15]<200)State[15]++;}else{if(State[15]>0)State[15]--;}
	if(!To16){if(State[16]<200)State[16]++;}else{if(State[16]>0)State[16]--;}
}

bit PhaseErrorCheck()
{
/*
A��:00,01,02,03,04,05,06,07,  08,09,10,11,12,13,14,15,  16,17,18,19,20,21,22,23,
B��:16,17,18,19,20,21,22,23,  00,01,02,03,04,05,06,07,  08,09,10,11,12,13,14,15,
C��:08,09,10,11,12,13,14,15,  16,17,18,19,20,21,22,23,  00,01,02,03,04,05,06,07,
*/
	unsigned char a[3];
	a[0]=PhaseTimer[0];
	a[2]=PhaseTimer[1];
	a[1]=PhaseTimer[2];

	//����7
	if(!FlagErrorPhase)//������
	{
		if((a[0]<40)&(a[1]<40)&(a[2]<40))//3�඼�е���ͨ��;200us*4*40=32ms
		{
			if(a[0]<a[1])//A��:00,01,02,03,04,05,06,07
			{
				if((a[2]>a[0])&(a[2]<a[1]))
				{
					if(PhaseError!=0)PhaseError--;
				}
				else
				{
					/*����*/if(PhaseError<200)PhaseError++;
				}
			}
			else
			{
				if((a[2]>a[0])|(a[2]<a[1]))//A��:08,09,10,11,12,13,14,15,  16,17,18,19,20,21,22,23,
				{
					if(PhaseError!=0)PhaseError--;
				}
				else
				{
					/*����*/if(PhaseError<200)PhaseError++;
				}				
			}
		}
	}
	if(PhaseError>100)//����
	{
		PhaseError=150;
		ErrorState=Alarm+Error+7;//ERROR
		FlagErrorPhase=1;
	}
	
	return FlagErrorPhase;
}

bit PowerCheck()   //zjb
{
//��ѹ���������������

	//if(SecCount>(255-20*1))//25ms*20=500ms
	if(SecCount>(255-100))//25ms*100=2.5s��
	{//����2.5s����⣬�ȴ���·�ȶ�   20130924
	}
	else if((LoadCurrent[0]<LosePower)&(LoadCurrent[1]<LosePower)&(LoadCurrent[2]<LosePower)&(LoadCurrent[3]<LosePower))
	{//δ��⵽��Ч��ѹ������⣬�Ա��ڵ�·����2017-5-25
	}
	else
	{
		if(FlagPowerL)//Ƿѹʱ
		{
			if((LoadCurrent[0]>MinPower)&(LoadCurrent[1]>MinPower)&(LoadCurrent[2]>MinPower))FlagPowerL=0;//��Ƿѹ
			else {FlagPowerL=1;ErrorState=Alarm+Error+21;}
		}else //����ʱ
		{
			if((LoadCurrent[0]<MinPower)|(LoadCurrent[1]<MinPower)|(LoadCurrent[2]<MinPower)){FlagPowerL=1;ErrorState=Alarm+Error+21;}
			else FlagPowerL=0;//��Ƿѹ
		}
	
		if(FlagPowerH)//��ѹʱ
		{
			if((LoadCurrent[0]<MaxPower)&(LoadCurrent[1]<MaxPower)&(LoadCurrent[2]<MaxPower))FlagPowerH=0;//����ѹ
			else {FlagPowerH=1;ErrorState=Alarm+Error+20;}
		}else //����ʱ
		{
			if((LoadCurrent[0]>MaxPower)|(LoadCurrent[1]>MaxPower)|(LoadCurrent[2]>MaxPower)){FlagPowerH=1;ErrorState=Alarm+Error+20;}
			else FlagPowerH=0;//����ѹ
		}
	}

	if(Sec5>1){FlagPowerH=0;FlagPowerL=0;}//2015-08-12

	
	/*ȱ��*/
	//if((((LoadCurrent[0]>MinPower)|(LoadCurrent[1]>MinPower)|(LoadCurrent[2]>MinPower))
	//&((LoadCurrent[0]<LosePower)|(LoadCurrent[1]<LosePower)|(LoadCurrent[2]<LosePower)))//��⵽��Ч��ѹ
	//|(LoadCurrent[3]<MinPower))//L1~N��ѹ����
	if(
	    ((LoadCurrent[0]>MinPower)|(LoadCurrent[1]>MinPower)|(LoadCurrent[2]>MinPower))//��⵽��Ч��ѹ
	   &((LoadCurrent[0]<LosePower)|(LoadCurrent[1]<LosePower)|(LoadCurrent[2]<LosePower)|(LoadCurrent[3]<LosePower))//L1~N��ѹ����
	  )

	{FlagLosePhase=1;ErrorState=Alarm+Error+8;}
	else FlagLosePhase=0;
	/*����*/
	PhaseErrorCheck();
	
	//FlagPower=0;//test ���������ж�
	return FlagPowerH|FlagPowerL|FlagLosePhase|FlagErrorPhase;
}
bit OverLoadCheck()
{
	//����10
	if(OverLoadCount>500)//200us*4*500=400ms
	{
		OverLoad=OverLoad1/OverLoadCount;
		OverLoad1=OverLoad;
//		OverLoad1 = OverLoad1*5000;
//		OverLoad1 = OverLoad1/1024;
		OverLoadCount=1;
		TI=0;		   						//�˶δ����Ӱ�����ͨѶ..����ʱ�ò��ܵ�����ʽ����ʹ��
		SBUF = OverLoad1>>24;
		while(!TI);
		TI=0;
		SBUF = OverLoad1>>16;
		while(!TI);
		TI=0;		   						//�˶δ����Ӱ�����ͨѶ..����ʱ�ò��ܵ�����ʽ����ʹ��
		SBUF = OverLoad1>>8;
		while(!TI);
		TI=0;
		SBUF = OverLoad1;
		while(!TI);
	}
	//���ٲ������췽ʽ����������� 20130503
	
	//if(OverLoad<OverLoadLimite1)FlagOverLoad=0;//������
	if((OverLoad-(GaoDu/GaoDuK))<OverLoadLimite1){FlagOverLoad=0;LED2=1;IN2=1;}//2014/3/19
	else
	{
		IN2 = 0;
		LED2 = 0;
		ErrorState=Alarm+Error+10;//ERROR
		FlagOverLoad=1;
	}
	if(FlagRun)FlagOverLoad=0;//�����н�ʱ���������ؼ�⣬��ֹ��ͣ����
	
	return FlagOverLoad;
}

//2012-10-26
bit MainSwitchCheck()
{
bit sta;
/*
	���⼱ͣ
	���ڼ�ͣ
	�Ž�
	��������λ
	��ȫ��
	�ȼ�
*/

	sta=1;FlagRun=0;	
	if(State[0]<50)		ErrorState=Error+1;//AC24V����
	if(State[1]<50)		ErrorState=Error+1;//Կ�׿���
	if(State[2]<50)		ErrorState=Error+3;//���⼱ͣ3
	else if(State[3]<50)ErrorState=Error+2;	//���ڼ�ͣ2
	else if(State[4]<50)ErrorState=Error+1;	//�̵���6
	else if(State[5]<50)ErrorState=Error+17;//������17
	else if(State[6]<50)ErrorState=Error+16;//�Ž�16
	else if(State[7]<50)ErrorState=Alarm+Error+4;//��������λ4
	else if(State[8]<50)ErrorState=Alarm+Error+18;//��ȫ��18
	else if(State[9]<50)ErrorState=Alarm+Error+9;//�ȼ�9
	else {sta=0;FlagRun=1;}


	return sta;
}

//2012-10-26
bit UpSwitchCheck()
{
	bit sta;
	sta=1;
	if(State[10]>0)//����
	{
		FlagStart=1;
		if(State[11]<50)
		ErrorState=Error+11;//����λ����11
		else if(State[12]<50)
		ErrorState=Alarm+Error+19;//��������λ����19
		else if(State[13]<50)
		ErrorState=Alarm+Error+1;//�̵���2����
		//else {ErrorState=2;FlagRun=1;}//����״̬
		else 
		{
			ErrorState=2;
			FlagRun=1;
			if(GaoDu<60000)
			GaoDu++;
		}//2014/3/19
	}
	else
    {
		sta=0;
		FlagRun=0;
	}
	
	return sta;
}

//2012-10-26
bit DownSwitchCheck()
{
	bit sta;
	sta=1;
	if(State[14]>150)//�½�
	{
		FlagStart=1;
		//if(State[15]<50)ErrorState=Error+12;//����λ����12
		if(State[15]<50)
		{
			ErrorState=Error+12;
			GaoDu=0;
		}//2014/3/19
		else if(State[16]<50)
		ErrorState=Alarm+Error+1;//�̵���1����
		//else {ErrorState=3;FlagRun=1;}//�½�״̬
		else 
		{
			ErrorState=3;
			FlagRun=1;
			if(GaoDu>0)
			GaoDu--;
		}//2014/3/19
	}
	else
    {
		sta=0;
		FlagRun=0;
	}
	
	return sta;
}

void StateShow()
{
	FlagRun=0;
	if(FlagStart)ErrorState=1;//ֹͣ״̬1
	else ErrorState=0;
}

//���Ϸ���
void StateErrorCheck()
{
	if(OverLoadCheck()){}//������ʾ����
	else if(PowerCheck()){}//����ѹ��Ƿ��ѹ��ȱ����������
	else if(FlagStart)//����Ѿ���ʼ����������ʾ�������ϣ�������ʾ����ʱ��
	{
		if(MainSwitchCheck());
		else if(UpSwitchCheck());
		else if(DownSwitchCheck());
		else StateShow();
	}
	else
	{
		if(UpSwitchCheck());
		else if(DownSwitchCheck());
		else StateShow();
	}
	
/*
B7��=B6��B5��B4��B3��B2��B1��B0
B6 B5=0״̬��ʾ
B4 B3 B2 B1 B0=
	0����ʾ�����תʱ��
1������״̬
2������״̬
3���½�״̬
B6 B5=1������ʾ(���澯)
	B4 B3 B2 B1 B0=
1����Դ24�����ϻ��ߵ�·����ϣ�ok
2�����ڼ�ͣ������ok
3�����⼱ͣ������ok
4����������λ������ok
5����������ʧ�ܣ����룩��
6����������ʧ�ܣ���ʱ����ok
7���������
8�����ࣻ
9�������������ȼ̵��� ok
10�����ر�����ok
11������λ������ok
12������λ������ok
16��ǰ��δ�ر� ok
17������δ�ر� ok
18����ȫ������ ok
19����������λ���� ok

20����ѹ����
21����ѹ����
30~31�����Զ˿�1~2�������Զ˿��ϳ���24V��ѹʱ����ʾ���Զ˿����ݣ�ͬʱ���ư�ĺ����˸��
B6 B5=3������ʾ(�澯)����������ͬB6 B5=1
*/

	if(SecCount>(255-20*1))//25ms*20=500ms
	{
		if(FlagErrorPhase)//��֤500ms��������������У��Ա��ж��˿��
		{
			DrvRun=0;
			SecCount=255;//2012-3-24 ��������ֹͣ����
		}

		if(State[14]>150)DrvSl3=0;	//����½����Ͽ���ȫ���̵���2017-05-22
	}
	else
	{
		DrvRun=FlagOverLoad|FlagErrorPhase|FlagLosePhase|OverTimer;//2017-5-25
		DrvSl3=0;	//�Ͽ���ȫ���̵���
	}
}

void Verify()
{
	unsigned char check;
	check=0;
	if(ErrorState&0x01)check++;
	if(ErrorState&0x02)check++;
	if(ErrorState&0x04)check++;
	if(ErrorState&0x08)check++;
	if(ErrorState&0x10)check++;
	if(ErrorState&0x20)check++;
	if(ErrorState&0x40)check++;
	if(check&0x01)ErrorState=ErrorState+0x80;
}

//**********************************************************************2017-5-31
void SxBUF_Init()
{
//����
	TH1=0x100-Crystal/12/16/BaudRate;TL1=0x100-Crystal/12/16/BaudRate;
//1
	SCON=0x58;
	T2H=(65536-Crystal/4/BaudRate)/256;
	T2L=(65536-Crystal/4/BaudRate)%256;
//2
	S2CON=0x58;//S2SM0/NC/S2SM2/S2REN/S2TB8/S2RB8/S2TI/S2RI
//3	
	S3CON=0x98;//S3SM0/S3ST3/S3SM2/S3REN/S3TB8/S3RB8/S3TI/S3RI
//4	
	S4CON=0x98;//S4SM0/S4ST4/S4SM2/S4REN/S4TB8/S4RB8/S4TI/S4RI
   	//�����Ĵ���
	AUXR=0X14;//T0*12/T1*12/UART_M0*6/T2R/T2_CT/T2*12/EXTRAM/S1ST2;S1ST2���ô���1ʹ�ö�ʱ��T2
	AUXR|=0X01;
}

//200us��ʱ�жϳ���
//ʱ��ʱ�����׼ȷ
void timer0(void) interrupt 1 /*T0�ж�*/
{
	TH0=(65536-Crystal/12000*InitTime/1000)/256;
	TL0=(65536-Crystal/12000*InitTime/1000)%256;

	WDT_CONTR=0x3e;//2.72�뿴�Ź�

	Clock_1s++;
	if(Clock_1s>9999)
	{
		if(ShowTimer<4)ShowTimer++;
		Clock_1s=0;
	}

	if(FlagStart)ShowTimer=5;
	
	if(Flag_CheckID)//����ڼ����Ȩ�������в���
	{
		LedShow();
		Flag_Timer0=1;
		goto timer0end;
	}

	AdStart();//ok
	
	StateCheck();//ok
	
	//���ز���ֵȡ����ֵ����ʱ���ڲ���A�࣬�����Ѳ������
	if(LoadIndex==0)//0.8msһ��
	{
		OverLoad1=OverLoad1+LoadDot[3];//������ֵ����
		OverLoadCount++;
		if(DelaySecCount>0)DelaySecCount--;
 		SumCount++;
 		
		CountIndex=SumCount%CheckCycle;
 		if(CountIndex==0)//0.8ms*31=24.8ms
 		{
			StateErrorCheck();
			//2017-5-25
			//start
			if(SumCount>(CheckCycle*40))//800us*31*40=992ms
			{
				LoadCurrent[0]=SumLoadCurrent[0]/SumCount;SumLoadCurrent[0]=0;
				LoadCurrent[1]=SumLoadCurrent[1]/SumCount;SumLoadCurrent[1]=0;
				SumLoadCurrent[2]=SumLoadCurrent[2]/1.06;//ʵ��50hz��1.053�������ģ��50hz��1.053~1.056��60hz��1.064~1.066������ȡ1.06
				//LoadCurrent[2]=SumLoadCurrent[2]/SumCount;SumLoadCurrent[2]=0;
				LoadCurrent[2]=LoadCurrent[1];SumLoadCurrent[2]=0;//���ǵ�����ֱ�Ӳ���������ϴ󣬲���ʹ��20130503
				LoadCurrent[3]=SumLoadCurrent[3]/SumCount;SumLoadCurrent[3]=0;

				SumCount=0;
				Sec5++;
				Sec5=Sec5%5;//2015-08-12
			}

			if(FlagRun)L_Run=(SumCount>(CheckCycle*20));//1������˸
			else L_Run=0;
			
			Verify();
			S2CON&=~S2TI;
			S2BUF=ErrorState;
			//Verify();TI=0;SBUF=OverLoad;//zjbtest

			if(SecCount>0)SecCount--;
			if(SecCount==0)
			{
				SecCount=100;//2.5sec
			}

		}
		else if(CountIndex&0x01)LedShow();//1.6msһ��
	}
	
/*
	LoadIndex==1ʱ��A�������������ʱ���ڲ���B�࣬A���Ѳ������
	LoadIndex==2ʱ��B�������������ʱ���ڲ���C�࣬B���Ѳ������
	LoadIndex==3ʱ��C�������������ʱ���ڲ������أ�C���Ѳ������
	
	������LoadDot[]>(LoadZero+LoadMin)ʱ����ʾ��������������Polarity[]��Ϊ1
	������LoadDot[]>(LoadZero-LoadMin)ʱ����ʾ�и�����������Polarity[]��Ϊ0
	������Polarity[]��0��Ϊ1ʱ��PhaseTimer[]��ʼ��ʱ,
	������Polarity[]��0��Ϊ1ʱ��LoadCurrent[]���㲢�ۼ�,

*/
	//AB���ѹ
	if(LoadIndex==1)
	{
		LoadDot[0]=1024-LoadDot[0];//��������BA��ѹ��תAB
	 	if(LoadDot[0]>LoadZero)
	 	{
	 		SumLoadCurrent[0]=SumLoadCurrent[0]+LoadDot[0]-LoadZero;//2012-11-28
	 	}
		else
	 	{
	 		SumLoadCurrent[0]=SumLoadCurrent[0]+LoadZero-LoadDot[0];//2012-11-28	
	 	}
		
		if(!Polarity0)//���������������
		{
			if(LoadDot[0]>(LoadZero+LoadMin))//����������
			{
				Polarity0=1;
				PhaseTimer[0]=0;
			}
			
		}
		else
		{
			if(LoadDot[0]<(LoadZero-LoadMin))//���븺����
			{
				Polarity0=0;
			}
		}
		
		if(PhaseTimer[0]<199)PhaseTimer[0]++;
	}
	
	//CA���ѹ
	if(LoadIndex==2)
	{
	 	if(LoadDot[1]>LoadZero)
	 	{
	 		SumLoadCurrent[1]=SumLoadCurrent[1]+LoadDot[1]-LoadZero;//2012-11-28
	 	}
	 	else 
	 	{
	 		SumLoadCurrent[1]=SumLoadCurrent[1]+LoadZero-LoadDot[1];//2012-11-28
	 	}
		
		if(!Polarity1)//���������������
		{
			if(LoadDot[1]>(LoadZero+LoadMin))//����������
			{
				Polarity1=1;
				PhaseTimer[1]=0;
				
			}
		}else
		{
			if(LoadDot[1]<(LoadZero-LoadMin))//���븺����
			{
				Polarity1=0;
			}
		}
		
		if(PhaseTimer[1]<199)PhaseTimer[1]++;
	}

	
	//BC���ѹ��A�ߵ�ѹ
	if(LoadIndex==3)
	{
	 	//A�ߵ�ѹ
	 	if(LoadDot[2]>LoadZero)SumLoadCurrent[3]=SumLoadCurrent[3]+LoadDot[2]-LoadZero;//2012-11-28
	 	else SumLoadCurrent[3]=SumLoadCurrent[3]+LoadZero-LoadDot[2];//2012-11-28
	
	 	//BC���ѹ
	 	LoadDot[2]=1024-(LoadDot[0]+LoadDot[1]-LoadZero);
	 	if(LoadDot[2]>LoadZero)
	 	{
	 		SumLoadCurrent[2]=SumLoadCurrent[2]+LoadDot[2]-LoadZero;//2012-11-28
	 	}else 
	 	{
	 		SumLoadCurrent[2]=SumLoadCurrent[2]+LoadZero-LoadDot[2];//2012-11-28
	 	}
		
		if(!Polarity2)//���������������
		{
			if(LoadDot[2]>(LoadZero+LoadMin))//����������
			{
				Polarity2=1;
				PhaseTimer[2]=0;
			}
			
		}else
		{
			if(LoadDot[2]<(LoadZero-LoadMin))//���븺����
			{
				Polarity2=0;
			}
		}
		
		if(PhaseTimer[2]<199)PhaseTimer[2]++;
	}
	
 	LoadDot[LoadIndex]=GetAdResult();

	LoadIndex=(LoadIndex+1)%4;
	AdReady(LoadIndex);

timer0end:;
}
//-------------------�жϺ���--------------------------//
void Int0(void) 	interrupt 0//int0
{while(1){TI=0;SBUF=0x00;while(!TI){};/*Delay(1000)*/;}}
void Int1(void) 	interrupt 2//int1
{while(1){TI=0;SBUF=0x02;while(!TI){};/*Delay(1000)*/;}}
void Timer1(void) 	interrupt 3//timer1
{while(1){TI=0;SBUF=0x03;while(!TI){};/*Delay(1000)*/;}}
void UART1(void) 	interrupt 4//uart ͨ��
{while(1){TI=0;SBUF=0x04;while(!TI){};/*Delay(1000)*/;}}
void ADC(void) 		interrupt 5//adc ģ��ת��
{while(1){TI=0;SBUF=0x05;while(!TI){};/*Delay(1000)*/;}}
void LVC(void) 		interrupt 6//pca ��ѹ���
{while(1){TI=0;SBUF=0x06;while(!TI){};/*Delay(1000)*/;}}
void PCA(void) 		interrupt 7
{while(1){TI=0;SBUF=0x07;while(!TI){};/*Delay(1000)*/;}}
void UART2(void) 	interrupt 8
{while(1){TI=0;SBUF=0x08;while(!TI){};/*Delay(1000)*/;}}
void SPI(void) 		interrupt 9
{while(1){TI=0;SBUF=0x09;while(!TI){};/*Delay(1000)*/;}}
void Int2(void) 	interrupt 10
{while(1){TI=0;SBUF=0x10;while(!TI){};/*Delay(1000)*/;}}
void Int3(void) 	interrupt 11
{while(1){TI=0;SBUF=0x11;while(!TI){};/*Delay(1000)*/;}}
void Timer2(void) 	interrupt 12
{while(1){TI=0;SBUF=0x12;while(!TI){};/*Delay(1000)*/;}}
void zd13(void) 	interrupt 13
{while(1){TI=0;SBUF=0x13;while(!TI){};/*Delay(1000)*/;}}
void zd14(void) 	interrupt 14
{while(1){TI=0;SBUF=0x14;while(!TI){};/*Delay(1000)*/;}}
void zd15(void) 	interrupt 15
{while(1){TI=0;SBUF=0x15;while(!TI){};/*Delay(1000)*/;}}
void Int4(void) 	interrupt 16
{while(1){TI=0;SBUF=0x16;while(!TI){};/*Delay(1000)*/;}}
void S3(void) 		interrupt 17
{while(1){TI=0;SBUF=0x17;while(!TI){};/*Delay(1000)*/;}}
void S4(void) 		interrupt 18
{while(1){TI=0;SBUF=0x18;while(!TI){};/*Delay(1000)*/;}}
void Timer3(void) 	interrupt 19
{while(1){TI=0;SBUF=0x19;while(!TI){};/*Delay(1000)*/;}}
void Timer4(void) 	interrupt 20
{while(1){TI=0;SBUF=0x20;while(!TI){};/*Delay(1000)*/;}}
void Comparator(void) interrupt 21
{while(1){TI=0;SBUF=0x21;while(!TI){};/*Delay(1000)*/;}}
void PWM(void) 		interrupt 22
{while(1){TI=0;SBUF=0x22;while(!TI){};/*Delay(1000)*/;}}
void PWMFD(void) 	interrupt 23
{while(1){TI=0;SBUF=0x23;while(!TI){};/*Delay(1000)*/;}}


//������
void main()
{
//unsigned int ii;
//unsigned char i;
//���Ź�����
/*
	i=WDT_CONTR;
	if(i>127)//������Ź���λ
	{
		while(1){};
	}else WDT_CONTR=0x3e;//2.72�뿴�Ź�
*/

//*****************************************
//Ӳ����ʼ��
	IE=0;
//T0
	TH0=(65536-Crystal/12000*InitTime/1000)/256;
	TL0=(65536-Crystal/12000*InitTime/1000)%256;
	TMOD=0x21;TR0=1;ET0=1;

	SxBUF_Init();

	ADC_CONTR=0x60;//��ת���ٶ���Ϊ���,210��ʱ������
	ADC_CONTR=ADC_CONTR|0x80;//��ת����Դ
	PX0=0;PT0=1;IP2=0;

	ADC_CONTR=0x60;//��ת���ٶ���Ϊ���,210��ʱ������
	ADC_CONTR=ADC_CONTR|0x80;//��ת����Դ
//��P1.3,P1.4,P1.5����ΪA/Dת��ģʽ

/*
M1	M0
0	0	˫��
0	1	���
1	0	����
1	1	��©��AD

O	���
I	����
NC	δ������
N0	û������
*/
	//01234567
    //TO14,TO15,TO16,TO3,TO2,S1,S0,NC
	P0M1=0x00;P0M0=0x00;

	//RXD2,TXD2,LOAD,AD0,AD1,AD2,2.5V,NC
	P1M1=~0x03;P1M0=~0x03;

	//NC,NC,NC,NC,TO4,TO5,TO6,TO7
	P2M1=0x00;P2M0=0x00;

    //RXD,TXD,IN8,IN7,IN6,IN3,IN2,IN1
	P3M1=0x00;P3M0=~0x03;//01����+6��2803

	//NC,DF,DB,DA,NC,TO12,TO13,NC
	P4M1=0x00;P4M0=0x00;

	//IN5,IN4,TO1,NC,NC,NC,NO,NO
	P5M1=0x00;P5M0=0x03;//2��2803+1����ż

	//DG,DE,DD,DC,TO8,TO9,TO10,TO11
	P7M1=0x00;P7M0=0x00;

//test end
	ErrorState	=0;
	DrvUpSta	=0;
	FlagOverLoad	=0;
	FlagErrorPhase	=0;
	FlagLosePhase	=0;
	FlagStart	=0;
	DrvSl3		=1;//������ȫ��
	DrvRun		=0;//��������
	
	OverLoad=0;
	OverLoadCount=1;
	Polarity0=0;
	Polarity1=0;
	Polarity2=0;
	PhaseError=0;
	PhaseTimer[0]=0;
	PhaseTimer[1]=0;
	PhaseTimer[2]=0;
	PhaseLose=0;
	
	CountIndex=0;
	SecCount=255;

	GaoDu=0;

//2017-5-25 start
	OneMin=0;
	FlagOverTimer=0;
	OverTimer=0;

	CheckID();

//2017-5-25 end
	EA=1;	//����ʱ�ض�
	while(1)
	{
	//	IN1 = 0;
	//	IN2 = 0;
	//	IN3=0;
	//	LED2 =0;
	//	LED1= 0;
		LED3 = 0;
		RstPower();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
	}
}

