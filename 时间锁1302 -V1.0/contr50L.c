//zjbtest
/*
*****************************必须使用自动增量模式，起始地址0x22，增量长度3，增量值1，十进制格式
*****************************注意，
*****************************烧录采用18.4320M，
*****************************低电压禁止eeprom操作，
*****************************eeprom填充为FF

ID存储位置 0x20,0x21,0x22
授权有效时间存储位置页码0，1，2，地址0x00(年)，0x01(月),0x02(日)

保存开机时时钟页码4，5，6，地址0x00(年)，0x01(月),0x02(日),0x03(时),0x04(分),0x05(秒)

保存时钟停摆自动修复次数页码8，地址0x00，当大于10次报错

开机授权方式，开机进行0.1秒钟的检测，判断是否存在授权需求

2017-5-31

在以前的程序上修改升级为stc15w4k32s4的新版本程序，以前程序的修改记录保留，但可能已经没有意义
stc15w4k32s4使用头文件方式，不再自己定义

以下是没有1302前的修改记录
2017-5-27
	1\使用外部存储器编译后，一些变量需要初始化
	2\屏蔽了没有太大意义的E31故障
2017-5-25
1\新增按照运行时间来上锁
	只计上升运行时间，
	运行时间用分钟计,使用2个字节,65535分=1092小时，
	运行时间超时时,只限制上升,不限制下降,并只在新开电源时才有效，

2\时间锁最大时间用程序地址修改,程序地址为[0x0010,0x0011],最大时间=[0x0010]*256+[0x0011]
对16进制,可以粗算0x40为一小时,0x80,0xc0,0x100......

3\按照每周使用1次,每次20分钟,上升为10分钟,超过10分钟当次不在计时

4\如果没有特殊要求，最大工作时长设为300小时

2017-05-22
由于客户在安全锁触发状态下，错误使用解锁方式，即上电时按下降按钮（应该按上升按钮然后手动解锁），所以屏蔽掉下降有效

2017-01-04测试将互感器限流电阻由200k增加到600k,同时输出阻抗由330增到1k

2015-08-12
由于使用发电机电压过低报警会覆盖其他故障报警,严重影响客服调试,改电压报警不完全覆盖原有故障信息

2015-04-08
现场发现频繁报电压过低,修改原来360为330伏

2014/3/19
增加一个时间计数器,以下限位触发归零,记录电梯上升和下降的时间,以便粗估升降机的位置,判定过载时刨除电缆的重量和钢丝绳的影响,变量名int GaoDu,另设一个常量(加权系数)GaoDuK

2013-05-03修改
1、不再计算第三相电流
2、过载不再才用推挽方式
3、

2012-12-3测试结果
1、开机报21号错误，然后消失
2、开机没有时间显示————————————有显示，1.2暂不需处理

3、设定的360~440，实测360~400
4、缺相报电压低
5、过载未处理
*/


/*
程序修改记录
2012-10-25 在二代电梯程序基础上制作5代电梯程序，源程序contr204.c
1、继电器控制功能通过
2、照明失败，灯闪烁，考虑三端稳压恒流源

2012-11-28记录
1、电压测试通过
2、照明失败，稳压芯片过热，考虑专用照明恒流源

2012-8-2
断相保护和错相保护时间由250毫秒 增加到2.5秒
程序修改记录
2012-3-24
加入备用门报警
断相保护电流门限由0.6A调到1.8A

编成日期：2009-12-25
完成的调试
1、测试点的采集和顺序整理
2、过载的检测和控制
3、安全锁继电器的控制
4、故障继电器的控制
5、过载的标定
6、主电路的控制
7、上升电路的控制
8、下降电路的控制
9、开机的控制
未完成的调试
1、错相的检测和控制
2、缺相的检测和控制
完成日期：2010-1-11
*/
//库文件
#include <intrins.h>
#include <ABSACC.h>
#include <STC15W4K32S4.h>

//#include <STDIO.H>
//包括的函数

//常数
#define OverLoadLimite	(128*4+6*10)	//过载载重值 0.2mV<==>25kg<==>6
#define OverLoadLimite1	(OverLoadLimite-1)	

#define Crystal 	18432000
#define BaudRate 	19200
#define InitTime 	200//(us)最大256*12/18432000=42.6ms

#define Alarm	 	0x40
#define Error	 	0x20

#define CheckCycle	31//大约25ms 800us*31=24.8ms
/*
#define MaxPower	((440-40)/2.5)//电压上限，440V
//#define MaxPower	(30/3.3)//电压上限，理论上应该=电压/3，可实测是电压/3.3
#define MinPower	((360-40)/2.5)//电压下限  360V
#define LosePower	((110-40)/2.5)//电压下限  110V
*/
#define MaxPower	(440/3.1)//电压上限，实测420V
//#define MinPower	(360/3.1)//电压下限  实测360V
#define MinPower	(330/3.1)//2015-04-08
//#define MinPower	(430/3.1)//2015-08-12//test
#define LosePower	(110/3.1)//电压下限  110V

/*实测结果
380V	80h

388V	87h

400V	7Eh	
360V	6Dh


20121212
第一次测量，其中第一个互感器正装，第二个反装，第三个是计算得出
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

第一次测量，其中第一个互感器正装，第二个正装，第三个是计算得出
	60hz		50hz
460	a7 a3 92	a9 a5 96
400	8f 8c 7d	90 8c 80
340	78 75 69	77 75 6a


得到关系式，数据=(电压-40)/2.5

以上每个数据大概在+1或-1上浮动
*/


//IO位定义
//双向
 
//A/D，这几个定义只是为了说明io位置，实际程序中并不出现
sbit	Load	=P1^2;	//称重传感器，引脚
sbit	AD0		=P1^3;	//A相
sbit	AD1		=P1^4;	//B相
sbit	AD2		=P1^5;	//C相
sbit	AD25	=P1^6;	//互感器基准电压

//输出
sbit	DrvRun	=P3^2;	//故障控制(缺相，错相，过载)
sbit	DrvSl3	=P3^3;	//安全锁控制
sbit	Ss0	=P0^6;	//备用，电路板扩展用
sbit	Ss1	=P0^5;	//备用，电路板扩展用

sbit	L_Run	=P5^0;	//灯
/*595和165，不再使用，修改完程序删除
sbit	Pl	=P2^4;
sbit	DatOut	=P2^5;
sbit	Lsetb	=P2^6;
sbit	Lclk	=P2^7;
sbit	Nc2	=P3^2;	//空
sbit	Nc3	=P3^3;	//空
sbit	Nc4	=P3^4;	//空
sbit	Clk	=P3^5;
sbit	Ldat	=P3^7;

//特殊功能寄存器
sfr	P0M0		=0x93;
sfr	P0M1		=0x94;
sfr	P1M0		=0x91;
sfr	P1M1		=0x92;
sfr	P2M0		=0x95;
sfr	P2M1		=0x96;
sfr	P3M0		=0xb1;
sfr	P3M1		=0xb2;

sfr	ISP_DATA	=0xe2;//ISP/IAP操作数据寄存器
sfr	ISP_ADDRH	=0xe3;//ISP/IAP操作地址寄存器高八位
sfr	ISP_ADDRL	=0xe4;//ISP/IAP操作地址寄存器低八位
sfr	ISP_CMD		=0xe5;//ISP/IAP操作命令寄存器，需命令触发寄存器触发方可生效
				//=0：待机，无操作=1=2=3
				//=1：读
				//=2：写
				//=3：擦除
sfr	ISP_TRIG	=0xe6;//ISP/IAP操作命令触发寄存器
				//在ISPEN(ISP_CONTR.7)=1时，先写入0x46，再写入0xb9，ISP/IAP命令才会生效
sfr	ISP_CONTR	=0xe7;////ISP/IAP控制寄存器

sfr	IPH		=0xb7;

sfr	AUXR		=0x8e;


sfr	WDT_CONTR		=0xe1;//看门狗

*/

//定义全局变量
unsigned char 	DelayStart;	//开机电压检测延时
unsigned char	Sec5;		//一个5秒的循环计数器2015-08-12

unsigned char xdata	State[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	//采集点的状态寄存器
unsigned char Tcount;		//采集计数器

unsigned char 	ErrorState;	//传送的命令字节,字节定义见StateErrorCheck()
unsigned char 	CountIndex;	//定时器计数,保证20ms发送一次指令
unsigned char  	SecCount;	//定时器，用于1秒的计时,25ms减一

unsigned int xdata 	OverLoad=0;
unsigned long xdata	OverLoad1=0;
unsigned int xdata	OverLoadCount=0;

unsigned int xdata	Clock_1s=0;

//位
bit DrvUpSta;		//?
bit FlagPowerL;		//用以检测电压故障 欠压
bit FlagPowerH;		//用以检测电压故障 过压
bit FlagOverLoad;	//用以检测过载故障
bit FlagErrorPhase;	//用以检测错相故障
bit FlagLosePhase;	//用以检测缺相故障
bit FlagStart;		//用以标示开机=0显示时间，=1显示状态

bit FlagRun;					//状态标示 =1运行状态 =0空闲状态 用以保障运行中不对过载测量，避免意外停机
bit FlagReadCurrent;			//=1读电流操作

#define LoadZero 	(128*4)		//0电流值
//#define LoadMin 	(16*4)		//检测电流阀值 相当于20mA*16*4=1.2A
//#define PhaseLoseMax	0x60	//缺相衡量有电流基准，相当于20mA*0x60/2=0.9A
//#define PhaseLoseMin	0x40	//缺相衡量无电流基准，相当于20mA*0x40/2=0.6A

#define LoadMin 	64		//检测电流阀值 相当于20mA*16*4=1.2A
#define PhaseLoseMax	200	//缺相衡量有电流基准，相当于20mA*200/2=2A
#define PhaseLoseMin	180	//缺相衡量无电流基准，相当于20mA*180/2=1.8A

/*
	原始数据：	ta12-200 5A==>2.5mA
	最大量程：	2.5mA*470/5V*1024=240
	分辨率：	5A/240=20mA
*/
unsigned int xdata 	DelaySecCount=0;//延时计数器
//电流检测

unsigned int xdata	LoadDot[4]={0,0,0,0};	//任一时刻测量到的电流
//unsigned int 	CurrentMax[3];				//电流波峰
//unsigned int 	CurrentMin[3];				//电流波谷

//0~2是ABC三相相间电压，3是A线电压
unsigned int xdata	LoadCurrent[4]={0,0,0,0};//电压均值=SumLoadCurrent[3]/SumCount
unsigned long xdata	SumLoadCurrent[4]={0,0,0,0};

unsigned int	SumCount;
unsigned char	PhaseLose;		//缺相错误计数器，至少10个周期才判断

bit				Polarity0;		//a相线当前极性
bit				Polarity1;		//b相线当前极性
bit				Polarity2;		//c相线当前极性
unsigned char	PhaseTimer[3];	//相序计时
unsigned char	PhaseError;		//相序错误计数器，至少10个周期才判断
unsigned char 	LoadIndex;		//设备工作电流测量索引


/*2014/3/19
假定升降机运行10分钟加重50公斤
10分钟=600秒=600000毫秒
GaoDu每25毫秒计数一次,计数值=600000毫秒/25毫秒=24000
而25公斤,数模转换值是6(这个值是否准确未知,先假定准确)
50公斤就是12,24000/12=2000,这就是GaoDuK的值
*/
unsigned int	GaoDu;//2014/3/19
#define GaoDuK	2000//2014/3/19

//2017-5-25 start
unsigned char 	OldErrorState;	//
bit 			FlagOverTimer;	//上电时记录超时状态
bit				OverTimer;		//超时后有上升操作报错
unsigned int	OneMin;
unsigned int	OldWorkTime;	//设备工作时间
unsigned int	WorkTime;		//设备工作时间
unsigned int	Pc_WorkTime;	//工作时间地址偏移量 
//#define F_WorkTime	0x2800//stc12c5410ad
#define F_WorkTime	0x0000//stc15w4k32s4，在stc15系列，eeprom不在偏移（硬件偏移，stc15w4k32s4偏移为0x8000，如果使用movc指令，需要考虑偏移量）
//#define Max_WorkTime (x*60+y)//x小时又y分钟
#define Max_WorkTime (300*60+0)//最大时长300*2=600小时
//#define Max_WorkTime (1*60+0)//设定1小时
unsigned char	WorkTimeStart;//本次开机时间
//2017-5-25 end

unsigned char RL1,RL2;//电位器调整显示动画2017-8-22

//ds1302程序
//寄存器宏定义 
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

unsigned char xdata DATE[7];	//0年1月2日3时4分5秒 

sbit 	SCLK 	= P2^1;			//DS1302时钟信号
sbit 	DIO		= P2^2;			// DS1302数据信号
sbit 	CE 		= P2^3;			// DS1302片选 

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
	0x63,// 20>>o上
	0x5C,// 21>>o下
	0x73,// 22>>p
	0x67,// 23>>q
	0x50,// 24>>r
	0x3E,// 25>>U
	0x40,// 26>>-
	0x23,// 27>>上升
	0x1C,// 28>>下降
	0x00,// 29>>空格
	0x00,// 30>>空格
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
	0xC6,// 20>>o上
	0x3A,// 21>>o下
	0xCE,// 22>>p
	0xE6,// 23>>q
	0x0A,// 24>>r
	0x7C,// 25>>U
	0x02,// 26>>-
	0xC4,// 27>>上升
	0x38,// 28>>下降

	0x00,// 29>>空格
	0x00,// 30>>空格
	0x00,// 31>>空格
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

//地址、数据发送子程序 
void Write1302 ( unsigned char addr,dat )     
{ 
	unsigned char i,temp; 
	
	CE=0;                         //CE引脚为低，数据传送中止 
	SCLK=0;                    //清零时钟总线 
	Delay_us(4);
	CE = 1;                       //CE引脚为高，逻辑控制有效 
	Delay_us(4);
	//发送地址 
	for ( i=8; i>0; i-- ) //循环8次移位 
	{     
	      SCLK = 0; 
	      temp = addr; 
	      DIO = (bit)(temp&0x01);          //每次传输低字节 
		  Delay_us(1);
	      addr >>= 1;                //右移一位 
	      SCLK = 1; 
	} 
	//发送数据 
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


//数据读取子程序 
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
       //发送地址 
		for ( i=8; i>0; i-- )                      //循环8次移位 
		{     
		      SCLK = 0; 
			  DIO = (bit)(addr&0x01);          //每次传输低字节
			  Delay_us(1);
		      addr >>= 1;                              //右移一位
		      SCLK = 1; 
		} 

//读取数据

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



//初始化DS1302 
void Initial(void)    
{ 
   Write1302 (WRITE_PROTECT,0X00);       	//禁止写保护 

   Write1302 (WRITE_YEAR,DATE[0]);      		//秒位初始化 
   Write1302 (WRITE_MONTH,DATE[1]);        	//分钟初始化 
   Write1302 (WRITE_DATE,DATE[2]);      		//秒位初始化 
   Write1302 (WRITE_HOUR,DATE[3]);          	//小时初始化
   Write1302 (WRITE_MINUTE,DATE[4]);        	//分钟初始化 
   Write1302 (WRITE_SECOND,0x00);      		//秒位初始化 

   Write1302 (WRITE_POWER,0xAA);          	//2个二极管，4千欧
   Write1302 (WRITE_PROTECT,0x80);         	//允许写保护 
} 



//测试
unsigned int t_int1;
unsigned char t_char;

#define	CMD_IDLE	0	//空闲模式
#define	CMD_READ	1	//IAP读
#define	CMD_PROGRAM	2	//IAP写
#define	CMD_ERASE	3	//IAP擦除

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
bit Flag_Timer0;		//用于开机时计算时间，每次定时中断置1
unsigned char code ID2 _at_ 	0x0020;
unsigned char code ID1 _at_ 	0x0021;
unsigned char code ID0 _at_ 	0x0022;

unsigned char code EndYear _at_ 0x8000;//如果用iap读写，地址为0x0000
unsigned char code EndMon  _at_ 0x8001;
unsigned char code EndDay  _at_ 0x8002;

unsigned char code EndYear1 _at_ 0x8200;//如果用iap读写，地址为0x0000
unsigned char code EndMon1  _at_ 0x8201;
unsigned char code EndDay1  _at_ 0x8202;

unsigned char code EndYear2 _at_ 0x8400;//如果用iap读写，地址为0x0000
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
	if(RI)//在程序运行过程中也可以授权校正时间
	{
		RI=0;
		t=SBUF;
		if(t==TB_NowTime)//校时同步
		{
			if(Sint_value==TB_NowTime)Sint_count++;
			else Sint_count=0;
	 	}
		if(t==TB_EndTime)//授权同步
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
				Sint_value=Sint_value|0x01;//如果数据是写，强制改为读
				TI=0;
				SBUF=Read1302(Sint_value);
				while(!TI){};
			}
		}
		Sint_value1=Sint_value;
		Sint_value=t;
		if(Sint_count>10)IAP_CONTR=0x20;//重新启动
 	}
}


void CheckID()
{
	unsigned int i,j;
	unsigned char t,t0;
	unsigned int crc_value;
	unsigned char BZ;//=0:检测ID，=1:包含本板ID
	
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
//检查是否开启新的授权
//检查时钟是否正常，检查时钟是否停摆，如果停摆报E01，同时显示A01，并禁止运行
//检查时钟是否正常，检查时钟充电设置，如果错误报E01，同时显示A03，并禁止运行
//检查是否授权过期，如果过期报E01，同时显示A01，并禁止运行
//时间校正

	RI=0;Flag_Timer0=0;i=0;

	//等待授权指令，不论有还是没有，确认没有后检查授权状态
	while((i<200))//500*200us=100ms,14ms有10%的机会完成通信，16ms有80%机会完成通信，18ms与16ms差不多，20ms基本100%，最后取40ms
	{
		if(Flag_Timer0)	//这个变量只有在检查授权时置位,所以其实这个函数就是延时,上电时检查是否要授权
		{
			i++;
			Flag_Timer0=0;
		}
		if(RI)	  		//如果串口接收到数据
		{
			RI=0;
			t=SBUF;		//接收到的数据存入t变量中
			if(t==TB_EndTime) //收到同步头为 0xed
			{
				while(t==TB_EndTime)//接受同步码，同时等待首位信息
				{
					TI=0;
					SBUF=0xaa;	  //返回0XAA
					while(!RI){;}
					RI=0;
					t0=SBUF;	  //下个数据存入t0变量中
					t=t0;		  //将t0中的数据给t确保t0一直是最新收到的值
				}

				//开始接收数据,先做CRC程序
				crc_value=0xffff;
				crc_value=crc_cal_value(t,crc_value);//随机码

				for(i=0;i<3;i++)//接收授权日期
				{
					while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[3]=t;crc_value=crc_cal_value(t,crc_value);
					while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[2]=t;crc_value=crc_cal_value(t,crc_value);
					while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[1]=t;crc_value=crc_cal_value(t,crc_value);
				}
				EndDate.value=AcceptDat.value;

				for(i=0;i<1000;i++)//接收ID
				{
					while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[1]=t;crc_value=crc_cal_value(t,crc_value);
					while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[2]=t;crc_value=crc_cal_value(t,crc_value);
					while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[3]=t;crc_value=crc_cal_value(t,crc_value);
					if(AcceptDat.value==MeID.value)BZ=1;
				}

				//接收crc
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[1]=t;crc_value=crc_cal_value(t,crc_value);
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[1]=t;crc_value=crc_cal_value(t,crc_value);

				if(crc_value==0)  //crc无错误
				{
					if(BZ==1)	 //id号在授权范围内
					{
						t=0x55;	//信息接收完毕
					//修正授权时间
						for(i=0;i<3;i++)
						{
							BZ=0;
							while(!BZ)
							{
								EraseFlash(i*512);	//第1,2,3扇区存储授权的终止时间
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
				while(t==TB_NowTime)//接受同步码，同时等待首位信息
				{
					TI=0;
					SBUF=0xa5;
					while(!RI){};
					RI=0;
					t0=SBUF;
					t=t0;
				}

				//开始接收数据,先做CRC程序
				crc_value = 0xffff;
				crc_value = crc_cal_value(t,crc_value);//随机码

				//接收秒分时
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[3]=t;crc_value=crc_cal_value(t,crc_value);
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[2]=t;crc_value=crc_cal_value(t,crc_value);
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[1]=t;crc_value=crc_cal_value(t,crc_value);
				EndDate.value=AcceptDat.value;//借用寄存器
				
				//接收日月年
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[3]=t;crc_value=crc_cal_value(t,crc_value);
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[2]=t;crc_value=crc_cal_value(t,crc_value);
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;AcceptDat.Addr[1]=t;crc_value=crc_cal_value(t,crc_value);

				//接收crc
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;crc_value=crc_cal_value(t,crc_value);
				while(!RI){};RI=0;t=(t0<<1);t0=SBUF;t=t^t0;crc_value=crc_cal_value(t,crc_value);

				if(crc_value==0)
				{
					//修正时钟
					BZ=0;
					while(!BZ)
					{
				     	Write1302 (WRITE_PROTECT,0X00);       				//禁止写保护 
				       	Write1302 (WRITE_SECOND,EndDate.Addr[3]);      		//秒位初始化 
				       	Write1302 (WRITE_MINUTE,EndDate.Addr[2]);        	//分钟初始化 
				       	Write1302 (WRITE_HOUR,EndDate.Addr[1]);          	//小时初始化
				       	EndDate.value=AcceptDat.value;						//借用寄存器
						Write1302 (WRITE_DATE,EndDate.Addr[3]);      		//日位初始化 
				       	Write1302 (WRITE_MONTH,EndDate.Addr[2]);        	//月钟初始化 
				       	Write1302 (WRITE_YEAR,EndDate.Addr[1]);          	//年时初始化
				       	Write1302 (WRITE_POWER,0xAA);          				//2个二极管，4千欧
				       	Write1302 (WRITE_PROTECT,0x80);         			//允许写保护 
	
						BZ=(Read1302(READ_DATE)==EndDate.Addr[3])&(Read1302(READ_MONTH)==EndDate.Addr[2])&(Read1302(READ_YEAR)==EndDate.Addr[1]);
						//BZ=1;
					}
					EraseFlash(8*512);
					WriteFlash(8*512,0);//每次校时后，时钟自动修复限次归0
					
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


//检查时钟是否正常，检查时钟是否停摆，如果停摆报E01，同时显示A01，并禁止运行
	
	do
	{
		t=Read1302(READ_PROTECT);
	}while(t&0x7f);//确保1302有0数据输出，0x80或0x00
	
	t=Read1302(READ_SECOND);
	if(t&0x80)
	{
		t0=0;
		//尝试修复
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
		}//修复时钟限制为10次
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
				DrvRun=1;//禁止运行
				LED1 = 0;
				IN1 =0;
				RstPower();

				//产生一个50000*200us=10秒的周期
				while(!Flag_Timer0){}
				i++;
				Flag_Timer0=0;
				if(i>50000)i=0;

				//20ms向B板发状态			
				t=i%100;
				if(t==1){ErrorState=Error+1;Verify();S2CON&=~S2TI;S2BUF=ErrorState;}

				//显示地址和故障代码
				t=i/10000;ShowTimer=7;
				if(t==2){t0=ID2;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
				if(t==3){t0=ID1;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
				if(t==4){t0=ID0;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
				if(t<2){LedStr[0]=10;LedStr[1]=0;LedStr[2]=1;}//=7显示bcd7段编码，=8直接显示数据
			}
		}
	}

//检查时钟是否正常，检查时钟充电设置，如果错误报E01，同时显示A03，并禁止运行
	t=Read1302(READ_POWER);
	if(t!=0xAA)
	{
		t0=0;
		while(1)
		{
			//ErrorState=Error+1;Verify();S2CON&=~S2TI;S2BUF=ErrorState;while(!(S2CON&S2TI)){}
			DrvRun=1;//禁止运行
			LED1 = 0;
			IN1 =0;
			RstPower();

			//产生一个50000*200us=10秒的周期
			while(!Flag_Timer0){}
			i++;Flag_Timer0=0;
			if(i>50000)i=0;

			//20ms向B板发状态
			t=i%100;
			if(t==1){ErrorState=Error+1;Verify();S2CON&=~S2TI;S2BUF=ErrorState;}

			//显示地址和故障代码
			t=i/10000;ShowTimer=7;
			if(t==2){t0=ID2;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
			if(t==3){t0=ID1;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
			if(t==4){t0=ID0;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
			if(t<2){LedStr[0]=10;LedStr[1]=0;LedStr[2]=3;}//=7显示bcd7段编码，=8直接显示数据
		}
	}

//检查是否授权过期，如果过期报E01，同时显示A02，并禁止运行
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
			DrvRun=1;//禁止运行
			LED1 = 0;
			IN1 =0;
			RstPower();

			//产生一个50000*200us=10秒的周期
			while(!Flag_Timer0){}
			i++;
			Flag_Timer0=0;
			if(i>50000)i=0;

			//20ms向B板发状态
			t=i%100;
			if(t==1){ErrorState=Error+1;Verify();S2CON&=~S2TI;S2BUF=ErrorState;}

			//显示地址和故障代码
			t=i/10000;ShowTimer=7;
			if(t==2){t0=ID2;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
			if(t==3){t0=ID1;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
			if(t==4){t0=ID0;LedStr[0]=t-2;LedStr[1]=t0/16;LedStr[2]=t0%16;}
			if(t<2){LedStr[0]=10;LedStr[1]=0;LedStr[2]=2;}//=7显示bcd7段编码，=8直接显示数据
		}
	}

//存储时钟的年月日时分秒，以备数据丢失可以自行回复使用
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

void ShowID()//ShowTimer=0,1显示4秒ID2；=2显示2秒ID1；=3显示2秒ID0；
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

void ShowLoad()//显示过载实测数据与基准数据的差值，绝对值3位数不显示符号
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
			t2=0x40;//负号-
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
//ShowTimer==0,1,2,3;开机显示ID,显示过程，3-,D1,D2,D3
//ShowTimer==4;然后显示过载数据，为过载标定提供方便
//ShowTimer==5;运行后显示运行状态

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

//A/D转换准备
void AdReady(unsigned char INDEX)
{
	if(INDEX==0)ADC_CONTR=0xe3;//A相测量
	else if(INDEX==1)ADC_CONTR=0xe4;//B相测量
	else if(INDEX==2)ADC_CONTR=0xe5;//C相测量
	else if(INDEX==3)ADC_CONTR=0xe2;//过载测量
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

	return ADC_RES*4+ADC_RESL%4;  //<<2的2次方 左移两位
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

void StateCheck(void)//状态检测
{
//数据读取
//如果有24V，DatOut=0,State[]!=0
//读取顺序有电路板决定
	
	if(!To1){if(State[1]<200)State[1]++;}else{if(State[1]>0)State[1]--;}
	if(!To2){if(State[2]<200)State[2]++;}else{if(State[2]>0)State[2]--;}
	if(!To3){if(State[3]<200)State[3]++;}else{if(State[3]>0)State[3]--;}
	if(!To7){if(State[7]<200)State[7]++;}else{if(State[7]>0)State[7]--;}
	if(!To8){if(State[8]<200)State[8]++;}else{if(State[8]>0)State[8]--;}
	if(!To9){if(State[9]<200)State[9]++;}else{if(State[9]>0)State[9]--;}
	if(!To10){if(State[10]<200)State[10]++;}else{if(State[10]>0)State[10]--;} //上升
	if(!To11){if(State[11]<200)State[11]++;}else{if(State[11]>0)State[11]--;}
	if(!To12){if(State[12]<200)State[12]++;}else{if(State[12]>0)State[12]--;}
	if(!To13){if(State[13]<200)State[13]++;}else{if(State[13]>0)State[13]--;}
	if(!To14){if(State[14]<200)State[14]++;}else{if(State[14]>0)State[14]--;} //下降
	if(!To15){if(State[15]<200)State[15]++;}else{if(State[15]>0)State[15]--;}
	if(!To16){if(State[16]<200)State[16]++;}else{if(State[16]>0)State[16]--;}
}

bit PhaseErrorCheck()
{
/*
A相:00,01,02,03,04,05,06,07,  08,09,10,11,12,13,14,15,  16,17,18,19,20,21,22,23,
B相:16,17,18,19,20,21,22,23,  00,01,02,03,04,05,06,07,  08,09,10,11,12,13,14,15,
C相:08,09,10,11,12,13,14,15,  16,17,18,19,20,21,22,23,  00,01,02,03,04,05,06,07,
*/
	unsigned char a[3];
	a[0]=PhaseTimer[0];
	a[2]=PhaseTimer[1];
	a[1]=PhaseTimer[2];

	//错相7
	if(!FlagErrorPhase)//不错相
	{
		if((a[0]<40)&(a[1]<40)&(a[2]<40))//3相都有电流通过;200us*4*40=32ms
		{
			if(a[0]<a[1])//A相:00,01,02,03,04,05,06,07
			{
				if((a[2]>a[0])&(a[2]<a[1]))
				{
					if(PhaseError!=0)PhaseError--;
				}
				else
				{
					/*错相*/if(PhaseError<200)PhaseError++;
				}
			}
			else
			{
				if((a[2]>a[0])|(a[2]<a[1]))//A相:08,09,10,11,12,13,14,15,  16,17,18,19,20,21,22,23,
				{
					if(PhaseError!=0)PhaseError--;
				}
				else
				{
					/*错相*/if(PhaseError<200)PhaseError++;
				}				
			}
		}
	}
	if(PhaseError>100)//错相
	{
		PhaseError=150;
		ErrorState=Alarm+Error+7;//ERROR
		FlagErrorPhase=1;
	}
	
	return FlagErrorPhase;
}

bit PowerCheck()   //zjb
{
//电压报警采用推挽设计

	//if(SecCount>(255-20*1))//25ms*20=500ms
	if(SecCount>(255-100))//25ms*100=2.5s，
	{//启动2.5s不检测，等待电路稳定   20130924
	}
	else if((LoadCurrent[0]<LosePower)&(LoadCurrent[1]<LosePower)&(LoadCurrent[2]<LosePower)&(LoadCurrent[3]<LosePower))
	{//未检测到有效电压，不检测，以便于电路板检测2017-5-25
	}
	else
	{
		if(FlagPowerL)//欠压时
		{
			if((LoadCurrent[0]>MinPower)&(LoadCurrent[1]>MinPower)&(LoadCurrent[2]>MinPower))FlagPowerL=0;//不欠压
			else {FlagPowerL=1;ErrorState=Alarm+Error+21;}
		}else //正常时
		{
			if((LoadCurrent[0]<MinPower)|(LoadCurrent[1]<MinPower)|(LoadCurrent[2]<MinPower)){FlagPowerL=1;ErrorState=Alarm+Error+21;}
			else FlagPowerL=0;//不欠压
		}
	
		if(FlagPowerH)//超压时
		{
			if((LoadCurrent[0]<MaxPower)&(LoadCurrent[1]<MaxPower)&(LoadCurrent[2]<MaxPower))FlagPowerH=0;//不超压
			else {FlagPowerH=1;ErrorState=Alarm+Error+20;}
		}else //正常时
		{
			if((LoadCurrent[0]>MaxPower)|(LoadCurrent[1]>MaxPower)|(LoadCurrent[2]>MaxPower)){FlagPowerH=1;ErrorState=Alarm+Error+20;}
			else FlagPowerH=0;//不超压
		}
	}

	if(Sec5>1){FlagPowerH=0;FlagPowerL=0;}//2015-08-12

	
	/*缺相*/
	//if((((LoadCurrent[0]>MinPower)|(LoadCurrent[1]>MinPower)|(LoadCurrent[2]>MinPower))
	//&((LoadCurrent[0]<LosePower)|(LoadCurrent[1]<LosePower)|(LoadCurrent[2]<LosePower)))//检测到有效电压
	//|(LoadCurrent[3]<MinPower))//L1~N电压过低
	if(
	    ((LoadCurrent[0]>MinPower)|(LoadCurrent[1]>MinPower)|(LoadCurrent[2]>MinPower))//检测到有效电压
	   &((LoadCurrent[0]<LosePower)|(LoadCurrent[1]<LosePower)|(LoadCurrent[2]<LosePower)|(LoadCurrent[3]<LosePower))//L1~N电压过低
	  )

	{FlagLosePhase=1;ErrorState=Alarm+Error+8;}
	else FlagLosePhase=0;
	/*错相*/
	PhaseErrorCheck();
	
	//FlagPower=0;//test 屏蔽以上判断
	return FlagPowerH|FlagPowerL|FlagLosePhase|FlagErrorPhase;
}
bit OverLoadCheck()
{
	//过载10
	if(OverLoadCount>500)//200us*4*500=400ms
	{
		OverLoad=OverLoad1/OverLoadCount;
		OverLoad1=OverLoad;
//		OverLoad1 = OverLoad1*5000;
//		OverLoad1 = OverLoad1/1024;
		OverLoadCount=1;
		TI=0;		   						//此段代码会影响软件通讯..测试时用不能当做正式程序使用
		SBUF = OverLoad1>>24;
		while(!TI);
		TI=0;
		SBUF = OverLoad1>>16;
		while(!TI);
		TI=0;		   						//此段代码会影响软件通讯..测试时用不能当做正式程序使用
		SBUF = OverLoad1>>8;
		while(!TI);
		TI=0;
		SBUF = OverLoad1;
		while(!TI);
	}
	//不再才用推挽方式，提高灵敏度 20130503
	
	//if(OverLoad<OverLoadLimite1)FlagOverLoad=0;//不过载
	if((OverLoad-(GaoDu/GaoDuK))<OverLoadLimite1){FlagOverLoad=0;LED2=1;IN2=1;}//2014/3/19
	else
	{
		IN2 = 0;
		LED2 = 0;
		ErrorState=Alarm+Error+10;//ERROR
		FlagOverLoad=1;
	}
	if(FlagRun)FlagOverLoad=0;//电梯行进时，不做过载检测，防止悬停空中
	
	return FlagOverLoad;
}

//2012-10-26
bit MainSwitchCheck()
{
bit sta;
/*
	箱外急停
	箱内急停
	门禁
	紧急上限位
	安全锁
	热继
*/

	sta=1;FlagRun=0;	
	if(State[0]<50)		ErrorState=Error+1;//AC24V故障
	if(State[1]<50)		ErrorState=Error+1;//钥匙开关
	if(State[2]<50)		ErrorState=Error+3;//箱外急停3
	else if(State[3]<50)ErrorState=Error+2;	//箱内急停2
	else if(State[4]<50)ErrorState=Error+1;	//继电器6
	else if(State[5]<50)ErrorState=Error+17;//备用门17
	else if(State[6]<50)ErrorState=Error+16;//门禁16
	else if(State[7]<50)ErrorState=Alarm+Error+4;//紧急上限位4
	else if(State[8]<50)ErrorState=Alarm+Error+18;//安全锁18
	else if(State[9]<50)ErrorState=Alarm+Error+9;//热继9
	else {sta=0;FlagRun=1;}


	return sta;
}

//2012-10-26
bit UpSwitchCheck()
{
	bit sta;
	sta=1;
	if(State[10]>0)//上升
	{
		FlagStart=1;
		if(State[11]<50)
		ErrorState=Error+11;//上限位动作11
		else if(State[12]<50)
		ErrorState=Alarm+Error+19;//轮廓上限位动作19
		else if(State[13]<50)
		ErrorState=Alarm+Error+1;//继电器2错误
		//else {ErrorState=2;FlagRun=1;}//上升状态
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
	if(State[14]>150)//下降
	{
		FlagStart=1;
		//if(State[15]<50)ErrorState=Error+12;//下限位动作12
		if(State[15]<50)
		{
			ErrorState=Error+12;
			GaoDu=0;
		}//2014/3/19
		else if(State[16]<50)
		ErrorState=Alarm+Error+1;//继电器1错误
		//else {ErrorState=3;FlagRun=1;}//下降状态
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
	if(FlagStart)ErrorState=1;//停止状态1
	else ErrorState=0;
}

//故障分析
void StateErrorCheck()
{
	if(OverLoadCheck()){}//过载显示故障
	else if(PowerCheck()){}//过电压或欠电压或缺相或相序错误
	else if(FlagStart)//如果已经开始操作检查和显示其他故障，否则显示运行时间
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
B7：=B6⊕B5⊕B4⊕B3⊕B2⊕B1⊕B0
B6 B5=0状态显示
B4 B3 B2 B1 B0=
	0：显示电机运转时间
1：空闲状态
2：上升状态
3：下降状态
B6 B5=1故障显示(不告警)
	B4 B3 B2 B1 B0=
1：电源24伏故障或者电路板故障；ok
2：厢内急停触发；ok
3：厢外急停触发；ok
4：紧急上限位触发；ok
5：接受命令失败（误码）；
6：接受命令失败（超时）；ok
7：相序错误；
8：断相；
9：过流保护；热继电器 ok
10：过载保护；ok
11：上限位触发；ok
12：下限位触发；ok
16：前门未关闭 ok
17：后门未关闭 ok
18：安全锁动作 ok
19：轮廓上限位触发 ok

20：电压过高
21：电压过低
30~31：测试端口1~2（当测试端口上出现24V电压时，显示测试端口数据，同时控制板的红灯闪烁）
B6 B5=3故障显示(告警)，故障类型同B6 B5=1
*/

	if(SecCount>(255-20*1))//25ms*20=500ms
	{
		if(FlagErrorPhase)//保证500ms秒内允许错相运行，以便拆卸钢丝绳
		{
			DrvRun=0;
			SecCount=255;//2012-3-24 报错后，如果停止操作
		}

		if(State[14]>150)DrvSl3=0;	//如果下降，断开安全锁继电器2017-05-22
	}
	else
	{
		DrvRun=FlagOverLoad|FlagErrorPhase|FlagLosePhase|OverTimer;//2017-5-25
		DrvSl3=0;	//断开安全锁继电器
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
//串口
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
   	//辅助寄存器
	AUXR=0X14;//T0*12/T1*12/UART_M0*6/T2R/T2_CT/T2*12/EXTRAM/S1ST2;S1ST2设置串口1使用定时器T2
	AUXR|=0X01;
}

//200us定时中断程序
//时钟时间节奏准确
void timer0(void) interrupt 1 /*T0中断*/
{
	TH0=(65536-Crystal/12000*InitTime/1000)/256;
	TL0=(65536-Crystal/12000*InitTime/1000)%256;

	WDT_CONTR=0x3e;//2.72秒看门狗

	Clock_1s++;
	if(Clock_1s>9999)
	{
		if(ShowTimer<4)ShowTimer++;
		Clock_1s=0;
	}

	if(FlagStart)ShowTimer=5;
	
	if(Flag_CheckID)//如果在检测授权跳过所有操作
	{
		LedShow();
		Flag_Timer0=1;
		goto timer0end;
	}

	AdStart();//ok
	
	StateCheck();//ok
	
	//过载测试值取绝对值，此时正在测量A相，过载已测量完毕
	if(LoadIndex==0)//0.8ms一次
	{
		OverLoad1=OverLoad1+LoadDot[3];//过载数值计算
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
				SumLoadCurrent[2]=SumLoadCurrent[2]/1.06;//实测50hz是1.053，计算机模拟50hz是1.053~1.056，60hz是1.064~1.066，折中取1.06
				//LoadCurrent[2]=SumLoadCurrent[2]/SumCount;SumLoadCurrent[2]=0;
				LoadCurrent[2]=LoadCurrent[1];SumLoadCurrent[2]=0;//考虑到不是直接测量，问题较大，不再使用20130503
				LoadCurrent[3]=SumLoadCurrent[3]/SumCount;SumLoadCurrent[3]=0;

				SumCount=0;
				Sec5++;
				Sec5=Sec5%5;//2015-08-12
			}

			if(FlagRun)L_Run=(SumCount>(CheckCycle*20));//1赫兹闪烁
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
		else if(CountIndex&0x01)LedShow();//1.6ms一次
	}
	
/*
	LoadIndex==1时，A相电流分析，此时正在测量B相，A相已测量完毕
	LoadIndex==2时，B相电流分析，此时正在测量C相，B相已测量完毕
	LoadIndex==3时，C相电流分析，此时正在测量过载，C相已测量完毕
	
	当电流LoadDot[]>(LoadZero+LoadMin)时，表示有正电流，极性Polarity[]标为1
	当电流LoadDot[]>(LoadZero-LoadMin)时，表示有负电流，极性Polarity[]标为0
	当极性Polarity[]由0变为1时，PhaseTimer[]开始计时,
	当极性Polarity[]由0变为1时，LoadCurrent[]计算并累计,

*/
	//AB相电压
	if(LoadIndex==1)
	{
		LoadDot[0]=1024-LoadDot[0];//测量的是BA电压，转AB
	 	if(LoadDot[0]>LoadZero)
	 	{
	 		SumLoadCurrent[0]=SumLoadCurrent[0]+LoadDot[0]-LoadZero;//2012-11-28
	 	}
		else
	 	{
	 		SumLoadCurrent[0]=SumLoadCurrent[0]+LoadZero-LoadDot[0];//2012-11-28	
	 	}
		
		if(!Polarity0)//如果电流负半周期
		{
			if(LoadDot[0]>(LoadZero+LoadMin))//进入正半周
			{
				Polarity0=1;
				PhaseTimer[0]=0;
			}
			
		}
		else
		{
			if(LoadDot[0]<(LoadZero-LoadMin))//进入负半周
			{
				Polarity0=0;
			}
		}
		
		if(PhaseTimer[0]<199)PhaseTimer[0]++;
	}
	
	//CA相电压
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
		
		if(!Polarity1)//如果电流负半周期
		{
			if(LoadDot[1]>(LoadZero+LoadMin))//进入正半周
			{
				Polarity1=1;
				PhaseTimer[1]=0;
				
			}
		}else
		{
			if(LoadDot[1]<(LoadZero-LoadMin))//进入负半周
			{
				Polarity1=0;
			}
		}
		
		if(PhaseTimer[1]<199)PhaseTimer[1]++;
	}

	
	//BC相电压和A线电压
	if(LoadIndex==3)
	{
	 	//A线电压
	 	if(LoadDot[2]>LoadZero)SumLoadCurrent[3]=SumLoadCurrent[3]+LoadDot[2]-LoadZero;//2012-11-28
	 	else SumLoadCurrent[3]=SumLoadCurrent[3]+LoadZero-LoadDot[2];//2012-11-28
	
	 	//BC相电压
	 	LoadDot[2]=1024-(LoadDot[0]+LoadDot[1]-LoadZero);
	 	if(LoadDot[2]>LoadZero)
	 	{
	 		SumLoadCurrent[2]=SumLoadCurrent[2]+LoadDot[2]-LoadZero;//2012-11-28
	 	}else 
	 	{
	 		SumLoadCurrent[2]=SumLoadCurrent[2]+LoadZero-LoadDot[2];//2012-11-28
	 	}
		
		if(!Polarity2)//如果电流负半周期
		{
			if(LoadDot[2]>(LoadZero+LoadMin))//进入正半周
			{
				Polarity2=1;
				PhaseTimer[2]=0;
			}
			
		}else
		{
			if(LoadDot[2]<(LoadZero-LoadMin))//进入负半周
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
//-------------------中断函数--------------------------//
void Int0(void) 	interrupt 0//int0
{while(1){TI=0;SBUF=0x00;while(!TI){};/*Delay(1000)*/;}}
void Int1(void) 	interrupt 2//int1
{while(1){TI=0;SBUF=0x02;while(!TI){};/*Delay(1000)*/;}}
void Timer1(void) 	interrupt 3//timer1
{while(1){TI=0;SBUF=0x03;while(!TI){};/*Delay(1000)*/;}}
void UART1(void) 	interrupt 4//uart 通信
{while(1){TI=0;SBUF=0x04;while(!TI){};/*Delay(1000)*/;}}
void ADC(void) 		interrupt 5//adc 模数转换
{while(1){TI=0;SBUF=0x05;while(!TI){};/*Delay(1000)*/;}}
void LVC(void) 		interrupt 6//pca 低压检测
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


//主程序
void main()
{
//unsigned int ii;
//unsigned char i;
//看门狗程序
/*
	i=WDT_CONTR;
	if(i>127)//如果看门狗复位
	{
		while(1){};
	}else WDT_CONTR=0x3e;//2.72秒看门狗
*/

//*****************************************
//硬件初始化
	IE=0;
//T0
	TH0=(65536-Crystal/12000*InitTime/1000)/256;
	TL0=(65536-Crystal/12000*InitTime/1000)%256;
	TMOD=0x21;TR0=1;ET0=1;

	SxBUF_Init();

	ADC_CONTR=0x60;//将转换速度设为最高,210个时钟周期
	ADC_CONTR=ADC_CONTR|0x80;//打开转换电源
	PX0=0;PT0=1;IP2=0;

	ADC_CONTR=0x60;//将转换速度设为最高,210个时钟周期
	ADC_CONTR=ADC_CONTR|0x80;//打开转换电源
//将P1.3,P1.4,P1.5设置为A/D转换模式

/*
M1	M0
0	0	双向
0	1	输出
1	0	输入
1	1	开漏、AD

O	输出
I	输入
NC	未用引脚
N0	没有引脚
*/
	//01234567
    //TO14,TO15,TO16,TO3,TO2,S1,S0,NC
	P0M1=0x00;P0M0=0x00;

	//RXD2,TXD2,LOAD,AD0,AD1,AD2,2.5V,NC
	P1M1=~0x03;P1M0=~0x03;

	//NC,NC,NC,NC,TO4,TO5,TO6,TO7
	P2M1=0x00;P2M0=0x00;

    //RXD,TXD,IN8,IN7,IN6,IN3,IN2,IN1
	P3M1=0x00;P3M0=~0x03;//01串口+6个2803

	//NC,DF,DB,DA,NC,TO12,TO13,NC
	P4M1=0x00;P4M0=0x00;

	//IN5,IN4,TO1,NC,NC,NC,NO,NO
	P5M1=0x00;P5M0=0x03;//2个2803+1个光偶

	//DG,DE,DD,DC,TO8,TO9,TO10,TO11
	P7M1=0x00;P7M0=0x00;

//test end
	ErrorState	=0;
	DrvUpSta	=0;
	FlagOverLoad	=0;
	FlagErrorPhase	=0;
	FlagLosePhase	=0;
	FlagStart	=0;
	DrvSl3		=1;//闭锁安全锁
	DrvRun		=0;//允许运行
	
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
	EA=1;	//调试时关断
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

