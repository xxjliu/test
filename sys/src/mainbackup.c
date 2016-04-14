/**
  ******************************************************************************
  * @file main.c
  * @brief This file contains the main function for TIM2 PWM Output example.
  * @author STMicroelectronics - MCD Application Team
  * @version V2.0.0
  * @date 15-March-2011
  ******************************************************************************
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  * @image html logo.bmp
  ******************************************************************************
  */
/*


/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"
#include "type.h"
#include "stm8s_clk.h"
#include "stm8s_tim1.h"
#include "stm8s_tim5.h"
#include "stm8s_tim6.h"

#include "adc.h"
#include "uart.h"
#include "temp.h"
#include "sysctrl.h"
#include "at24.h"	
#include "key.h"
#include "user.h"
#include "debug.h"
#include "iostm8s903k3.h"
//PWM1 SET
const unsigned  int TabL[10]={763,772,780,789,797,805,813,821,828,836};//797,789,780,772,763,754,};
const unsigned  int TabH[61]={754,745,736,727,718,709,699,690,680,670,660,
651,641,631,621,611,601,591,581,571,561,551,541,532,522,512,502,493,483,474,
464,455,446,437,428,419,410,401,393,384,376,368,360,352,344,336,329,321,314,
307,300,293,286,279,273,267,260,254,248,242,237};
//------------------------------------------------------------------

//const unsigned  int TabL[11]={836,828,821,813,805,797,789,780,772,763,754,745};

void down_line(void);
void  SendUart(void);
void  Dwrite_eeprom(void);
void HexBcd(unsigned int hex);
 void DisplayTemp(void);
//------------------------------------------------------------------
//const  float @0x0204 Tab[4]={1.000,2.000,3.000,5.000};
//#pragma location="MYCONST"
//const   float Tab[4]={1.000,2.000,3.000,5.000};
//1、__no_init char alpha @ 0x0200; 
 
//2、#pragma location=0x0202
//const int beta; 
 
//3、const int gamma @ 0x0204 = 3; 

//********************************************************************

#define LED1                    PD_ODR_ODR2
#define LED2                    PD_ODR_ODR3
#define LED3                    PD_ODR_ODR4
#define LED4                    PD_ODR_ODR5

#define TEST                	PA_ODR_ODR1
#define UART_TXD                PA_ODR_ODR3
#define UART_RXD                PF_IDR_IDR4


#define PRECHARGE_ENABLE        PB_ODR_ODR6
#define PRECHARGE_BYPASS        PB_ODR_ODR5
#define SUPPLY_LATCH_COUPLED    PB_ODR_ODR0

#define VAC_MOTOR               PE_ODR_ODR5
#define SUPPLY_LATCH            PC_ODR_ODR2
#define BRUSH_MOTOR             PC_ODR_ODR5
#define BRUSH_MOTOR_ONOFF       PC_IDR_IDR1
#define SUPPLY_DETECT           PC_IDR_IDR7

#define CHARGE_VOLTAGE          PB_IDR_IDR2
#define VOP_MONITOR             PB_IDR_IDR7

//#define CHARGER_DETECT          PB_IDR_IDR3


#define CHG_TEMP_HIGHT            71  //60=2.9704
#define CHG_TEMP_LOW              229 //0=28.2671
#define BIGCUR_PRE_TEMP_HIGHT     101 //45=4.8520
#define BIGCUR_PRE_TEMP_LOW       200 //10=18.2801

#define DIS_TEMP_HIGHT            71  //60=2.9704
#define DIS_TEMP_LOW              254 //-10=45.0676

#define CELLS_OVER_DIS_VOL1       658
#define CELLS_OVER_DIS_VOL2       587
#define CELLS_LOW_DIS_VOL         658

#define CELLS_OVER_CHG_VOL        975

unsigned int Temp_val,CellsVol_val,Motor_val,Chger_val,Motor_Amp;

unsigned int MotorOvCur_DelayTimer,LedDisLow_Timer,LedCellsErr_Timer,OvChgFirstTimer,LedChgTemp_Timer,Led8HzTimer,Led8Hz3SecondTimer;
unsigned int OvChg_DelayTimer,CellOvChg_DelayTimer,LedDisTemp_Timer,LedChgerErr_Timer,SleepTimer;
unsigned int PreChg_OvTimer_Pre,HighTemp_PreChg_OvTimer_Pre,LowTemp_PreChg_OvTimer_Pre,BigChg_OvTimer_Pre;
unsigned int MotorOvCur_DelayTimer_Set,OvChg_DelayTimer_Set,CellOvChg_DelayTimer_Set;
unsigned int PreChg_OvTimer_Set,BigChg_OvTimer_Set,HighTemp_PreChg_OvTimer_Set,LowTemp_PreChg_OvTimer_Set,UnderDis1_DelayTimer_Set,UnderDis2_DelayTimer_Set;
unsigned int PreChg_OvTimer,BigChg_OvTimer,HighTemp_PreChg_OvTimer,LowTemp_PreChg_OvTimer,UnderDis1_DelayTimer,UnderDis2_DelayTimer;
unsigned int PreChg_OvTimer_pre,BigChg_OvTimer_pre,BigChg_OvTimer_mid,HighTemp_PreChg_OvTimer_pre,LowTemp_PreChg_OvTimer_pre,LowTemp_PreChg_OvTimer_mid;
int FirstChgMark,FirstDisMark,SLEEP,Brush_on,Chger_on,FirstDisVolMark;
struct
{
  unsigned char ChgerErr          : 1;
  unsigned char Chg               : 1;
  unsigned char Dischg            : 1;
  unsigned char OvChg             : 1;
  unsigned char OvDis             : 1;
  unsigned char PreChg            : 1;
  unsigned char ChgVolLow         : 1;
  unsigned char ChgUnderLow       : 1;
  
  unsigned char PreChgTempH       : 1;
  unsigned char PreChgTempL       : 1;
  unsigned char DisTemp           : 1;
  unsigned char UnderDisLow       : 1;
  unsigned char UnderDis1         : 1;
  unsigned char UnderDis2         : 1;
  unsigned char MotorOvCur        : 1;
  unsigned char ChgTemp           : 1;

  unsigned char CellOvChg         : 1;
  
}
BITS_FLAG;
//--------------------------------------
void Delayms(int num)
{
	int i,k;
	k=num*715;
	while(k != 0)
		{
			for(i=0;i<1000;i++);
			k--;
		}
}
//-----------------------
void OffLed(void)
{
	GPIO_WriteLow(GPIOD, GPIO_PIN_2);
	GPIO_WriteLow(GPIOD, GPIO_PIN_3);
	GPIO_WriteLow(GPIOD, GPIO_PIN_4);
	GPIO_WriteLow(GPIOD, GPIO_PIN_5);
}

//--------------------------LedDisTemp_Timer,LedChgerErr_Timer
void LedDisTemp(void)
{
  if(LedDisTemp_Timer <6000)//闪烁6s，
    {
      if(LedDisTemp_Timer < 500)
        {
          if(LedDisTemp_Timer <250)
            {
              LED1 =1;
              LED2 =1;
              LED3 =1;
              LED4 =1;
            }
          else
            {
              LED1 =0;
              LED2 =0;
              LED3 =0;
              LED4 =0;
            }
        }
     else
        {
          LedDisLow_Timer =0;
        }
    }
  else
    {
      BITS_FLAG.ChgTemp =0;
    }
}
//--------------------------
void LedDisLowVol(void)
{
  if(Led8Hz3SecondTimer <3000)
    {
      if(Led8HzTimer > 62)//8Hz 3second flashing,then sleep
        {
          Led8HzTimer =0;
          LED1 =~LED1;
          LED2 =~LED2;
          LED3 =~LED3;
          LED4 =~LED4;
        }
    }
  else
    {
      SLEEP=1;
    }
}
//----------------------------------
    
void LedCellsDisLow(void)//亮 3v/cell以下 闪烁一个LED
{
  if(LedDisLow_Timer < 1000)
    {
      if(LedDisLow_Timer <500)
        {
          LED1 =1;
          LED2 =0;
          LED3 =0;
          LED4 =0;
        }
      else
        {
          LED1 =0;
          LED2 =0;
          LED3 =0;
          LED4 =0;
        }
    }
  else
    {
      LedDisLow_Timer =0;
    }
}

//--------------------------
void LedDis(void) //亮  3v/cell 以上
{
  if(CellsVol_val > 949)//18.360)
    {
      LED1 =1;
      LED2 =1;
      LED3 =1;
      LED4 =1;
    }
  else if(CellsVol_val > 901)//17.435)
    {
      LED1 =1;
      LED2 =1;
      LED3 =1;
      LED4 =0;
    }
  else if(CellsVol_val > 874)//16.915)
    {
      LED1 =1;
      LED2 =1;
      LED3 =0;
      LED4 =0;
    }
  else if(CellsVol_val > 874 && CellsVol_val > 828)//16.015)
    {
      LED1 =1;
      LED2 =0;
      LED3 =0;
      LED4 =0;
    }    

}

//--------------------------
void LedCellsErr(void)//亮
{
  if(LedCellsErr_Timer < 140)
    {
      if(LedChgTemp_Timer <70)
        {
          LED1 =1;
          LED2 =1;
          LED3 =1;
          LED4 =1;
        }
      else
        {
          LED1 =0;
          LED2 =0;
          LED3 =0;
          LED4 =0;
        }
    }
}

//--------------------------
void LedChgerErr(void)//亮
{
  if(LedChgerErr_Timer < 1000)
    {
      if(LedChgTemp_Timer <62)
        {
          LED1 =1;
          LED2 =1;
          LED3 =1;
          LED4 =1;
        }
      else if(LedChgTemp_Timer <124)
        {
          LED1 =0;
          LED2 =0;
          LED3 =0;
          LED4 =0;
        }
      else if(LedChgTemp_Timer <186)
        {
          LED1 =1;
          LED2 =1;
          LED3 =1;
          LED4 =1;
        }
      else
        {
          LED1 =0;
          LED2 =0;
          LED3 =0;
          LED4 =0;
        }
    }
  else
    {
      LedChgTemp_Timer =0;
    }
}

//---------------------------
void LedOvChg(void) //暗
{
  if(FirstChgMark ==1 && OvChgFirstTimer < 3000)//亮3秒
    {
      LED1 =1;
      LED2 =1;
      LED3 =1;
      LED4 =1;
    }
  else
    {
      FirstChgMark =0;
      LED1 =0;
      LED2 =0;
      LED3 =0;
      LED4 =0;
    }
}
//--------------------------
void LedChg(void) //暗
{
  if(CellsVol_val < 938)//18.15)
    {
      LED1 =1;
      LED2 =0;
      LED3 =0;
      LED4 =0;
    }
  else if(CellsVol_val < 956)//18.50)
    {
      LED1 =1;
      LED2 =1;
      LED3 =0;
      LED4 =0;
    }
  else if(CellsVol_val < 1003)//19.4)
    {
      LED1 =1;
      LED2 =1;
      LED3 =1;
      LED4 =0;
    }
  else if(CellsVol_val < 1070)//20.75)
    {
      LED1 =1;
      LED2 =1;
      LED3 =1;
      LED4 =1;
    }
  else
    {
      LED1 =0;
      LED2 =0;
      LED3 =0;
      LED4 =0;
    }
}
//--------------------------
void LedChgTemp(void)
{
  if(LedChgTemp_Timer < 760)
    {
      if(LedChgTemp_Timer <250)
        {
          LED1 =1;
          LED2 =1;
          LED3 =1;
          LED4 =1;
        }
      else if(LedChgTemp_Timer <500)
        {
          LED1 =0;
          LED2 =0;
          LED3 =0;
          LED4 =0;
        }
      else if(LedChgTemp_Timer <510)
        {
          LED1 =1;
          LED2 =1;
          LED3 =1;
          LED4 =1;
        }
      else
        {
          LED1 =0;
          LED2 =0;
          LED3 =0;
          LED4 =0;
        }
    }
  else
    {
      LedChgTemp_Timer =0;
    }
}
//--------------------------
void OffDis(void)
{
  VAC_MOTOR =0;
  BRUSH_MOTOR =0;
  Brush_on =0;
}
//--------------------------
void OnDis(void)
{
  VAC_MOTOR =1;
  Delayms(3);
  if(BRUSH_MOTOR_ONOFF && (!SUPPLY_DETECT))//刷头开关在主马达开启前不能工作
  	{
	  if(Brush_on ==1)
	  	{
			 Brush_on =1;
			 BRUSH_MOTOR =1;
	  	}
	  else
	  	{
	  	  BRUSH_MOTOR =0;
	  	  Brush_on =0;
	  	}
  	}
  else if(!BRUSH_MOTOR_ONOFF)
  	{
  		Brush_on =0;
  	}
}
//--------------------------    
void DisCurCheck(void)
{
  if(Motor_val  < 290)
    {
      MotorOvCur_DelayTimer =0;
    }
  else if(MotorOvCur_DelayTimer >MotorOvCur_DelayTimer_Set)
    {
      BITS_FLAG.MotorOvCur =1;
      MotorOvCur_DelayTimer =0;
    }
  if(BITS_FLAG.MotorOvCur && Motor_val  > 290 && SUPPLY_DETECT)
    {
      BITS_FLAG.MotorOvCur =0;
    }
}
//--------------------------
void DisTempCheck(void)
{
  if(Temp_val < 71 || Temp_val > 254)
    {
      BITS_FLAG.DisTemp =1;
    }
  else if(BITS_FLAG.DisTemp && SUPPLY_DETECT)
    {
      BITS_FLAG.DisTemp =0;
    }
  
}
//--------------------------
void DisVolCheck(void)
{
  if(FirstDisVolMark ==1 && CellsVol_val < 658)//起放电压 2.8v/cell
    {
      BITS_FLAG.UnderDisLow =1;
    }
  else 
    {
      FirstDisVolMark =0;
      //BITS_FLAG.UnderDisLow =0;
      if(CellsVol_val >658)
        {
          UnderDis1_DelayTimer =0;
        }
      else if(UnderDis1_DelayTimer >UnderDis1_DelayTimer_Set)//放电保护电压1: 14v = 2.8v/cell
        {
          BITS_FLAG.UnderDis1 =1;
          UnderDis1_DelayTimer =0;
        }
      
      if(CellsVol_val >587)
        {
          UnderDis2_DelayTimer =0;
        }
      else if(UnderDis2_DelayTimer >UnderDis2_DelayTimer_Set)//放电保护电压2: 12.5v = 2.5v/cell
        {
          BITS_FLAG.UnderDis2 =1;
          UnderDis2_DelayTimer =0;
        }
      
      if(SUPPLY_DETECT)//重启开关，解除标示符
      {
        
        if(CellsVol_val >658)
        { 
          BITS_FLAG.UnderDis1 =0;
          BITS_FLAG.UnderDis2 =0;
          BITS_FLAG.UnderDisLow =0;
        }
        else if(CellsVol_val >587)
        {
          BITS_FLAG.UnderDis2 =0;
        }
        
    }
  
}
}
//--------------------------
void ChgTempCheck(void)
  {
    if(Temp_val >229 || Temp_val <71)
      {
        BITS_FLAG.ChgTemp =1;
      }
    else
      {
        BITS_FLAG.ChgTemp =0;
      }
    
    if(Temp_val >200 && Temp_val <229)//预充低温
      {
        BITS_FLAG.PreChgTempL =1;
      }
    else
      {
        BITS_FLAG.PreChgTempL =0;
      }
    
    if(Temp_val >71 && Temp_val <101)//预充高温
      {
        BITS_FLAG.PreChgTempH =1;
      }
    else
      {
        BITS_FLAG.PreChgTempH =0;
      }
  }
//-------------------------
void VarInit(void)
{
  OvChg_DelayTimer =0;
  OvChg_DelayTimer_Set =190;          //整体过充延时 190ms
   
  CellOvChg_DelayTimer =0;
  CellOvChg_DelayTimer_Set =1000;     //单节过充延时 1000ms
  
  PreChg_OvTimer =0;
  PreChg_OvTimer_Set =500;            //低压预充计时 50min
  
  HighTemp_PreChg_OvTimer =0;
  HighTemp_PreChg_OvTimer_Set =1000;  //高温预充计时 100min
   
  LowTemp_PreChg_OvTimer =0;
  LowTemp_PreChg_OvTimer_Set =2700;   //低温预充计时 24hours 
  
  BigChg_OvTimer =0;
  BigChg_OvTimer_Set =3000;           //大电流充电计时 6.5hours
  
  UnderDis1_DelayTimer =0;
  UnderDis1_DelayTimer_Set =6000;     // 整体过放保护延时1 6s
  
  UnderDis2_DelayTimer =0;
  UnderDis2_DelayTimer =300;          // 整体过放保护延时2 300ms
  
  MotorOvCur_DelayTimer =0;
  MotorOvCur_DelayTimer_Set =300;     //副马达过载保护延时 300ms
  
  
  LedDisLow_Timer =0;
  LedCellsErr_Timer =0;
  OvChgFirstTimer =0;
  LedChgTemp_Timer =0;
  Led8HzTimer =0;
  Led8Hz3SecondTimer =0;
  LedDisTemp_Timer =0;
  LedChgerErr_Timer =0;

  Brush_on =0;
  Chger_on =0;

  FirstChgMark =1;
  FirstDisMark =1;
  FirstDisVolMark =1;
}
//-----------------------------
void ChgerCheck(void)
{
	if(Chger_val >888 && Chger_val <666)//888, and 666 need to rewrite
		{
			BITS_FLAG.ChgerErr =0;
		}
	else
		{
			BITS_FLAG.ChgerErr =1;
		}
}

//-----------------------------
void ChgVolCheck(void)
{
  if(CellsVol_val <235)// 起充电压 5v
    {
      BITS_FLAG.ChgUnderLow =1;
    }
  else
    {
      BITS_FLAG.ChgUnderLow =0;
      if(CellsVol_val <975)//  过充电压为 20.75v 
        {
          OvChg_DelayTimer =0;
          BITS_FLAG.OvChg =0;
        }
      else if(OvChg_DelayTimer > OvChg_DelayTimer_Set)
        {
          BITS_FLAG.OvChg =1;
          OvChg_DelayTimer =0;
        }
      
      if(CellsVol_val <750) //充电低压 3v
        {
          BITS_FLAG.ChgVolLow =1;
        }
      else 
        {
          BITS_FLAG.ChgVolLow =0;
        }
    }
  
}
//------------------------------------------------------
void PreChgCheck(void)
{
    if((Temp_val > 200 || Temp_val < 101) && (CellsVol_val <750 && CellsVol_val >235))
    {
      BITS_FLAG.PreChg =1;
    }
    else if((Temp_val < 200 && Temp_val > 101) && (CellsVol_val > 750 && CellsVol_val < 975))
    {
      BITS_FLAG.PreChg =0;
    }
}
//------------------------------------------------------
/*
void Uart1Init(void)	
  {
     CLK_PCKENR1|=0x0c;//0xf3;  //使能fmaster与UART连接
     UART1_TX_OUT();		//TRISC6 = 0; 定义该IO输出
     //UART1_RX_IN();		//TRISC7 = 1;
     UART1_BRR2 = 0x02;         // 设置波特率9600
     UART1_BRR1 = 0x68;         // 16M/9600 = 0x0682
     UART1_CR1 = 0x00;//UART使能，一个起始位，8个数据位，禁止奇偶校验，禁止中断
     UART1_CR3 = 0x00; //一个停止位  
     UART1_CR2 =0x08;//发送使能，屏蔽中断0x06=发送及接收使能
     
  } 

void Uart1SendByte(BYTE chr)
{ 
   FeedWatchDog();
  while(UART1_SR_RXNE);
  while(!UART1_SR_TXE);
  UART1_DR = chr;   //通过UART1发送AD采样结果
  chr=UART1_SR;
  while(!UART1_SR_TC);
  UART1_SR_TC=0;
 
}  
*/
void Init(void)
{
	//-------system clock

									      
   CLK_ICKR=0X01;//internal RC enable
   CLK_ECKR=0;//0X01;
   CLK_CKDIVR=0X02;    //主频为16MHz, cpu时钟为4MHz
   CLK_PCKENR1=0XFF;
   CLK_PCKENR2=0xFF;

 /*  
   
	//-------PORT A, 
	PA_DDR |= 0x0E; //PA1、PA2、PA3 as output
	PA_CR1 = 0x00; // output:0为模拟开漏输出，1为推挽输出
								 // input：0为浮空输入，1为带上拉电阻输入
	PA_CR2 = 0x00; //output:0 为输出速度最大为2MHz，1 为10MHz
								 //input: 0 为禁止外部中断，1为允许
	
	//-------PORT B 
	PB_DDR = 0x71; //
	PB_CR1 = 0x00; // 
	PB_CR2 = 0x00; //
	
        //-------PORT C 
	PC_DDR = 0x65; //
	PC_CR1 = 0x00; // 
	PC_CR2 = 0x00; //
	
	//-------PORT D 
	PD_DDR = 0xBD; //
	PD_CR1 = 0x00; // 
	PD_CR2 = 0x00; //


	//-------PORT E 
	PE_DDR |= 0x20; // PE5 as output port
	PE_CR1 = 0x00; // 
	PE_CR2 = 0x00; //

	//-------PORT F 
	PF_DDR &= 0xEF; // PF4 as input port
	PF_CR1 = 0x00; // 
	PF_CR2 = 0x00; //
*/

//------------------A/D conversion
	
	ADC_CR1 = 0x00; // ADC时钟=主时钟/2=8MHZ,单次转换模式Tad =1/8
	ADC_CR2 = 0x08; // A/D 结果数据右对齐
	ADC_CSR = 0x00; // 清转换结束标志位EOC，
    ADC_CR3 = 0x00;
    ADC_TDRH = 0xFF;  //禁止施密特触发功能
    ADC_TDRL = 0xFF;
}


  unsigned int ADConverse(int channel)
  {
          unsigned int result,i;
          result =0;
          CLK_PCKENR2 |=0x08;//0XFE;
          ADC_CSR = channel; // 选择通道0~15
          for(i=0;i<100;i++); 
          ADC_CR1 |= 0x01; // ADON置1,开始转换
          for(i=0;i<100;i++); //确保ADC 模块的上电完成
          ADC_CR1 = ADC_CR1 | 0x01; // 再次 ADON置1,开始转换
          while(!(ADC_CSR & 0x80)); // 等待ADC 结束，1为结束
      
          result = ADC_DRL; 
          result = (ADC_DRH<<8)|result;
          ADC_CSR &= ~0x80;       // 清除EOC转换结束标志 
          return result;
          
  }
/*
 void Time6Init(void)// T = 2ms
 {
		// Init Timer6, 1ms, up mode
	TIM6_IER = 0x00; // 禁止中断
	
	TIM6_EGR = 0x01; // 允许产生更新事件
	
	TIM6_PSCR = 0x07; // 计数器时钟=主时钟/128=16MHZ/128  // 相当于计数器周期为8uS
	TIM6_ARR = 255; // 设定重装载时的寄存器值，255 是最大值
	
	TIM6_CNTR = 125; // 设定计数器的初值// 定时周期=(ARR+1)*8=2048uS
	
	TIM6_CR1 = 0x01; // b0 = 1,允许计数器工作

	TIM6_IER |= 0x01; // 允许中断


 }

*/
//-------------------------- 
void Get_AD(void)
{
          int i;
	  Motor_val =0;
	  for(i=0;i<16;i++)
	   	 Motor_val += ADConverse(0X02);
	  Motor_val >>=4;

	 Temp_val =0;
	  for(i=0;i<16;i++)
	   	 Temp_val += ADConverse(0X06);
	  Temp_val >>=4;

	   CellsVol_val =0;
	  for(i=0;i<16;i++)
	   	 CellsVol_val += ADConverse(0X01);
	  CellsVol_val >>=4;

	  Chger_val =0;
	  for(i=0;i<16;i++)
	   	 Chger_val += ADConverse(0X03);
	  Chger_val >>=4;
	  
}
//-----------------------------
void OffChg(void)
{
       PRECHARGE_ENABLE =0;
       PRECHARGE_BYPASS =0;
}
//-------------------------------------
void ClearBITS_FLAG(void)
{
  BITS_FLAG.ChgerErr =0;
  BITS_FLAG.Chg =0;
  BITS_FLAG.Dischg =0;
  BITS_FLAG.OvChg =0;
  BITS_FLAG.OvDis =0;
  BITS_FLAG.PreChg =0;
  BITS_FLAG.ChgVolLow =0;
  BITS_FLAG.ChgUnderLow =0;
  
  BITS_FLAG.PreChgTempH  =0;
  BITS_FLAG.PreChgTempL  =0;
  BITS_FLAG.DisTemp =0;
  BITS_FLAG.UnderDisLow =0;
  BITS_FLAG.UnderDis1  =0;
  BITS_FLAG.UnderDis2  =0;
  BITS_FLAG.MotorOvCur =0;
  BITS_FLAG.CellOvChg =0;
  
}
//-------------------------------------
//*********************************************************************


//pwmfreq : 01-65535uS  2Khz=500uS => pwmfreq=500
//pulsewidth  :00-pwmfreq   占空比 pulsewidth =500 = 100%
void Pwm1Set(WORD pwmfreq,WORD pulsewidth )
{
    TIM1_DeInit();
	//set define freq    
    TIM1_TimeBaseInit(14,TIM1_COUNTERMODE_UP,pwmfreq,0);  //16M分频16后，(pwmfreq)１格为1US,
    TIM1_Cmd(ENABLE);
    TIM1_CtrlPWMOutputs(ENABLE);
    
		 /*   
		    TIM1_OC3Init(TIM1_OCMode_TypeDef TIM1_OCMode,
		                  TIM1_OutputState_TypeDef TIM1_OutputState,
		                  TIM1_OutputNState_TypeDef TIM1_OutputNState,
		                  uint16_t TIM1_Pulse,
		                  TIM1_OCPolarity_TypeDef TIM1_OCPolarity,
		                  TIM1_OCNPolarity_TypeDef TIM1_OCNPolarity,
		                  TIM1_OCIdleState_TypeDef TIM1_OCIdleState,
		                  TIM1_OCNIdleState_TypeDef TIM1_OCNIdleState)    
		*/    
    
		//define CHn    
		//TIM1_OC1  对应那一路 PWM ==TIM1_CH1N/PE5,  TIM1_OC2 对应那一路 PWM ==TIM1_CH2N /PC1
		//TIM1_OC1Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_DISABLE,TIM1_OUTPUTNSTATE_ENABLE,pulsewidth, TIM1_OCPOLARITY_HIGH,TIM1_OCNPOLARITY_HIGH,TIM1_OCIDLESTATE_RESET,TIM1_OCNIDLESTATE_RESET);//此模式或产生任意频率的50%占空比方波 (FOSC/分频系数)/(2*TIM2_Period)   
    TIM1_OC1Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_DISABLE,TIM1_OUTPUTNSTATE_ENABLE,pulsewidth, TIM1_OCPOLARITY_HIGH,TIM1_OCNPOLARITY_HIGH,TIM1_OCIDLESTATE_RESET,TIM1_OCNIDLESTATE_RESET);//此模式或产生任意频率的50%占空比方波 (FOSC/分频系数)/(2*TIM2_Period)   
    TIM1_OC1PreloadConfig(ENABLE);  
    
		// TIM1_OC2Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_ENABLE,TIM1_OUTPUTNSTATE_ENABLE,pulsewidth, TIM1_OCPOLARITY_HIGH,TIM1_OCNPOLARITY_HIGH,TIM1_OCIDLESTATE_RESET,TIM1_OCNIDLESTATE_RESET);//此模式或产生任意频率的50%占空比方波 (FOSC/分频系数)/(2*TIM2_Period)   
		//TIM1_OC2PreloadConfig(ENABLE);
		// TIM1_OC3Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_ENABLE,TIM1_OUTPUTNSTATE_ENABLE,pulsewidth, TIM1_OCPOLARITY_HIGH,TIM1_OCNPOLARITY_HIGH,TIM1_OCIDLESTATE_RESET,TIM1_OCNIDLESTATE_RESET);//此模式或产生任意频率的50%占空比方波 (FOSC/分频系数)/(2*TIM2_Period)     
		// TIM1_OC3PreloadConfig(ENABLE);      
		//TIM1_OC4Init(TIM1_OCMODE_PWM1,TIM1_OUTPUTSTATE_ENABLE,pulsewidth,TIM1_OCPOLARITY_HIGH,TIM1_OCIDLESTATE_RESET);
		//TIM1_OC4PreloadConfig(ENABLE);  
    
   TIM1_ARRPreloadConfig(ENABLE);    

		/*
		    TIM1_OC4Init(TIM1_OCMode_TypeDef TIM1_OCMode,
		                  TIM1_OutputState_TypeDef TIM1_OutputState,
		                  uint16_t TIM1_Pulse,
		                  TIM1_OCPolarity_TypeDef TIM1_OCPolarity,
		                  TIM1_OCIdleState_TypeDef TIM1_OCIdleState)
		*/   

    TIM1_Cmd(ENABLE);
}

//------------------------------------------------------------------------------
//PWM5 SET
void Pwm5Set(WORD	pwmfreq,//01-65535uS  2Khz=500uS => pwmfreq=500
	  		WORD	pulsewidth)	//00-pwmfreq   占空比 pulsewidth =500 = 100%
	  
{


    TIM5_DeInit();
	//set freq    
    TIM5_TimeBaseInit( TIM5_PRESCALER_1024,pwmfreq);   
		/*
		    TIM5_OC1Init(TIM5_OCMode_TypeDef TIM5_OCMode,
		                  TIM5_OutputState_TypeDef TIM5_OutputState,
		                  uint16_t TIM5_Pulse,
		                  TIM5_OCPolarity_TypeDef TIM5_OCPolarity)
		*/
		//set tim5 CHn    
	   //TIM5_OC1  对应那一路 PWM ==TIM1_CH1N/PE5,  TIM1_OC2 对应那一路 PWM ==TIM1_CH2N /PC1
	   //TIM5_OC1Init(TIM5_OCMODE_PWM1, TIM5_OUTPUTSTATE_ENABLE,pulsewidth, TIM5_OCPOLARITY_HIGH);//此模式或产生任意频率的50%占空比方波 (FOSC/分频系数)/(2*TIM2_Period)   
	   //TIM5_OC1PreloadConfig(ENABLE); 
   
   TIM5_OC1Init(TIM5_OCMODE_PWM1, TIM5_OUTPUTSTATE_ENABLE,pulsewidth, TIM5_OCPOLARITY_HIGH);//此模式或产生任意频率的50%占空比方波 (FOSC/分频系数)/(2*TIM2_Period)   
   TIM5_OC1PreloadConfig(ENABLE); 
    
   TIM5_ARRPreloadConfig(ENABLE);

   TIM5_Cmd(ENABLE);

}

//----------------------------------------------------------------
/**/
//1ms inturrupt
void	Time6Init(void)
{
    TIM6_DeInit();//fCK_CNT = fCK_PSC/2(PSCR[3:0])
    TIM6_TimeBaseInit(TIM6_PRESCALER_64,250);  //16M/16*1000=1ms    
    TIM6_ITConfig(TIM6_IT_UPDATE,ENABLE);
    TIM6_Cmd(ENABLE);
}

//------------------------------------------------------
/*
void SystemClkSet(void)
{
   CLK_ICKR=0X01;//select fosc source  内部RC //CLK_HSICmd(ENABLE);
   CLK_ECKR=0;//0X01;
   CLK_CKDIVR=0X02;
  // CLK_PCKENR1=0XFF;
   CLK_PCKENR2=0xff;
          //Fmaster = Fosc/1    外设应用频率 16M
          // CLK_SYSCLKConfig(CLK_PRESCALER_HSIDIV1); 
          //Fcpu = Fmaster/1    CPU应用频率 2倍频=8M
          //CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV2); 
          //enable clk Fmaster to Peripheral clk(reset default all enable)  
          //CLK_PeripheralClockConfig(CLK_PERIPHERAL_I2C | CLK_PERIPHERAL_SPI, ENABLE);
          //enable CCS
          //CLK_ClockSecuritySystemEnable();
          //Output Fcpu on CLK_CCO pin
          //CLK_CCOConfig(CLK_OUTPUT_MASTER);
          //设置时钟输出IO
          // GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_HIGH_FAST);
          //CLK_CCOConfig(CLK_OUTPUT_CPUDIV4);
}
*/
//-----------------------------------------------------
void Uart1Init(void)	
{
   CLK_PCKENR1|=0x0c;//0xf3;
   UART1_BRR2 = 0x02;         // 设置波特率9600
   UART1_BRR1 = 0x68;         // 16M/9600 = 0x0682
   UART1_CR1 = 0x00;//0x00;          // 8 bit,n 串口有效 
   UART1_CR2 =0x08;////0x2c;//0;// 0x08;//0x2C;  // 允许接收中断，允许接收，允许发送
      		//UART1_TX_OUT();		//TRISC6 = 0; 定义该IO输出
  			//UART1_RX_IN();		//TRISC7 = 1;
 			// Uart1CmdSend(" Uart1 enable!\n", 14);
}                 
//***********************************************************
        /*#pragma vector=UART1_R_RXNE_vector
        __interrupt void UART1_RX_RXNE1(void)
        {
          unsigned char c;
          //c = UART1_DR;          // 接收到数据了
           while(!UART1_SR_TXE);
          UartRecData[UartRecCnt]=UART1_DR;
          UartRecCnt++;
          c = UART1_SR;
          UART1_SR_RXNE=0;
          while(MASK_UART1_SR_RXNE & c);
          
        }
        */
//******************************************************
//send a byte    
void Uart1SendByte(BYTE c)
{ 
  FeedWatchDog();
  while(UART1_SR_RXNE);
  while(!UART1_SR_TXE);
  UART1_DR = c;   //通过UART1发送AD采样结果
  c=UART1_SR;
  while(!UART1_SR_TC);
  UART1_SR_TC=0;
 
}  

//********************************************    
//send n bytes to the uart send fifo
/*
 void  Uart1CmdSend(const BYTE* str, BYTE n)
{ 
    BYTE i = 0;  
//disable tx interrupt    
    for(i = 0; i < n; i++)
        {
           Uart1SendByte(str[i]);
        }    
}
*/
//--------------------------------------------------
/*
unsigned int AD_start(unsigned char cell)
{
   WORD	 value;
    u8 tempH;
    u8 tempL;
    u8 j;
    value=0;
        // Then read MSB
    CLK_PCKENR2|=0x08;//0XFE;
    ADC_CSR=cell;   // 清除EOC转换结束标志, select channel
    nop();
    nop();
    nop();
    nop();
    /////ADC_CSR &= ~0x80;       // 清除EOC转换结束标志
    ADC_CR1 |= 0x01;           // 开始单次转换
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    ADC_CR1 |= 0x01;           // 开始单次转换
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    nop();
    j=0;
    while(!(ADC_CSR & 0x80))
      { 
         nop();
         nop();    // 等待单次转换完成
         nop();
         nop();
         nop();
         nop();
         nop();
         nop();
         nop();
         nop();
         nop();
         nop();
         j+=1;
         if(j>250)
           break;
      }
    tempL = ADC_DRL;                //先 读低8位
    tempH = ADC_DRH; // 再读高8位，设置数据左对齐
    value=(tempH<<8)|tempL;
    ADC_CSR &= ~0x80;       // 清除EOC转换结束标志   
    return(value); 
}
*/
//***************************************************************************

void PortInit(void)
{
	//GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_Pin_TypeDef GPIO_Pin, GPIO_Mode_TypeDef GPIO_Mode);
/*
	GPIO_MODE_IN_FL_NO_IT 无中断功能的浮动输入。  
	
	GPIO_MODE_IN_PU_NO_IT 无中断功能的上拉输入。  
	
	GPIO_MODE_IN_FL_IT 带中断功能的浮动输入。  
	
	GPIO_MODE_IN_PU_IT 带中断功能的上拉输入。	
	
	GPIO_MODE_OUT_OD_LOW_FAST 高速开漏低电平输出，可工作到10MHz。  
	
	GPIO_MODE_OUT_PP_LOW_FAST 高速推挽低电平输出，可工作到10MHz。  
	
	GPIO_MODE_OUT_OD_LOW_SLOW 低速开漏低电平输出，可工作到2MHz。	
	
	GPIO_MODE_OUT_PP_LOW_SLOW 低速推挽低电平输出，可工作到2MHz。	
	
	GPIO_MODE_OUT_OD_HIZ_FAST 高速开漏高阻态输出，可工作到10MHz。  
	
	GPIO_MODE_OUT_PP_HIGH_FAST 高速推挽高电平输出，可工作到10MHz。  
	
	GPIO_MODE_OUT_OD_HIZ_SLOW 低速开漏高阻态输出，可工作到2MHz。	
	
	GPIO_MODE_OUT_PP_HIGH_SLOW 低速推挽高电平输出，可工作到2MHz。

	*/
/**/
	GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);//AD input
	GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);
	GPIO_Init(GPIOB, GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);
	GPIO_Init(GPIOB, GPIO_PIN_1, GPIO_MODE_IN_FL_NO_IT);
	
	//GPIO_Init(GPIOA, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_FAST);//PORT A
	//GPIO_Init(GPIOA, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_FAST);
	//GPIO_Init(GPIOA, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
	//GPIO_Init(GPIOA, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);	
	GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOA, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOA, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_FAST);
	//GPIO_Init(GPIOA, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST);
	
	GPIO_Init(GPIOB, GPIO_PIN_7, GPIO_MODE_IN_PU_NO_IT);//PORT B
	GPIO_Init(GPIOB, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);	
	GPIO_Init(GPIOB, GPIO_PIN_2, GPIO_MODE_IN_PU_NO_IT);
	//GPIO_Init(GPIOB, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOB, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST);

	
	GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_IN_PU_NO_IT);//PORT C
	GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_IN_PU_NO_IT);	
	GPIO_Init(GPIOC, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOC, GPIO_PIN_1, GPIO_MODE_IN_PU_NO_IT);

	GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_OUT_PP_LOW_FAST);//PORT D
	GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);	
	GPIO_Init(GPIOD, GPIO_PIN_7, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOD, GPIO_PIN_0, GPIO_MODE_OUT_PP_LOW_FAST);

	
	GPIO_Init(GPIOE, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST);
	GPIO_Init(GPIOF, GPIO_PIN_4, GPIO_MODE_IN_PU_NO_IT);

}
//------------------------------
	
void SystemInit(void)
{

  disableInterrupts(); 
                 /* //----------------------------------------------------------------------------
                    //PA_DDR=0x0f;
                    PA_DDR=0x0f;
                    PA_CR1=0x0f;
                    PA_CR2=0x00;
                    //---------------------------------------------------------------------------
                    PB_DDR=0xc8;//0xc0;
                    PB_CR1=0xf0;//0xc0;
                    PB_CR2=0x00;
                    //--------------------------------------------------------------------------------
                    PC_DDR=0x3e;
                    PC_CR1=0xbe;
                    PC_CR2=0x00;
                    //-----------------------------------------------------------------------------------
                    //PD_DDR=0x10;
                    PD_DDR=0x14;
                    PD_CR1=0x17;
                    PD_CR2=0x00;
                    //----------------------------------------------------------------------------
                    PE_DDR=0x20;
                    PE_CR1=0x20;
                    //-------------------------------------------------------------
                    PF_DDR=0x10;
                    PF_CR1=0x10;
                    PF_CR2=0x00;
                    K2=1;
                  */

  //---------------------------------------------------------------------------
  Init();
	PortInit();
  Time6Init();      //１MS时基TIME初始化
  //--------------------------------------------------------------------------
  nop();
  //SystemClkSet();   //系统时钟设置,内部RC 16MHz
  //Uart1Init();
  UART1_CR1 = 0x20; //低功耗，关闭串口  
  //UART1_CR2 = 0x08;//0x2C;          // 允许接收中断，允许接收，允许发送
  UART1_CR2 = 0x00;//0x2C;  
  CLK_PCKENR1&=0xf0;
  
  //AdcInit();        //ADC初始化

 // Pwm1Set(500,0);   //电机调速PWM设置2K;500US
 // Pwm5Set(250,0);   //充电PWM设置500HZ;2000US
 // AT24Init();       //E2PROM初始化
  WatchDogInit();

//---------------------

//-------------------------  
 
  enableInterrupts();
            
            /* 
              EE_L=AT24ReadByte(0x50); 
              EE_H=AT24ReadByte(0x51); 
             
              if(EE_L==0xc3||EE_H==0x3c)
              {
               FloatData1.Data1[0]=AT24ReadByte(0x03);
               FloatData1.Data1[1]=AT24ReadByte(0x02);
               FloatData1.Data1[2]=AT24ReadByte(0x01);
               FloatData1.Data1[3]=AT24ReadByte(0x00);
               r1=FloatData1.Fdata1;
               
               FloatData1.Data1[0]=AT24ReadByte(0x07);
               FloatData1.Data1[1]=AT24ReadByte(0x06);
               FloatData1.Data1[2]=AT24ReadByte(0x05);
               FloatData1.Data1[3]=AT24ReadByte(0x04);
               r2=FloatData1.Fdata1;
               
              // FloatData1.Data1[0]=AT24ReadByte(0xb);
              // FloatData1.Data1[1]=AT24ReadByte(0xa);
              // FloatData1.Data1[2]=AT24ReadByte(0x9);
              // FloatData1.Data1[3]=AT24ReadByte(0x8);
              // r3=FloatData1.Fdata1;
               
               //FloatData1.Data1[0]=AT24ReadByte(0x0f);
               //FloatData1.Data1[1]=AT24ReadByte(0x0e);
               //FloatData1.Data1[2]=AT24ReadByte(0x0d);
               //FloatData1.Data1[3]=AT24ReadByte(0x0c);
               //r4=FloatData1.Fdata1;
               
              // FloatData1.Data1[0]=AT24ReadByte(0x13);
              // FloatData1.Data1[1]=AT24ReadByte(0x12);
               //FloatData1.Data1[2]=AT24ReadByte(0x11);
               //FloatData1.Data1[3]=AT24ReadByte(0x10);
               //r5=FloatData1.Fdata1;
               
               FloatData1.Data1[0]=AT24ReadByte(0x57);
               FloatData1.Data1[1]=AT24ReadByte(0x56);
               FloatData1.Data1[2]=AT24ReadByte(0x55);
               FloatData1.Data1[3]=AT24ReadByte(0x54);
               VCC=FloatData1.Fdata1;
              }
             else
             {
              r1=1.990;///B1
              r2=3.000;//B2
              VCC=3.408;//VCC
             }
             Slect_addr=AT24ReadByte(90);  //slect_adr=90;
             DisLowT=AT24ReadWord(200);
             CTempH=AT24ReadWord(202);
             Rtemp8H=AT24ReadWord(204); 
             CTempL=AT24ReadWord(206); 
             DischarTempT=AT24ReadWord(208);
             CerrTime=AT24ReadWord(210);
             CvT=AT24ReadWord(212);
             OVer6HT=AT24ReadWord(214);
             OVC=AT24ReadWord(216);
             IE=AT24ReadWord(218);
             DischarTempL=AT24ReadWord(220);
              //AT24WriteWord(220,DischarTempL);
             */
 }
//***********************************************************
//************************************************************************************************
//*************************************************************************************************
void main(void)
{
		 /* unsigned char i;
		  nop();
		  nop();
		  nop();
		  nop();
		  SystemInit();
		  K2=1;
		  WaitMs(10);

		 // clearChargFlag();
		  // clearDisFlag();
		 

		   //-------------------------------------------------
		   while(1)
		    {  
		       FeedWatchDog();
		        K1=1;  
		    }
		  */
  int i;
  FirstDisMark =1;//初上电
  FirstChgMark =1;
  Delayms(5);
  //OffDis();
  SystemInit();
  VarInit();
  ClearBITS_FLAG();
  OffLed();
  //-----------------------零漂
  Motor_val =0;
  for(i=0;i<16;i++)
    Motor_val += ADConverse(0x02);
  Motor_val >>=4;
  if(Motor_val <120)
    Motor_Amp =Motor_val;
  else
    Motor_Amp =120;
  
    //-------------------------
  //SUPPLY_LATCH =1;

  SLEEP =0;

  
  while(1)
  {
	 FeedWatchDog();

	Delayms(5);
	
    Get_AD();

	if(Chger_val >50)
	  {
	  	Chger_on =1;
	    if(FirstChgMark ==1 )//如果第一次充电，check 充电器电压
		    {
			      ChgerCheck();
			    
			    if(BITS_FLAG.ChgerErr)//充电器错误
				      {
					    OffChg();	
				        LedChgerErr();
				      }
		    }
		if(BITS_FLAG.ChgerErr ==0 )//充电器电压正常
			{
			    if(VOP_MONITOR)//没有单节电芯过充
			      {
			        SleepTimer =0;
			        FirstChgMark =0;
			        CellOvChg_DelayTimer =0;
			        PreChgCheck();
			        ChgVolCheck();
			        //ChgTempCheck();

			        if(BITS_FLAG.ChgTemp || BITS_FLAG.OvChg || BITS_FLAG.ChgUnderLow)
			          {
						OffChg();
			            if(BITS_FLAG.OvChg)
			              {
			                LedOvChg();
			              }
			            else if(BITS_FLAG.ChgTemp)
			              {
			                LedChgTemp();
			              }
			            else if(BITS_FLAG.ChgUnderLow)//低于起充电压
			            {
			              LedCellsErr();
			            }
			          }
			        else
			          {
			            if(BITS_FLAG.PreChg)
			               {

			                  if(PreChg_OvTimer > PreChg_OvTimer_Set && BITS_FLAG.ChgVolLow)//低压预充超时
			                    {
						          OffChg();
			                      PreChg_OvTimer =0;
			                    }
			                  else if(HighTemp_PreChg_OvTimer >HighTemp_PreChg_OvTimer_Set && BITS_FLAG.PreChgTempH) //高温预充超时
			                    {
						 		  OffChg();
			                      HighTemp_PreChg_OvTimer =0;
			                    }
			                  else if(LowTemp_PreChg_OvTimer >LowTemp_PreChg_OvTimer_Set && BITS_FLAG.PreChgTempL) //低温预充超时
			                    {
						 			OffChg();
			                        LowTemp_PreChg_OvTimer =0;
			                    }
			                  else
			                    {
			                      PRECHARGE_BYPASS =0;
			                      PRECHARGE_ENABLE =1;
			                      LedChg();
			                    }
			              }
			           else if(BigChg_OvTimer > BigChg_OvTimer_Set)
			              {
					  		OffChg();
			                BigChg_OvTimer =0;
			              }
						
			           else
			              {
			                PRECHARGE_ENABLE =0;
			                PRECHARGE_BYPASS =1;
			                LedChg();
			              }

			          }
			      }
			    else 
					{
						if(CellOvChg_DelayTimer > CellOvChg_DelayTimer_Set)//单节过充保护延时 1s
					      {
						 	OffChg();
					        CellOvChg_DelayTimer =0;
							BITS_FLAG.CellOvChg =1;//单节过充
					      }
						else
							{
								BITS_FLAG.CellOvChg =0;
								LedOvChg();
							}
			    	}
								

				}
		}
	else 
		{
			Chger_on =0;//充电器移出
		}


    if(CHARGE_VOLTAGE || Chger_on ==1)//正在充电
      {
        BITS_FLAG.Chg =1;
        BITS_FLAG.Dischg =0;
		FirstDisMark =1;
		OffDis();
      }
    else
      {
        BITS_FLAG.Chg =0;
        FirstChgMark =1;
		OffChg();
        if(FirstDisMark ==1)// 断开充电后或休眠后，需按重启开关产品
          {
            if(SUPPLY_DETECT)// 等待开关重启，断开为高
              {
                FirstDisMark =0;
                BITS_FLAG.Dischg =1;
              }
            else
	            {
	              BITS_FLAG.Dischg =0;
	              OffDis();
	            }
          }
        else
          {
            BITS_FLAG.Dischg =1;
          }
      }

    if(BITS_FLAG.Dischg)// 放电过程
    {

      DisVolCheck();
     //DisTempCheck();
      DisCurCheck();

      if(BITS_FLAG.UnderDisLow || BITS_FLAG.DisTemp ||BITS_FLAG.UnderDis1 ||BITS_FLAG.UnderDis2 ||BITS_FLAG.MotorOvCur)
		{
		  OffDis();
		  if(SleepTimer >10000)//10秒后休眠
		    {
		      SLEEP =1;
		    }

		  if(BITS_FLAG.UnderDisLow)
		    {
		      LedDisLowVol();
		    }
		  else if(BITS_FLAG.UnderDis1 ||BITS_FLAG.UnderDis2)
		    {
		      LedCellsDisLow();
		    }
		  else if(BITS_FLAG.DisTemp)
		  	{
		  		LedDisTemp();
		  	}
		}
      else
        {
          if(!SUPPLY_DETECT)// 开关按下，进行放电状态
          	{
          		SleepTimer =0;
				LedDis();
				OnDis();
          	}
		  else
		  	{	
				OffLed();
				if(SleepTimer >5000)//10秒后休眠
		            {
		              SLEEP =1;
		            }
		  	}
        }
    }
	//GPIO_WriteReverse(GPIOA, GPIO_PIN_1);  
   TEST = ~TEST;
  }
  
}
//***********************************************************************************

              /*
              void  Dwrite_eeprom(void)
              { 
                  //AT24WriteWord(BYTE	adrdata,WORD	n)
                  //Adrr_number+=1;
                unsigned char Adrr_number;
                  
                  if(Slect_addr>=3)
                      Slect_addr=0;
                  
                 
                 if(Slect_addr==0)  //slect_adr=90;
                   Adrr_number=100;
                 else if(Slect_addr==1)
                   Adrr_number=110;
                 else
                    Adrr_number=120;
                 Slect_addr+=1;
                 AT24WriteByte(90,Slect_addr);
                 //----------------------------------------------------------------------------------
                if(Dischargebits.Bocd_f)
                    alarm_type=0x34; 
                 AT24WriteByte(Adrr_number,alarm_type);
                  Adrr_number+=1; 
              
               AT24WriteWord(200,DisLowT);//多少次过放
               AT24WriteWord(202,CTempH);//充电的时候多少次超高温；
               AT24WriteWord(204,Rtemp8H);//充电的时候多少次8H后没有恢复温度
               AT24WriteWord(208,DischarTempT);//充电的时候多少次8H后没有恢复温度
               AT24WriteWord(206, CTempL);
               AT24WriteWord(210,CerrTime);
               AT24WriteWord(212,CvT);  
               AT24WriteWord(214,OVer6HT);  
               AT24WriteWord(216,OVC);  
               AT24WriteWord(218,IE);
               AT24WriteWord(220,DischarTempL);
               
              }
              //**************************************************************************
              //**************************************************************************
              void SendUart(void)
               {
                 
                   unsigned char recode;
                   Uart1CmdSend("-------\n ",8);  //FOR DEBUG 
                   recode=AT24ReadByte(100);
                   if(recode)      //第一次报警
                   {
                     Uart1CmdSend("3 log 1:", 8);
                     Uart1SendByte(recode);
                     Uart1SendByte(0X0D);
                     Uart1SendByte(0X0A);
                   } 
                //---------
                   recode=AT24ReadByte(110);
                   if(recode)      //第二次报警
                   {
                     Uart1CmdSend("3 log 2:", 8);
                     Uart1SendByte(recode);
                     Uart1SendByte(0X0D);
                     Uart1SendByte(0X0A);
                    } 
               //--------    
                   recode=AT24ReadByte(120); //第三次报警
                   if(recode)
                   {
                     Uart1CmdSend("3 log 3:", 9);
                     Uart1SendByte(recode);
                    Uart1SendByte(0X0D);
                    Uart1SendByte(0X0A);
                    } 
                   //-----------
                   Uart1CmdSend("OV_CH:",6 );
                   HexBcd(OVC);     //多少次过充
                   Uart1SendByte(0x0d);
                    Uart1SendByte(0x0a);
                   
                   Uart1CmdSend("HT_CH:",6 );
                   HexBcd(CTempH);       //充电的时候多少次超高温；
                   Uart1SendByte(0x0d);
                    Uart1SendByte(0x0a);
                   
                   Uart1CmdSend("LT_CH:",6 );
                   HexBcd(CTempL);       //充电的时候多少次超低温；
                   Uart1SendByte(0x0d);
                    Uart1SendByte(0x0a);
                   
                   Uart1CmdSend("8H_CH:",6);
                   HexBcd(Rtemp8H);          //充电的时候多少次8H后没有恢复温度
                   Uart1SendByte(0x0d);
                    Uart1SendByte(0x0a);
                   
                   Uart1CmdSend("6H_6H:",6);
                   HexBcd(OVer6HT);//多少次6小时还没有充饱；
                   Uart1SendByte(0x0d);
                    Uart1SendByte(0x0a);
                   
                   Uart1CmdSend("PRE_CH:",7);
                   HexBcd(CvT);//多少次CV充电没有恢复到大电流充电；
                    Uart1SendByte(0x0d);
                    Uart1SendByte(0x0a);
                    
                    Uart1CmdSend("IE_CH:",6);
                    HexBcd(CerrTime);//插错充电器多少次；
                    Uart1SendByte(0x0d);
                    Uart1SendByte(0x0a);
                  //------------------------------     
                   Uart1CmdSend("UV_DIS:",7 );
                   HexBcd(DisLowT);     //多少次过放
                   Uart1SendByte(0x0d);
                   Uart1SendByte(0x0a);
                   
                   Uart1CmdSend("HT_DIS:",7);
                   HexBcd(DischarTempT);//放电多少次超温
                   Uart1SendByte(0x0d);
                    Uart1SendByte(0x0a);
                    
                   Uart1CmdSend("LT_DIS:",7);
                   HexBcd(DischarTempL);//放电多少次超低温
                   Uart1SendByte(0x0d);
                    Uart1SendByte(0x0a);
              
                   Uart1CmdSend("OI_DIS:",7);
                   HexBcd(IE);//放电多少次过流
                  Uart1SendByte(0x0d);
                   Uart1SendByte(0x0a);
                   Uart1CmdSend("-------",8);  //FOR DEBUG 
                   Uart1SendByte(0x0d);
                   Uart1SendByte(0x0a);
                  //-------------------------------- 
                    Uart1CmdSend("TOV:",4);  //FOR DEBUG 
                    HexBcd(TOV);
                    Uart1SendByte('m');
                    Uart1SendByte('V');
                    Uart1SendByte(0x0d);
                    Uart1SendByte(0x0a);
                    
                    Uart1CmdSend("B1:",3); 
                    HexBcd(B1);//
                    Uart1SendByte('m');
                    Uart1SendByte('V');
                    Uart1SendByte(0x0d);
                    Uart1SendByte(0x0a);
                    
                    Uart1CmdSend("B2:",3); 
                    HexBcd(B2);//
                    Uart1SendByte('m');
                    Uart1SendByte('V');
                    Uart1SendByte(0x0d);
                    Uart1SendByte(0x0a);
                    
                   // Uart1CmdSend("B3:",3); 
                    //HexBcd(B3);//充
                    //Uart1SendByte('m');
                    //Uart1SendByte('V');
                    //Uart1SendByte(0x0d);
                    //Uart1SendByte(0x0a);
                    
                   // Uart1CmdSend("B4:",3); 
                   // HexBcd(B4);//充
                   // Uart1SendByte('m');
                   // Uart1SendByte('V');
                   // Uart1SendByte(0x0d);
                   // Uart1SendByte(0x0a);
                    
                   // Uart1CmdSend("B5:",3); 
                    //HexBcd(B5);//充
                    //Uart1SendByte('m');
                   // Uart1SendByte('V');
                    //Uart1SendByte(0x0d);
                    //Uart1SendByte(0x0a);
                  
                   Uart1CmdSend("Temp:",5);  //FOR DEBUG 
                   DisplayTemp();
                   Uart1CmdSend("C",1);
                   Uart1SendByte(0x0d);
                   Uart1SendByte(0x0a);
                   
                   if(Charge.CH_f)
                      Uart1CmdSend("Charge",6); 
                  else
                       Uart1CmdSend("Dis",3); 
                   Uart1SendByte(0X0D);
                   Uart1SendByte(0X0A);
                
                   Uart1CmdSend("MOT:",4);
                   Uart1CmdSend(" ",2);
                   if(Charge.CH_f==0&&mark.work)
                    {
                    if(Dischargebits.Speed_f)
                       Uart1CmdSend("Max",3);
                    else
                       Uart1CmdSend("Min",3);
                    }
                   else
                   { 
                   Uart1CmdSend("Stop",4);
                   }
                  //-----------------------------------------------------------------------------     
                   Uart1SendByte(0X0D);
                   Uart1SendByte(0X0A);
                   Uart1CmdSend("Alarm:",7); 
                   Uart1SendByte(alarm_type);
                   Uart1SendByte(0X0D);
                   Uart1SendByte(0X0A);
                   //-----------------------------------------------------------------------------
                   // Uart1CmdSend("B5:",3); 
                    //HexBcd(B5);//
                   //------------------------------------------------------------------------------      
                  
               }
              //**************************************************************************
              //***************************************************************************
              void HexBcd(unsigned int hex)
                {
                     
                    unsigned char v1,v2,v3,v4,v5;
                    v1=0;
                    v2=0;
                    v3=0;
                    v4=0;
                    v5=0;
                    if(hex>=65000)
                      hex=65000;
                     while(hex>=10000)
                    {
                     hex-=10000;
                     v5+=1;
                    }
                    while(hex>=1000)
                    {
                     hex-=1000;
                     v4+=1;
                    }
                     while(hex>=100)
                    {
                     hex-=100;
                     v3+=1;
                    } 
                    while(hex>=10)
                    {
                     hex-=10;
                     v2+=1;
                    }  
                   v1=(char)hex;
                   v1|=0x30;
                   v2|=0x30;
                   v3|=0x30;
                   v4|=0x30;
                   v5|=0x30;
                    Uart1SendByte(v5);
                   Uart1SendByte(v4);
                   Uart1SendByte(v3);
                   Uart1SendByte(v2);
                   Uart1SendByte(v1);
                 }
              //*************************************************************************
              //************************************************************************* 
              */
