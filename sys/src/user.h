/*
 *****************************************************************************
 *					PACE ELECTRONICS CO.,LTD
 *	Copyright 2012, PACE ELECTRONICS CO.,LTD  Shenzhen, China
 *					All rights reserved.
 *
 * Filename:			bat.h
 * Description:			bat head file
 *
 * Change History:
 *			Goldjun	- 09/13'2012 - Ver 0.1
 *					- created
 *			xxxxx	- xx/xx/20xx - Verx.x
 *					- change code
 ******************************************************************************
 */ 

//#include "macro_def.h"
 //#include "bq769x0.h"

#ifndef	__USER_H__
#define __USER_H__

#include "user.h"
#include "macro_def.h"
#include "iostm8s003f3.h" 
//==================================================================
/*

项目                      Item										常温25℃                                                                     Normal Temperature  25℃ 																				
								最小值(Min)	                常值(Type)	                        最大值(Max)						测试(Test)			特殊控制点		
过放保护电压 (V)        Over discharge detection voltage	4.2		                4.23		                        4.26											
过放保护延时 (mS)       Over discharge detection voltage	500				1000					1500											
过放保护电压 (V)        Over discharge detection voltage	2.65				2.7					2.75											
过放保护延迟时间(ms)    Over discharge detection delay 		200				400					600											
放电过流1保护电流(A)    Over current1  detection  current  	17				20					23											
放电过流1保护延时(s)    Over current  detection delay 		10				20					30											
放电过流2保护电流(A)    Over current2  detection  current  	22				25					28											
放电过流2保护延时(s)    Over current2  detection delay 		0.4				0.5					0.6											
充电低温保护(°C)       charge　Over temperature protect 	-3（NTC阻值：32.4037KΩ）	0（NTC阻值：28.2671KΩ）		+3（NTC阻值：24.7264KΩ）											
充电高温保护(°C)       charge　Over temperature protect 	42（NTC阻值：5.3793KΩ）	45（NTC阻值：4.8520KΩ）		48（NTC阻值：4.3840KΩ）											
"放电低温保护(°C)      
discharge Over temperature protect  "	-25（NTC阻值：96.862KΩ）	-20（NTC阻值：74.3538KΩ）		-15（NTC阻值：57.6261KΩ）											
"放电高温保护(°C)      
discharge Over temperature protect "	62（NTC阻值：2.7905KΩ）	65（NTC阻值：2.5442KΩ）		68（NTC阻值：2.3229KΩ）											
静态(不按开关)自耗电(uA)Current consumption  				                 	10											
充电不放电																														
短路保护  Circuit   Short																														
ESD防护										需重点加强ESD防护（如NTC端子，LED灯)																		T1		
充电电量显示		亮四个灯LED1,LED2,LED3，LED4(绿灯)					11.4±0.3																		电量显示及照明灯测试时,产线只需测绿灯和白灯亮即可,各电压点显示情况需抽检测试.		
			亮三个灯LED1,LED2,LED3(绿灯)						10.1±0.3									11.4±0.3											
			亮两个灯 LED2,LED3(绿灯)						8.8±0.3									10.1±0.3											
			亮一个灯LED1(绿灯)						        8.1±0.3									8.8±0.3											
			LED1闪烁								8.1±0.3											
烧录程序信息							烧录程序版本：																						
								PCM 设计 VCC 电压：						5V																
								校准基准电压：																						
								校准误差计算公式：																						
	注意：																													

*/
//==================================================================
//==================================================================
void PWM2_Init(void);
void PWM1_Init(void);
unsigned int ADC(int channel);
unsigned int ADConverse(unsigned char channel);
void SysInit(void);
void ADC_Init(void);
void ClrWdt(void);
void PortInit(void);
void VarInit(void);
void Timer2Init(void);
void Timer4Init(void);
void Timer2Init_backup(void);
void TempCheck(void);
void CurrentCheck(void);
void BatVolCheck(void);
void ModeCheck(void);
void FET_ChgDis_Cntrl(void);
void WorkLedShow(void);
void LedShow(void);
void StatusClear(void);
void VarClear(void);


//===========================================================
#define ADDR_BASE       0x004000
#define SOC_ADDR          0x004004 
//=== ADDR_BASE ==0x004000    ADDR_END ==0x00407F
//void FLASH_ProgramByte(uint32_t Address, uint8_t Data);
//uint8_t FLASH_ReadByte(uint32_t Address);
//void FLASH_ProgramWord(uint32_t Address, uint32_t Data);
void SOC_Init(void);
void SOC_SavedtoEEPROM(void);


extern unsigned int ChargeMode_Exchange_Delay ;
extern unsigned int DischargeMode_Exchange_Delay ;
extern unsigned int IdleMode_Exchange_Delay ;
extern unsigned int Adc_value[18];
extern unsigned int Cell_Volt_Ad[3]; 
extern unsigned int Cell_Volt[3];  
extern unsigned int Cell_Ratio[3];  
extern unsigned int Vcc_Volt;  
extern unsigned int Cell_Volt_Tol;  
extern unsigned int Cell_Volt_Max;    
extern unsigned int Cell_Volt_Min;    
extern unsigned int Cell_Volt_Avg;  
extern unsigned int DisCur_Val;  
extern unsigned int DisCur_Bias_Val;  
extern unsigned int Temp_Val; 

extern unsigned int DisOv_t;
extern unsigned int ChgOv_t;
extern unsigned int DisCurOv_I_t;
extern unsigned int DisCurOv_II_t;
extern unsigned int DisCurOv_Re_t;  
extern unsigned int ChgCurOv_Re_t;
extern unsigned int  DisCur_Val_Tmp_Last_Len ;
extern unsigned int  DisCur_Val_Tmp_Last[16];
 
extern unsigned int  LedShow_Flash_t;







//--------------------------------------
extern union UCHAR_ULONG
{
    unsigned char uchardata[4]; 
    unsigned long longdata;
}SendComData,tt;

extern union UINT_UCHAR
{
    unsigned int uintdata;
    unsigned char uchardata[2];
} RevcComData,ReadEEPData;

//------------------------------------------------------------------
extern struct FLAG_BITS
{
    unsigned char Chg             : 1;
    unsigned char ChgOv           : 1; 
    unsigned char ChgTemp         : 1; 
    unsigned char Dis             : 1;
    unsigned char DisOv           : 1;
    unsigned char DisTemp         : 1;   
    unsigned char DisCurOv        : 1;  
    unsigned char ChgCurOv        : 1;  
} Bits_flag;
 
extern enum  em_workmode
{
   IDLE_MODE 	    =0x00,
   CHARGE_MODE 	    =0x01,
   DISCHARGE_MODE   =0x02
}WorkMode;




#endif










