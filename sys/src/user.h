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

��Ŀ                      Item										����25��                                                                     Normal Temperature  25�� 																				
								��Сֵ(Min)	                ��ֵ(Type)	                        ���ֵ(Max)						����(Test)			������Ƶ�		
���ű�����ѹ (V)        Over discharge detection voltage	4.2		                4.23		                        4.26											
���ű�����ʱ (mS)       Over discharge detection voltage	500				1000					1500											
���ű�����ѹ (V)        Over discharge detection voltage	2.65				2.7					2.75											
���ű����ӳ�ʱ��(ms)    Over discharge detection delay 		200				400					600											
�ŵ����1��������(A)    Over current1  detection  current  	17				20					23											
�ŵ����1������ʱ(s)    Over current  detection delay 		10				20					30											
�ŵ����2��������(A)    Over current2  detection  current  	22				25					28											
�ŵ����2������ʱ(s)    Over current2  detection delay 		0.4				0.5					0.6											
�����±���(��C)       charge��Over temperature protect 	-3��NTC��ֵ��32.4037K����	0��NTC��ֵ��28.2671K����		+3��NTC��ֵ��24.7264K����											
�����±���(��C)       charge��Over temperature protect 	42��NTC��ֵ��5.3793K����	45��NTC��ֵ��4.8520K����		48��NTC��ֵ��4.3840K����											
"�ŵ���±���(��C)      
discharge Over temperature protect  "	-25��NTC��ֵ��96.862K����	-20��NTC��ֵ��74.3538K����		-15��NTC��ֵ��57.6261K����											
"�ŵ���±���(��C)      
discharge Over temperature protect "	62��NTC��ֵ��2.7905K����	65��NTC��ֵ��2.5442K����		68��NTC��ֵ��2.3229K����											
��̬(��������)�Ժĵ�(uA)Current consumption  				                 	10											
��粻�ŵ�																														
��·����  Circuit   Short																														
ESD����										���ص��ǿESD��������NTC���ӣ�LED��)																		T1		
��������ʾ		���ĸ���LED1,LED2,LED3��LED4(�̵�)					11.4��0.3																		������ʾ�������Ʋ���ʱ,����ֻ����̵ƺͰ׵�������,����ѹ����ʾ����������.		
			��������LED1,LED2,LED3(�̵�)						10.1��0.3									11.4��0.3											
			�������� LED2,LED3(�̵�)						8.8��0.3									10.1��0.3											
			��һ����LED1(�̵�)						        8.1��0.3									8.8��0.3											
			LED1��˸								8.1��0.3											
��¼������Ϣ							��¼����汾��																						
								PCM ��� VCC ��ѹ��						5V																
								У׼��׼��ѹ��																						
								У׼�����㹫ʽ��																						
	ע�⣺																													

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










