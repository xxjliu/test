/*
*****************************************************************************
*				All rights reserved.
*
* Filename:			sysctrl.h
* Description:		sysctrl head file
*
* Change History:
*			Goldjun     -- 10/30'2012 - Ver0.1
*			            -- created
*
*
*
*
******************************************************************************
*/
#ifndef __SYSCTRL_H__
#define __SYSCTRL_H__
#include "stm8s.h"
#include "type.h"

//timer���ݽṹ����
typedef struct _SYS_DATA
{
	BYTE		State;						//״̬��
	BYTE		KeyValue;					//����ֵ 
        BYTE            T10msFlag;
        BYTE            T100msFlag;        
        BYTE            T500msFlag;        
	BYTE		Sec;						//Even        
	BYTE		Min;						//Even
	BYTE		Hour;						//State
        
}SYS_DATA;

extern  SYS_DATA gSys;




//timer���ݽṹ����
typedef struct _TIME_DATA
{
	WORD		MS;						//Freq
	BYTE		Sec;						//Width
	BYTE		Min;						//Even
	BYTE		Hour;						//State
}TIME_DATA;

extern  TIME_DATA gRTC,gTime;

extern  WORD  gSysTick;


//����1MSʱ����Ԫ����
extern void	
	TimerCnt(
		void
		);
extern void
        MotoSetPwm(
              WORD  pulse
                );










/* Includes ------------------------------------------------------------------*/
// * xu add*
//// define const ***********************************
//1) DeviceStatus   �豸״̬������
typedef enum {
  DeviceOFF_0 = 0,
  MIN_Discharge_1,
  MAX_Discharge_2,
  Charge_Start_after_discharge_3,
  Charge_Start_at_power_up_4,
  Charging_5,
  AlarmProtection_6,
  Standby_7,
  PowerUp_8
}DeviceStatus_TypeDef;


/*
2��TempStatus �¶�״̬����					
0��û�б���					
3�������¾�ʾ��ֹͣ��ǰ��������磩					
4���ŵ���¾�ʾ��ֹͣ��ǰ�������ŵ磩					
1: �ȴ�ִ�е�ǰ��������磩����������¶�(������ʱ����	
*/
typedef enum {
  T_NoAlarm_0=0,		//	0��û�б���
  T_OCT_WAIT_1,		//	1: �ȴ�ִ�е�ǰ��������磩����������¶�(������ʱ����
  T_REVD_2,		//	2: ����
  T_OCT_3,		//	3�������¾�ʾ��ֹͣ��ǰ��������磩
  T_ODT_4		//	4���ŵ���¾�ʾ��ֹͣ��ǰ�������ŵ磩
}TempStatus_TypeDef;



//3��VoltStatus ��ѹ״̬����
typedef enum {			
ALARM_0=0,		//	0��ALARM
V_FULL_1,		//	1������
V_DISCHARGE_I_2,		//	2���ŵ�����1
V_DISCHARGE_II_3,		//	3���ŵ�����2
V_OVERDISCHARGE_4,		//	4������
V_PRE_CHARGE_5,		//	5�����Ԥ��
V_CC_CHARGING_6,		//	6���������
V_CV_CHARGE_4V_7,		//	7�����>4V
V_CHARGING_STANDBY_8		//	8��OCV,������
}VoltStatus_TypeDef;



//4��CurrStatus ����״̬����
typedef enum {	
  C_NORMAL_0=0,         //����
  C_OVERCURRENT_1 //����
}CurrStatus_TypeDef;


//5��AlarmProtectionStatus ���汣��״̬����
typedef enum {	
A_NORMAL_0=0,		//	0��Normal
A_OCT_1,		//	1��������
A_OCV_2,		//	2������
A_ODT_3,		//	3���ŵ����
A_ODV_4,	//	4������
A_ODC_5,		//	5������
}AlarmProtectionStatus_TypeDef;


//6��OnOff_Status ���ػ�״̬����	
typedef enum {		
S_OFF_0=0,	//	�ػ�
S_ON_1	    //	����
}OnOff_Status_TypeDef;	


//7��II_Status ��ﵵλ״̬����	
typedef enum {			
D_MIN_0=0,	//	0�� MIN ����
D_MAX_1	    //	1�� MAX ����
}II_Status_TypeDef;




extern  DeviceStatus_TypeDef DeviceStatus;
extern  TempStatus_TypeDef TempStatus;
extern  VoltStatus_TypeDef VoltStatus;
extern  CurrStatus_TypeDef CurrStatus;
extern  AlarmProtectionStatus_TypeDef AlarmProtectionStatus;
extern  OnOff_Status_TypeDef OnOff_Status;	
extern  II_Status_TypeDef II_Status;
//extern  u8 CurrStatus;
//***********************
//8��Cells_V(5) ��о��ѹ��������
extern  u16 Cells_mV[7];
//Cells_V[5]  -> Current OC mV
//Cells_V[6]  -> NTC mV
//u16 AdValues(10);
//9��TempValue �¶ȱ�������
extern  s16 TempValue;

extern  u16 Pack_V;	//�����ѹ
extern  u16 Cell_MinV;	//��о��Сֵ
extern  u16 Cell_MaxV;	//��о���ֵ
extern  u16 CompensatesV; //��ѹ����
extern  s16 CompensateT;  //�¶Ȳ���ֵ
extern  u16 CompensateI; //��������ֵ
extern  u16 ADVmultiple;  //AD�Ŵ���
extern  u8 MotorActiveStatus; //���״̬�� 2-���٣�1- ���٣�0-ֹͣ			
extern  u8 TestMode;  //����ģʽ:1,����ģʽ��0������ģʽ��2��У׼ģʽ

extern  u8 ChargeFlag;  //����ģʽ:1,����ģʽ��0������ģʽ��2��У׼ģʽ

/**************************************************/
extern   u16 LED_Time_mS;	//LED flash jishi
extern   u8 LED_Time_flash;	//LED flash flag
extern   u16 LED_Time_flash_uS;	//LED flash jishi
extern   u8 LED_Time_flash_b;	//LED flash ON LEN
extern   u8 LED_Time_flash_a;	//LED flash per
extern   u8 LED_Time_flash_c;	//LED flash 2s

 extern  u8 TempTimeOut;
 extern  u8 TempTimeOutStart;
 extern  u16 TempTime_Min;


/*******************************************/
extern void GPIO_INIT_5S(void);
extern void LED_Slow_ONOFF(void);
/*********************************************/
extern void LEDShow(VoltStatus_TypeDef cVoltStatus);





#endif


