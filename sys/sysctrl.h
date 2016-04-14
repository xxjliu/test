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

//timer数据结构定义
typedef struct _SYS_DATA
{
	BYTE		State;						//状态机
	BYTE		KeyValue;					//按键值 
        BYTE            T10msFlag;
        BYTE            T100msFlag;        
        BYTE            T500msFlag;        
	BYTE		Sec;						//Even        
	BYTE		Min;						//Even
	BYTE		Hour;						//State
        
}SYS_DATA;

extern  SYS_DATA gSys;




//timer数据结构定义
typedef struct _TIME_DATA
{
	WORD		MS;						//Freq
	BYTE		Sec;						//Width
	BYTE		Min;						//Even
	BYTE		Hour;						//State
}TIME_DATA;

extern  TIME_DATA gRTC,gTime;

extern  WORD  gSysTick;


//用作1MS时基单元产生
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
//1) DeviceStatus   设备状态机定义
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
2）TempStatus 温度状态定义					
0：没有报警					
3：充电过温警示，停止当前动作（充电）					
4：放电过温警示，停止当前动作（放电）					
1: 等待执行当前动作（充电），继续检测温度(启动计时器）	
*/
typedef enum {
  T_NoAlarm_0=0,		//	0：没有报警
  T_OCT_WAIT_1,		//	1: 等待执行当前动作（充电），继续检测温度(启动计时器）
  T_REVD_2,		//	2: 保留
  T_OCT_3,		//	3：充电过温警示，停止当前动作（充电）
  T_ODT_4		//	4：放电过温警示，停止当前动作（放电）
}TempStatus_TypeDef;



//3）VoltStatus 电压状态定义
typedef enum {			
ALARM_0=0,		//	0：ALARM
V_FULL_1,		//	1：满电
V_DISCHARGE_I_2,		//	2：放电正常1
V_DISCHARGE_II_3,		//	3：放电正常2
V_OVERDISCHARGE_4,		//	4：过放
V_PRE_CHARGE_5,		//	5：充电预充
V_CC_CHARGING_6,		//	6：充电正常
V_CV_CHARGE_4V_7,		//	7：充电>4V
V_CHARGING_STANDBY_8		//	8：OCV,充电待命
}VoltStatus_TypeDef;



//4）CurrStatus 电流状态定义
typedef enum {	
  C_NORMAL_0=0,         //正常
  C_OVERCURRENT_1 //过流
}CurrStatus_TypeDef;


//5）AlarmProtectionStatus 警告保护状态定义
typedef enum {	
A_NORMAL_0=0,		//	0：Normal
A_OCT_1,		//	1：充电过温
A_OCV_2,		//	2：过充
A_ODT_3,		//	3：放电过温
A_ODV_4,	//	4：过放
A_ODC_5,		//	5：过流
}AlarmProtectionStatus_TypeDef;


//6）OnOff_Status 开关机状态定义	
typedef enum {		
S_OFF_0=0,	//	关机
S_ON_1	    //	开机
}OnOff_Status_TypeDef;	


//7）II_Status 马达档位状态定义	
typedef enum {			
D_MIN_0=0,	//	0： MIN 低速
D_MAX_1	    //	1： MAX 高速
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
//8）Cells_V(5) 电芯电压变量定义
extern  u16 Cells_mV[7];
//Cells_V[5]  -> Current OC mV
//Cells_V[6]  -> NTC mV
//u16 AdValues(10);
//9）TempValue 温度变量定义
extern  s16 TempValue;

extern  u16 Pack_V;	//整体电压
extern  u16 Cell_MinV;	//电芯最小值
extern  u16 Cell_MaxV;	//电芯最大值
extern  u16 CompensatesV; //电压补偿
extern  s16 CompensateT;  //温度补偿值
extern  u16 CompensateI; //电流补偿值
extern  u16 ADVmultiple;  //AD放大倍数
extern  u8 MotorActiveStatus; //马达状态： 2-高速，1- 低速，0-停止			
extern  u8 TestMode;  //测试模式:1,正常模式，0：测试模式，2；校准模式

extern  u8 ChargeFlag;  //测试模式:1,正常模式，0：测试模式，2；校准模式

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


