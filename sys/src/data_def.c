//#include "stdio.h"
#include "macro_def.h" 


  unsigned int ChargeMode_Exchange_Delay = 0;
  unsigned int DischargeMode_Exchange_Delay = 0;
  unsigned int IdleMode_Exchange_Delay = 0;
  unsigned int Adc_value[18];
  unsigned int Cell_Volt_Ad[3]; 
  unsigned int Cell_Volt[3];  
  unsigned int Cell_Ratio[3];  
  unsigned int Vcc_Volt; 
  unsigned int Cell_Volt_Tol;  
  unsigned int Cell_Volt_Max;    
  unsigned int Cell_Volt_Min;    
  unsigned int Cell_Volt_Avg;  
  unsigned int DisCur_Val;  
  unsigned int DisCur_Bias_Val;  
  unsigned int Temp_Val; 

  unsigned int DisOv_t;
  unsigned int ChgOv_t;
  unsigned int DisCurOv_I_t;
  unsigned int DisCurOv_II_t; 
  unsigned int DisCurOv_Re_t; 
  unsigned int ChgCurOv_Re_t;
  unsigned int DisCur_Val_Tmp_Last_Len ;
  unsigned int DisCur_Val_Tmp_Last[16];
  


  unsigned int  LedShow_Flash_t =0;





//--------------------------------------
union UCHAR_ULONG
{
    unsigned char uchardata[4]; 
    unsigned long longdata;
}SendComData,tt;

  union UINT_UCHAR
{
    unsigned int uintdata;
    unsigned char uchardata[2];
} RevcComData,ReadEEPData;

//------------------------------------------------------------------
struct FLAG_BITS
{
    unsigned char Chg             : 1;
    unsigned char ChgOv           : 1; 
    unsigned char ChgTemp         : 1; 
    unsigned char Dis             : 1;
    unsigned char DisOv           : 1;
    unsigned char DisTemp         : 1;   
    unsigned char DisCurOv        : 1;  
} Bits_flag;
 
enum  em_workmode
{
   IDLE_MODE 	    =0x00,
   CHARGE_MODE 	    =0x01,
   DISCHARGE_MODE   =0x02
}WorkMode;