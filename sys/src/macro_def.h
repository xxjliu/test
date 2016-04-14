 
#include "type.h"
//==================================
#define   IS_CHG_OC_ON()        (PA_IDR_IDR1==0)
#define   IS_CHG_OC_OFF()       (PA_IDR_IDR1==1)
#define   IS_BIG_LOAD_DET()     (PA_IDR_IDR2==0)
#define   IS_CHGER_IN()         (PB_IDR_IDR5==1) 
#define   IS_MCU_DO_OFF()       (PB_IDR_IDR4==1) 
#define   IS_Pre_Dis_ON()      (PD_IDR_IDR1==1) 
#define   IS_Pre_Dis_OFF()      (PD_IDR_IDR1==0) 

#define   MCU_CO_ON()    (PC_ODR_ODR3 = 1) 
#define   MCU_CO_OFF()   (PC_ODR_ODR3 = 0) 

#define   MCU_DO_ON()    (PB_ODR_ODR4 = 0) 
#define   MCU_DO_OFF()   (PB_ODR_ODR4 = 1) 
#define   MCU_DO_XOR()   (PB_ODR_ODR4 = ~PB_ODR_ODR4) 

#define   Pre_Dis_ON()    (PD_ODR_ODR1 = 1) 
#define   Pre_Dis_OFF()   (PD_ODR_ODR1 = 0) 
#define   VREF_ON()       (PA_ODR_ODR3 = 1) 
#define   VREF_OFF()      (PA_ODR_ODR3 = 0) 
#define   VREF_XOR()      (PA_ODR_ODR3 = ~PA_ODR_ODR3) 

#define   CON_ON()      // (PD_ODR_ODR4 = 1) 
#define   CON_OFF()     // (PD_ODR_ODR4 = 0) 
#define   CON_XOR()     // (PD_ODR_ODR4 = ~PD_ODR_ODR4) 

#define   LED1_ON()    (PC_ODR_ODR6 =0)    
#define   LED1_OFF()   (PC_ODR_ODR6 =1)    
#define   LED2_ON()    (PC_ODR_ODR7 =0)  
#define   LED2_OFF()   (PC_ODR_ODR7 =1)  
//#define   LED_3ON_4OFF()    (PC_ODR_ODR5 =0); GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_FAST) 
#define   LED_3ON_4OFF()    (PC_ODR_ODR5 =0); GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_LOW_FAST) 
#define   LED_3OFF_4ON()    (PC_ODR_ODR5 =1); GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST); 
#define   LED_3OFF_4OFF()   (PC_ODR_ODR5 =1); GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_FAST)    
#define   LED1_XOR()    (PC_ODR_ODR6 =~PC_ODR_ODR6)    
#define   LED2_XOR()    (PC_ODR_ODR7 =~PC_ODR_ODR7)   
//============================================================================= 
 
#define MAX_UINT16_NUM        0x7FFF
#define DisCur_Val_I_SET      800   // (20 *21 *0.005 *1024 )/5 = 430
#define DisCur_Val_II_SET     1500   // (25 *21 *0.005 *1024 )/5 = 537
#define DisCurOv_I_t_SET      300//2000
#define DisCurOv_II_t_SET     50 

#define ChgTempH_ON_SET       334   //45
#define ChgTempH_OFF_SET      417   //35 
#define ChgTempL_OFF_SET      662   //10
#define ChgTempL_ON_SET       757   //0 

#define DisTempH_ON_SET       207   //65
#define DisTempH_OFF_SET      263   //55
#define DisTempL_OFF_SET      839   //-10
#define DisTempL_ON_SET       903   //-20


#define ChgOv_SET             4230  //866   // 4.23v
#define ChgOv_Re_SET          4100  //839   // 4.10v
#define DisOv_Min_SET         2700  //553   // 2.7v
#define DisOv_Avg_SET         3000  //614   // 3.0v
#define DisOv_Re_SET          3200  //655   // 3.2v
  
#define ChgOv_t_SET           100   // 1000ms
#define DisOv_t_SET           40    // 400ms
#define ChgCurOv_Re_t_SET     2000  // 20s