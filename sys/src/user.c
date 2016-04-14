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
//===========================================================
#define ADJUST_ADDR       0x004000
#define SOC_ADDR          0x004004 
//=== ADDR_BASE ==0x004000    ADDR_END ==0x00407F
//void FLASH_ProgramByte(uint32_t Address, uint8_t Data);
//uint8_t FLASH_ReadByte(uint32_t Address);
//void FLASH_ProgramWord(uint32_t Address, uint32_t Data); 
//==================================================================
//==================================================================
void PWM2_Init(void)
{ 
    CLK_PCKENR1 |= 0xA0; 
    TIM2_CR1 =0;              //关闭TIM2
    TIM2_IER = 0; 
    TIM2_PSCR = 0;  
    TIM2_ARRH = 0x07;
    TIM2_ARRL = 0xCF;         //16M/(1999+1)=8KHz   //2MHz/(1999+1) = 1KHz 
    TIM2_CR1 |= 0x80;         //使能ARP,边沿对齐，向上计数
    TIM2_EGR |= 0x01;         //更新TIM1，使PSC有效
    TIM2_EGR |= 0x20;         //重新初始化TIM1 
    TIM2_CCR1H =0;           
    TIM2_CCR1L =0; 
    TIM2_CCMR1 =0x68;         //配置TIM2_CH1为PWM1模式输出 
    TIM2_CCER1 =0x01;         //Enable TIM2_CH1 channel 
    TIM2_CR1 |= 0x01;         //使能TIM2 
}
//==================================================================
//==================================================================
void PWM1_Init(void)
{
    CLK_PCKENR1 |= 0x80;
    TIM1_CR1 &= ~0x01;      //关闭TIM1 
    TIM1_PSCRH = 0;
    TIM1_PSCRL = 0;         //不分频2MHz 
    TIM1_ARRH = 0x07;
    TIM1_ARRL = 0xCF;      //2MHz/(1999+1) = 1KHz 
    TIM1_CR1 |= 0x80;      //使能ARP,边沿对齐，向上计数
    TIM1_EGR |= 0x01;      //更新TIM1，使PSC有效
    TIM1_EGR |= 0x20;      //重新初始化TIM1 
    
    TIM1_CCR1H = 0;
    TIM1_CCR1L = 0; 
    TIM1_CCMR1 = 0x68;       //配置TIM1_CH1为PWM1模式输出
    TIM1_CCER1 |= 0x01;      //Enable TIM1_CH1 channel
    
    TIM1_CCR2H = 0;
    TIM1_CCR2L = 0;
    TIM1_CCMR2 = 0x68;       //配置TIM1_CH2为PWM1模式输出
    TIM1_CCER1 |= 0x10;      //Enable TIM1_CH2 channel
    
    //TIM1_CCR3H = 0x07;
    //TIM1_CCR3L = 0xE6;       //占空比50% 
    //TIM1_CCMR3 = 0x68;       //配置TIM1_CH3为PWM1模式输出
    //TIM1_CCER2 |= 0x01;      //使能TIM1_CH3通道
    
    //TIM1_CCR4H = 0x03;
    //TIM1_CCR4L = 0xE6;       //占空比50%
    //TIM1_CCMR4 = 0x68;       //配置TIM1_CH4为PWM1模式输出 
    //TIM1_CCER2 |= 0x10;      //Enable TIM1_CH4 channel 
    
    TIM1_BKR |= 0x80;          //
    TIM1_CR1 |= 0x01;          //使能TIM1 
}
//==================================================================
//==================================================================
unsigned int ADC(int channel)
{
    unsigned int value =0;
    unsigned char tempH,tempL; 
    //CLK_PCKENR2 |=0x08; 
    ADC_CSR = channel;                // 清除EOC转换结束标志, select channel
    nop();  nop(); nop(); nop(); 
    ADC_CR1 |= 0x01;                  // 开始单次转换 
    nop(); nop(); nop(); nop(); nop();
    nop(); nop(); nop(); nop(); nop();
    ADC_CR1 |= 0x01;                  // 开始单次转换
    nop(); nop(); nop(); nop();
    nop(); nop(); nop(); nop();
    while(!(ADC_CSR & 0x80))nop();    // 等待单次转换完成 
    tempL = ADC_DRL;                  // 先读低8位
    tempH = ADC_DRH;                  // 再读高8位，设置数据左对齐
    value=(tempH<<8)|tempL;
    ADC_CSR &= ~(0x80);               // 清除EOC转换结束标志  
    ADC_CR1 &= ~(0x01); 
    return(value);  
}
//==================================================================
//==================================================================
unsigned int ADConverse(unsigned char channel)
{ 
    unsigned int value=0;
    unsigned int max_value =0;
    unsigned int min_value =0x7FFF;
    unsigned char i;  
    CLK_PCKENR2 |=0x08;  
    for(i =0;i <100;i++)
    {
      nop();nop();
    }
    for(i=0;i<10;i++)
    {
      Adc_value[i] = ADC(channel);  
      value += Adc_value[i];
      if(max_value <Adc_value[i])
      {
        max_value =Adc_value[i];
      }
       
      if(min_value >Adc_value[i])
      {
        min_value = Adc_value[i];
      }
    }
    value = value -max_value -min_value;
    value >>= 3; 
    CLK_PCKENR2 &=~(0x08); 
    return value; 
}
//================================================================================
//==================================================================
void SysInit(void)
{
    //-------system clock                                                       
    CLK_ICKR = 0X01;    //internal RC enable
    while(!(CLK_ICKR&0x02)); 
    CLK_SWCR = 0;       //禁止时钟切换及相关中断
    CLK_CKDIVR = 0X02;  //主频为Fmaster为Fhsi：16MHz, Fcpu为4MHz  0.25us
    CLK_PCKENR1 = 0;    //禁止Fmaster 与外设连接
    CLK_PCKENR2 = 0;
}
void ADC_Init(void)
{
    //------------------A/D conversion 
    CLK_PCKENR2 |= 0x08;           //使能Fmaster与外设ADC模块连接 
    ADC_CR1 = 0x00;               // ADC时钟=主时钟/2=8MHZ,单次转换模式Tad =1/8,未使能ADC
    ADC_CR2 = 0x08;               // A/D 结果数据右对齐
    ADC_CSR = 0x00;               // 清转换结束标志位EOC，
    ADC_CR3 = 0x00;
    ADC_TDRH = 0xFF;              //禁止施密特触发功能
    ADC_TDRL = 0x00; 
    nop(); nop(); nop();
    ADC_TDRL = 0x7C;              // AIN2、AIN3、AIN4、AIN5、AIN6 0b01111100 
    CLK_PCKENR2 &=~(0x08);
}
//==================================================================
//==================================================================
//---------------WatchDog  LSI 128KHz/2 = 64KHz
void ClrWdt(void)
{
    IWDG_KR = 0x55;   //解除保护
    IWDG_RLR = 0xFF;  //刷新内容
    IWDG_PR = 4;//3;      //127ms
    IWDG_KR = 0xAA;   //刷新及恢复保护
    IWDG_KR = 0xCC;   //独立看门狗启动
} 
//==================================================================
//==================================================================
void PortInit(void)
{ 
  GPIO_Init(GPIOA, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);//Vref  //GPIO_MODE_IN_PU_NO_IT);        //chger in 
  GPIO_Init(GPIOA, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);        //BIG_LOAD_DET
  GPIO_Init(GPIOA, GPIO_PIN_1, GPIO_MODE_IN_PU_NO_IT);        //充电过流
  
  GPIO_Init(GPIOB, GPIO_PIN_5, GPIO_MODE_IN_FL_NO_IT);//chger in //GPIO_MODE_OUT_PP_LOW_FAST);    //VREF
  GPIO_Init(GPIOB, GPIO_PIN_4, GPIO_MODE_OUT_OD_LOW_SLOW);//GPIO_MODE_OUT_PP_LOW_FAST);    //MCU_DO 
 
  GPIO_Init(GPIOC, GPIO_PIN_7, GPIO_MODE_OUT_OD_HIZ_FAST);    //LED2
  GPIO_Init(GPIOC, GPIO_PIN_6, GPIO_MODE_OUT_OD_HIZ_FAST);    //LED1
  PC_ODR_ODR5 = 1;
  GPIO_Init(GPIOC, GPIO_PIN_5, GPIO_MODE_OUT_OD_HIZ_FAST);    //LED3
  GPIO_Init(GPIOC, GPIO_PIN_4, GPIO_MODE_IN_FL_NO_IT);        //B3_AD
  GPIO_Init(GPIOC, GPIO_PIN_3, GPIO_MODE_OUT_PP_LOW_FAST);    //MCU_CO 
   
  GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);         //NTC_AD
  GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_IN_FL_NO_IT);         //Dis_Cur_AD
  GPIO_Init(GPIOD, GPIO_PIN_4, GPIO_MODE_OUT_PP_LOW_FAST);     //CON
  GPIO_Init(GPIOD, GPIO_PIN_3, GPIO_MODE_IN_FL_NO_IT);         //B1_AD
  GPIO_Init(GPIOD, GPIO_PIN_2, GPIO_MODE_IN_FL_NO_IT);         //B2_AD   
#if 1
  CPU_CFG_GCR |=0x01;       //SWIN模式被禁用，SWIM引脚可被用作普通I/O口
  GPIO_Init(GPIOD, GPIO_PIN_1, GPIO_MODE_OUT_PP_LOW_FAST);   //Pre_Dis
#else
  GPIO_Init(GPIOD, GPIO_PIN_1, GPIO_MODE_IN_PU_NO_IT);        //SWIN/MUC_DO3
#endif
  //EXTI_CR1 |=0x40;//上升沿触发//0x80;//下降沿触发
/*
	GPIO_MODE_IN_FL_NO_IT 无中断功能的浮动输入。  //ADC input
	
	GPIO_MODE_IN_PU_NO_IT 无中断功能的上拉输入。  //normal digit input
	
	GPIO_MODE_IN_FL_IT 带中断功能的浮动输入。  
	
	GPIO_MODE_IN_PU_IT 带中断功能的上拉输入。	
	
	GPIO_MODE_OUT_OD_LOW_FAST 高速开漏低电平输出，可工作到10MHz。  
	
	GPIO_MODE_OUT_PP_LOW_FAST 高速推挽低电平输出，可工作到10MHz。  
	
	GPIO_MODE_OUT_OD_LOW_SLOW 低速开漏低电平输出，可工作到2MHz。	//undefined port
	
	GPIO_MODE_OUT_PP_LOW_SLOW 低速推挽低电平输出，可工作到2MHz。	
	
	GPIO_MODE_OUT_OD_HIZ_FAST 高速开漏高阻态输出，可工作到10MHz。  
	
	GPIO_MODE_OUT_PP_HIGH_FAST 高速推挽高电平输出，可工作到10MHz。  
	
	GPIO_MODE_OUT_OD_HIZ_SLOW 低速开漏高阻态输出，可工作到2MHz。	
	
	GPIO_MODE_OUT_PP_HIGH_SLOW 低速推挽高电平输出，可工作到2MHz。
*/
}
//==================================================================
//==================================================================
void VarInit(void)
{ 
}
//==================================================================
//==================================================================
void Delay400us(void)
{
    unsigned char t=71;//75;//40;//38; 
    while(t--)
    {
        nop(); nop(); nop(); nop(); nop(); nop(); nop(); nop();
    }
} 
//=======================================================================
//======================================================================= 
//======================================================================= 
void Timer2Init(void)
{ 
    CLK_PCKENR1 |= 0x20;    // Fmaster 与外设TIM2连接
    TIM2_CR1 = 0x85;        // TIM2_ARR寄存器通过缓冲预装载，使能计数器
    TIM2_IER = 0x00;        // 禁止中断
    TIM2_EGR = 0x01;        // 允许产生更新事件 
    TIM2_PSCR = 0x07;       // 计数器时钟=主时钟/8=16MHZ/128    8us// 相当于计数器周期为 
    TIM2_CNTRH = 0;         //     
    TIM2_CNTRL = 0;         //     
    TIM2_ARRH = 0x04;       //     10ms
    TIM2_ARRL = 0xE2;       //   
    TIM2_IER = 0x01;        // 使能更新中断
    TIM2_IER |= 0x01;       // 使能更新中断，禁止触发中断  
} 
//=======================================================================
//==================================================================
//==================================================================
//--Fmaster = CK_PSC =16MHz, CK_PSC prescaler(128) to CK_CNT(8us)   TIM4_SR1.UIF更新中断标志符
void Timer4Init(void)
{
    CLK_PCKENR1 |= 0x10;    //Fmaster 与外设TIM4连接
    TIM4_IER = 0x00;        // 禁止中断
    TIM4_EGR = 0x01;        // 允许产生更新事件 
    TIM4_PSCR = 0x07;       // 计数器时钟=主时钟/128=16MHZ/128  // 相当于计数器周期为8uS
    TIM4_CR1 = 0x15;        // TIM4_ARR寄存器通过缓冲预装载，使能计数器
    TIM4_CNTR = 0;          //     
    TIM4_ARR = 6;//12;//25;           // 6*8us = 200us 
    //TIM4_ARR = 125;       // 125*8us = 1ms 
    TIM4_IER |= 0x01;       //更新中断使能，禁止触发中断
    //TIM4_OFF();
    TIM4_CR1 &= ~0x01;
} 
void Timer2Init_backup(void)
{
    CLK_PCKENR1 |= 0xA0; 
    TIM2_CR1 =0;              // 关闭TIM2
    TIM2_IER = 0; 
    TIM2_PSCR = 0x07;         // 16MHz/128 = 8us
    TIM2_ARRH = 0x04;
    TIM2_ARRL = 0xE2;         // 10ms/8us = 1250
    TIM2_CR1 |= 0x80;         // 自动重装载预装载允许位, 允许更新中断
    TIM2_EGR |= 0x01;         // 更新TIM2，使PSC有效
     /* 
    TIM2_EGR |= 0x20;         // 重新初始化TIM1  
    TIM2_CCR1H =0;           
    TIM2_CCR1L =0; 
    TIM2_CCMR1 =0X68;         //配置TIM2_CH1为PWM1模式输出 
    TIM2_CCER1 =0x01;         //Enable TIM2_CH1 channel 
    */
    TIM2_IER = 0x01;          // 使能更新中断
    TIM2_CR1 |= 0x01;         // 使能TIM2 
} 
//====================================================================
//====================================================================
void TempCheck(void)
{ 
    static uint8_t ChgTemp_cnt =0;
    static uint8_t DisTemp_cnt =0; 
    Temp_Val = ADConverse(6); 
    if(WorkMode == CHARGE_MODE)  
    {
       DisTemp_cnt =0;
       if((Temp_Val > ChgTempL_ON_SET) || (Temp_Val < ChgTempH_ON_SET) || (ChgTemp_cnt >= 10))
       {
         if((ChgTemp_cnt ++) >= 10)
         {
           ChgTemp_cnt = 10;
           Bits_flag.ChgTemp = 1;
         }
       }
       else
       {
         ChgTemp_cnt = 0;
       }
      //==========================充电温度保护恢复
      if(Bits_flag.ChgTemp && (Temp_Val < ChgTempL_OFF_SET) && (Temp_Val > ChgTempH_OFF_SET))
      {
        ChgTemp_cnt = 0;
        Bits_flag.ChgTemp = 0;
      } 
    }
    else //if(WorkMode == DISCHARGE_MODE)
    {
       ChgTemp_cnt = 0;
       if((Temp_Val > DisTempL_ON_SET) || (Temp_Val < DisTempH_ON_SET) || (DisTemp_cnt >= 10))
       {
         if((DisTemp_cnt ++) >= 10)
         {
           DisTemp_cnt = 10;
           Bits_flag.DisTemp = 1;
         }
       }
       else
       {
         DisTemp_cnt =0;
       }
      //==========================充电温度保护恢复 
      //if(Bits_flag.DisTemp && (Temp_Val < DisTempL_OFF_SET) && (Temp_Val > DisTempH_OFF_SET))
      if(Bits_flag.DisTemp && (Temp_Val < DisTempL_OFF_SET) && (Temp_Val > DisTempH_OFF_SET))
      {
          DisTemp_cnt = 0;
          Bits_flag.DisTemp = 0;
      }  
    }   
}
//================================================================== 
//==================================================================
void CurrentCheck(void)
{ 
  static uint16_t IS_CHG_OC_cnt =0;
  static uint16_t DisCur_Val_Cur = 0;
  uint16_t DisCur_Val_Tmp = 0;
  uint16_t DisCur_Val_Sum =0;
  uint8_t i =0;
  if(WorkMode == CHARGE_MODE)
  {
    if(IS_CHG_OC_ON())
    {
      if((IS_CHG_OC_cnt ++) >= 5)
      {
        IS_CHG_OC_cnt = 5;
        Bits_flag.ChgCurOv = 1;
      }
    }
    else
    {
      IS_CHG_OC_cnt = 0;
    }
      
    //if(Bits_flag.ChgCurOv && ChgCurOv_Re_t >= ChgCurOv_Re_t_SET && IS_CHG_OC_OFF())
    if(Bits_flag.ChgCurOv && ChgCurOv_Re_t >= 500 && IS_CHG_OC_OFF())
    {
      IS_CHG_OC_cnt = 0;
      ChgCurOv_Re_t = 0;
      Bits_flag.ChgCurOv = 0;
    }
  }
  else if(WorkMode == DISCHARGE_MODE)//0.5A ==0.227V 
  {
    DisCur_Val_Cur = ADConverse(5);     //电池包电压检测  if(Current_float_Tmp_Last_Len <32)
    //if(DisCur_Val_Cur > DisCur_Bias_Val)
    if(DisCur_Val_Cur > 35)
    {
      //DisCur_Val_Cur -= DisCur_Bias_Val;
      DisCur_Val_Cur -= 35;
    }
    else
    {
      DisCur_Val_Cur = 0;
    }
    DisCur_Val_Tmp = DisCur_Val_Cur;
    DisCur_Val = (uint16_t)((uint32_t)465 * DisCur_Val_Tmp/10 );
    //DisCur_Val_Tmp = (DisCur_Val *21 *5 *1024 )/5
     /* 
    //==============================对放电电流进行平均滑动滤波 begin
    if(DisCur_Val_Tmp_Last_Len < 8)
    {
      DisCur_Val_Tmp_Last[DisCur_Val_Tmp_Last_Len] = DisCur_Val_Cur;//DisCur_Val;
      for(i =0;i < DisCur_Val_Tmp_Last_Len; i++)
      {
        DisCur_Val_Sum += DisCur_Val_Tmp_Last[i];
      }
      DisCur_Val =  DisCur_Val_Sum / DisCur_Val_Tmp_Last_Len ;
      DisCur_Val_Tmp_Last_Len += 1;
    }
    else
    {
      for(i =0;i < DisCur_Val_Tmp_Last_Len-1;i++)
      {
        DisCur_Val_Tmp_Last[i] =DisCur_Val_Tmp_Last[i+1];
      }
      DisCur_Val_Tmp_Last[i] = DisCur_Val_Cur;
      for(i =0;i < DisCur_Val_Tmp_Last_Len;i++)
      {
        DisCur_Val_Sum += DisCur_Val_Tmp_Last[i];
      }
      DisCur_Val = DisCur_Val_Sum >> 3;
    }
    //==============================对放电电流进行平均滑动滤波 end
    */
    //if(DisCur_Val_Tmp > 1500 || DisCurOv_II_t >= DisCurOv_II_t_SET || DisCurOv_I_t >= DisCurOv_I_t_SET)
    if(DisCur_Val > DisCur_Val_II_SET || DisCurOv_II_t >= DisCurOv_II_t_SET || DisCurOv_I_t >= DisCurOv_I_t_SET)
    { 
      if(DisCurOv_II_t >= DisCurOv_II_t_SET)
      {
        DisCurOv_II_t = DisCurOv_II_t_SET;
        Bits_flag.DisCurOv = 1; 
      }
    }
    else if(DisCur_Val > DisCur_Val_I_SET)//
    { 
      DisCurOv_II_t = 0;
      if(DisCurOv_I_t >= DisCurOv_I_t_SET)
      {
        DisCurOv_I_t = DisCurOv_I_t_SET;
        Bits_flag.DisCurOv = 1; 
      }
    }
    else
    {
      DisCurOv_I_t  = 0;
      DisCurOv_II_t = 0; 
    }
    if(Bits_flag.DisCurOv && DisCurOv_Re_t >= 500)
    {
      DisCurOv_I_t  = 0;
      DisCurOv_II_t = 0;
      Bits_flag.DisCurOv = 0;
    }
  }
}
//================================================================== 
void CellVolCalibrate(void)
{
  uint8_t i = 0;
  Cell_Ratio[0] = 6;
  Cell_Ratio[1] = 2;
  Cell_Ratio[2] = 1;
  Vcc_Volt = 5000; 
  for(i = 0;i<3;i++)
  {
    Cell_Volt[i] = (uint16_t)((uint32_t)Vcc_Volt *Cell_Ratio[i] * Cell_Volt_Ad[i]/1024);
  }
  //Cell_Volt[2] = Cell_Volt[2] - Cell_Volt[1];
  //Cell_Volt[1] = Cell_Volt[1] - Cell_Volt[0]; 
  Cell_Volt[0] = Cell_Volt[0] - Cell_Volt[1];
  Cell_Volt[1] = Cell_Volt[1] - Cell_Volt[2]; 
}
//==================================================================
void BatVolCheck(void)
{
  uint8_t i = 0;  
  uint16_t tmpval_tol = 0; 
  uint16_t tmpval_max = 0; 
  uint16_t tmpval_min = 0x7FFF;  
  for(i =0;i<3;i++)
  {
    Cell_Volt_Ad[i] = ADConverse(i+2);     //电池包电压检测 
  }
  CellVolCalibrate();
  
  for(i =0;i<3;i++)
  {
    tmpval_tol += Cell_Volt[i];
    if(Cell_Volt_Ad[i] < tmpval_min)
    {
      tmpval_min = Cell_Volt[i];
    }
    if(Cell_Volt_Ad[i] > tmpval_max)
    {
      tmpval_max = Cell_Volt[i];
    }
  }
  Cell_Volt_Tol = tmpval_tol;
  Cell_Volt_Max = tmpval_max;
  Cell_Volt_Min = tmpval_min;
  Cell_Volt_Avg = Cell_Volt_Tol/3; 
   
  //=====================================
  if(WorkMode == CHARGE_MODE)//(Bits_flag.Chg)
  {
    //if(Cell_Volt_Max > ChgOv_SET || ChgOv_t >= ChgOv_t_SET) 
    if(Cell_Volt_Max > 4230 || ChgOv_t >= ChgOv_t_SET) 
    {
      if(ChgOv_t >= ChgOv_t_SET)
      {
        ChgOv_t = ChgOv_t_SET;
        Bits_flag.ChgOv = 1;
      }
    }
    else
    { 
      ChgOv_t = 0;
    }
    
    //if(Bits_flag.ChgOv && (Cell_Volt_Avg < ChgOv_Re_SET) && (Cell_Volt_Max < ChgOv_Re_SET)) 
    if(Bits_flag.ChgOv && (Cell_Volt_Avg < 4100) && (Cell_Volt_Max < 4100)) 
    { 
      ChgOv_t = 0;
      Bits_flag.ChgOv = 0;
    }
  }
  else if(WorkMode == DISCHARGE_MODE)
  { 
    //if(Cell_Volt_Min < DisOv_Min_SET || Cell_Volt_Avg < DisOv_Avg_SET || DisOv_t >= DisOv_t_SET)//866
    if(Cell_Volt_Min < 2700 || Cell_Volt_Avg < 3000 || DisOv_t >= DisOv_t_SET)//866
    {
      if(DisOv_t >= DisOv_t_SET)
      {
        DisOv_t = DisOv_t_SET;
        Bits_flag.DisOv = 1;
      }
    }
    else
    { 
      DisOv_t = 0;
    }
    
    //if(Bits_flag.DisOv && (Cell_Volt_Avg >= DisOv_Re_SET)) 
    if(Bits_flag.DisOv && (Cell_Volt_Avg >= 3200))
    { 
      DisOv_t = 0;
      Bits_flag.DisOv = 0;
    }
  }
   
}
//================================================================== 
void ModeCheck(void)
{    
  //static uint8_t workmode_exchange_cnt = 0;
  
   if(IS_CHGER_IN())
   {
     if(ChargeMode_Exchange_Delay >= 50)
     {
        ChargeMode_Exchange_Delay = 100;
        DischargeMode_Exchange_Delay = 0;
        IdleMode_Exchange_Delay = 0;
        WorkMode = CHARGE_MODE;    
     }
   }
   /*-------------------------------------------------------------------------------------------------------
      无异常且检测到大电流放电时，为放电状态
      无保护且小电流时，Pre_Dsg为on，可关闭MCU_DO，来检测BIG_LOAD_DET是否为低电平
      有异常保护时，Pre_Dsg和MCU_DO均关闭，无法检测负载状态，可静等异常释放时，先打开Pre_Dsg，再检测负载状态
   ---------------------------------------------------------------------------------------------------------*/
   else if(DisCur_Val > 10 || (IS_MCU_DO_OFF() && IS_BIG_LOAD_DET() && IS_Pre_Dis_ON()) || (IS_MCU_DO_OFF() && IS_Pre_Dis_OFF()) )// ////  &&
   //else if((IS_BIG_LOAD_DET()))// || DisCur_Val > 5) 
   {
     if(DischargeMode_Exchange_Delay >= 50)
     {
        ChargeMode_Exchange_Delay = 0;
        DischargeMode_Exchange_Delay = 100;
        IdleMode_Exchange_Delay = 0;  
        WorkMode = DISCHARGE_MODE;    
     }
   }/**/
   else
   {
     if(IdleMode_Exchange_Delay >= 50)
     {
        ChargeMode_Exchange_Delay = 0;
        DischargeMode_Exchange_Delay = 0;
        IdleMode_Exchange_Delay = 100; 
        WorkMode = IDLE_MODE;  
     }
   } 
}  
void StatusClear(void)
{ 
  if(WorkMode == CHARGE_MODE)
  {
    //===========================
    DisOv_t = 0; 
    DisCurOv_II_t = 0; 
    DisCurOv_I_t = 0; 
    DisCurOv_Re_t = 0; 
  }   
  else if(WorkMode == DISCHARGE_MODE)
  {
    //===========================
    ChgOv_t = 0; 
    ChgCurOv_Re_t = 0; 
  }  
}

void VarClear(void)
{
  uint8_t i = 0;
  ChargeMode_Exchange_Delay = 0;
  DischargeMode_Exchange_Delay = 0;
  IdleMode_Exchange_Delay = 0;
  for(i =0; i <18;i++)
  {
    Adc_value[i] = 0;
    if(i <3)
    {
      Cell_Volt[i]    = 0; 
      Cell_Ratio[i]   = 0; 
      Cell_Volt_Ad[i] = 0; 
    }
    if(i <16 )
    {
      DisCur_Val_Tmp_Last[i] = 0;
    }
  }
  Vcc_Volt = 0;
  Cell_Volt_Tol = 0;
  Cell_Volt_Max = 0;
  Cell_Volt_Min = 0;
  Cell_Volt_Avg = 0;
  DisCur_Val = 0;
  DisCur_Bias_Val = 0;
  Temp_Val = 0;
  DisOv_t = 0;
  ChgOv_t = 0;
  DisCurOv_I_t = 0;
  DisCurOv_II_t = 0;
  DisCurOv_Re_t = 0;
  ChgCurOv_Re_t = 0;
  DisCur_Val_Tmp_Last_Len = 0;
  LedShow_Flash_t =0;
  
  Bits_flag.Chg = 0;
  Bits_flag.ChgOv = 0;
  Bits_flag.ChgTemp = 0;
  Bits_flag.Dis = 0;
  Bits_flag.DisOv = 0;
  Bits_flag.DisTemp = 0;
  Bits_flag.DisCurOv = 0;  
  
  WorkMode = IDLE_MODE;
  
  ChargeMode_Exchange_Delay = 100;
  DischargeMode_Exchange_Delay = 100;
  IdleMode_Exchange_Delay = 100; 
}

//================================================================== 
void FET_ChgDis_Cntrl(void)
{   
   if(WorkMode == CHARGE_MODE)
   {
     MCU_DO_OFF();
     Pre_Dis_OFF();
     if(Bits_flag.ChgCurOv || Bits_flag.ChgTemp || Bits_flag.ChgOv)
     {
       MCU_CO_OFF();
     }
     else
     {
       MCU_CO_ON();
     }
   }
   else //if(WorkMode == DISCHARGE_MODE)
   {
     MCU_CO_OFF();
     //==有异常保护时，Pre_Dsg和MCU_DO均关闭，无法检测负载状态， 异常释放时，先打开Pre_Dsg，再检测负载状态
     if(0)//(Bits_flag.DisCurOv || Bits_flag.DisTemp || Bits_flag.DisOv)
     {
       MCU_DO_OFF();
       Pre_Dis_OFF();
     }
     else
     {
       Pre_Dis_ON();
       if(0)//(WorkMode == DISCHARGE_MODE)
       {
          MCU_DO_ON();
       }
       else
       {
          MCU_DO_OFF();
       }
     }
   } 
}  

//================================================================== 
void FET_ChgDis_Cntrl_Backup(void)
{   
   if(WorkMode == CHARGE_MODE)
   {
     MCU_DO_OFF();
     Pre_Dis_OFF();
     if(Bits_flag.ChgCurOv || Bits_flag.ChgTemp || Bits_flag.ChgOv)
     {
       MCU_CO_OFF();
     }
     else
     {
       MCU_CO_ON();
     }
   }
   else if(WorkMode == DISCHARGE_MODE)
   {
     MCU_CO_OFF();
     if(Bits_flag.DisCurOv || Bits_flag.DisTemp || Bits_flag.DisOv)
     {
       MCU_DO_OFF();
       Pre_Dis_OFF();
     }
     else
     {
       Pre_Dis_ON();
       if(1) 
       {
          MCU_DO_ON();
       }
       else
       {
          MCU_DO_OFF();
       }
     }
   }
   else
   {
     MCU_CO_OFF();
     MCU_DO_OFF();
     Pre_Dis_OFF();
   }
     
}  


//======================
void WorkLedShow(void)
{ 
  if(WorkMode == CHARGE_MODE)//LED2 flashing
  {
    if(LedShow_Flash_t >= 10)
    {
      LedShow_Flash_t = 0;
      LED1_XOR();
      LED2_OFF();
      LED_3OFF_4OFF();
    }
  }
  else if(WorkMode == DISCHARGE_MODE)//LED1,LED2 flashing
  { 
    if(LedShow_Flash_t >= 10)
    {
      LedShow_Flash_t = 0;
      LED1_OFF();
      LED2_XOR();
      LED_3OFF_4OFF();
    } 
  }
  else if(WorkMode == IDLE_MODE)//LED1 flashing
  {
    LED1_OFF();
    LED2_OFF(); 
    if(LedShow_Flash_t < 10)
    { 
      LED_3ON_4OFF();
    }
    else if(LedShow_Flash_t < 20)
    {
      LED_3OFF_4ON();
    }
    else
    {
      LedShow_Flash_t = 0;
    }
    
  } 
}
//=============================================
/*----------------------------------------------------------------
  充电电量显示				
  亮四个灯LED1,LED2,LED3，LED4(绿灯)	11.4±0.3														
  亮三个灯LED1,LED2,LED3(绿灯)		10.1±0.3	11.4±0.3					
  亮两个灯LED1,LED2(绿灯)		8.8±0.3	10.1±0.3					
  亮一个灯LED1(绿灯)			8.1±0.3	8.8±0.3					
  LED1闪烁				8.1±0.3	 
------------------------------------------------------------------*/
void LedShow(void)
{
  static uint8_t ledshow_cnt = 0;
  if(Cell_Volt_Tol > 11400)//11.4
  {
    LED1_ON();
    LED2_ON(); 
    ledshow_cnt += 1;
    if(ledshow_cnt < 3)
    { 
      LED_3ON_4OFF();
    }
    else if(ledshow_cnt < 5)
    { 
      LED_3OFF_4ON();
    }
    else
    {
      ledshow_cnt = 0;
    }
  }
  else if(Cell_Volt_Tol > 10100)//10.1
  { 
    LED1_ON();
    LED2_ON(); 
    LED_3ON_4OFF(); 
  }
  else if(Cell_Volt_Tol > 8800)//8.8
  {
    LED1_ON();
    LED2_ON(); 
    LED_3OFF_4OFF(); 
  }
  else if(Cell_Volt_Tol > 8100)//8.1
  {
    LED1_ON();
    LED2_OFF(); 
    LED_3OFF_4OFF(); 
  }
  else
  {
    LED2_OFF(); 
    LED_3OFF_4OFF(); 
    if(LedShow_Flash_t >= 50)
    {
      LedShow_Flash_t = 0; 
      LED1_XOR();  
    }
  }
  
}