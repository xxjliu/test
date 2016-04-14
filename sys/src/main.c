
#include "stm8s.h"  
#include "user.h"  
int main( void )
{ 
  uint16_t j,i = 0;
  disableInterrupts();
  SysInit();
  PortInit();
  ADC_Init();
  Timer2Init();       // 10ms
  PWM2_Init();
  enableInterrupts();
  VREF_ON();  
  VarClear();
  Pre_Dis_ON();
  MCU_CO_OFF();
  MCU_DO_OFF();
  WorkMode = IDLE_MODE; 
  LED1_OFF();
  LED2_OFF(); 
  LED_3ON_4OFF() ; 
  //DisCur_Bias_Val = ADConverse(5);     //µç³Ø°üµçÑ¹¼ì²â  if(Current_float_Tmp_Last_Len <32) 
//=== ADDR_BASE ==0x004000    ADDR_END ==0x00407F
//void FLASH_ProgramByte(uint32_t Address, uint8_t Data);
//uint8_t FLASH_ReadByte(uint32_t Address);
//void FLASH_ProgramWord(uint32_t Address, uint32_t Data); 
  
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  FLASH_ProgramByte(ADDR_BASE, 0xAA);
  FLASH_Lock(FLASH_MEMTYPE_DATA);
  while(1)
  {
    ClrWdt(); 
    for(i =0;i<3000;i++)
    {
      ClrWdt();
      nop();nop();nop();nop();
    } 
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
    FLASH_ProgramByte(ADDR_BASE, 0xAA);
  FLASH_Lock(FLASH_MEMTYPE_DATA);
    if(0xAA == FLASH_ReadByte(ADDR_BASE))
    {
      LED1_ON();
    }
    else
    {
      LED1_ON();
    }
    for(i =0;i<3000;i++)
    {
      ClrWdt();
      nop();nop();nop();nop();
    } 
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
    FLASH_ProgramByte(ADDR_BASE, 0x00);
  FLASH_Lock(FLASH_MEMTYPE_DATA);
    if(0xAA == FLASH_ReadByte(ADDR_BASE))
    {
      LED1_ON();
    }
    else
    {
      LED1_ON();
    }
    
  }
  while(1)
  {
    ClrWdt();
    //LED1_XOR();  
    ModeCheck();   
        //WorkMode = DISCHARGE_MODE;    
    StatusClear();
    CurrentCheck();    
    TempCheck(); 
    BatVolCheck();
    FET_ChgDis_Cntrl(); 
    WorkLedShow();//LedShow();  //
  }
  while(1)
  {
    nop();
    nop();
    nop();
    if(LedShow_Flash_t < 100)
    {
      LED1_ON(); 
      LED2_OFF(); 
      LED_3OFF_4OFF(); 
    }
    else if(LedShow_Flash_t < 200)
    {
      LED1_OFF(); 
      LED2_ON(); 
      LED_3OFF_4OFF(); 
    }
    else if(LedShow_Flash_t < 300)
    {
      LED1_OFF(); 
      LED2_OFF(); 
      LED_3ON_4OFF(); 
    }
    else if(LedShow_Flash_t < 400)
    {
      LED1_OFF(); 
      LED2_OFF(); 
      LED_3OFF_4ON(); 
    }
    else if(LedShow_Flash_t < 500)
    { 
      LED2_OFF(); 
      LED1_OFF(); 
      LED_3OFF_4OFF();
    } 
    else
    {
      LedShow_Flash_t = 0; 
    } 
      
    
  } 
  return 0;
}
