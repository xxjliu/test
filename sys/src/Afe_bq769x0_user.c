//
#include "stm8s.h" 
#include "user.h"
#include "macro_def.h"
#include "iostm8s003f3.h"
#include "bq769x0.h"

#if 1
void Afe_CellBalance_Disable(void)
{ 
  //==============cell balance off
  CELLBAL1_Last = 0x00;
  I2C_Write(CELLBAL1_ADDR,CELLBAL1_Last);
  CELLBAL2_Last = 0x00;
  I2C_Write(CELLBAL2_ADDR,CELLBAL2_Last);
}
//====================================================================
uint16_t Afe_Get_Adc(uint8_t addr)
{ 
  uint8_t adcval,tmpval = 0; 
  uint16_t adv_result ;    
  I2C_Read(addr,&tmpval);
  adcval = tmpval;
  I2C_Read(addr + 1,&tmpval);
  adv_result = ((uint16_t)adcval << 8 ) + (uint16_t)tmpval;
  return adv_result; 
}
//==========================================================================
/*      SYS_STAT (0x00)/RESET:0x00
        BIT        7      6           5             4       3   2    1     0
        NAME   CC_READY  RSVD   DEVICE_XREADY   OVRD_ALERT  UV  OV  SCD   OCD
*/
void Afe_AbnormalCheck(void)
{
  if(SYS_STAT.Bit.DEVICE_XREADY)// || SYS_STAT.Bit.OVRD_ALERT)
  {
    Bits_flag.Bit.AfeErr = 1;
  }
  else
  {
    Bits_flag.Bit.AfeErr = 0;
  }
  /*  
  if(SYS_STAT.Bit.OVRD_ALERT)
  {
    SYS_STAT_Last |= 0x10;
    I2C_Write(SYS_STAT_ADDR,SYS_STAT_Last);
    SYS_STAT_Last &= ~0x10;
    SYS_STAT.Bit.OVRD_ALERT = 0;
  }*/
  if(Bits_flag.Bit.AfeErr && DEVICE_XREADY_Re_t >= DEVICE_XREADY_Re_SET)//SYS_STAT.Bit.DEVICE_XREADY
  {
    SYS_STAT_Last |= 0x20;
    I2C_Write(SYS_STAT_ADDR,SYS_STAT_Last);
    SYS_STAT_Last &= ~0x20;
    SYS_STAT.Bit.DEVICE_XREADY = 0;
  }
}

//==========================================================================
/*
  Waking from SHIP mode to NORMAL mode requires pulling the TS1 pin greater than VBOOT, which triggers the device boot-up sequence.
  In order to exit SHIP mode into NORMAL mode, the device must follow the standard boot sequence by applying a voltage greater than the VBOOT threshold on the TS1 pin.
  
To enter SHIP mode from NORMAL mode, the [SHUT_A] and [SHUT_B] bits in the SYS_CTRL1 register must be written with specific patterns across two consecutive writes:
   Write #1: [SHUT_A] = 0, [SHUT_B] = 1
   Write #2: [SHUT_A] = 1, [SHUT_B] = 0
  Table 7-7. SYS_CTRL1 (0x04)/RESET:0x00
  BIT       7         6   5     4         3       2     1       0
  NAME  LOAD_PRESENT  ！  ！  ADC_EN  TEMP_SEL  RSVD  SHUT_A  SHUT_B
*/

void Afe_EnterShipMode(void)
{
  SYS_CTRL1_Last &= ~0x02;
  SYS_CTRL1_Last |= 0x01;
  I2C_Write(SYS_CTRL1_ADDR,SYS_CTRL1_Last);
  
  SYS_CTRL1_Last |= 0x02;
  SYS_CTRL1_Last &= ~0x01;
  I2C_Write(SYS_CTRL1_ADDR,SYS_CTRL1_Last);
}
 
//==========================================================================
/* This bit is set automatically whenever the device enters NORMAL mode. 
   When enabled, the ADC ensures that the integrated OV and UV protections are functional.*/
void Afe_ADC_Enable(void)
{
  //==ADC enable command
  SYS_CTRL1_Last |= 0x10;
  I2C_Write(SYS_CTRL1_ADDR,SYS_CTRL1_Last);
}
 
//========================================================================== 
void Afe_Temp_Enable(void)
{
  //==TEMP_SEL (Bit 3): TSx_HI and TSx_LO temperature source
  SYS_CTRL1_Last |= 0x08;
  I2C_Write(SYS_CTRL1_ADDR,SYS_CTRL1_Last);
}

//===============================================
//ADCGAIN is stored in units of μV/LSB, while ADCOFFSET is stored in mV.
void Afe_Get_GainOffset(void)
{
  uint8_t tmpval = 0;
  uint8_t gain_val_tmp = 0;
  
  //==get adc gian value 
  I2C_Read(ADCGAIN1_ADDR,&tmpval);
  gain_val_tmp = (tmpval & 0x0C)>>2; 
  I2C_Read(ADCGAIN2_ADDR,&tmpval); 
  gain_val_tmp = (gain_val_tmp << 3) + ((tmpval & 0xE0) >> 5); 
  ADCGain_Val = 365 + gain_val_tmp;
  
  //===get adc offset value
  I2C_Read(ADCOFFSET_ADDR,&tmpval );
  ADCOffset_Val = tmpval ;
  
  
   // Uart_SendByte(gain_val_tmp); //0x0C =12
   // Uart_SendByte((uint8_t)(ADCGain_Val>>8)); //12
   // Uart_SendByte((uint8_t)ADCGain_Val); //12
   // Uart_SendByte(ADCOffset_Val); //0x31=49
}
//==========================================================================
void Afe_Device_Init(void)
{ 
  uint8_t tmp = 0;
  //==For optimal performance, these bits should be programmed to 0x19 upon device startup
  I2C_Write(CC_CFG_ADDR,CC_CFG_INIT_VAL); 
   
  //Delay_ms(100);
  I2C_Write(SYS_STAT_ADDR,0xFF); 
  SYS_STAT_Last = 0x00;
  //Delay_us(50); 
   /**/
  Afe_CellBalance_Disable();
  Afe_Get_SysStatus(); 
  Afe_ADC_Enable();
  Afe_CC_1Shot_Set();
  Afe_Temp_Enable();
  Afe_Get_GainOffset();  
  Afe_SCD_Set(SCD_THREHOLD_VAL_SET, SCD_DELAY_SET);
  Afe_OCD_Set(OCD_THREHOLD_VAL_SET, OCD_DELAY_SET);
  Afe_OV_UV_Delay_Set(OV_DELAY_SET,UV_DELAY_SET); 
  Afe_OV_UV_Threshold_Set(OV_THREHOLD_VAL_SET, UV_THREHOLD_VAL_SET);
}

//==========================================================================
void Afe_SCD_Set(uint16_t SCD_val, uint16_t SCD_delay)
{
  uint8_t RSNS_mark = 0;
  uint8_t SCD_val_tmp = 0;
  uint8_t SCD_delay_tmp = 0;
  SCD_val = SCD_val/100;
  if(SCD_val >= 100)
  {
    RSNS_mark = 0x01; 
    if(SCD_val > 178)
    {
      SCD_val_tmp = 0x07;  // 200mV
    }
    else if(SCD_val > 155)
    {
      SCD_val_tmp = 0x06;  // 178mV
    }
    else if(SCD_val > 133)
    {
      SCD_val_tmp = 0x05;  // 155mV
    } 
    else if(SCD_val > 111)
    {
      SCD_val_tmp = 0x04;  // 133mV
    }
    else if(SCD_val > 89)
    {
      SCD_val_tmp = 0x03;  // 111mV
    } 
    else if(SCD_val > 67)
    {
      SCD_val_tmp = 0x02;  // 89mV
    }
    else if(SCD_val > 44)
    {
      SCD_val_tmp = 0x01;  // 67mV
    } 
    else
    {
      SCD_val_tmp = 0x00;  // 44mV
    } 
  }
  else
  {
    if(SCD_val > 89)
    {
      SCD_val_tmp = 0x07;  // 100mV
    }
    else if(SCD_val > 78)
    {
      SCD_val_tmp = 0x06;  // 89mV
    }
    else if(SCD_val > 67)
    {
      SCD_val_tmp = 0x05;  // 78mV
    } 
    else if(SCD_val > 56)
    {
      SCD_val_tmp = 0x04;  // 67mV
    }
    else if(SCD_val > 44)
    {
      SCD_val_tmp = 0x03;  // 56mV
    } 
    else if(SCD_val > 33)
    {
      SCD_val_tmp = 0x02;  // 44mV
    }
    else if(SCD_val > 22)
    {
      SCD_val_tmp = 0x01;  // 33mV
    } 
    else
    {
      SCD_val_tmp = 0x00;  // 22mV
    } 
  } 
  //========================
  if(SCD_delay > 200)
  {
    SCD_delay_tmp = 0x03;  // 400uS
  }
  else if(SCD_delay > 100)
  {
    SCD_delay_tmp = 0x02;  // 200uS
  }
  else if(SCD_delay > 70)
  {
    SCD_delay_tmp = 0x01;  // 100uS
  } 
  else
  {
    SCD_delay_tmp = 0x00;  // 70uS
  }
  //== Protect page36
  //PROTECT1_Last =  (RSNS_mark <<8) + (SCD_delay_tmp << 3) + SCD_val_tmp; //SCD
  //I2C_Write(PROTECT1_ADDR,PROTECT1_Last);
  PROTECT1_Last = 0x98 + 0x07; //SCD
  I2C_Write(PROTECT1_ADDR,PROTECT1_Last);
  //Uart_SendByte(0x02);Uart_SendByte(PROTECT1_Last);
}

//==========================================================================
void Afe_OCD_Set(uint16_t OCD_val, uint16_t OCD_delay)
{ 
  uint8_t OCD_val_tmp,OCD_delay_tmp;
  OCD_val = OCD_val/100;
  if(SCD_THREHOLD_VAL_SET >= 1000)
  { 
    if(OCD_val > 94)
    {
      OCD_val_tmp = 0x0F;  //  
    }
    else if(OCD_val > 89)
    {
      OCD_val_tmp = 0x0E;  //  
    }
    else if(OCD_val > 83)
    {
      OCD_val_tmp = 0x0D;  //  
    } 
    else if(OCD_val > 78)
    {
      OCD_val_tmp = 0x0C;  //  
    }
    else if(OCD_val > 72)
    {
      OCD_val_tmp = 0x0B;  //  
    } 
    else if(OCD_val > 67)
    {
      OCD_val_tmp = 0x0A;  //  
    }
    else if(OCD_val > 61)
    {
      OCD_val_tmp = 0x09;  //  
    } 
    else if(OCD_val > 56)
    {
      OCD_val_tmp = 0x08;  //  
    } 
    else if(OCD_val > 50)
    {
      OCD_val_tmp = 0x07;  //  
    }
    else if(OCD_val > 44)
    {
      OCD_val_tmp = 0x06;  //  
    }
    else if(OCD_val > 39)
    {
      OCD_val_tmp = 0x05;  //  
    } 
    else if(OCD_val > 33)
    {
      OCD_val_tmp = 0x04;  //  
    }
    else if(OCD_val > 28)
    {
      OCD_val_tmp = 0x03;  //  
    } 
    else if(OCD_val > 22)
    {
      OCD_val_tmp = 0x02;  //  
    }
    else if(OCD_val > 17)
    {
      OCD_val_tmp = 0x01;  // 22mV
    } 
    else
    {
      OCD_val_tmp = 0x00;  // 17mV
    } 
  }
  else
  { 
    if(OCD_val > 47)
    {
      OCD_val_tmp = 0x0F;  //  
    }
    else if(OCD_val > 44)
    {
      OCD_val_tmp = 0x0E;  //  
    }
    else if(OCD_val > 42)
    {
      OCD_val_tmp = 0x0D;  //  
    } 
    else if(OCD_val > 39)
    {
      OCD_val_tmp = 0x0C;  //  
    }
    else if(OCD_val > 36)
    {
      OCD_val_tmp = 0x0B;  //  
    } 
    else if(OCD_val > 33)
    {
      OCD_val_tmp = 0x0A;  //  
    }
    else if(OCD_val > 31)
    {
      OCD_val_tmp = 0x09;  //  
    } 
    else if(OCD_val > 28)
    {
      OCD_val_tmp = 0x08;  //  
    } 
    else if(OCD_val > 25)
    {
      OCD_val_tmp = 0x07;  //  
    }
    else if(OCD_val > 22)
    {
      OCD_val_tmp = 0x06;  //  
    }
    else if(OCD_val > 19)
    {
      OCD_val_tmp = 0x05;  //  
    } 
    else if(OCD_val > 17)
    {
      OCD_val_tmp = 0x04;  //  
    }
    else if(OCD_val > 14)
    {
      OCD_val_tmp = 0x03;  //  
    } 
    else if(OCD_val > 11)
    {
      OCD_val_tmp = 0x02;  //  
    }
    else if(OCD_val > 8)
    {
      OCD_val_tmp = 0x01;  //  
    } 
    else
    {
      OCD_val_tmp = 0x00;  //  
    } 
  } 
  //========================
  if(OCD_delay > 640)
  {
    OCD_delay_tmp = 0x07;  // 1280mS
  }
  else if(OCD_delay > 320)
  {
    OCD_delay_tmp = 0x06;  // 640mS
  }
  else if(OCD_delay > 160)
  {
    OCD_delay_tmp = 0x05;  // 320mS
  }
  else if(OCD_delay > 80)
  {
    OCD_delay_tmp = 0x04;  // 160mS
  }
  else if(OCD_delay > 40)
  {
    OCD_delay_tmp = 0x03;  // 80mS
  }
  else if(OCD_delay > 20)
  {
    OCD_delay_tmp = 0x02;  // 40mS
  }
  else if(OCD_delay > 8)
  {
    OCD_delay_tmp = 0x01;  // 20mS
  }
  else
  {
    OCD_delay_tmp = 0x00;  // 8mS
  }
  //== Protect page36
  //PROTECT2_Last = (OCD_delay_tmp << 4) + OCD_val_tmp; //OCD
  //I2C_Write(PROTECT2_ADDR,PROTECT2_Last);
  PROTECT2_Last = 0x7F; //OCD
  I2C_Write(PROTECT2_ADDR,PROTECT2_Last);
}

//==========================================================================
void Afe_OV_UV_Delay_Set(uint8_t OV_delay, uint8_t UV_delay)
{
  if(OV_delay > 4)
  {
    OV_delay = 0x30;  // 8s
  }
  else if(OV_delay > 2)
  {
    OV_delay = 0x20;  // 4s
  }
  else if(OV_delay > 1)
  {
    OV_delay = 0x10;  // 2s
  }
  else
  {
    OV_delay = 0x00;  // 1s
  }
  //==============================
  if(UV_delay > 8)
  {
    UV_delay = 0x30;
  }
  else if(UV_delay > 4)
  {
    UV_delay = 0x20;
  }
  else if(UV_delay > 2)
  {
    UV_delay = 0x10;
  }
  else
  {
    UV_delay = 0x00;
  }
  PROTECT3_Last = (UV_delay << 2) + OV_delay ; //OV, UV delay time 4s
  I2C_Write(PROTECT3_ADDR,PROTECT3_Last);
}

//==========================================================================
void Afe_OV_UV_Threshold_Set(uint16_t OV_val, uint16_t UV_val)
{ 
  //==OV UV threshold setting
  if(OV_val >= 4250)
  {
    OV_val = 4250;
  }
  if(UV_val <= 2000)
  {
    UV_val = 2000;
  } 
  OV_TRIP_Last = (uint8_t)(((uint32_t)1000 * (OV_val - ADCOffset_Val)/ADCGain_Val) >> 4);  
  UV_TRIP_Last = (uint8_t)(((uint32_t)1000 * (UV_val - ADCOffset_Val)/ADCGain_Val) >> 4);  
  I2C_Write(UV_TRIP_ADDR,UV_TRIP_Last);
  I2C_Write(OV_TRIP_ADDR,OV_TRIP_Last);  
}

//==========================================================================
void Afe_CC_1Shot_Set(void)
{ 
  SYS_CTRL2.Bit.CC_EN = 0;
  SYS_CTRL2.Bit.CC_ONESHOT = 1;
  SYS_CTRL2.Bit.DELAY_DIS = 0;
  SYS_CTRL2_Last = SYS_CTRL2.Byte;
  //SYS_CTRL2_Last &= ~0x40;
  //SYS_CTRL2_Last |= 0x20;
  //I2C_Write(SYS_CTRL2_ADDR,SYS_CTRL2_Last); 
  I2C_Write(SYS_CTRL2_ADDR,SYS_CTRL2_Last);
  //SYS_CTRL2_Last = 0x20;
}


//==========================================================================
void Afe_CC_AlwaysOn_Set(void)
{
  SYS_CTRL2_Last |= 0x40;
  SYS_CTRL2_Last &= ~0x20;
  I2C_Write(SYS_CTRL2_ADDR,SYS_CTRL2_Last); 
}

//==========================================================================
/*-------------------------------------------------------------------------
      SYS_STAT (0x00)/RESET:0x00
      BIT      7      6           5             4       3   2    1     0
      NAME CC_READY  RSVD   DEVICE_XREADY   OVRD_ALERT  UV  OV  SCD   OCD
-------------------------------------------------------------------------*/
void Afe_Get_SysStatus(void)
{ 
    I2C_Read(SYS_STAT_ADDR,&SYS_STAT_Last); 
    SYS_STAT.Byte = SYS_STAT_Last;
    I2C_Read(SYS_CTRL1_ADDR,&SYS_CTRL1_Last); 
    SYS_CTRL1.Byte = SYS_CTRL1_Last;
    I2C_Read(SYS_CTRL2_ADDR,&SYS_CTRL2_Last); 
    SYS_CTRL2.Byte = SYS_CTRL2_Last;
    
   // I2C_Read(PROTECT1_ADDR,&PROTECT1_Last_Copy); 
   // I2C_Read(PROTECT2_ADDR,&PROTECT2_Last_Copy); 
   // I2C_Read(PROTECT3_ADDR,&PROTECT3_Last_Copy); 
}

void Afe_FET_ChgOn_DisOn(void)
{
  SYS_CTRL2_Last |= 0x03;
  I2C_Write(SYS_CTRL2_ADDR,SYS_CTRL2_Last);
}
void Afe_FET_ChgOn_DisOff(void)
{
  SYS_CTRL2_Last |= 0x01;
  SYS_CTRL2_Last &= ~0x02;
  I2C_Write(SYS_CTRL2_ADDR,SYS_CTRL2_Last);
}
void Afe_FET_ChgOff_DisOn(void)
{
  SYS_CTRL2_Last &= ~0x01;
  SYS_CTRL2_Last |= 0x02;
  I2C_Write(SYS_CTRL2_ADDR,SYS_CTRL2_Last);
}
void Afe_FET_ChgOff_DisOff(void)
{
  SYS_CTRL2_Last &= ~0x03;
  I2C_Write(SYS_CTRL2_ADDR,SYS_CTRL2_Last);
}

//==========================================================================
/*
      SYS_STAT (0x00)/RESET:0x00
      BIT      7      6           5             4       3   2    1     0
      NAME CC_READY  RSVD   DEVICE_XREADY   OVRD_ALERT  UV  OV  SCD   OCD
  
            CHG, DSG Response Under Various System Events
    EVENT                         [CHG_ON]              [DSG_ON]
    OV Fault                      Set to 0              ！
    UV Fault                      ！                    Set to 0
    OCD Fault                     ！                    Set to 0  
    SCD Fault                     ！                    Set to 0
    ALERT Override                Set to 0              Set to 0
    DEVICE_XREADY is set          Set to 0              Set to 0
    Enter SHIP mode from NORMAL   Set to 0              Set to 0
*/
void Afe_FET_ChgDis_Cntrl(void)
{
  if(WorkMode == IDLE_MODE)
  {
    Afe_FET_ChgOff_DisOff();
  }
  else if(WorkMode == CHARGE_MODE)
  {
    if(Bits_flag.Bit.ChgOv || Bits_flag.Bit.ChgTemp || Bits_flag.Bit.ChgCurOv)
    {
      Afe_FET_ChgOff_DisOff();
    }
    else
    { 
      Afe_FET_ChgOn_DisOn();
    }
  }
  else if(WorkMode == DISCHARGE_MODE)
  {
    if(Bits_flag.Bit.DisOv || Bits_flag.Bit.DisTemp || Bits_flag.Bit.DisCurOv)
    {
      Afe_FET_ChgOff_DisOff();
    }
    else
    { 
      Afe_FET_ChgOn_DisOn();
    }
  }
}

#endif
