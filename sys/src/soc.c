

#include"soc.h"

uint8_t bSocOCVCorrectEn = 0;

X_SOC_V0 SocReg;

X_SOC_CALC SocCalc;

/*
const X_SOC_2_OCV Soc2OcvTbl[11] =  // ����ʵ���޸�
{// soc, volt,
    {0, 3440}, {10, 3523}, {20, 3585}, {30, 3615}, {40, 3643}, {50, 3690}, // �����
    {60, 3755}, {70, 3850}, {80, 3930}, {90, 4040}, {100, 4190},
    //{0, 3435}, {10, 3510}, {20, 3572}, {30, 3610}, {40, 3635}, {50, 3675}, // ���ܵ��
    //{60, 3755}, {70, 3890}, {80, 3935}, {90, 4041}, {100, 4188},
};

const X_TEMP_2_CAP Temp2CapacityTbl[10] =   // ����ʵ���޸�
{// temp, ratedcap
    // -20drg -> 50%, -10drg -> 75%, 0drg -> 90%, 10drg -> 95%, 25drg -> 100%
    { -20, 11000}, { -15, 14000},
    { -10, 16500}, { -5, 17800},
    {0, 19800}, {5, 19950},
    {10, 21000}, {15, 21333},
    {20, 21666}, {25, 22000},
};
*/
///const 
	X_SOC_2_OCV Soc2OcvTbl[11] =  // ����ʵ���޸�
{
    //{0, 3440}, {10, 3523}, {20, 3585}, {30, 3615}, {40, 3643}, {50, 3690}, // ????
    //{60, 3755}, {70, 3850}, {80, 3930}, {90, 4040}, {100, 4190},
    {0, 3335}, {10, 3510}, {20, 3572}, {30, 3610}, {40, 3635}, {50, 3675}, // ????
    {60, 3755}, {70, 3890}, {80, 3935}, {90, 4041}, {100, 4188},
};
/*const X_SOC_2_OCV Soc2OcvTbl_10deg[11] = // 10'C
{
    {0, 3440}, {10, 3512}, {20, 3571}, {30, 3605}, {40, 3633}, {50, 3674},
    {60, 3749}, {70, 3836}, {80, 3931}, {90, 4038}, {100, 4191},
};*/
const X_SOC_2_OCV Soc2OcvTbl_0deg[11] = // 0'C - 20160131
{
    {0, 3339}, {10, 3500}, {20, 3555}, {30, 3594}, {40, 3629}, {50, 3671},
    {60, 3736}, {70, 3830}, {80, 3932}, {90, 4040}, {100, 4196},
};
const X_SOC_2_OCV Soc2OcvTbl_N10deg[11] = // -10'C
{
    {0, 3303}, {10, 3385}, {20, 3531}, {30, 3574}, {40, 3611}, {50, 3656},
    {60, 3720}, {70, 3811}, {80, 3918}, {90, 4029}, {100, 4184},
};

const X_TEMP_2_CAP Temp2CapacityTbl[10] =  // ����ʵ���޸�
{
    /*// -20drg -> 50%, -10drg -> 75%, 0drg -> 90%, 10drg -> 95%, 25drg -> 100%
    { -20, 11000}, { -15, 14000},
    { -10, 16500}, { -5, 17800},
    {0, 19800}, {5, 19950},
    {10, 21000}, {15, 21333},
    {20, 21666}, {25, 22000},*/
    // -20drg -> 83%, -10drg -> 87%, 0drg -> 92%, 10drg -> 98%, 25drg -> 100%
    { -20, 18260}, { -15, 18700}, // ??TNL-13-ITR-10 ???????????????
    { -10, 19140}, { -5, 19800},
    {0, 20020}, {5, 20900},
    {10, 21560}, {15, 21780},
    {20, 21890}, {25, 22000},
};


extern X_BMS_REG_V3 BMSRegs;

void SOCCorrectOCV(void);
void SOCCorrectTemp(void);
void SOCSmooth(void);


/*  ������soc  ����ã��Ը���bms ���ݽṹ��soc ��ص���ֵ��
*/
void BMSReflashFromSOC(void)
{
    //BMSRegs.soc_rt = SocReg.soc;  // NOTE: soc_rt ��smooth �������¡�
    //BMSRegs.dc_flt_t_rt=
    //BMSRegs.c_cont=
}

extern uint32_t Current_float;// = 44000; // 44000=44A
extern uint16_t Total_Volt_float;// = 560; // 560=56V

extern uint16_t Cell_Volt_float[13];
extern uint16_t CellVolt_Min_float;// = 3900;
extern uint16_t CellVolt_Max_float;// = 4187;

extern int8_t Temp_RT[5];
extern int8_t Temp_RT_Min;// = -20;
extern int8_t Temp_RT_Max;// = 30;

extern uint16_t Bat_Status_Rt;// = 0xff;
 
void BMSReflashFromAFE(void)
{
    bSocOCVCorrectEn = 1;
    SocReg.temp_corr = 0;
}

/*  ����������֣�ÿ10ms ����һ�Ρ�
*
*   �������:
*  SocReg.curr                  ʵʱ����
*  SocReg.curr_dir              ��������
*
*   ���:   ���ڼ�����м�ֵ��
*
*/
void SOCAhIntergrate(void)
{
    uint32_t tmpah;
    tmpah = SocReg.curr;    // xmA 10ms
#if 0

    if(tmpah < 10)  // ��С������Ϊ�Ǹ��Ż��ߴ��ھ���״̬��
    {
        SocCalc.totalInAh_bak = SocCalc.totalInAh;
        SocCalc.totalOutAh_bak = SocCalc.totalOutAh;
        return;
    }

#endif

    if(1 == SocReg.curr_dir)
    {
        SocCalc.inAh_bak += tmpah;

        if(SocCalc.inAh_bak > 360000)
        {
            SocCalc.totalInAh += SocCalc.inAh_bak / 360000;
            SocCalc.inAh_bak = SocCalc.inAh_bak % 360000; // ����1Ah��
        }
    }
    else
    {
        SocCalc.outAh_bak += tmpah;

        if(SocCalc.outAh_bak > 360000)
        {
            SocCalc.totalOutAh += SocCalc.outAh_bak / 360000;
            SocCalc.outAh_bak = SocCalc.outAh_bak % 360000; // �ų�1Ah��
        }
    }
}
//================= ��soc ��Χ 10~90 ת��Ϊ soc_rt ����� 0~100��
void SOCConvert(void)
{
	if(SocReg.soc>90)
	{
		BMSRegs.soc_rt=100;
	}
	else if(SocReg.soc<10)
	{
		BMSRegs.soc_rt=0;
	}
	else
	{
		BMSRegs.soc_rt = (SocReg.soc -10)*5/4;
	}
}
/*  ����SOC��ÿ100ms ����һ�Ρ�
*
*   �������:
*  SocReg.ah                ��ǰʣ������
*  SocReg.max_cell_vlt      ��ߵ��ڵ�ѹ
*  SocReg.max_cell_chg_vlt  �����ѹ
*  SocReg.min_cell_vlt      ��͵��ڵ�ѹ
*  SocReg.min_cell_dchg_vlt ���ŵ�ѹ
*  SocReg.rated_cap         �����
*
*   ���:
*  SocReg.ah            �µ�����
*  SocReg.soc           �µ�SOC
*
*/
void SOCCalculate(void)
{
    uint32_t deltAh;
    SocCalc.curAh = SocReg.ah;
    SOCCorrectOCV();    // OCV У׼
    //SOCCorrectTemp();   // �¶�У׼

    if(SocCalc.totalInAh > SocCalc.totalInAh_bak)   // ���롣
    {
        deltAh = (uint16_t)(SocCalc.totalInAh - SocCalc.totalInAh_bak);
        SocCalc.curAh += deltAh;
        //HAL_GPIO_TogglePin(GPIOB,  GPIO_PIN_9);

        if(SocCalc.curAh > SocReg.rated_cap)    // ��ֹ���ڶ������
        {SocCalc.curAh = SocReg.rated_cap;}

        if(SocReg.max_cell_vlt > SocReg.max_cell_chg_vlt)   // ����һ��ع����ˡ�
        {
            if(SocCalc.ov_cnt > 30)
            {
                SocCalc.ov_cnt = 30;
                SocCalc.curAh = SocReg.rated_cap;
            }
            else
            {SocCalc.ov_cnt++;}
        }
        else
        {SocCalc.ov_cnt = 0;}

        SocCalc.totalInAh_bak = SocCalc.totalInAh;
    }

    if(SocCalc.totalOutAh > SocCalc.totalOutAh_bak) // �ų���
    {
        deltAh = (uint16_t)(SocCalc.totalOutAh - SocCalc.totalOutAh_bak);

        if(SocCalc.curAh < deltAh)  // ��ֹС��0��
        {SocCalc.curAh = 0;}
        else
        { SocCalc.curAh -= deltAh; }

        if(SocReg.min_cell_vlt < SocReg.min_cell_dchg_vlt)  // ����һ��ع����ˡ�
        {
            if(SocReg.min_cell_vlt > 0)
            {
                if(SocCalc.uv_cnt > 30)
                {
                    SocCalc.uv_cnt = 30;
                    SocCalc.curAh = 0;
                }
                else
                {SocCalc.uv_cnt++;}
            }
        }
        else
        {SocCalc.uv_cnt = 0;}

        SocCalc.totalOutAh_bak = SocCalc.totalOutAh;
    }

    // ����SOC��
    SocReg.ah = SocCalc.curAh;
		// ƽ����о��ѹ4.1V ʱ SOCҪ��ʾ100%�� 3.1V ʱ��SOCҪ��ʾ0%
	  /*if(Avg_Volt_float >= 4100)
		{
			SocReg.soc = 100;
		}
		else if(Avg_Volt_float <= 3100)
		{
			SocReg.soc = 0;
		}
		else 	*/
    {
			SocReg.soc = (uint8_t)(SocCalc.curAh * 100 / SocReg.rated_cap);
		}
    //SOCSmooth();
    SOCConvert();//BMSRegs.soc_rt = SocReg.soc;
    BMSReflashFromSOC();
}
/*  ���ú����OCV ������
*
*   ��Ҫ����Ĳ���:
*
*  1. SocReg.curr           ʵʱ����
*  2. SocReg.min_cell_vlt   ��͵��ڵ�ѹ(�ؼ�����)
*
*   ���:
*
*  1. SocReg.soc
*  2. SocCalc.curAh
*/
void SOCCorrectOCV(void)
{
    uint8_t i, soc;
    uint32_t tmpah;
    uint16_t deltV;
    tmpah = SocReg.curr;    // xmA 10ms
#if 0

    if(tmpah < 10)   // ��С������Ϊ�Ǹ��Ż��ߴ��ھ���״̬��
    {
        if(SocCalc.stb_cnt < 36000) // ��ʱ1 Сʱ
        {SocCalc.stb_cnt++;}
    }
    else
    {SocCalc.stb_cnt = 0;}

    if(SocCalc.stb_cnt >= 36000) // ����1 Сʱ������һ��OCV У׼��
#else
    if(bSocOCVCorrectEn == 1)
#endif
    {
        //SocCalc.stb_cnt = 0;
        bSocOCVCorrectEn = 0;
/**/
        if(SocReg.min_cell_temp >= 0) // 20160131 - add for OCV tbl under different temp.
        {
            for(i = 0; i < 11; i++)
            { 
							Soc2OcvTbl[i].volt = Soc2OcvTbl_0deg[i].volt; 
						}
        }
        else
        {
            for(i = 0; i < 11; i++)
            { 
							Soc2OcvTbl[i].volt = Soc2OcvTbl_N10deg[i].volt; 
						}
        }

        if(SocReg.min_cell_vlt <= Soc2OcvTbl[0].volt)   // ��ֹ����Χ
        {
					soc = 0;
				}
        else if(SocReg.min_cell_vlt >= Soc2OcvTbl[10].volt)
        {
					soc = 100;
				}
        else
        {
            for(i = 1; i < 11; i++)     // ������OCV ��ȡsoc��
            {
                if(SocReg.min_cell_vlt == Soc2OcvTbl[i].volt)   // NOTE: ������͵�ѹ��һ�ڵ�����Ƚϡ�
                {
                    soc = Soc2OcvTbl[i].soc;
                    break;
                }
                else if(SocReg.min_cell_vlt < Soc2OcvTbl[i].volt)
                {
                    deltV = (Soc2OcvTbl[i].soc - Soc2OcvTbl[i - 1].soc) * 1000 / (Soc2OcvTbl[i].volt - Soc2OcvTbl[i - 1].volt);
                    soc = Soc2OcvTbl[i - 1].soc + (SocReg.min_cell_vlt - Soc2OcvTbl[i - 1].volt) * deltV / 1000;
                    break;
                }
            }
        }

        SocReg.soc = soc;
        SocCalc.curAh = (uint32_t)(SocReg.rated_cap * soc) / 100;
        SocReg.ah = SocCalc.curAh;
    }
}

/*  �����¶ȺͶ������У����
*
*   ��Ҫ����Ĳ���:
*
*  1. SocReg.min_cell_temp  ��͵����¶�
*  2. SocReg.temp_corr      У��ʹ�ܱ�־λ
*
*   ���:
*
*  1. SocReg.rated_cap      ������Ķ����
*/
void SOCCorrectTemp(void)
{
    int8_t minTemp;
    uint16_t deltT;
    uint8_t i;
    minTemp = SocReg.min_cell_temp;

    if(SocReg.temp_corr == 1)
    {
        SocReg.temp_corr = 0;

        if(minTemp <= Temp2CapacityTbl[0].temp)
        {SocReg.rated_cap = Temp2CapacityTbl[0].rated_cap;}
        else if(minTemp >= Temp2CapacityTbl[9].temp)
        {SocReg.rated_cap = Temp2CapacityTbl[9].rated_cap;}
        else
        {
            for(i = 1; i < 10; i++)
            {
                if(minTemp == Temp2CapacityTbl[i].temp)
                {
                    SocReg.rated_cap = Temp2CapacityTbl[i].rated_cap;
                    break;
                }
                else if(minTemp < Temp2CapacityTbl[i].temp)
                {
                    // ���Ի��ٵȷ֡�
                    deltT = (Temp2CapacityTbl[i].rated_cap - Temp2CapacityTbl[i - 1].rated_cap)
                            / (Temp2CapacityTbl[i].temp - Temp2CapacityTbl[i - 1].temp);
                    SocReg.rated_cap = Temp2CapacityTbl[i - 1].rated_cap
                                       + (SocReg.min_cell_temp - Temp2CapacityTbl[i - 1].temp) * deltT;
                    break;
                }
            }
        }
    }
}

/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/

uint8_t smooth_cnt;
sword smooth_delt_v;
uint16_t old_v = 0;
uint16_t new_v = 0;

/*  10 ���ڽ�SocReg.soc  ֵƽ���仯��BMSRegs.soc_rt  ֵ��
*/
void SOCSmooth(void)
{
    new_v = SocReg.soc * 10;

    //if((old_v != new_v)&&(abs(old_v-new_v)<10))
    if(old_v != new_v)
    {
        smooth_cnt++;

        if(smooth_cnt > 5) // 100ms=1 x 100ms
        {
            smooth_cnt = 0;

            if(old_v > new_v)
            {
                smooth_delt_v = (old_v - new_v) / 10;
                old_v -= smooth_delt_v;
            }
            else
            {
                smooth_delt_v = (new_v - old_v) / 10;
                old_v += smooth_delt_v;
            }

            if(smooth_delt_v == 0)
            { old_v = new_v; }
        }
    }
    else
    { smooth_cnt = 0; }

    BMSRegs.soc_rt = (uint8_t)(old_v / 10);
}


