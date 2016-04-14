

#include"soc.h"

uint8_t bSocOCVCorrectEn = 0;

X_SOC_V0 SocReg;

X_SOC_CALC SocCalc;

/*
const X_SOC_2_OCV Soc2OcvTbl[11] =  // 根据实际修改
{// soc, volt,
    {0, 3440}, {10, 3523}, {20, 3585}, {30, 3615}, {40, 3643}, {50, 3690}, // 润丰电池
    {60, 3755}, {70, 3850}, {80, 3930}, {90, 4040}, {100, 4190},
    //{0, 3435}, {10, 3510}, {20, 3572}, {30, 3610}, {40, 3635}, {50, 3675}, // 天能电池
    //{60, 3755}, {70, 3890}, {80, 3935}, {90, 4041}, {100, 4188},
};

const X_TEMP_2_CAP Temp2CapacityTbl[10] =   // 根据实际修改
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
	X_SOC_2_OCV Soc2OcvTbl[11] =  // 根据实际修改
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

const X_TEMP_2_CAP Temp2CapacityTbl[10] =  // 根据实际修改
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


/*  计算完soc  后调用，以更新bms 数据结构中soc 相关的数值。
*/
void BMSReflashFromSOC(void)
{
    //BMSRegs.soc_rt = SocReg.soc;  // NOTE: soc_rt 由smooth 函数更新。
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

/*  计算电流积分，每10ms 调用一次。
*
*   输入参数:
*  SocReg.curr                  实时电流
*  SocReg.curr_dir              电流方向
*
*   输出:   用于计算的中间值。
*
*/
void SOCAhIntergrate(void)
{
    uint32_t tmpah;
    tmpah = SocReg.curr;    // xmA 10ms
#if 0

    if(tmpah < 10)  // 极小电流认为是干扰或者处于静置状态。
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
            SocCalc.inAh_bak = SocCalc.inAh_bak % 360000; // 充入1Ah。
        }
    }
    else
    {
        SocCalc.outAh_bak += tmpah;

        if(SocCalc.outAh_bak > 360000)
        {
            SocCalc.totalOutAh += SocCalc.outAh_bak / 360000;
            SocCalc.outAh_bak = SocCalc.outAh_bak % 360000; // 放出1Ah。
        }
    }
}
//================= 将soc 范围 10~90 转换为 soc_rt 所需的 0~100。
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
/*  计算SOC，每100ms 调用一次。
*
*   输入参数:
*  SocReg.ah                当前剩余容量
*  SocReg.max_cell_vlt      最高单节电压
*  SocReg.max_cell_chg_vlt  过冲电压
*  SocReg.min_cell_vlt      最低单节电压
*  SocReg.min_cell_dchg_vlt 过放电压
*  SocReg.rated_cap         额定容量
*
*   输出:
*  SocReg.ah            新的容量
*  SocReg.soc           新的SOC
*
*/
void SOCCalculate(void)
{
    uint32_t deltAh;
    SocCalc.curAh = SocReg.ah;
    SOCCorrectOCV();    // OCV 校准
    //SOCCorrectTemp();   // 温度校准

    if(SocCalc.totalInAh > SocCalc.totalInAh_bak)   // 充入。
    {
        deltAh = (uint16_t)(SocCalc.totalInAh - SocCalc.totalInAh_bak);
        SocCalc.curAh += deltAh;
        //HAL_GPIO_TogglePin(GPIOB,  GPIO_PIN_9);

        if(SocCalc.curAh > SocReg.rated_cap)    // 防止大于额定容量。
        {SocCalc.curAh = SocReg.rated_cap;}

        if(SocReg.max_cell_vlt > SocReg.max_cell_chg_vlt)   // 有任一电池过充了。
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

    if(SocCalc.totalOutAh > SocCalc.totalOutAh_bak) // 放出。
    {
        deltAh = (uint16_t)(SocCalc.totalOutAh - SocCalc.totalOutAh_bak);

        if(SocCalc.curAh < deltAh)  // 防止小于0。
        {SocCalc.curAh = 0;}
        else
        { SocCalc.curAh -= deltAh; }

        if(SocReg.min_cell_vlt < SocReg.min_cell_dchg_vlt)  // 有任一电池过放了。
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

    // 计算SOC。
    SocReg.ah = SocCalc.curAh;
		// 平均电芯电压4.1V 时 SOC要显示100%， 3.1V 时，SOC要显示0%
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
/*  静置后进行OCV 修正。
*
*   需要输入的参数:
*
*  1. SocReg.curr           实时电流
*  2. SocReg.min_cell_vlt   最低单节电压(关键参数)
*
*   输出:
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

    if(tmpah < 10)   // 极小电流认为是干扰或者处于静置状态。
    {
        if(SocCalc.stb_cnt < 36000) // 计时1 小时
        {SocCalc.stb_cnt++;}
    }
    else
    {SocCalc.stb_cnt = 0;}

    if(SocCalc.stb_cnt >= 36000) // 静置1 小时后启动一次OCV 校准。
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

        if(SocReg.min_cell_vlt <= Soc2OcvTbl[0].volt)   // 防止超范围
        {
					soc = 0;
				}
        else if(SocReg.min_cell_vlt >= Soc2OcvTbl[10].volt)
        {
					soc = 100;
				}
        else
        {
            for(i = 1; i < 11; i++)     // 查表根据OCV 表取soc。
            {
                if(SocReg.min_cell_vlt == Soc2OcvTbl[i].volt)   // NOTE: 采用最低电压的一节电池来比较。
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

/*  进行温度和额定容量的校正。
*
*   需要输入的参数:
*
*  1. SocReg.min_cell_temp  最低单节温度
*  2. SocReg.temp_corr      校正使能标志位
*
*   输出:
*
*  1. SocReg.rated_cap      修正后的额定容量
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
                    // 线性化再等分。
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

/*  10 秒内将SocReg.soc  值平滑变化到BMSRegs.soc_rt  值。
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


