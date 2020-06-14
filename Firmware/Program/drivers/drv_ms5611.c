
#include "math.h"
#include "drv_ms5611.h"
#include "math.h"
#include "Filter.h"
#include "Flow.h"
#include "MotionCal.h"
#include "FlightDataCal.h"
#define BARO_CAL_CNT 200

int32_t ms5611_pre,baroAlt,baroAltOld,baroAlt_lpf;
float baro_alt_speed;
int32_t baro_Offset;
uint32_t ms5611_ut;  // static result of temperature measurement
uint32_t ms5611_up;  // static result of pressure measurement
uint16_t ms5611_prom[PROM_NB];  // on-chip ROM
uint8_t t_rxbuf[3],p_rxbuf[3];

void MS5611_Reset(void)
{
    IIC_Write_1Byte(MS5611_ADDR, CMD_RESET, 1);
}

u8 MS5611_Read_Prom(void)
{
	uint8_t rxbuf[2] = { 0, 0 };
	u8 check = 0;
	u8 i;

	for (i = 0; i < PROM_NB; i++)
	{
		check += IIC_Read_nByte(MS5611_ADDR, CMD_PROM_RD + i * 2, 2, rxbuf); // send PROM READ command
		ms5611_prom[i] = rxbuf[0] << 8 | rxbuf[1];
	}

	if(check==PROM_NB)
		return 1;
	else
		return 0;
}


void MS5611_Read_Adc_T(void)
{
	IIC_Read_nByte( MS5611_ADDR, CMD_ADC_READ, 3, t_rxbuf ); // read ADC
}

void MS5611_Read_Adc_P(void)
{
	IIC_Read_nByte(MS5611_ADDR, CMD_ADC_READ, 3, p_rxbuf); // read ADC
}

void MS5611_Start_T(void)
{
	IIC_Write_1Byte(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + MS5611_OSR, 1); // D2 (temperature) conversion start!
}

void MS5611_Start_P(void)
{
  IIC_Write_1Byte(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + MS5611_OSR, 1); // D1 (pressure) conversion start!
}

u8 ms5611_ok;
void MS5611_Init(void)
{
	
	
	//传感器复位
	MS5611_Reset();
	Delay_ms(100);
	ms5611_ok = !( MS5611_Read_Prom() );
	Delay_ms(200);
	//开始读取温度
	MS5611_Start_T();
}

int MS5611_Update(void)
{
	static int state = 0;
	
//	I2C_FastMode = 0;
	
	if (state) 
	{
			MS5611_Read_Adc_P();
			MS5611_Start_T();
			MS5611_BaroAltCalculate();
			state = 0;
	} 
	else 
	{
			MS5611_Read_Adc_T();
			MS5611_Start_P();
			state = 1;
	}
	return (state);
}

float temperature_5611;
u16 baro_start;
void MS5611_BaroAltCalculate(void)
{
	
	
  int32_t temperature, off2 = 0, sens2 = 0, delt;
  int32_t pressure;
	float alt_3;
	
	int32_t dT;
	int64_t off;
	int64_t sens;
	
		static vs32 sum_tmp_5611 = 0;
//		static u8 sum_cnt = BARO_CAL_CNT + 10;
	
		ms5611_ut = (t_rxbuf[0] << 16) | (t_rxbuf[1] << 8) | t_rxbuf[2];
		ms5611_up = (p_rxbuf[0] << 16) | (p_rxbuf[1] << 8) | p_rxbuf[2];
		
    dT = ms5611_ut - ((uint32_t)ms5611_prom[5] << 8);
    off = ((uint32_t)ms5611_prom[2] << 16) + (((int64_t)dT * ms5611_prom[4]) >> 7);
    sens = ((uint32_t)ms5611_prom[1] << 15) + (((int64_t)dT * ms5611_prom[3]) >> 8);
    temperature = 2000 + (((int64_t)dT * ms5611_prom[6]) >> 23);

    if (temperature < 2000) { // temperature lower than 20degC 
        delt = temperature - 2000;
        delt = delt * delt;
        off2 = (5 * delt) >> 1;
        sens2 = (5 * delt) >> 2;
        if (temperature < -1500) { // temperature lower than -15degC
            delt = temperature + 1500;
            delt = delt * delt;
            off2  += 7 * delt;
            sens2 += (11 * delt) >> 1;
        }
    }
    off  -= off2; 
    sens -= sens2;
    pressure = (((ms5611_up * sens ) >> 21) - off) >> 15;
	alt_3 = (101000 - pressure)/1000.0f;
	pressure = 0.0082f *alt_3 * alt_3 *alt_3 + 0.09f *(101000 - pressure)*100.0f ;
	//pressure = 4430000 * ( 1.0f - powf( pressure / 101325.0f , 1.0f / 5.256f ) );
	baroAlt = ( pressure - baro_Offset ) ; //cm
	
	if( baro_start < 200 )
	{
		baro_start++;		
		baroAlt = 0;
		if(baro_start<50)
		{
			baro_Offset = baroAlt;
			
		}
		else
		{
			baro_Offset += 10.0f *3.14f *0.05f *(pressure - baro_Offset);
			//WCZ_Data_Reset();
		}
	}	
//	if(sum_cnt)
//		{
//			sum_cnt--;
// 			if(sum_cnt < BARO_CAL_CNT)
// 				sum_tmp_5611 += pressure;
// 			if(sum_cnt==0)
// 				baro_Offset = sum_tmp_5611 / (BARO_CAL_CNT - 1);
//		}
		temperature_5611 += 0.01f *( ( 0.01f *temperature ) - temperature_5611 );
}

int32_t MS5611_Get_BaroAlt(void)
{
	return baroAlt;
}


s32 baro_speed_o,baro_speed_lpf;
float speed_delta;
float speed_delta_lpf;
//Butter_Parameter Hight_Parameter,Hight_vel_Parameter;
//Butter_BufferData Buffer_Hight[2],Buffer_Hight_Vel[2];
//void Hight_Init(void)
//{
//  Set_Cutoff_Frequency(50, 0.5,&Hight_Parameter);//20
//  Set_Cutoff_Frequency(50, 2,&Hight_vel_Parameter);//20
//}
//void Height_Get(float dT)
//{

//	baro_height_ms5611 = (MS5611_Get_BaroAlt()); 
//	baro_height_lpf=LPButterworth(baro_height_ms5611,&Buffer_Hight[0],&Hight_Parameter);

//}
int32_t BaroGetAlt(void)
{
	return baroAlt;
}
float BaroGetVelocity(void)
{
    return baro_speed_o;
}
