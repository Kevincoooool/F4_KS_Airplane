#ifndef _MS5611_H
#define _MS5611_H

#include "stm32f4xx.h"
#include "Drv_i2c_soft.h"
#include "Drv_icm20689.h"

#define MS5611_ADDR             0x77<<1   //0xee //

#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8
#define MS5611_OSR				0x08	//CMD_ADC_4096


	
	//��ѹ�Ƴ�ʼ��
	void MS5611_Init(void);
	//��ȡ��ѹ������
	int MS5611_Update(void);	
	//������ѹ�߶�
	
	int32_t MS5611_Get_BaroAlt(void);
void Hight_Init(void);
	void MS5611_Reset(void);
	u8 MS5611_Read_Prom(void);
	void MS5611_Start_T(void);
	void MS5611_Start_P(void);
	void MS5611_Read_Adc_T(void);
	void MS5611_Read_Adc_P(void);
	void MS5611_BaroAltCalculate(void);
extern u16 baro_start;
extern u8 ms5611_ok;
	extern int32_t ms5611_pre,baroAlt,baroAltOld;
	extern float baro_alt_speed;
	void Height_Get(float dT);
	int32_t BaroGetAlt(void);
	float BaroGetVelocity(void);
#endif
