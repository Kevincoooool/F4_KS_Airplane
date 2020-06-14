
#include "Voltage.h"
#include "Drv_ADC.h"
#include "RC.h"

float voltage = 4000;//单位 1mv

void voltage_check()
{
	static u16 cnt0,cnt1;
	u16 power0,power1;
	
	voltage += 0.2f *(2 *(3300 *AdcValue/4096) - voltage);

	if(flag.fly_ready && flag.thr_low==0)//飞行状态下
	{
		power0 = 3400;
		power1 = 3450;
	}
	else
	{
		power0 = 3700;
		power1 = 3750;
	}
	
	if(voltage < power0)//低压
	{
		cnt0++;
		cnt1=0;
		if(cnt0>60)
		{
			if(LED_warn==0)
			{
				LED_warn = 1;
			}
		}
	}
	else if(voltage > power1)//正常
	{
		cnt1++;
		cnt0=0;
		if(cnt1>60)
		{
			if(LED_warn==1)
			{
				LED_warn = 0;
			}
		}
	}
	else
	{
		cnt0=0;
		cnt1=0;
	}
}




