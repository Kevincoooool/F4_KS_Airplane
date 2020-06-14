
#include "Drv_LED.h"
#include "RC.h"

void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_LED,ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin   = LED_R|LED_G| LED_B;
	GPIO_Init(GPIO_LED, &GPIO_InitStructure);


	GPIO_SetBits(GPIO_LED, LED_R);
	GPIO_SetBits(GPIO_LED, LED_G);
	GPIO_SetBits(GPIO_LED, LED_B);
}



u16 led_accuracy = 20;//该时间应与LED_Duty()调用周期相同
float on_time=20;

void LED_1ms_DRV()
{
	static u16 led_cnt;
	
	if(led_cnt<on_time)
	{
		if(flag.fly_ready==1)
		{
			LED_R_ON;
			//LED_G_ON;
		}
		
		else
		{
			LED_B_ON;
		}
	}
	else
	{
		if(flag.fly_ready==1)
		{
			LED_R_OFF;
			//LED_G_OFF;
		}
		
		else
		{
			LED_B_OFF;
		}
	}
	
	if(++led_cnt>=led_accuracy)
	{
		led_cnt = 0;
	}

}
u8 LED_warn;
//亮-灭 为一组
void LED_Duty(float dT)
{
	switch(LED_warn)
	{
		case 0:
			if(!flag.fly_ready)
			{
				led_breath(dT,1000);
			}
			else
			{
				led_flash(dT,2,100,100,500);//调用周期（s），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
			}			
		break;
			
		case 1://没电
			
			led_flash(dT,1,80,80,0);//调用周期（s），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
		break;
		
		case 2://校准gyro
			if(led_flash(dT,6,100,100,0)) //调用周期（s），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
			{
				LED_warn = 0;
			}
		break;
		
		case 3://校准acc
			if(led_flash(dT,12,100,100,0)) //调用周期（s），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
			{
				LED_warn = 0;
			}
			
		break;
		
		default:break;
	}



}



u8 led_breath(float dT,u16 T)// 一次半程的时间，单位ms
{
	u8 f = 0;
	static u8 dir;
	switch(dir)
	{
		case 0:
			on_time += safe_div(led_accuracy,(T/(dT*1000)),0);
			if(on_time>20)
			{
				dir = 1;
			}
		break;
		case 1:
			on_time -= safe_div(led_accuracy,(T/(dT*1000)),0);
			if(on_time<0)
			{
				dir = 0;
				f = 1;
			}
		break;
			
		default:
			dir = 0;
		break;
			
	}
	return (f);

}
//亮-灭 为一组
//调用周期（s），组数，亮时间(ms)，灭时间(ms)，组间隔 ,ms>led_accuracy;
u8 led_flash(float dT,u16 group_n,u16 on_ms,u16 off_ms,u16 group_dT_ms)
{
	u8 f=0;
	static u16 ms_cnt;
	static u16 group_n_cnt;
	
	if(group_n_cnt < group_n)   //组数没到
	{
		if(ms_cnt<on_ms)
		{
			on_time = 20;
		}
		else if(ms_cnt<(on_ms+off_ms))
		{
			on_time = 0;
		}
		if(ms_cnt>=(on_ms+off_ms))
		{
			group_n_cnt ++;
			ms_cnt = 0;
		}
	}
	else						//进入组间隔
	{
		if(ms_cnt<group_dT_ms)
		{
			on_time = 0;
		}
		else
		{
			group_n_cnt = 0;
			ms_cnt = 0;
			f = 1; //流程完成1次
		}
	}
	
	ms_cnt += (dT*1000);        //计时
	return (f); //0，未完成，1完成
}
