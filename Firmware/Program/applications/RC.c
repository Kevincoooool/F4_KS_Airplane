

#include "RC.h"
#include "KS_Data_Transfer.h"
#include "include.h"
#include "WIFI_UFO.h"
#define UN_YAW_VALUE  200
#define UN_THR_VALUE -200

#define TIME 600 //100*10ms=1s

u8 fly_ready = 0;
float turn_head = 1;//改变机头变量
u16 fly_ulock_cnt = 0;
u16 fly_ready_cnt = 0;
u16 Calibrate_cnt = 0;

u16 RX_CH[CH_NUM];

s16 CH_N[CH_NUM] = {0,0,0,0};

void unlock()
{

	if(CH_N[CH_THR] < UN_THR_VALUE)
	{
			
		//解锁检测
		if(CH_N[CH_YAW]>UN_YAW_VALUE)
		{
			fly_ready_cnt++;
			if(fly_ready_cnt>1000) //100*dT秒
			{
				//fly_ready_cnt = 0;
				auto_take_off = 0;
				flag.fly_ready = 1;
				flag.locking = 0;
				start = 0;//起飞
				high_start = 80;//初始化起飞高度
				LED_warn = 2;
			}
		}
//		else
//		{
//			
//			fly_ready_cnt = 0;
//			
//		}
		//上锁检测
		if(CH_N[CH_YAW]<-UN_YAW_VALUE)
		{
			fly_ready_cnt++;

			if(fly_ready_cnt>1000) //200*dT秒
			{
				flag.locking = 1;
				flag.fly_ready = 0;
				//fly_ready_cnt = 0;
			}
		}
//		else
//		{
//			fly_ready_cnt = 0;
//			
//		}
		if(flag.fly_ready==0)	flag.thr_low = 1;//上锁状态下油门都标记为拉低
		
		//if(flag.NS==3 || flag.NS==0) return;//无信号或者蓝牙信号，不进行手势检测
		
		//校准检测
		if((flag.fly_ready==0) && (CH_N[CH_THR]<-300) && (CH_N[CH_PIT]<-300)&&flag.NS ==1)
		{
			Calibrate_cnt++;
			if(Calibrate_cnt>TIME)
			{
				sensor.acc_CALIBRATE = 1;
				sensor.gyr_CALIBRATE = 1;
				baro_start = 1;
				LED_warn = 2;
				Calibrate_cnt = 0;
				//auto_take_off = 1;
			}
		}
		else
		{
			Calibrate_cnt = 0;
		}
		
	}
	else
	{
		fly_ready_cnt = 0;
	}
	
	if(CH_N[CH_THR]>-300&&flag.fly_ready)
	{
		flag.thr_low = 0;//油门非低
	}
	else
	{
		flag.thr_low = 1;//油门拉低
	}
		
}

void RC_duty(float dT) //建议2ms调用一次
{
	u8 i ;
	for(i=0;i<CH_NUM;i++)
	{
		CH_N[i] = ((s16)RX_CH[i]-1500);
	}

		//空翻过程中如果出现意外，拉低油门可以取消空翻
	if(roll_en && CH_N[CH_THR]<-400) roll_en = 0;
	
	unlock();
	
//失控保护检查
	fail_safe_check();
}

void fail_safe()
{
	if(flag.NS == 0)
	{
		RX_CH[CH_YAW]=1500;
		RX_CH[CH_ROL]=1500;
		RX_CH[CH_PIT]=1500;
		RX_CH[CH_THR]=1500;
//		if(RX_CH[THR]>1000)
//		{
//			RX_CH[THR]-=1;
//		}
//		else 
//		{
//			RX_CH[THR] = 1000;
//		}
		if (fly_ready) start = 3;
	}
}


void fail_safe_check() //dT秒调用一次
{
	static u16 cnt;
	
	if(flag.NS == 0)
	{
		cnt++;
		if(cnt >= 500) //500*dT 秒
		{
			fail_safe();
//			fly_ready  = 0;
			cnt = 0;
		}
	}
	else
	{
		

	}

	
	
	//flag.NS = 0;
}
void key_function(u8 key)
{
	extern u8 yaw_lock;
	static u8 count1=0,count2=0;

	///////////////////////////////////////////////////////		
	if(key&0x02)//左边按键,一键起飞和降落
	{
		count1++;
		if(count1>=200)count1=200;
	}
	else
	{
		if(count1>=2)	
		{
			//定高模式下才能启动一键起飞
			if(keep_high_mode)
			{
				if(flag.fly_ready==0)
				{
					auto_landing=0;
					auto_take_off = 1;
					flag.fly_ready = 1;
					high_start = HIGH_START;//目标高度
				}
				else
				{
					auto_landing = 1;
				}
			}
			LED_warn = 2;
		}
		count1=0;
	}
	if(key&0x04)//右键，空翻
	{
		count2++;
		if(count2>=200)count2=200;
	}
	else
	{
		if(count2>=2)	
		{
			//非定高模式下才能空翻
			if(fly_ready) 
			{
				roll_en = 1;
			}
			LED_warn = 2;
		}
		count2=0;
	}
}
void Rc_Connect(void)
{
	//控制数据优先级：
	//1、遥控器。2：WiFi图传。3：蓝牙模块
	if(NRF_Connect()==0)
	{
//		if(WIFI_UFO_Connect()==0)
//		{
//			//Bluet_Connect();
//		}
	}

}
u16 test_flag,set_flag;

void Flag_Check(void)
{

	set_flag = 0;
	
	if(fly_ready)	      set_flag |= BIT0;
	if(No_Head_Mode)	set_flag |= BIT1;
	if(No_Head_Mode)		set_flag |= BIT2;
	if(roll_en == 1)   set_flag |= BIT3;
//	if(sensor.acc_CALIBRATE == 1)  set_flag |= BIT4;
	if(sys_init_ok == 1)  set_flag |= BIT5;
	
	test_flag = 0;
	
	if(!sens_hd_check.gyro_ok)		test_flag |= BIT0;
	if(!sens_hd_check.baro_ok)		test_flag |= BIT1;
	if(!sens_hd_check.nrf_ok)		test_flag |= BIT2;

}



