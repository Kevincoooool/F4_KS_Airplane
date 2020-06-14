#include "MotorCtrl.h"
#include "mymath.h"
#include "RC.h"
#include "Drv_icm20689.h"
#include "Imu.h"
#include "Drv_pwm_out.h"
#include "MotionCal.h"
#include "Filter.h"
//#include "Navigate.h"

/*
电机顺序
      前
   m2     m1
     \   /
      \ /
      / \
     /   \
   m3     m4
      后
*/
s16 motor[MOTOR_NUM];
s16 motor_step[MOTOR_NUM];
//float motor_lpf[MOTOR_NUM];

static u16 motor_prepara_cnt;
_mc_st mc;
u16 IDLING;//10*Fly_Parame.set.idle_speed_pwm  //200
void Motor_Ctrl_Task(u8 dT_ms)
{
	u8 i;
	
//	if(flag.taking_off)
//	{
//		flag.motor_preparation = 1;
//		motor_prepara_cnt = 0;			
//	}
	
	if(flag.fly_ready)
	{		
		IDLING = 10*LIMIT(Param.idle_speed_pwm,0,10);
		
		if(flag.motor_preparation == 0)
		{
			motor_prepara_cnt += dT_ms;
			
			if(flag.motor_preparation == 0)
			{			
				if(motor_prepara_cnt<300)
				{
					motor[m1] = IDLING;
				}
				else if(motor_prepara_cnt<600)
				{
					motor[m2] = IDLING;
				}
				else if(motor_prepara_cnt<900)
				{
					motor[m3] = IDLING;
				}	
				else if(motor_prepara_cnt<1200)
				{	
					motor[m4] = IDLING;
				}
				else
				{
					flag.motor_preparation = 1;
					motor_prepara_cnt = 0;
				}
			}
			
		}	
	}
	else
	{
		flag.motor_preparation = 0;
	}
	

			
	if(flag.motor_preparation == 1)
	{	
		motor_step[m1] = mc.ct_val_thr  -mc.ct_val_yaw;
		motor_step[m2] = mc.ct_val_thr  +mc.ct_val_yaw;
		motor_step[m3] = mc.ct_val_thr  -mc.ct_val_yaw;
		motor_step[m4] = mc.ct_val_thr  +mc.ct_val_yaw;
		
	
		for(i=0;i<MOTOR_NUM;i++)
		{	
			motor_step[i] = LIMIT(motor_step[i],IDLING,MAX_THR_SET*10);
		}
		
		motor[m1] = motor_step[m1] +mc.ct_val_rol +mc.ct_val_pit;
		motor[m2] = motor_step[m2] -mc.ct_val_rol +mc.ct_val_pit;
		motor[m3] = motor_step[m3] -mc.ct_val_rol -mc.ct_val_pit;
		motor[m4] = motor_step[m4] +mc.ct_val_rol -mc.ct_val_pit;
	}
	
	for(i=0;i<MOTOR_NUM;i++)
	{
		if(flag.fly_ready)
		{
			if(flag.motor_preparation == 1)
			{
				motor[i] = LIMIT(motor[i],IDLING,1000);
			}
	
		}
		else
		{		
			motor[i] = 0;
		}	
	
	}
	motor_out(motor);
}



