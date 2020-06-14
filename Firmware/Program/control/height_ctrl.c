#include "include.h"
#include "height_ctrl.h"
/*
基础单位：cm

*/
s32 baro_height,baro_height_old;
s32 baro_speed_o,baro_speed;
s32 baro_acc_o,baro_acc;

_height_fusion_st Rol_fus;
_height_fusion_st Pit_fus;

#define MONUM 10
float speed_av_arr[MONUM];
float speed_av;
u16 speed_av_cnt;

float speed_delta;
float speed_delta_lpf;

/*
基础单位：cm
*/
s32 baro_height,baro_height_old,baro_Offset;
s32 baro_speed_o,baro_speed;
u8 baro_start=1;
void Height_Get(float dT)
{

	//气压高度清零
	if( baro_start )
	{
		baro_height = 0;
		baro_height_old = 0;
		baro_Offset = Drv_Spl0601_Read();
		WCZ_Data_Reset();
		baro_start = 0;
	}
	else
	{
		//读取气压计高度
		baro_height = Drv_Spl0601_Read() - baro_Offset;
		baro_speed_o = safe_div(baro_height - baro_height_old,dT,0);
		baro_height_old = baro_height;
	}

	//计算速度
	Moving_Average(speed_av_arr,MONUM ,&speed_av_cnt,baro_speed_o,&speed_av);
	speed_delta = LIMIT(speed_av - baro_speed,-2000*dT,2000*dT);
	LPF_1_(0.5f,dT,speed_delta,speed_delta_lpf);
	baro_speed += speed_delta *LIMIT((ABS(speed_delta_lpf)/(2000*dT)),0,1);
}

u8 start=0,auto_landing=0,auto_take_off=0;
float high_begin,high_start;
//float Now_High,Target_Speed;
pid_param h_axis;//高度控制参数

//高度控制，控制频率100hz
void High_Ctrl(float thr,float high,float speed)
{
	static u8 _cnt;

	//遥控数据归1
	fs.speed_set_norm[Z] = thr*0.002f;
	fs.speed_set_norm_lpf[Z] += 0.2f *(fs.speed_set_norm[Z] - fs.speed_set_norm_lpf[Z]);//低通滤波

	//标记为未起飞状态
	if(flag.fly_ready == 0)
	{
		start = 0;
		
		high_begin = wcz_hei_fus.out; //记录起飞前高度
		return;
	}
	
	if(start && ( ct_val_thr<200) )//降落判断
	{
		_cnt++;
		if(_cnt>=100)
		{
			_cnt = 0;         //清除标记
			if(CH_N[CH_THR]<-400 || auto_landing || flag.NS==0)
			{
				start = 0;        //标记为未起飞状态
				auto_landing = 0;	//取消自动降落
				//Throw_Fly = 0;		//取消抛飞
				fly_ready = 0;		//上锁
				flag.fly_ready = 0;				//上锁
			}
		}
	}
	else _cnt = 0;
	
	switch(start)
	{
		//没有起飞
		case 0:
		{
			if(thr>=0)
			{
				if(auto_take_off) 	start = 1;//自动起飞
				else				start = 2;//手动起飞
			}
			else
			{
				fs.speed_set[Z] = 0 ;
			}
		}
		break;
		
		//自动起飞阶段
		case 1:
		{
			//如果起飞高度大于目标高度，则切换到正常飞行状态
			if(wcz_hei_fus.out >= high_start)	start = 2;
			else	fs.speed_set[Z] = 200 +  fs.speed_set_norm_lpf[Z]*MAX_D_SPEED;//自动起飞速度
			fs.speed_set[Z] = LIMIT(fs.speed_set[Z],0,300);//自动起飞速度限幅
		
			//自动起飞过程中拉低油门，可以取消自动起飞
			if(fs.speed_set_norm_lpf[Z] < -0.8) start = 2;
		}
		break;
		
		//飞行中
		default:
		{
			//自动降落
			if(auto_landing || flag.NS==0)
			{
				fs.speed_set[Z] = -50 + fs.speed_set_norm_lpf[Z]*MAX_D_SPEED;
				fs.speed_set[Z] = LIMIT(fs.speed_set[Z],-MAX_D_SPEED,MAX_U_SPEED);
			}
			//遥控控制高度
			else
			{
				if(fs.speed_set_norm_lpf[Z]>=0)	
					fs.speed_set[Z] = fs.speed_set_norm_lpf[Z]*MAX_U_SPEED;//设置速度
				else
					fs.speed_set[Z] = fs.speed_set_norm_lpf[Z]*MAX_D_SPEED;//设置速度
			}
		}
		break;
	}
	
	Alt_1level_Ctrl(10e-3f);/*高度速度环控制*/
	Alt_2level_Ctrl(10e-3f);/*高度环控制*/
}

