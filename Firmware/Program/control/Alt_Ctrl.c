#include "include.h"

#include "Alt_Ctrl.h"

//flight_state_st fs;

PID_arg_t alt_arg_high;
PID_val_t alt_val_high;

/*高度环PID参数初始化*/
void Alt_2level_PID_Init()
{
	alt_arg_high.kp = Param.PID_high.kp*0.001f;
	alt_arg_high.ki = Param.PID_high.ki*0.001f;
	alt_arg_high.kd = Param.PID_high.kd*0.001f;
}

float exp_hv,fb_hv;
float exp_h,fb_h;
u8 ct_alt_hold;

//高度环
void Alt_2level_Ctrl(float dT_s)
{
	u8 out_en;
	
	if(start) out_en = 1;
	else			out_en = 0;
	
	//设置高度控制速度
	fs.alt_ctrl_speed_set[Z] = fs.speed_set[Z];

	//反馈高度
	fb_h = wcz_hei_fus.out;
	
	//遥控高度变化阶段
	if(fs.alt_ctrl_speed_set[Z] != 0)
	{
		ct_alt_hold = 0;//关闭定高
		alt_val_high.out = 0;//输出为零
	}
	else
	{
		//重新回到定高阶段
		if(ct_alt_hold==0)
		{
			//如果反馈速度接近0，启动定高
			if(ABS(fb_hv)<=10)
			{
				ct_alt_hold = 1;
				exp_h = fb_h;
			}
			//否则减速
			else
			{
				alt_val_high.out += (-fb_hv*alt_arg_high_s.kp*0.5f - alt_val_high.out)*0.5f;//减速
			}
		}
	}
	
	//非定高模式下期望高度一直等于当前高度
	if(!keep_high_mode) exp_h = fb_h;
	
	if(out_en)
	{
		if(ct_alt_hold == 1)
		{
			Alt_PID_calculate( 	dT_s,     	//周期（单位：秒）
													0,					//前馈值
													exp_h,			//期望值（设定值）
													fb_h,				//反馈值（）
													&alt_arg_high, //PID参数结构体
													&alt_val_high,	//PID数据结构体
													100,				//积分误差限幅
													100					//积分限幅									
												);
		}
	}
	else
	{
		exp_h = fb_h;
		alt_val_high.out = 0;
	}
	
	alt_val_high.out = LIMIT(alt_val_high.out,-200,200);
}

PID_arg_t alt_arg_high_s;
PID_val_t alt_val_high_s;

/*高度速度环PID参数初始化*/
void Alt_1level_PID_Init()
{
	alt_arg_high_s.kp = Param.PID_high_s.kp*0.001f;
	alt_arg_high_s.ki = Param.PID_high_s.ki*0.001f;
	alt_arg_high_s.kd = Param.PID_high_s.kd*0.001f;
}


float err_i_comp;
float ct_val_thr;

//高度速度环
void Alt_1level_Ctrl(float dT_s)
{
	u8 out_en;
	
	if(start) out_en = 1;
	else			out_en = 0;	
	
	//期望速度
	exp_hv = fs.alt_ctrl_speed_set[Z] + alt_val_high.out;
	
	//反馈速度
	fb_hv = wcz_spe_fus.out;

	Alt_PID_calculate( 	dT_s,         //周期（单位：秒）
											0,						//前馈值
											exp_hv,				//期望值（设定值）
											fb_hv,				//反馈值（）
											&alt_arg_high_s, 	//PID参数结构体
											&alt_val_high_s,		//PID数据结构体
											100,					//积分误差限幅
											(MAX_THR - err_i_comp) * out_en//积分限幅									
										);
	
	//设置油门起调量，越大起飞速度越快。
//	if(Throw_Fly) THR_START =300;
//	else					THR_START =200;
	
	if(out_en)
	{
		err_i_comp = 200;
	}
	else
	{
		err_i_comp = 0;
	}
	
	mc.ct_val_thr=ct_val_thr = out_en *(alt_val_high_s.out + err_i_comp );
}

