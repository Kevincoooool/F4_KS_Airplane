
#include "Loc_Ctrl.h"
#include "Control.h"
#include "opticalflow.h"
#include "IMU.h"

//位置环控制参数
PID_arg_t loc_arg_2[VEC_XYZ];

//位置环控制数据
PID_val_t loc_val_2[VEC_XYZ];

//位置环PID参数初始化
void Loc_2level_PID_Init(void)
{

	loc_arg_2[X].kp = Param.PID_loc.kp *0.001f;
	loc_arg_2[X].ki = Param.PID_loc.ki *0.001f;
	loc_arg_2[X].kd = Param.PID_loc.kd *0.001f;

	loc_arg_2[Y] = loc_arg_2[X];
}

float exp_v[VEC_XYZ],fb_v[VEC_XYZ];
float exp_[VEC_XYZ],fb_[VEC_XYZ];
u8 ct_loc_hold[VEC_XYZ];

//位置环
void Loc_2level_Ctrl(float dT_s)
{
	u8 out_en,axis;
	
	if(Fixed_Point) out_en = 1;
	else						out_en = 0;
	
	//设置位置控制速度
	fs.alt_ctrl_speed_set[X] = fs.speed_set[X];
	fs.alt_ctrl_speed_set[Y] = fs.speed_set[Y];
	
	//反馈位置
	fb_[X] = mini_flow.out_x_i;
	fb_[Y] = mini_flow.out_y_i;
	
	for(u8 i=0;i<2;i++)
	{
		//XY轴循环控制
		if(i)	axis = X; else axis = Y;
		
		//遥控位置变化阶段
		if(fs.alt_ctrl_speed_set[axis] != 0)
		{
			ct_loc_hold[axis] = 0;//关闭定位
			loc_val_2[axis].out = 0;//输出为零
		}
		//重新回到定位阶段
		else
		{
			if(ct_loc_hold[axis] == 0)
			{
				//如果反馈速度足够小，启动定位
				if(ABS(fb_v[axis])<20)
				{
					ct_loc_hold[axis] = 1;
					
//					if(!locat.err)	exp_[axis] = 0;//如果是视觉定位模块，期望位置一直是0
//					else						
					exp_[axis] = fb_[axis];
				}
				//否则减速
				else
				{
					loc_val_2[axis].out += (-fb_v[axis]*0.05f - loc_val_2[axis].out)*0.5f;//减速
				}
			}
		}

		if(out_en)
		{
			if(ct_loc_hold[axis] == 1)
			{
				Alt_PID_calculate( 	dT_s,     				//周期（单位：秒）
														0,								//前馈值
														exp_[axis],				//期望值（设定值）
														fb_[axis],				//反馈值（）
														&loc_arg_2[axis], //PID参数结构体
														&loc_val_2[axis],	//PID数据结构体
														100,							//误差限幅
														MAX_LOC_SPEED			//积分限幅
													);
			}
		}
		else
		{
			exp_[axis] = fb_[axis];
			loc_val_2[axis].out = 0;
		}
		
		loc_val_2[axis].out  = LIMIT(loc_val_2[axis].out,-MAX_LOC_SPEED,MAX_LOC_SPEED);
	}
}

//////////////////////////////////////////////////////////////////

//位置速度环控制参数
PID_arg_t loc_arg_1[VEC_XYZ];

//位置速度环控制数据
PID_val_t loc_val_1[VEC_XYZ];

//位置速度环PID参数初始化
void Loc_1level_PID_Init(void)
{

	loc_arg_1[X].kp = Param.PID_loc_s.kp *0.001f;
	loc_arg_1[X].ki = Param.PID_loc_s.ki *0.001f;
	loc_arg_1[X].kd = Param.PID_loc_s.kd *0.001f;

	loc_arg_1[Y] = loc_arg_1[X];
}

#define LOC_INTE_LIM 500
#define LOC_CTRL_MAX 15
float ct_loc_out[VEC_XYZ];

//位置速度环
void Loc_1level_Ctrl(float dT_s)
{
	u8 out_en,axis;
	
	if(Fixed_Point) out_en = 1;
	else						out_en = 0;	
	
	//期望速度
	exp_v[X] = fs.alt_ctrl_speed_set[X] + loc_val_2[X].out;
	exp_v[Y] = fs.alt_ctrl_speed_set[Y] + loc_val_2[Y].out;
	
	//反馈速度
	fb_v[X] = mini_flow.fix_x;
	fb_v[Y] = mini_flow.fix_y;
	
	for(u8 i=0;i<2;i++)
	{
		//XY轴循环控制
		if(i)	axis = X; else axis = Y;

		Alt_PID_calculate( 	dT_s,         			//周期（单位：秒）
												0,									//前馈值
												exp_v[axis],				//期望值（设定值）
												fb_v[axis],					//反馈值（）
												&loc_arg_1[axis], 	//PID参数结构体
												&loc_val_1[axis],		//PID数据结构体
												200,								//误差限幅
												LOC_CTRL_MAX*out_en	//积分限幅
											);
		
		loc_val_1[axis].out = LIMIT(loc_val_1[axis].out,-LOC_CTRL_MAX,LOC_CTRL_MAX);
		
		ct_loc_out[axis] = out_en *(loc_val_1[axis].out);
	}
}



#define Fixed_Point_Max 15.0f
u8 Fixed_Point_Mode = 1,Fixed_Point = 0;

//定点控制，控制频率50hz
void Fixed_Point_Ctrl(float x,float y)
{
	//遥控数据归1化
	fs.speed_set_norm[X] = x*0.002f;
	fs.speed_set_norm[Y] = y*0.002f;
	fs.speed_set_norm_lpf[X] += 0.2f *(fs.speed_set_norm[X] - fs.speed_set_norm_lpf[X]);//低通滤波
	fs.speed_set_norm_lpf[Y] += 0.2f *(fs.speed_set_norm[Y] - fs.speed_set_norm_lpf[Y]);//低通滤波
	
//	//非定高模式下不进行定点运算
//	if(!Fixed_Point_Mode || !keep_high_mode)	return;

	switch(start)
	{
		case 0://没有起飞
			Fixed_Point =  0;//不定点
			fs.speed_set[X] = 0;//速度为零
			fs.speed_set[Y] = 0;//速度为零
		break;
			
		case 1://自动起飞阶段
			Fixed_Point =  1;		//开启定点
			fs.speed_set[X] = 0;//速度为零
			fs.speed_set[Y] = 0;//速度为零
		break;
		
		default://飞行过程中
			if(auto_landing)
			{
				Fixed_Point =  1;		//开启定点
				fs.speed_set[X] = 0;//速度为零
				fs.speed_set[Y] = 0;//速度为零
			}
			else
			{
				Fixed_Point =  1;//开启定点
				fs.speed_set[X] = fs.speed_set_norm_lpf[X]*MAX_LOC_SPEED;//设置速度
				fs.speed_set[Y] = fs.speed_set_norm_lpf[Y]*MAX_LOC_SPEED;//设置速度
			}
		break;
	}
	
	Loc_1level_Ctrl(20e-3f);//位置速度环控制
	Loc_2level_Ctrl(20e-3f);//位置环控制
}








