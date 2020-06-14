
#include "Control.h"
#include "IMU.h"
#include "include.h"

//角度环参数
PID_arg_t arg_2_rol ;
PID_arg_t arg_2_pit ;
PID_arg_t arg_2_yaw ;

//角速度环参数
PID_arg_t arg_1_rol ;
PID_arg_t arg_1_pit ;
PID_arg_t arg_1_yaw ;

PID_val_t val_2_rol;
PID_val_t val_2_pit;
PID_val_t val_2_yaw;

PID_val_t val_1_rol;
PID_val_t val_1_pit;
PID_val_t val_1_yaw;

void CTRL_2_PID_Init()//角度环
{
	arg_2_rol.kp = Param.PID_rol.kp *0.001f;
	arg_2_rol.ki = Param.PID_rol.ki *0.001f;
	arg_2_rol.kd = Param.PID_rol.kd *0.001f;
	arg_2_rol.k_pre_d = 0.0f;
	arg_2_rol.k_ff = 0.0f;
	
	arg_2_pit.kp = Param.PID_pit.kp *0.001f;
	arg_2_pit.ki = Param.PID_pit.ki *0.001f;
	arg_2_pit.kd = Param.PID_pit.kd *0.001f;
	arg_2_pit.k_pre_d = 0.0f;
	arg_2_pit.k_ff = 0.0f;

	arg_2_yaw.kp = Param.PID_yaw.kp *0.001f;
	arg_2_yaw.ki = Param.PID_yaw.ki *0.001f;
	arg_2_yaw.kd = Param.PID_yaw.kd *0.001f;
	arg_2_yaw.k_pre_d = 0.0f;
	arg_2_yaw.k_ff = 0.0f;		
}
void CTRL_1_PID_Init()//角速度环
{
	arg_1_rol.kp = Param.PID_rol_s.kp *0.001f;
	arg_1_rol.ki = Param.PID_rol_s.ki *0.001f;
	arg_1_rol.kd = 0.0f   ;
	arg_1_rol.k_pre_d = Param.PID_rol_s.kd *0.001f;
	arg_1_rol.k_ff = 0.2f;
	
	arg_1_pit.kp = Param.PID_pit_s.kp *0.001f;
	arg_1_pit.ki = Param.PID_pit_s.ki *0.001f;
	arg_1_pit.kd = 0.0f   ;
	arg_1_pit.k_pre_d = Param.PID_pit_s.kd *0.001f; 
	arg_1_pit.k_ff = 0.2f;

	arg_1_yaw.kp = Param.PID_yaw_s.kp *0.001f;
	arg_1_yaw.ki = Param.PID_yaw_s.ki *0.001f;
	arg_1_yaw.kd = 0.0f   ;
	arg_1_yaw.k_pre_d = Param.PID_yaw_s.kd *0.001f;
	arg_1_yaw.k_ff = 0.5f;	
}

void pid_init(void)
{
	CTRL_2_PID_Init();
	CTRL_1_PID_Init();
	Alt_1level_PID_Init();
	Alt_2level_PID_Init();
	Loc_1level_PID_Init();
	Loc_2level_PID_Init();
}

void CTRL_2(float dT,float weight,_copter_ctrl_st *data) //角度环
{
	PID_Calculate(  dT,            			//周期
					0,						//前馈
					data->exp_rol,			//期望值（设定值）
					data->fb_rol,			//反馈值
					&arg_2_rol, 			//PID参数结构体
					&val_2_rol,				//PID数据结构体
					CTRL_2_INTER_LIMIT *weight,			//integral limit，积分限幅
					&(data->out_rol)  );	//输出
	
	PID_Calculate(  dT,            			//周期
					0,						//前馈
					data->exp_pit,			//期望值（设定值）
					data->fb_pit,			//反馈值
					&arg_2_pit, 			//PID参数结构体
					&val_2_pit,				//PID数据结构体
					CTRL_2_INTER_LIMIT *weight,			//integral limit，积分限幅
					&(data->out_pit)  );	//输出
	
 	PID_Calculate(  dT,            			//周期
 					0,						//前馈
 					data->exp_yaw,			//期望值（设定值）
 					data->fb_yaw,			//反馈值
 					&arg_2_yaw, 			//PID参数结构体
 					&val_2_yaw,				//PID数据结构体
 					CTRL_2_INTER_LIMIT *weight,						//integral limit，积分限幅
 					&(data->out_yaw)  );	//输出

}

void CTRL_1(float dT,float weight,_copter_ctrl_st *data) //角速率环
{
	
	PID_Calculate(  dT,            			//周期
					data->exp_rol,			//前馈
					data->exp_rol,			//期望值（设定值）
					data->fb_rol,			//反馈值
					&arg_1_rol, 			//PID参数结构体
					&val_1_rol,				//PID数据结构体
					CTRL_1_INTER_LIMIT *weight,			//integral limit，积分限幅
					&(data->out_rol)  );			//输出
	
	PID_Calculate(  dT,            			//周期
					data->exp_pit,			//前馈
					data->exp_pit,			//期望值（设定值）
					data->fb_pit,			//反馈值
					&arg_1_pit, 			//PID参数结构体
					&val_1_pit,				//PID数据结构体
					CTRL_1_INTER_LIMIT *weight,			//integral limit，积分限幅
					&(data->out_pit)  );			//输出
	
	PID_Calculate(  dT,            			//周期
					data->exp_yaw,			//前馈
					data->exp_yaw,			//期望值（设定值）
					data->fb_yaw,			//反馈值
					&arg_1_yaw, 			//PID参数结构体
					&val_1_yaw,				//PID数据结构体
					2 *CTRL_1_INTER_LIMIT *weight,		//integral limit，积分限幅
					&(data->out_yaw) );			//输出

}
/*=====================================================================================================================
						CH_N  1横滚，2俯仰，3油门，4航向 范围：+-500
=====================================================================================================================*/
_copter_ctrl_st ctrl_1; 
_copter_ctrl_st ctrl_2; 

volatile u32 ctrl_H_time;
volatile u32 ctrl_2_time; 
volatile u32 ctrl_1_time; 

float out_rol_curve,out_pit_curve,out_yaw_curve,out_thr_curve;
float est_rol,est_pit,est_yaw;

u8 keep_high_mode = 1;//开机默认是定高模式

#define PI 3.14f
u8 No_Head_Mode = 0;

//3D翻滚控制
#define SPEED_MIN 100
u8 roll_en=0,roll_dir=0;

//3D翻滚控制
void Roll_3D(void)
{
	switch(roll_en)
	{
		case 1://第一步：判断翻滚方向
			if	(CH_N[ROL] < -200) roll_dir = 1,roll_en = 2;//左翻
			else if(CH_N[ROL] >  200) roll_dir = 2,roll_en = 2;//右翻
			else if(CH_N[PIT] < -200) roll_dir = 3,roll_en = 2;//后翻
			else if(CH_N[PIT] >  200) roll_dir = 4,roll_en = 2;//前翻
		break;
		
		case 2://第二步：加速上升
			if(wcz_spe_fus.out >= 150)
			{
				//初始化积分角度
				imu_data.inter_rol = imu_data.rol;
				imu_data.inter_pit = imu_data.pit;
				
				roll_en = 3;
			}
		break;
		
		case 3://第三步：翻滚阶段
			if(ABS(imu_data.inter_rol)>=225.0f || ABS(imu_data.inter_pit)>=225.0f)
			{
				roll_en = 4;//翻滚半周后进入自稳减速阶段
			}
		break;
			
		case 4://第四步：自稳减速阶段
			if(ABS(sensor.Gyro_deg[X])<=50.0f && ABS(sensor.Gyro_deg[Y])<=50.0f)
			{
				roll_en = 0;//恢复正常状态
				//arg_1_pit.kp = arg_1_rol.kp = kp_temp;
			}
		break;
		
		default://非空翻模式下姿态角大于80度直角锁定飞机
//			if(ABS(imu_data.rol)>=80 || ABS(imu_data.pit)>=80) 
//			{
//				if(!Throw_Fly_Mode) fly_ready = 0;
//			}
		break;
	}
}

u8 cnt = 0,yaw_lock = 0,Send_Check = 0;
//无头模式控制
float Rc_Angle,Rc_Abs;

void No_Head(float *x,float *y,float angle)
{
	angle    = angle / 0.0174f;				 //转向角度转弧度
	Rc_Angle = asin( (*x)/Rc_Abs );          //遥杆角度
	
	//角度转换到-PI~0~PI
	if((*x)<0 && (*y)<0)
	{
		Rc_Angle = -PI - Rc_Angle;
	}
	else if((*x)>0 && (*y)<0)
	{
		Rc_Angle = PI - Rc_Angle;
	}
	else if((*x)==0 && (*y)<0)
	{
		Rc_Angle = PI;
	}
	//无头控制输出
	*x = Rc_Abs*sin(Rc_Angle+angle);
	*y = Rc_Abs*cos(Rc_Angle+angle);
}

_height_ctrl_st h_c;
void CTRL_Duty(float ch1,float ch2,float ch3,float ch4)//2ms
{
	static float Turn_Angle,Star_Angle;
	static u8 ctrl_2_cnt;
	float dT;
	
	//判断是否输出
	gf.out_weight = (start==0)?0:1;
	
	if(start)
	{
		if(gf.out_weight_slow<1)
		{
			gf.out_weight_slow += 2.0f *0.003f;
		}
		else
		{
			gf.out_weight_slow = 1;
		}
	}
	else
	{
		gf.out_weight_slow = 0 ;
	}
	
/*=====================================================================================================================
						角度环
=====================================================================================================================*/	
	ctrl_2_cnt++;
	ctrl_2_cnt%=2;
	
	if(ctrl_2_cnt==0)//6ms
	{
		//周期时间
		dT = 0.000001f *(GetSysTime_us() - ctrl_2_time);
		ctrl_2_time = GetSysTime_us();
		
		Rc_Abs = sqrt(ch1*ch1 + ch2*ch2);  //遥杆绝对值
		if(No_Head_Mode)	No_Head(&ch1,&ch2,Turn_Angle+Star_Angle);
		
		//反馈角度
		ctrl_2.fb_rol = -imu_data.rol;
		ctrl_2.fb_pit = -imu_data.pit;

		//期望角度
		ctrl_2.exp_rol = (0.002f *(-ch1))  *(MAX_ROL_ANGLE )- ct_loc_out[X];
		ctrl_2.exp_pit = (0.002f *(-ch2))  *(MAX_PIT_ANGLE) -ct_loc_out[Y];
		ctrl_2.exp_rol = LIMIT(ctrl_2.exp_rol,-MAX_ROL_ANGLE,MAX_ROL_ANGLE);
		ctrl_2.exp_pit = LIMIT(ctrl_2.exp_pit,-MAX_PIT_ANGLE,MAX_PIT_ANGLE);
		
		//YAW反馈角度
		if(start==0)
		{
			//初始化起飞航向
			Star_Angle = imu_data.yaw;
			Turn_Angle = -imu_data.yaw;
		}
		else
		{
			if(!yaw_lock)
			{
				if(ctrl_2.fb_yaw > -MAX_YAW_ANGLE && ctrl_2.fb_yaw < MAX_YAW_ANGLE)	
				{
					Turn_Angle += my_deadzone(((0.002f *(+ch4))),0,0.1f);
				}
				if		 (Turn_Angle > 360)	Turn_Angle =-360;
				else if(Turn_Angle <-360)	Turn_Angle =360;
			}
		}
		ctrl_2.fb_yaw = imu_data.yaw + Turn_Angle;
		
		//反馈角度限制在+-180
		if		 (ctrl_2.fb_yaw >= 180)	ctrl_2.fb_yaw -= 360;
		else if(ctrl_2.fb_yaw <=-180)	ctrl_2.fb_yaw += 360;
		
		//YAW期望角度
		ctrl_2.exp_yaw = 0;
		
		//角度环控制
		CTRL_2(dT,gf.out_weight_slow,&ctrl_2);
		
		//输出限幅
		ctrl_2.out_rol = LIMIT(ctrl_2.out_rol,-MAX_ROL_ANGLE,MAX_ROL_ANGLE);
		ctrl_2.out_pit = LIMIT(ctrl_2.out_pit,-MAX_PIT_ANGLE,MAX_PIT_ANGLE);
		ctrl_2.out_yaw = LIMIT(ctrl_2.out_yaw,-MAX_YAW_ANGLE,MAX_YAW_ANGLE);
	}
/*=====================================================================================================================
						角速度环
=====================================================================================================================*/	
	//周期
	dT = 0.000001f *(GetSysTime_us() - ctrl_1_time);
	ctrl_1_time = GetSysTime_us();
	
	//期望角速度
	ctrl_1.exp_rol = 8 *ctrl_2.out_rol *(0.55f + 0.45f *(ABS(ctrl_2.out_rol)/(MAX_ROL_ANGLE *arg_2_rol.kp)));//误差二次曲线
	ctrl_1.exp_pit = 8 *ctrl_2.out_pit *(0.55f + 0.45f *(ABS(ctrl_2.out_pit)/(MAX_PIT_ANGLE *arg_2_pit.kp)));//误差二次曲线
	ctrl_1.exp_yaw = 8 *ctrl_2.out_yaw *(0.55f + 0.45f *(ABS(ctrl_2.out_yaw)/(MAX_YAW_ANGLE *arg_2_yaw.kp)));//误差二次曲线
	
	ctrl_1.exp_rol = LIMIT(ctrl_1.exp_rol,-MAX_ROL_SPEED,MAX_ROL_SPEED);
	ctrl_1.exp_pit = LIMIT(ctrl_1.exp_pit,-MAX_PIT_SPEED,MAX_PIT_SPEED);
	ctrl_1.exp_yaw = LIMIT(ctrl_1.exp_yaw,-MAX_YAW_SPEED,MAX_YAW_SPEED);
	
	//反馈角速度
	ctrl_1.fb_rol = -sensor.Gyro_deg[X];
	ctrl_1.fb_pit =  sensor.Gyro_deg[Y];
	ctrl_1.fb_yaw = -sensor.Gyro_deg[Z];
	
	//角速度环控制
	CTRL_1(dT,gf.out_weight_slow,&ctrl_1);
	
	//输出限幅
	ctrl_1.out_rol = gf.out_weight *LIMIT(ctrl_1.out_rol,-1000,1000);
	ctrl_1.out_pit = gf.out_weight *LIMIT(ctrl_1.out_pit,-1000,1000);
	ctrl_1.out_yaw = gf.out_weight *LIMIT(ctrl_1.out_yaw,-1000,1000);
	
/*=====================================================================================================================
						控制量输出
=====================================================================================================================*/	
	float angle_z;
	
	//求偏离Z轴的角度,进行油门补偿
	angle_z = sqrt(imu_data.rol*imu_data.rol + imu_data.pit*imu_data.pit);
	angle_z = LIMIT(angle_z,0,30) * 0.0174f;	//限幅并转弧度
	
	//油门补偿并且限幅
	out_thr_curve = mc.ct_val_thr / cos(angle_z);
	mc.ct_val_thr=out_thr_curve = LIMIT(out_thr_curve,0,1000-balance_max);
	
	//自稳量输出
	mc.ct_val_rol= out_rol_curve = ctrl_1.out_rol;
	mc.ct_val_pit= out_pit_curve = ctrl_1.out_pit;
	mc.ct_val_yaw= out_yaw_curve = ctrl_1.out_yaw;

//	motor_ctrl((s16)out_rol_curve,(s16)out_pit_curve,(s16)out_yaw_curve,(s16)out_thr_curve);
}


#define Fixed_Point_Max 15.0f


#define SPEED_HIGH 130
#define SPEED_LOW  80

u8 Throw_Fly_Mode=0,Throw_Fly=0;
//抛飞检测
void Throw_Fly_Check(void)
{
	if(flag.fly_ready || !Throw_Fly_Mode) return;
	
	//检测上升
	if(wcz_spe_fus.out > SPEED_HIGH)
	{
	//	ready = 2;
		flag.fly_ready = 1;//解锁
		auto_take_off = 0;//手动起飞
		Throw_Fly = 1;//抛飞
	}
}

#define STATIC_MAX_GYRO 10

//静止检测，20Hz
void Static_Check(void)
{
	static u8 cnt;
	
	if(flag.fly_ready==1) return;
	
	if( ABS(sensor.Gyro[X]) <= STATIC_MAX_GYRO &&
			ABS(sensor.Gyro[Y]) <= STATIC_MAX_GYRO &&
			ABS(sensor.Gyro[Z]) <= STATIC_MAX_GYRO 
		)		cnt ++;
	else 	cnt = 0;
	
	//静止3s后高度清零
	if(cnt>=100) baro_start = 1;
}
/*
四轴：
      机头
   m2     m1
     \   /
      \ /
      / \
     /   \
   m3     m4
      屁股

六轴：
			机头
     m2   m1
      \  /
 m3----------m6
			/  \
     m4   m5
*/
#define ROLL_THR_MAX 750
#define ROLL_THR_MIN 50
#define ROLL_KP      1.0f


s16 balance[MOTOR_NUM],balance_max;

s16 roll_balance,roll_thr;
void motor_ctrl(s16 ct_val_rol,s16 ct_val_pit,s16 ct_val_yaw,s16 ct_val_thr)
{
	u8 i;

	//平衡输出
	balance[m1] = +ct_val_rol +ct_val_pit -ct_val_yaw;
	balance[m2] = -ct_val_rol +ct_val_pit +ct_val_yaw;
	balance[m3] = -ct_val_rol -ct_val_pit -ct_val_yaw;
	balance[m4] = +ct_val_rol -ct_val_pit +ct_val_yaw;
	
	//求平衡输出最大值
	balance_max = balance[m1];
	for(i=m3;i<MOTOR_NUM;i++)
	{
		if( balance_max < ABS(balance[i]) ) balance_max = balance[i];
	}
	
	balance_max = LIMIT(balance_max,0,500);
	
	//电机最终输出
	motor[m1] = ct_val_thr + balance[m1];
	motor[m2] = ct_val_thr + balance[m2];
	motor[m3] = ct_val_thr + balance[m3];
	motor[m4] = ct_val_thr + balance[m4];
	
	for(i=0;i<MOTOR_NUM;i++)
	{
		if(flag.fly_ready)
		{
			motor[i] = LIMIT(motor[i],0,1000);
		}
//		else if(ready)
//		{
//			motor[i] = (CH_N[CH_THR]+500)*0.4;
//		}
		else
		{
			motor[i] = 0;
		}
	}
	motor_out(motor);
	
}

u8 Run_state = 0;
#define Stay_on_the_ground 0
#define Taking_off 1
#define Normal_fly 2
#define Traking_face 3
#define landing 4
void state_transfer(void)
{
	switch (Run_state)
	{
		case Stay_on_the_ground:  
			
			break;
		case Taking_off:  
			
			break;
		case Normal_fly:  
			
			break;
		case Traking_face:  
			
			break;
		case landing:  
			
			break;
		default:  break;
	}
			
}


/*PID参数初始化*/
void All_PID_Init(void)
{
	/*姿态控制，角速度PID初始化*/
	Att_1level_PID_Init();
	
	/*姿态控制，角度PID初始化*/
	Att_2level_PID_Init();
	
	/*高度控制，高度速度PID初始化*/
	Alt_1level_PID_Init();	
	
	/*高度控制，高度PID初始化*/
	Alt_2level_PID_Init();
	
	/*位置速度控制PID初始化*/
	Loc_2level_PID_Init();
	Loc_1level_PID_Init();
}
/*控制参数改变任务*/
void ctrl_parameter_change_task()
{

	
	if(0)
	{
		Set_Att_2level_Ki(0);
		
	}
	else
	{
		if(flag.auto_take_off_land ==AUTO_TAKE_OFF)
		{

			Set_Att_1level_Ki(2);
		}
		else
		{

			Set_Att_1level_Ki(1);
		}
		
		Set_Att_2level_Ki(1);
	}
}



static u16 one_key_taof_start;
/*一键起飞任务（主要功能为延迟）*/
void one_key_take_off_task(u16 dt_ms)
{
	if(one_key_taof_start != 0)
	{
		one_key_taof_start += dt_ms;
		
		
		if(one_key_taof_start > 1400 && flag.motor_preparation == 1)
		{
			one_key_taof_start = 0;
				if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL)
				{
					flag.auto_take_off_land = AUTO_TAKE_OFF;
					//解锁、起飞

					flag.taking_off = 1;
				}
			
		}
	}
	

}
/*一键起飞*/
void one_key_take_off()
{
	if(flag.unlock_en)
	{	
		one_key_taof_start = 1;
		flag.fly_ready = 1;
	}
}
/*降落检测*/
void one_key_land()
{
	flag.auto_take_off_land = AUTO_LAND;
}

//////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////
_flight_state_st fs;

s16 flying_cnt,landing_cnt;

extern s32 ref_height_get;

float stop_baro_hpf;

/*降落检测*/

static s16 ld_delay_cnt ;
void land_discriminat(s16 dT_ms)
{
//	static s16 acc_delta,acc_old;
	
//	acc_delta = imu_data.w_acc[Z]- acc_old;
//	acc_old = imu_data.w_acc[Z];
	
	/*油门归一值小于0.1并且垂直方向加速度小于阈值  或者启动自动降落*/
	if((fs.speed_set_h_norm[Z] < 0.1f && imu_data.w_acc[Z]<200) || flag.auto_take_off_land == AUTO_LAND)
	{
		if(ld_delay_cnt>0)
		{
			ld_delay_cnt -= dT_ms;
		}
	}
	else
	{
		ld_delay_cnt = 200;
	}
	
	/*意义是：如果向上推了油门，就需要等垂直方向加速度小于200cm/s2 保持200ms才开始检测*/	
	if(ld_delay_cnt <= 0 && (flag.thr_low || flag.auto_take_off_land == AUTO_LAND) )
	{
		/*油门最终输出量小于250并且没有在手动解锁上锁过程中，持续1秒，认为着陆，然后上锁*/
		if(mc.ct_val_thr<250 && flag.fly_ready == 1 && flag.locking != 2)//ABS(wz_spe_f1.out <20 ) //还应当 与上速度条件，速度小于正20厘米每秒。
		{
			if(landing_cnt<1500)
			{
				landing_cnt += dT_ms;
			}
			else
			{

				flying_cnt = 0;
				flag.taking_off = 0;
				///////
					landing_cnt =0;	
					flag.fly_ready =0;				

				flag.flying = 0;

			}
		}
		else
		{
			landing_cnt = 0;
		}
			
		
	}
	else
	{
		landing_cnt  = 0;
	}

}


/*飞行状态任务*/
float vel_z_tmp[2];
void Flight_State_Task(u8 dT_ms,s16 *CH_N)
{
	s16 thr_deadzone;
	static float max_speed_lim;
	/*设置油门摇杆量*/
	thr_deadzone = (flag.wifi_ch_en != 0) ? 0 : 50;
	fs.speed_set_h_norm[Z] = my_deadzone(CH_N[CH_THR],0,thr_deadzone) *0.002f;
	fs.speed_set_h_norm_lpf[Z] += 0.5f *(fs.speed_set_h_norm[Z] - fs.speed_set_h_norm_lpf[Z]);
	
	/*推油门起飞*/
	if(flag.fly_ready)
	{
		if(fs.speed_set_h_norm[Z]>0.01f && flag.motor_preparation == 1) // 0-1
		{
			flag.taking_off = 1;
		}
	}
	
	if(flag.taking_off)
	{
			
		if(flying_cnt<1000)//800ms
		{
			flying_cnt += dT_ms;
		}
		else
		{
			/*起飞后1秒，认为已经在飞行*/
			flag.flying = 1;  
		}
		
		if(fs.speed_set_h_norm[Z]>0)
		{
			/*设置上升速度*/
			vel_z_tmp[0] = (fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_UP);
		}
		else
		{
			/*设置下降速度*/
			vel_z_tmp[0] = (fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_DW);
		}

		//
		vel_z_tmp[1] = vel_z_tmp[0];// + program_ctrl.vel_cmps_h[Z];
		//
//		vel_z_tmp[1] = LIMIT(vel_z_tmp[1],fc_stv.vel_limit_z_n,fc_stv.vel_limit_z_p);
		//
		fs.speed_set_h[Z] += LIMIT((vel_z_tmp[1] - fs.speed_set_h[Z]),-0.8f,0.8f);//限制增量幅度
	}
	else
	{
		fs.speed_set_h[Z] = 0 ;
	}
	float speed_set_tmp[2];
	/*速度设定量，正负参考ANO坐标参考方向*/
	fs.speed_set_h_norm[X] = (my_deadzone(+CH_N[CH_PIT],0,50) *0.002f);
	fs.speed_set_h_norm[Y] = (my_deadzone(-CH_N[CH_ROL],0,50) *0.002f);
		
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[X],fs.speed_set_h_norm_lpf[X]);
	LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[Y],fs.speed_set_h_norm_lpf[Y]);
	
	max_speed_lim = MAX_SPEED;
	
//	if(switchs.of_flow_on)
//	{
//		max_speed_lim = 1.5f *wcz_hei_fus.out;
//		max_speed_lim = LIMIT(max_speed_lim,50,150);
//	}	
	
	speed_set_tmp[X] = max_speed_lim *fs.speed_set_h_norm_lpf[X];
	speed_set_tmp[Y] = max_speed_lim *fs.speed_set_h_norm_lpf[Y];
	
	length_limit(&speed_set_tmp[X],&speed_set_tmp[Y],max_speed_lim,fs.speed_set_h_cms);

	fs.speed_set_h[X] = fs.speed_set_h_cms[X];
	fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];	
	
	/*调用检测着陆的函数*/
	land_discriminat(dT_ms);
	
	/*倾斜过大上锁*/

	if(imu_data.z_vec[Z]<0.25f)//75度  ////////////////////////////////////////*************************** 倾斜过大上锁，慎用。
	{

		flag.fly_ready = 0;
	}
	
		//////////////////////////////////////////////////////////
	/*校准中，复位重力方向*/
	if(sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 ||sensor.acc_z_auto_CALIBRATE)
	{
		imu_state.G_reset = 1;
	}
	
	/*复位重力方向时，认为传感器失效*/
	if(imu_state.G_reset == 1)
	{
		flag.sensor_imu_ok = 0;
		//LED_STA.rst_imu = 1;
		//NavigationReset();
		WCZ_Data_Reset(); //复位高度数据融合
	}
	else if(imu_state.G_reset == 0)
	{	
		if(flag.sensor_imu_ok == 0)
		{
			flag.sensor_imu_ok = 1;
			//LED_STA.rst_imu = 0;
			//KS_DT_SendString("IMU OK!");
		}
	}
	
	/*飞行状态复位*/
	if(flag.fly_ready == 0)
	{
		flag.flying = 0;
		landing_cnt = 0;
		flag.taking_off = 0;
		flying_cnt = 0;
		
		flag.rc_loss_back_home = 0;
		
		//复位融合
		if(flag.taking_off == 0)
		{
//			wxyz_fusion_reset();
		}
	}
	

}
