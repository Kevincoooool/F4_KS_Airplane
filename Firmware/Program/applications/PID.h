#ifndef __KS_PID_H
#define __KS_PID_H

#include "stm32f4xx.h"

/*=====================================================================================================================
						   PID控制器设定参数
=====================================================================================================================*/
#define INNER_INTEGRAL 1.0f 		//内环积分系数
#define INTEGRAL_LIMIT_EN 1		//开启积分限幅
/*=====================================================================================================================
						
=====================================================================================================================*/

typedef struct
{
	u8 fb_d_mode;
	float kp;			 //比例系数
	float ki;			 //积分系数
	float kd_ex;		 	 //微分系数
	float kd_fb; //previous_d 微分先行
//	float inc_hz;  //不完全微分低通系数
//	float k_inc_d_norm; //Incomplete 不完全微分 归一（0,1）
	float k_ff;		 //前馈 

}_PID_arg_st;


typedef struct
{
	float kp;			 //比例系数
	float ki;			 //积分系数
	float kd;		 	 //微分系数
	float k_pre_d; //previous_d 微分先行
	float k_ff;		 //前馈 
}__attribute__((packed)) PID_arg_t;
typedef struct
{
	float err;
	float err_old;
	float feedback_old;
	float feedback_d;
	
	float exp_old;
	float fb_d;
	float fb_d_ex;
	float exp_d;
	
	float err_d;
	
	float err_i;
	float ff;
	float pre_d;
	
	float out;
}__attribute__((packed)) PID_val_t;
typedef struct
{
	float err;
	float exp_old;
	float feedback_old;
	
	float fb_d;
	float fb_d_ex;
	float exp_d;
//	float err_d_lpf;
	float err_i;
	float ff;
	float pre_d;

	float out;
}_PID_val_st;


float PID_calculate( float T,            //周期
										float in_ff,				//前馈
										float expect,				//期望值（设定值）
										float feedback,			//反馈值
										_PID_arg_st *pid_arg, //PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_d_lim,
										float inte_lim			//integration limit，积分限幅
										   );			//输出

void PID_Calculate( float T,  //周期
					float in_ff,				//前馈
					float expect,				//期望值（设定值）
					float feedback,			//反馈值
					PID_arg_t *pid_arg, //PID参数结构体
					PID_val_t *pid_val,	//PID数据结构体
					float inte_lim,			//积分限幅
					float *out  );			//输出
					
float Alt_PID_calculate( float dT_s,    //周期（单位：秒）
										float in_ff,				//前馈值
										float expect,				//期望值
										float feedback,			//反馈值
										PID_arg_t *pid_arg, //PID参数结构体
										PID_val_t *pid_val,	//PID数据结构体
										float inte_d_lim,		//积分误差限幅
										float inte_lim			//积分限幅									
										 );

typedef struct
{
	float exp;	//期望值
	float fb; 	//反馈值
	float speed;//速度值
	float err;	//误差值
	float inte;	//积分值
	float out;	//输出值
	
	float kp;		//比例系数
	float ki;		//积分系数
	float kd;		//微分系数
	
}__attribute__((packed)) pid_param; 

void Pid_Contrl(float T, pid_param *Pid_Param, s16 inte_lim);

#endif

