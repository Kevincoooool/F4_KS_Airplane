/******************** (C) COPYRIGHT 2016 ANO Tech ***************************
 * 作者		 ：匿名科创
 * 文件名  ：KS_PID.c
 * 描述    ：PID函数
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
*****************************************************************************/
#include "PID.h"
#include "filter.h"

float PID_calculate( float dT_s,            //周期（单位：秒）
										float in_ff,				//前馈值
										float expect,				//期望值（设定值）
										float feedback,			//反馈值（）
										_PID_arg_st *pid_arg, //PID参数结构体
										_PID_val_st *pid_val,	//PID数据结构体
										float inte_d_lim,//积分误差限幅
										float inte_lim			//integration limit，积分限幅									
										 )	
{
	float differential,hz;
	hz = safe_div(1.0f,dT_s,0);
	
//	pid_arg->k_inc_d_norm = LIMIT(pid_arg->k_inc_d_norm,0,1);
	
	pid_val->err = (expect - feedback);
	
	pid_val->exp_d = (expect - pid_val->exp_old) *hz;
	
	if(pid_arg->fb_d_mode == 0)
	{
		pid_val->fb_d = (feedback - pid_val->feedback_old) *hz;
	}
	else
	{
		pid_val->fb_d = pid_val->fb_d_ex;
	}	
	differential = (pid_arg->kd_ex *pid_val->exp_d - pid_arg->kd_fb *pid_val->fb_d);
	
	if(pid_arg->kp != 0)
	{
		pid_val->err = (expect - feedback + (differential/pid_arg->kp));	
	}
	else
	{
		pid_val->err = 0;
	}

		pid_val->err_i += pid_arg->ki *LIMIT((pid_val->err ),-inte_d_lim,inte_d_lim )*dT_s;//)*T;//+ differential/pid_arg->kp
	//pid_val->err_i += pid_arg->ki *(pid_val->err )*T;//)*T;//+ pid_arg->k_pre_d *pid_val->feedback_d
		pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);
	
	
	
	pid_val->out = pid_arg->k_ff *in_ff 
	    + pid_arg->kp *pid_val->err  
//			+	differential
//	    + pid_arg->k_inc_d_norm *pid_val->err_d_lpf + (1.0f-pid_arg->k_inc_d_norm) *differential
    	+ pid_val->err_i;
	
	pid_val->feedback_old = feedback;
	pid_val->exp_old = expect;
	
	return (pid_val->out);
}

//单环PID控制器
void Pid_Contrl(float T, pid_param *Pid_Param, s16 inte_lim)
{
	Pid_Param->err = Pid_Param->exp - Pid_Param->fb;	//误差
	
	Pid_Param->inte += Pid_Param->err * T;//积分
	Pid_Param->inte = LIMIT(Pid_Param->inte, -inte_lim, inte_lim);//积分限幅

	Pid_Param->out = Pid_Param->kp*Pid_Param->err + Pid_Param->ki*Pid_Param->inte + Pid_Param->kd*Pid_Param->speed;
}
/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

void PID_Calculate( float T,  //周期（单位：秒）
					float in_ff,				//前馈值
					float expect,				//期望值（设定值）
					float feedback,			//反馈值（）
					PID_arg_t *pid_arg, //PID参数结构体
					PID_val_t *pid_val,	//PID数据结构体
					float inte_lim,			//积分限幅
					float *out  )				//输出
{
	pid_val->err = pid_arg->kp *(expect - feedback);//比例
	pid_val->err_d = pid_arg->kd *(pid_val->err - pid_val->err_old) *safe_div(1.0f,T,0);//微分
	pid_val->feedback_d = - ( pid_arg->k_pre_d ) *(feedback - pid_val->feedback_old) *safe_div(1.0f,T,0);//反馈
	pid_val->err_i += (pid_arg->ki *pid_val->err + INNER_INTEGRAL *pid_val->feedback_d)*T;//积分
	
	//积分限幅
	pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);
	
	*out = pid_arg->k_ff *in_ff + pid_val->err + pid_val->err_d + pid_val->feedback_d + pid_val->err_i;
	
	pid_val->feedback_old = feedback;
	pid_val->err_old = pid_val->err;
}

float Alt_PID_calculate( float dT_s,    //周期（单位：秒）
										float in_ff,				//前馈值
										float expect,				//期望值（设定值）
										float feedback,			//反馈值（）
										PID_arg_t *pid_arg, //PID参数结构体
										PID_val_t *pid_val,	//PID数据结构体
										float inte_d_lim,		//误差限幅
										float inte_lim			//积分限幅									
										 )	
{
	float differential,hz;
	hz = safe_div(1.0f,dT_s,0);
	
	pid_val->err = (expect - feedback);//误差
	
	pid_val->exp_d = (expect - pid_val->exp_old) *hz;//期望误差
	
	pid_val->fb_d = (feedback - pid_val->feedback_old) *hz;

	differential = - pid_arg->kd *pid_val->fb_d;

	pid_val->err_i += pid_arg->ki *LIMIT((pid_val->err ),-inte_d_lim,inte_d_lim )*dT_s;
	pid_val->err_i = LIMIT(pid_val->err_i,-inte_lim,inte_lim);
	
	pid_val->out = pid_arg->k_ff *in_ff 
	    + pid_arg->kp *pid_val->err  
			+	differential
    	+ pid_val->err_i;
	
	pid_val->feedback_old = feedback;
	pid_val->exp_old = expect;
	
	return (pid_val->out);
}
