#ifndef __KS_CTRL_H
#define __KS_CTRL_H

#include "stm32f4xx.h"
#include "PID.h"
#include "filter.h"

/*=====================================================================================================================
						飞机控制参数
=====================================================================================================================*/
#define MAX_ZSPEED_UP 2000  //最大控制上升速度（mm/s）
#define MAX_ZSPEED_DN 1000	//最大控制下降速度（mm/s）

#define MAX_ROL_ANGLE 30		//最大控制横滚角误差（度）
#define MAX_PIT_ANGLE 30		//最大控制俯仰角误差（度）
//#define MAX_YAW_ANGLE 30    //最大控制偏航角误差（度）
//#define MAX_ROL_ANGLE 15		//最大控制横滚角误差（度）
//#define MAX_PIT_ANGLE 15		//最大控制俯仰角误差（度）
#define MAX_YAW_ANGLE 100   //最大控制偏航角误差（度）
#define MAX_HIG_VALUE 50   	//最大控制高度误差  （cm）


#define MAX_ROL_SPEED 300		//最大控制横滚角速率（度/s）
#define MAX_PIT_SPEED 300		//最大控制俯仰角速率（度/s）
#define MAX_YAW_SPEED 300		//最大控制偏航角速率（度/s）
#define MAX_LOC_SPEED 150		//最大控制位置速度
#define HIGH_START  	(high_begin + 80)    //一键起飞目标高度


#define MAX_U_SPEED 200  	//最大控制速度（cm/s）
#define MAX_D_SPEED 200  	//最大控制速度（cm/s）
#define START_SPEED  60   //一键起飞目标速度

#define THR_INTEGRA_LIM      60    //自动油门积分最大值（百分比）
#define THR_INIT             30    //油门pid起调量

#define CTRL_1_INTER_LIMIT 200  //角速度积分幅度限制
#define CTRL_2_INTER_LIMIT 5    //角度积分幅度限制

#define CTRL_0_HIG_INTER_LIMIT    200  //油门速度控制积分
#define CTRL_1_HIG_INTER_LIMIT    800  //升降速度积分限幅
#define CTRL_2_HIG_INTER_LIMIT    10   //高度积分幅度限制

#define X_KP 0.05f	//光流定点PID基数
#define X_KI 0.00f
#define X_KD 0.05f

/*=====================================================================================================================
						
=====================================================================================================================*/
typedef struct
{
	float exp_rol;
	float exp_pit;
	float exp_yaw;
	float exp_hig;
	float exp_thr;
	float exp_yaw_i_comp;
	
	float fb_rol;
	float fb_pit;
	float fb_yaw;
	float fb_hig;
	float fb_thr;
	
	float out_rol;
	float out_pit;
	float out_yaw;
	float out_hig;
	float out_thr;
	
}_copter_ctrl_st;

typedef struct
{
//	s16 alt_ctrl_speed_set;
	float speed_set_h[VEC_XYZ];	
	float speed_set_h_cms[VEC_XYZ];
	
	float speed_set_h_norm[VEC_XYZ];
	float speed_set_h_norm_lpf[VEC_XYZ];
	
	s16 speed_set[VEC_XYZ];//设置速度
	
	s16 alt_ctrl_speed_set[VEC_XYZ];//控制速度
	
	/*设置油门摇杆量0~50*/
	float speed_set_norm[VEC_XYZ];
	float speed_set_norm_lpf[VEC_XYZ];
	
	
}_flight_state_st;
extern _flight_state_st fs;
void user_fun(float dT,u8 action_num);

void All_PID_Init(void);

void one_key_take_off(void);
void one_key_land(void);

void one_key_roll(void);
void app_one_key_roll(void);
void app_one_key_roll_reset(void);
void one_key_take_off_task(u16 dt_ms);

void ctrl_parameter_change_task(void);
	
void Flight_State_Task(u8,s16 *CH_N);

void Flight_Mode_Set(u8 dT_ms);

void Swtich_State_Task(u8 dT_ms);
extern float thr_value;
void CTRL_Duty(float ch1,float ch2,float ch3,float ch4);
void High_Ctrl(float thr,float high,float speed);
void Fixed_Point_Ctrl(float x,float y);
void Throw_Fly_Check(void);
void CTRL_O_A_PID_Init(void);
void CTRL_I_A_PID_Init(void);
void CTRL_I_H_PID_Init(void);
void pid_init(void);
void Roll_3D(void);
void Static_Check(void);
void motor_ctrl(s16 ct_val_rol,s16 ct_val_pit,s16 ct_val_yaw,s16 ct_val_thr);
extern s16 motor[],balance_max;
void Speed_Set(void);
extern int Speed_state;
extern float exp_hspeed;


extern _copter_ctrl_st ctrl_1;
extern _copter_ctrl_st ctrl_2;

extern u8 keep_high_mode;
extern u8 No_Head_Mode;
extern u8 start,auto_landing,auto_takeoff;
extern float Target_High,high_start;
extern float high_begin;
extern float Now_High;
extern u8 Throw_Fly_Mode;
extern float rol_angle,pit_angle;
extern u8 Fixed_Point_Mode,Fixed_Point;
extern float Rc_Angle,Rc_Abs;
extern u8 roll_en;
#endif
