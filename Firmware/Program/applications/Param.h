#ifndef __Param_H
#define __Param_H

#include "stm32f4xx.h"
#include "drv_flash.h"
#include "PID.h"
#include "Data.h"
/////////////////////////////////////////////////
#define FIRST_INIT_FLAG 		0XAA

typedef struct
{
	s32 kp;			 //比例系数
	s32 ki;			 //积分系数
	s32 kd;		 	 //微分系数

}PID_param_st_pk; 
typedef struct
{
	u8 save_en;
	u8 save_trig;
	u16 time_delay;
}_parameter_state_st ;
extern _parameter_state_st para_sta;

struct _save_param_st_pk{	
	u8 firstintiflag;
	u16 hardware;
	u16 software;
	u8 sensor_type;
	
	float acc_offset[VEC_XYZ];
	float acc_scale[VEC_XYZ];
	float gyro_offset[VEC_XYZ];
	
	float surface_vec[VEC_XYZ];
	
	float gyr_temprea_offset;
	float acc_temprea_offset;

	float auto_take_off_height;
	float auto_take_off_speed;
	float auto_landing_speed;
	float idle_speed_pwm;
	
	PID_param_st_pk PID_rol; //12字节，3个float
	PID_param_st_pk PID_pit;
	PID_param_st_pk PID_yaw;
	PID_param_st_pk PID_loc;
	PID_param_st_pk PID_high;

	PID_param_st_pk PID_rol_s; //12字节，3个float
	PID_param_st_pk PID_pit_s;
	PID_param_st_pk PID_yaw_s;
	PID_param_st_pk PID_loc_s;
	PID_param_st_pk PID_high_s;

};

/////////////////////////////////////////////////
extern struct _save_param_st_pk Param;
void data_save(void);
void Param_Init(void);
void Param_Read(void);
void Param_Save(void);
void PID_Save_Overtime(u16 ms,u16 dTms);//PID参数超时写入
extern u16 save_pid_en;
#endif

