#ifndef __KS_HEIGHT_CTRL_H
#define __KS_HEIGHT_CTRL_H

#include "stm32f4xx.h"
#include "PID.h"

typedef struct
{
	float speed_ei;
	float speed;
	float speed_fix;
	float height;
	float Locat;
} _height_fusion_st;
typedef struct
{
	float exp_speed;
	float exp_disp;
	
	float fb_speed;
	float fb_disp;
	
	float pid_exp_speed;
	float pid_exp_acc;
	
	s16 thr_out;

} _height_ctrl_st;
extern _height_fusion_st Rol_fus;
extern _height_fusion_st Pit_fus;
extern _height_ctrl_st h_c;
void Height_Get(float dT);
void High_Ctrl(float thr,float high,float speed);	
extern s32 baro_height,baro_speed,baro_speed_o;
extern float baro_speed_av,auto_takeoff_speed;
extern s32 baro_acc;
extern u8 auto_take_off,baro_start;
#endif

