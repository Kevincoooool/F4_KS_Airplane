#ifndef __IMU_H
#define __IMU_H
#include "Data.h"
#include "Drv_icm20689.h"
//#include "FlightDataCal.h"

typedef struct
{
	float w;//q0;
	float x;//q1;
	float y;//q2;
	float z;//q3;

	float x_vec[VEC_XYZ];
	float y_vec[VEC_XYZ];
	float z_vec[VEC_XYZ];
	float hx_vec[VEC_XYZ];

	float a_acc[VEC_XYZ];
	float w_acc[VEC_XYZ];
	float h_acc[VEC_XYZ];
	
	float w_mag[VEC_XYZ];
	
	float gacc_deadzone[VEC_XYZ];
	
	float obs_acc_w[VEC_XYZ];
	float obs_acc_a[VEC_XYZ];
	float gra_acc[VEC_XYZ];
	
	float est_acc_a[VEC_XYZ];
	float est_acc_h[VEC_XYZ];
	float est_acc_w[VEC_XYZ];
	
	float est_speed_h[VEC_XYZ];
	float est_speed_w[VEC_XYZ];

	
	float rol;
	float pit;
	float yaw;
	float inter_rol;
	float inter_pit;
	float inter_yaw;
	float Cos_Roll;
	float Sin_Roll;
	float Cos_Pitch;
	float Sin_Pitch;
	float Cos_Yaw;
	float Sin_Yaw;
} _imu_st ;
extern _imu_st imu_data;

typedef struct
{
	float gkp;
	float gki;
	
	float mkp;
	float drag_p;
	
	u8 G_reset;
	u8 M_reset;
	u8 G_fix_en;
	u8 M_fix_en;
	
	u8 obs_en;
}_imu_state_st;
extern _imu_state_st imu_state;

void IMU_duty(float);
void IMU_update(float dT,_imu_state_st *,float gyr[VEC_XYZ],s32 acc[VEC_XYZ],s16 mag_val[VEC_XYZ],_imu_st *imu);
void calculate_RPY(void);

void w2h_2d_trans(float w[VEC_XYZ],float ref_ax[VEC_XYZ],float h[VEC_XYZ]);

void h2w_2d_trans(float h[VEC_XYZ],float ref_ax[VEC_XYZ],float w[VEC_XYZ]);
void IMU_Update_Task(u8 dT_ms);
#endif

