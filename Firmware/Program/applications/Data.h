#ifndef __KS_Data_H
#define __KS_Data_H

#include "stm32f4xx.h"

#define TRUE 1
#define FALSE 0 

enum
{
	AUTO_TAKE_OFF_NULL = 0,
	AUTO_TAKE_OFF = 1,
	AUTO_TAKE_OFF_FINISH,
	AUTO_LAND,
};

enum
{
	THR_MANUAL = 0,
	THR_AUTO,
	
};

enum
{
	X = 0,
	Y = 1,
	Z = 2,
	VEC_XYZ,
};
enum
{
	ROL = 0,
	PIT = 1,
	YAW = 2,
	VEC_RPY,
};

enum
{
	KP = 0,
	KI = 1,
	KD = 2,
	PID,
};
typedef struct
{
	float x;
	float y;
	float z;
} _xyz_f_st;

typedef struct
{
	s16 x;
	s16 y;
	s16 z;
} _xyz_s16_st;

typedef struct
{
	float x;
	float y;
	float z;
}
__attribute__((packed)) _xyz_f_st_pk;

typedef struct
{
	s16 x;
	s16 y;
	s16 z;
}__attribute__((packed)) _xyz_s16_st_pk;

typedef struct
{
	u8 first_f;
	float acc_offset[VEC_XYZ];
	float acc_scale[VEC_XYZ];
	float gyro_offset[VEC_XYZ];
	
	float surface_vec[VEC_XYZ];
	
	float mag_offset[VEC_XYZ];
	float mag_gain[VEC_XYZ];

} _save_st ;
extern _save_st save;
enum
{
 CH_ROL = 0,
 CH_PIT ,
 CH_THR ,
 CH_YAW ,
 AUX1 ,
 AUX2 ,
 AUX3 ,
 AUX4 ,
 CH_NUM,
};
typedef struct
{
	u8 thr_low;
	u8 locking;
	u8 NS; //信号来源，0：无信号
	u8 signal_loss;
	u8 low_power;
	u8 landed;
	u8 espDownloadMode;	
	
	u8 start_ok;
	u8 sensor_imu_ok;
	u8 motionless;
	u8 power_state;
	u8 wifi_ch_en;
	u8 chn_failsafe;
	u8 rc_loss;	
	u8 rc_loss_back_home;
	u8 gps_ok;	
	u8 nrf_ok;	
	u8 LC302_ok;	
	
	
	u8 manual_locked;
	u8 unlock_en;
	u8 unlock_err;
	u8 unlock_cmd;
	u8 unlock_sta;//unlocked
	u8 fly_ready;//unlocked
	u8 taking_off; //??
	u8 set_yaw;
	u8 ct_loc_hold;
	u8 ct_alt_hold;

	
	//????
	u8 flying;
	u8 auto_take_off_land;
	u8 home_location_ok;	
	u8 speed_mode;
	u8 thr_mode;	
	u8 flight_mode;
	u8 gps_mode_en;
	u8 motor_preparation;
	u8 locked_rotor;
}_flag_st;
extern _flag_st flag;

typedef struct
{
	float thr_en;
	float out_weight;
	float out_weight_slow;

} _global_f_st;
extern _global_f_st gf;
typedef struct
{
	u8 gyro_ok;
	u8 acc_ok;
	u8 mag_ok;
	u8 baro_ok;
	u8 gps_ok;
	u8 sonar_ok;
	u8 tof_ok;
	u8 of_ok;
	u8 nrf_ok;
} _sensor_hd_check_st; //Hardware
extern _sensor_hd_check_st sens_hd_check;
enum
{
 A_X = 0,
 A_Y ,
 A_Z ,
 G_X ,
 G_Y ,
 G_Z ,
 TEM ,
 ITEMS ,
};

enum
{
	
	m1=0,
	m2,
	m3,
	m4,
	m5,
	m6,
	m7,
	m8,

};
enum
{
	MPU_6050_0 = 0,
	MPU_6050_1,
	
};

void data_save(void);
#endif

