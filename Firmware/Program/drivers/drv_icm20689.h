#ifndef __DRV_ICM20689_H
#define	__DRV_ICM20689_H
#include "include.h"
#include "mathTool.h"
#include "Filter.h"
#define RANGE_PN2000_TO_RAD 0.001065f
#define RANGE_PN8G_TO_CMSS  0.2395f

typedef struct
{
	float center_pos_cm[VEC_XYZ];
	float gyro_rad[VEC_XYZ];
	float gyro_rad_old[VEC_XYZ];
	float gyro_rad_acc[VEC_XYZ];
	float linear_acc[VEC_XYZ];
}_center_pos_st;
typedef struct
{
  u8 surface_CALIBRATE;
	float surface_vec[VEC_XYZ];
	float surface_unitvec[VEC_XYZ];
	
}_sensor_rotate_st;
extern _sensor_rotate_st sensor_rot ;
extern _center_pos_st center_pos;

typedef struct 
{
	u8 acc_CALIBRATE;
	u8 gyr_CALIBRATE;
	u8 acc_z_auto_CALIBRATE;
	
	s16 Acc_Original[VEC_XYZ];
	s16 Gyro_Original[VEC_XYZ];
	
	s16 Acc[VEC_XYZ];
	s32 Acc_cmss[VEC_XYZ];
	float Gyro[VEC_XYZ];
	float Gyro_deg[VEC_XYZ];
	float Gyro_deg_lpf[VEC_XYZ];
	float Gyro_rad[VEC_XYZ];
	float my_acc_z;

	s16 Tempreature;
	float Tempreature_C;
	
}_sensor_st;//__attribute__((packed)) 

enum ORIENTATION_STATUS
{
    ORIENTATION_UP,
    ORIENTATION_DOWN,
    ORIENTATION_LEFT,
    ORIENTATION_RIGHT,
    ORIENTATION_FRONT,
    ORIENTATION_BACK,
};
typedef struct
{
    Vector3f_t offset;	    //��ƫ���
    Vector3f_t scale;		//缩放
    bool should_cali;		//������У׼��־λ
    bool success;           //У׼�ɹ���־λ
    uint8_t step;           //�����־λ
} SENSOR_CALI_t;

//typedef struct
//{
//    float gyro_offset;
//    float acc_offset;
//    float mag_offset;
//    enum SENSOR_HEALTH gyro;
//    enum SENSOR_HEALTH acc;
//    enum SENSOR_HEALTH mag;
//} SENSOR_HEALTH_t;
typedef struct {
    Vector3f_t data;
    Vector3f_t dataLpf;
    float mag;
    float vibraCoef;
    LPF2ndData_t lpf_2nd;
    SENSOR_CALI_t cali;
    SENSOR_CALI_t levelCali;
} ACCELEROMETER_t;
extern Vector3f_t AccRaw;
extern _sensor_st sensor;
void Drv_Icm20689CSPin_Init(void);
bool ICM20689_Detect(void);
u8 Drv_Icm20689Reg_Init(void);
void Drv_Icm20689_Read(void);
void Sensor_Data_Prepare(u8 dT_ms);
void Center_Pos_Set(void);
void AccCalibration(Vector3f_t accRaw);
void ImuOrientationDetect(Vector3f_t acc);
#endif








