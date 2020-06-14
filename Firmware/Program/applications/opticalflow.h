#ifndef _USER_FLOW_H_
#define _USER_FLOW_H_

#define angle_to_rad1 0.0174f
#define sq(x) ((x)*(x))
#include "stm32f4xx.h"
#include "filter.h"
//����������ݽṹ��
struct _mini_flow_
{
	float x_i;			//x�����ԭʼֵ
	float y_i;			//y�����ԭʼֵ
	float fix_x_i;		//x������˲�ֵ
	float fix_y_i;		//y������˲�ֵ
	float ang_x;			
	float ang_y;			
	float out_x_i;			//x��������ֵ
	float out_y_i;			//y��������ֵ
	float out_x_i_o;			
	float out_y_i_o;		
	
	float x;				//x���ٶ�ԭʼֵ
	float y;				//y���ٶ�ԭʼֵ
	float fix_x;		//x���ٶ��ں�ֵ
	float fix_y;		//y���ٶ��ں�ֵ
	
	float gyro_x;				//x���ٶ�ԭʼֵ
	float gyro_y;				//y���ٶ�ԭʼֵ
	float gyro_fix_x;		//x���ٶ��ں�ֵ
	float gyro_fix_y;		//y���ٶ��ں�ֵ
	s16 high;
	u8 qual;	//��������
};
extern struct _mini_flow_ mini_flow;

typedef struct
{
	float speed_ei;
	float speed;
	float speed_fix;
	float height;
	float Locat;
} _flow_fusion_st;
extern _fix_inte_filter_st opt_spe_fus,opt_hei_fus;
typedef struct
{
	float vAccDeadband; /* ���ٶ����� */
	float accBias[3];	/* ���ٶ� ƫ��(cm/s/s)*/
	float acc[3];		/* ������ٶ� ��λ(cm/s/s)*/
	float vel[3];		/* �����ٶ� ��λ(cm/s)*/
	float pos[3]; 		/* ����λ�� ��λ(cm)*/
} estimator_t;

extern u16 VL53L0X_Dist;
extern s16 user_flow_x,user_flow_y;
extern s16 user_flow_x_i,user_flow_y_i;//��������ԭʼ����
extern s16 user_flow_x_o,user_flow_y_o;
extern float user_flow_fix_x_i,user_flow_fix_y_i;
extern float user_flow_i_x,user_flow_i_y;
extern float user_flow_fix_x,user_flow_fix_y;
extern float user_gyro_x,user_gyro_y;
void OPTIACL_FLOW_Data_Receive_Prepare(u8 data);
extern u8 OF_Err;
extern _flow_fusion_st flowx_fus;
extern _flow_fusion_st flowy_fus;

void Opt_Flow_Receive(u8 data);
void Opt_Flow_Duty(u8 *data_buf);
void Mini_Flow_Receive(u8 data);
void mini_flow_Fix(void);
void Flow_Duty(void);

#define Filter_Num 3

struct _Filter
{
	int sum;
	int old[Filter_Num];
};
typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;
typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;
void mini_flow_reset(void);
void OpticalFlow_Init(void);
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF);
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);
void Flow_Window_Filter(s16 *data0, s16 *data1);
void Gyro_Window_Filter(float *data0, float *data1);
float Flow_Speed_Filter(float ImuSpeed,float FlowSpeed);
#endif
