#include "MotionCal.h"
#include "mymath.h"
#include "IMU.h"
#include "height_ctrl.h"
#include "Control.h"
#include "include.h"
_inte_fix_filter_st wcz_acc_fus;
_fix_inte_filter_st wcz_spe_fus,wcz_hei_fus;

s32 ref_height_old,ref_speed_old;

s32 wcz_ref_height,wcz_ref_speed,wcz_ref_acc;

float wcz_acc_deadzone;	
float wcz_acc_use;	
static s32 wcz_acc;
#define N_TIMES 5
void WCZ_Acc_Get_Task(void)//��С����
{
	wcz_acc_use += 0.1f *(imu_data.w_acc[Z] - wcz_acc_use);
}
void WCZ_Data_Calc(u8 dT_ms)
{
	static u8 cyc_xn;
	float hz,ntimes_hz;	
	hz = safe_div(1000,dT_ms,0);
	ntimes_hz = hz/N_TIMES;
	wcz_ref_height = baro_height;
/////////////////////////////////////////////////////////////	
	wcz_acc_deadzone = LIMIT(5 *(0.996f - imu_data.z_vec[Z] *imu_data.z_vec[Z]),0,1) *10;
	
	cyc_xn ++;
	cyc_xn %= N_TIMES;
	
	if(cyc_xn == 0)
	{
		wcz_ref_speed = (wcz_ref_height - ref_height_old) *ntimes_hz;
		
		wcz_ref_acc = (wcz_ref_speed - ref_speed_old) *ntimes_hz;
		
		ref_height_old = wcz_ref_height;	
		ref_speed_old = wcz_ref_speed;
	
	}
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////
	
	wcz_acc_fus.fix_ki = 0.1f;
	wcz_acc_fus.in_est = wcz_acc_use;
	wcz_acc_fus.in_obs = wcz_ref_acc;
	wcz_acc_fus.ei_limit = 100;
	inte_fix_filter(dT_ms*1e-3f,&wcz_acc_fus);

	
	wcz_spe_fus.fix_kp = 0.8f;
	wcz_spe_fus.in_est_d = my_deadzone(wcz_acc_fus.out,0,wcz_acc_deadzone);
	wcz_spe_fus.in_obs = wcz_ref_speed;
	wcz_spe_fus.e_limit = 200;
	fix_inte_filter(dT_ms*1e-3f,&wcz_spe_fus);
	
	
	wcz_hei_fus.fix_kp = 0.8f;
	wcz_hei_fus.in_est_d = wcz_spe_fus.out;
	wcz_hei_fus.in_obs = wcz_ref_height;
	//wcz_hei_fus.e_limit = 200;
	fix_inte_filter(dT_ms*1e-3f,&wcz_hei_fus);
	
}

void WCZ_Data_Reset(void)
{

	wcz_acc_fus.out = 0;
	wcz_acc_fus.ei = -wcz_acc_use;

	wcz_spe_fus.out = 0;
	wcz_spe_fus.e = 0;
	
	wcz_hei_fus.out = 0;
	wcz_hei_fus.e = 0;	
}
_st_Height Height;
 float pos_correction;
 float acc_correction;
 float vel_correction;
/****��ѹ�����׻����˲����������ο���Դ�ɿ�APM****/
//#define TIME_CONTANST_ZER       1.5f
const float TIME_CONTANST_ZER=1.5f; //�ںϲ���
#define K_ACC_ZER 	        (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
#define K_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))														
#define K_POS_ZER           (3.0f / TIME_CONTANST_ZER)
float high_test;

void Strapdown_INS_High(float high)
{
	float dt;
	float Altitude_Estimate=0;
	static uint16_t delay_cnt;
	static float high_offset;
	{//��ȡϵͳǰ������׼ȷ���ںϼ��
		static	float last_time;
		float now_time;
		now_time = GetSysTime_us()* 1e-6f;
		dt = now_time - last_time;
		last_time = now_time;

	}
	if(delay_cnt<200)//�ո��ϵ�ʱ ����ѹ�Ƶ��油��
	{
		pos_correction = acc_correction = vel_correction = 0;
		high_offset = high;
		delay_cnt++;
		return;
	}
	high -= high_offset;

	const uint8_t High_Delay_Cnt=10;//150ms
	Altitude_Estimate=high;//�߶ȹ۲���

	high_test = high;
	//�ɹ۲�������ѹ�ƣ��õ�״̬���
	static float History_Z[High_Delay_Cnt+1]; 
	float temp;
	temp= Altitude_Estimate- History_Z[High_Delay_Cnt];//��ѹ��(������)��SINS�������Ĳ��λcm
	//��·���ַ����������ߵ�
	acc_correction += temp * K_ACC_ZER * dt ;//���ٶȽ�����
	vel_correction += temp * K_VEL_ZER * dt ;//�ٶȽ�����
	pos_correction += temp * K_POS_ZER * dt ;//λ�ý�����
	//���ٶȼƽ��������
	static float last_accZ;
	static float Current_accZ; //��ǰZ���ϵļ��ٶ�
	last_accZ=Current_accZ;//��һ�μ��ٶ���
	
	Current_accZ = imu_data.w_acc[Z] + acc_correction;
	//�ٶ�������������£����ڸ���λ��,���ڲ���h=0.005,��Խϳ���
	//������ö����������������΢�ַ��̣��������ø��߽׶Σ���Ϊ���ٶ��źŷ�ƽ��
	float speed_Z;
	static float t_high;
	static float t_speed;
	speed_Z =+(last_accZ+Current_accZ)*dt/2.0f;
	//ԭʼλ�ø���
	t_high += (Height.Speed+0.5f*speed_Z)*dt;
	//λ�ý��������
	Height.High = t_high + pos_correction;
	//��ֱԭʼ�ٶȸ���
	t_speed +=speed_Z;
	//��ֱ�ٶȽ��������
	Height.Speed = t_speed + vel_correction;

	//---------------------------------------------------------------
	static uint16_t Save_Cnt=0;
	Save_Cnt++;//���ݴ洢����
	if(Save_Cnt>=5)//5ms
	{
		for(uint8_t Cnt=High_Delay_Cnt;Cnt>0;Cnt--)//5ms����һ��
		{
			History_Z[Cnt]=History_Z[Cnt-1];
		}
		History_Z[0]=Height.High; //
		Save_Cnt=0;
	}
}

