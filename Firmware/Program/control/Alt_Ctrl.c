#include "include.h"

#include "Alt_Ctrl.h"

//flight_state_st fs;

PID_arg_t alt_arg_high;
PID_val_t alt_val_high;

/*�߶Ȼ�PID������ʼ��*/
void Alt_2level_PID_Init()
{
	alt_arg_high.kp = Param.PID_high.kp*0.001f;
	alt_arg_high.ki = Param.PID_high.ki*0.001f;
	alt_arg_high.kd = Param.PID_high.kd*0.001f;
}

float exp_hv,fb_hv;
float exp_h,fb_h;
u8 ct_alt_hold;

//�߶Ȼ�
void Alt_2level_Ctrl(float dT_s)
{
	u8 out_en;
	
	if(start) out_en = 1;
	else			out_en = 0;
	
	//���ø߶ȿ����ٶ�
	fs.alt_ctrl_speed_set[Z] = fs.speed_set[Z];

	//�����߶�
	fb_h = wcz_hei_fus.out;
	
	//ң�ظ߶ȱ仯�׶�
	if(fs.alt_ctrl_speed_set[Z] != 0)
	{
		ct_alt_hold = 0;//�رն���
		alt_val_high.out = 0;//���Ϊ��
	}
	else
	{
		//���»ص����߽׶�
		if(ct_alt_hold==0)
		{
			//��������ٶȽӽ�0����������
			if(ABS(fb_hv)<=10)
			{
				ct_alt_hold = 1;
				exp_h = fb_h;
			}
			//�������
			else
			{
				alt_val_high.out += (-fb_hv*alt_arg_high_s.kp*0.5f - alt_val_high.out)*0.5f;//����
			}
		}
	}
	
	//�Ƕ���ģʽ�������߶�һֱ���ڵ�ǰ�߶�
	if(!keep_high_mode) exp_h = fb_h;
	
	if(out_en)
	{
		if(ct_alt_hold == 1)
		{
			Alt_PID_calculate( 	dT_s,     	//���ڣ���λ���룩
													0,					//ǰ��ֵ
													exp_h,			//����ֵ���趨ֵ��
													fb_h,				//����ֵ����
													&alt_arg_high, //PID�����ṹ��
													&alt_val_high,	//PID���ݽṹ��
													100,				//��������޷�
													100					//�����޷�									
												);
		}
	}
	else
	{
		exp_h = fb_h;
		alt_val_high.out = 0;
	}
	
	alt_val_high.out = LIMIT(alt_val_high.out,-200,200);
}

PID_arg_t alt_arg_high_s;
PID_val_t alt_val_high_s;

/*�߶��ٶȻ�PID������ʼ��*/
void Alt_1level_PID_Init()
{
	alt_arg_high_s.kp = Param.PID_high_s.kp*0.001f;
	alt_arg_high_s.ki = Param.PID_high_s.ki*0.001f;
	alt_arg_high_s.kd = Param.PID_high_s.kd*0.001f;
}


float err_i_comp;
float ct_val_thr;

//�߶��ٶȻ�
void Alt_1level_Ctrl(float dT_s)
{
	u8 out_en;
	
	if(start) out_en = 1;
	else			out_en = 0;	
	
	//�����ٶ�
	exp_hv = fs.alt_ctrl_speed_set[Z] + alt_val_high.out;
	
	//�����ٶ�
	fb_hv = wcz_spe_fus.out;

	Alt_PID_calculate( 	dT_s,         //���ڣ���λ���룩
											0,						//ǰ��ֵ
											exp_hv,				//����ֵ���趨ֵ��
											fb_hv,				//����ֵ����
											&alt_arg_high_s, 	//PID�����ṹ��
											&alt_val_high_s,		//PID���ݽṹ��
											100,					//��������޷�
											(MAX_THR - err_i_comp) * out_en//�����޷�									
										);
	
	//���������������Խ������ٶ�Խ�졣
//	if(Throw_Fly) THR_START =300;
//	else					THR_START =200;
	
	if(out_en)
	{
		err_i_comp = 200;
	}
	else
	{
		err_i_comp = 0;
	}
	
	mc.ct_val_thr=ct_val_thr = out_en *(alt_val_high_s.out + err_i_comp );
}

