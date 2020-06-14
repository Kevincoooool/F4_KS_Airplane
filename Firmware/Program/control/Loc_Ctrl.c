
#include "Loc_Ctrl.h"
#include "Control.h"
#include "opticalflow.h"
#include "IMU.h"

//λ�û����Ʋ���
PID_arg_t loc_arg_2[VEC_XYZ];

//λ�û���������
PID_val_t loc_val_2[VEC_XYZ];

//λ�û�PID������ʼ��
void Loc_2level_PID_Init(void)
{

	loc_arg_2[X].kp = Param.PID_loc.kp *0.001f;
	loc_arg_2[X].ki = Param.PID_loc.ki *0.001f;
	loc_arg_2[X].kd = Param.PID_loc.kd *0.001f;

	loc_arg_2[Y] = loc_arg_2[X];
}

float exp_v[VEC_XYZ],fb_v[VEC_XYZ];
float exp_[VEC_XYZ],fb_[VEC_XYZ];
u8 ct_loc_hold[VEC_XYZ];

//λ�û�
void Loc_2level_Ctrl(float dT_s)
{
	u8 out_en,axis;
	
	if(Fixed_Point) out_en = 1;
	else						out_en = 0;
	
	//����λ�ÿ����ٶ�
	fs.alt_ctrl_speed_set[X] = fs.speed_set[X];
	fs.alt_ctrl_speed_set[Y] = fs.speed_set[Y];
	
	//����λ��
	fb_[X] = mini_flow.out_x_i;
	fb_[Y] = mini_flow.out_y_i;
	
	for(u8 i=0;i<2;i++)
	{
		//XY��ѭ������
		if(i)	axis = X; else axis = Y;
		
		//ң��λ�ñ仯�׶�
		if(fs.alt_ctrl_speed_set[axis] != 0)
		{
			ct_loc_hold[axis] = 0;//�رն�λ
			loc_val_2[axis].out = 0;//���Ϊ��
		}
		//���»ص���λ�׶�
		else
		{
			if(ct_loc_hold[axis] == 0)
			{
				//��������ٶ��㹻С��������λ
				if(ABS(fb_v[axis])<20)
				{
					ct_loc_hold[axis] = 1;
					
//					if(!locat.err)	exp_[axis] = 0;//������Ӿ���λģ�飬����λ��һֱ��0
//					else						
					exp_[axis] = fb_[axis];
				}
				//�������
				else
				{
					loc_val_2[axis].out += (-fb_v[axis]*0.05f - loc_val_2[axis].out)*0.5f;//����
				}
			}
		}

		if(out_en)
		{
			if(ct_loc_hold[axis] == 1)
			{
				Alt_PID_calculate( 	dT_s,     				//���ڣ���λ���룩
														0,								//ǰ��ֵ
														exp_[axis],				//����ֵ���趨ֵ��
														fb_[axis],				//����ֵ����
														&loc_arg_2[axis], //PID�����ṹ��
														&loc_val_2[axis],	//PID���ݽṹ��
														100,							//����޷�
														MAX_LOC_SPEED			//�����޷�
													);
			}
		}
		else
		{
			exp_[axis] = fb_[axis];
			loc_val_2[axis].out = 0;
		}
		
		loc_val_2[axis].out  = LIMIT(loc_val_2[axis].out,-MAX_LOC_SPEED,MAX_LOC_SPEED);
	}
}

//////////////////////////////////////////////////////////////////

//λ���ٶȻ����Ʋ���
PID_arg_t loc_arg_1[VEC_XYZ];

//λ���ٶȻ���������
PID_val_t loc_val_1[VEC_XYZ];

//λ���ٶȻ�PID������ʼ��
void Loc_1level_PID_Init(void)
{

	loc_arg_1[X].kp = Param.PID_loc_s.kp *0.001f;
	loc_arg_1[X].ki = Param.PID_loc_s.ki *0.001f;
	loc_arg_1[X].kd = Param.PID_loc_s.kd *0.001f;

	loc_arg_1[Y] = loc_arg_1[X];
}

#define LOC_INTE_LIM 500
#define LOC_CTRL_MAX 15
float ct_loc_out[VEC_XYZ];

//λ���ٶȻ�
void Loc_1level_Ctrl(float dT_s)
{
	u8 out_en,axis;
	
	if(Fixed_Point) out_en = 1;
	else						out_en = 0;	
	
	//�����ٶ�
	exp_v[X] = fs.alt_ctrl_speed_set[X] + loc_val_2[X].out;
	exp_v[Y] = fs.alt_ctrl_speed_set[Y] + loc_val_2[Y].out;
	
	//�����ٶ�
	fb_v[X] = mini_flow.fix_x;
	fb_v[Y] = mini_flow.fix_y;
	
	for(u8 i=0;i<2;i++)
	{
		//XY��ѭ������
		if(i)	axis = X; else axis = Y;

		Alt_PID_calculate( 	dT_s,         			//���ڣ���λ���룩
												0,									//ǰ��ֵ
												exp_v[axis],				//����ֵ���趨ֵ��
												fb_v[axis],					//����ֵ����
												&loc_arg_1[axis], 	//PID�����ṹ��
												&loc_val_1[axis],		//PID���ݽṹ��
												200,								//����޷�
												LOC_CTRL_MAX*out_en	//�����޷�
											);
		
		loc_val_1[axis].out = LIMIT(loc_val_1[axis].out,-LOC_CTRL_MAX,LOC_CTRL_MAX);
		
		ct_loc_out[axis] = out_en *(loc_val_1[axis].out);
	}
}



#define Fixed_Point_Max 15.0f
u8 Fixed_Point_Mode = 1,Fixed_Point = 0;

//������ƣ�����Ƶ��50hz
void Fixed_Point_Ctrl(float x,float y)
{
	//ң�����ݹ�1��
	fs.speed_set_norm[X] = x*0.002f;
	fs.speed_set_norm[Y] = y*0.002f;
	fs.speed_set_norm_lpf[X] += 0.2f *(fs.speed_set_norm[X] - fs.speed_set_norm_lpf[X]);//��ͨ�˲�
	fs.speed_set_norm_lpf[Y] += 0.2f *(fs.speed_set_norm[Y] - fs.speed_set_norm_lpf[Y]);//��ͨ�˲�
	
//	//�Ƕ���ģʽ�²����ж�������
//	if(!Fixed_Point_Mode || !keep_high_mode)	return;

	switch(start)
	{
		case 0://û�����
			Fixed_Point =  0;//������
			fs.speed_set[X] = 0;//�ٶ�Ϊ��
			fs.speed_set[Y] = 0;//�ٶ�Ϊ��
		break;
			
		case 1://�Զ���ɽ׶�
			Fixed_Point =  1;		//��������
			fs.speed_set[X] = 0;//�ٶ�Ϊ��
			fs.speed_set[Y] = 0;//�ٶ�Ϊ��
		break;
		
		default://���й�����
			if(auto_landing)
			{
				Fixed_Point =  1;		//��������
				fs.speed_set[X] = 0;//�ٶ�Ϊ��
				fs.speed_set[Y] = 0;//�ٶ�Ϊ��
			}
			else
			{
				Fixed_Point =  1;//��������
				fs.speed_set[X] = fs.speed_set_norm_lpf[X]*MAX_LOC_SPEED;//�����ٶ�
				fs.speed_set[Y] = fs.speed_set_norm_lpf[Y]*MAX_LOC_SPEED;//�����ٶ�
			}
		break;
	}
	
	Loc_1level_Ctrl(20e-3f);//λ���ٶȻ�����
	Loc_2level_Ctrl(20e-3f);//λ�û�����
}








