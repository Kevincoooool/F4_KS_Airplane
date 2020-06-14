#include "include.h"
#include "height_ctrl.h"
/*
������λ��cm

*/
s32 baro_height,baro_height_old;
s32 baro_speed_o,baro_speed;
s32 baro_acc_o,baro_acc;

_height_fusion_st Rol_fus;
_height_fusion_st Pit_fus;

#define MONUM 10
float speed_av_arr[MONUM];
float speed_av;
u16 speed_av_cnt;

float speed_delta;
float speed_delta_lpf;

/*
������λ��cm
*/
s32 baro_height,baro_height_old,baro_Offset;
s32 baro_speed_o,baro_speed;
u8 baro_start=1;
void Height_Get(float dT)
{

	//��ѹ�߶�����
	if( baro_start )
	{
		baro_height = 0;
		baro_height_old = 0;
		baro_Offset = Drv_Spl0601_Read();
		WCZ_Data_Reset();
		baro_start = 0;
	}
	else
	{
		//��ȡ��ѹ�Ƹ߶�
		baro_height = Drv_Spl0601_Read() - baro_Offset;
		baro_speed_o = safe_div(baro_height - baro_height_old,dT,0);
		baro_height_old = baro_height;
	}

	//�����ٶ�
	Moving_Average(speed_av_arr,MONUM ,&speed_av_cnt,baro_speed_o,&speed_av);
	speed_delta = LIMIT(speed_av - baro_speed,-2000*dT,2000*dT);
	LPF_1_(0.5f,dT,speed_delta,speed_delta_lpf);
	baro_speed += speed_delta *LIMIT((ABS(speed_delta_lpf)/(2000*dT)),0,1);
}

u8 start=0,auto_landing=0,auto_take_off=0;
float high_begin,high_start;
//float Now_High,Target_Speed;
pid_param h_axis;//�߶ȿ��Ʋ���

//�߶ȿ��ƣ�����Ƶ��100hz
void High_Ctrl(float thr,float high,float speed)
{
	static u8 _cnt;

	//ң�����ݹ�1
	fs.speed_set_norm[Z] = thr*0.002f;
	fs.speed_set_norm_lpf[Z] += 0.2f *(fs.speed_set_norm[Z] - fs.speed_set_norm_lpf[Z]);//��ͨ�˲�

	//���Ϊδ���״̬
	if(flag.fly_ready == 0)
	{
		start = 0;
		
		high_begin = wcz_hei_fus.out; //��¼���ǰ�߶�
		return;
	}
	
	if(start && ( ct_val_thr<200) )//�����ж�
	{
		_cnt++;
		if(_cnt>=100)
		{
			_cnt = 0;         //������
			if(CH_N[CH_THR]<-400 || auto_landing || flag.NS==0)
			{
				start = 0;        //���Ϊδ���״̬
				auto_landing = 0;	//ȡ���Զ�����
				//Throw_Fly = 0;		//ȡ���׷�
				fly_ready = 0;		//����
				flag.fly_ready = 0;				//����
			}
		}
	}
	else _cnt = 0;
	
	switch(start)
	{
		//û�����
		case 0:
		{
			if(thr>=0)
			{
				if(auto_take_off) 	start = 1;//�Զ����
				else				start = 2;//�ֶ����
			}
			else
			{
				fs.speed_set[Z] = 0 ;
			}
		}
		break;
		
		//�Զ���ɽ׶�
		case 1:
		{
			//�����ɸ߶ȴ���Ŀ��߶ȣ����л�����������״̬
			if(wcz_hei_fus.out >= high_start)	start = 2;
			else	fs.speed_set[Z] = 200 +  fs.speed_set_norm_lpf[Z]*MAX_D_SPEED;//�Զ�����ٶ�
			fs.speed_set[Z] = LIMIT(fs.speed_set[Z],0,300);//�Զ�����ٶ��޷�
		
			//�Զ���ɹ������������ţ�����ȡ���Զ����
			if(fs.speed_set_norm_lpf[Z] < -0.8) start = 2;
		}
		break;
		
		//������
		default:
		{
			//�Զ�����
			if(auto_landing || flag.NS==0)
			{
				fs.speed_set[Z] = -50 + fs.speed_set_norm_lpf[Z]*MAX_D_SPEED;
				fs.speed_set[Z] = LIMIT(fs.speed_set[Z],-MAX_D_SPEED,MAX_U_SPEED);
			}
			//ң�ؿ��Ƹ߶�
			else
			{
				if(fs.speed_set_norm_lpf[Z]>=0)	
					fs.speed_set[Z] = fs.speed_set_norm_lpf[Z]*MAX_U_SPEED;//�����ٶ�
				else
					fs.speed_set[Z] = fs.speed_set_norm_lpf[Z]*MAX_D_SPEED;//�����ٶ�
			}
		}
		break;
	}
	
	Alt_1level_Ctrl(10e-3f);/*�߶��ٶȻ�����*/
	Alt_2level_Ctrl(10e-3f);/*�߶Ȼ�����*/
}

