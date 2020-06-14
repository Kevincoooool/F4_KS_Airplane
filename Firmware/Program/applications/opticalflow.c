
#include "opticalflow.h"
#include "include.h"

s16 user_flow_x,user_flow_y;//��������ԭʼ����
s16 user_flow_x_i,user_flow_y_i;//��������ԭʼ����
s16 user_flow_x_o,user_flow_y_o;//��������ԭʼ���ֵ

float user_flow_i_x,user_flow_i_y;//������������ԭʼֵ
float user_flow_fix_x,user_flow_fix_y;//���������ں�����
float user_flow_fix_x_i,user_flow_fix_y_i;
float user_gyro_x,user_gyro_y;//���ٶ�����
float flow_gyr_x,flow_gyr_y;
float flow_gyr_fix_x,flow_gyr_fix_y;
void Opt_Flow_Receive(u8 data)
{
	static u8 RxBuffer[32];
	static u8 _data_len = 0,_data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xAA)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1&&data==0xAA)
	{
		state=2;
		RxBuffer[1]=data;
	}
	else if(state==2&&data<0XFF)
	{
		state=3;
		RxBuffer[2]=data;
	}
	else if(state==3&&data<33)
	{
		state = 4;
		RxBuffer[3]=data;
		_data_len = data;
		_data_cnt = 0;
	}
	else if(state==4&&_data_len>0)
	{
		_data_len--;
		RxBuffer[4+_data_cnt++]=data;
		if(_data_len==0)
			state = 5;
	}
	else if(state==5)
	{
		state = 0;
		RxBuffer[4+_data_cnt]=data;
		Opt_Flow_Duty(RxBuffer);
	}
	else
		state = 0;
}
u16 VL53L0X_Dist;
u16 use_dis=0;
u8 OF_Err=1;
void Opt_Flow_Duty(u8 *data_buf)
{
	switch( *(data_buf+2) )
	{
		case 0xF1:
	
			user_flow_x   = (s16)(*(data_buf+4)<<8)|*(data_buf+5) ;
			user_flow_y   = (s16)(*(data_buf+6)<<8)|*(data_buf+7) ;
			user_flow_i_x = (s16)(*(data_buf+8)<<8)|*(data_buf+9) ;
			user_flow_i_y = (s16)(*(data_buf+10)<<8)|*(data_buf+11) ;
		
			VL53L0X_Dist  = (s16)(*(data_buf+12)<<8)|*(data_buf+13) ;
		if( wcz_hei_fus.out<=0 )
		{
			user_flow_x = 0;
			user_flow_y = 0;
		}
		else if(wcz_hei_fus.out<50)
		{
			user_flow_x = user_flow_x * wcz_hei_fus.out * 0.02f;
			user_flow_y = user_flow_y * wcz_hei_fus.out * 0.02f;
		}
		
		break;
	}
}				
float k;

//����������߶ȼ��ں�ϵ��
void Flow_High_Cal(void)
{
	if( wcz_hei_fus.out<=0 ) k = 0.0f;
	else
	{
		k = wcz_hei_fus.out * 0.02f;
		k = LIMIT(k,0.0f,1.0f);
	}
}
u8 RxBuffer_Flow[9];
//�������ݽ���
void Mini_Flow_Receive(u8 data)
{
	
	static u8 _data_cnt = 0;
	static u8 state = 0;
	u8 sum = 0;
	
	switch(state)
	{
		case 0:
			if(data==0xFE)//��ͷ
			{
				state=1;
				RxBuffer_Flow[_data_cnt++]=data;
			}else state = 0;
		break;
			
		case 1:
			if(data==0x04)//���ֽ�����
			{
				state=2;
				RxBuffer_Flow[_data_cnt++]=data;
			}else state = 0;
		break;
			
		case 2:
			RxBuffer_Flow[_data_cnt++]=data;
		
			if(_data_cnt==9)
			{
				state = 0;
				_data_cnt = 0;

				sum =  (RxBuffer_Flow[2] + RxBuffer_Flow[3] + RxBuffer_Flow[4] + RxBuffer_Flow[5]);
				
				if((0xAA == data) && (sum == RxBuffer_Flow[6])) //��У��
				{

					user_flow_x = -( (s16)(*(RxBuffer_Flow+3)<<8)|*(RxBuffer_Flow+2) );
					user_flow_y = ( (s16)(*(RxBuffer_Flow+5)<<8)|*(RxBuffer_Flow+4) );
					mini_flow.x_i += user_flow_x*0.21f;
					mini_flow.y_i += user_flow_y*0.21f;
				}
			}
		break;
			
		default:	
			state = 0;
			_data_cnt = 0;
		break;
	}
}

struct _mini_flow_ mini_flow;//����������ݽṹ��
Butter_Parameter OpticalFlow_Parameter,Opt_vel_Parameter;
Butter_BufferData Buffer_OpticalFlow[2],Buffer_Opt_vel[2];
void OpticalFlow_Init()
{
  Set_Cutoff_Frequency(50, 20,&OpticalFlow_Parameter);//20
  Set_Cutoff_Frequency(50, 20,&Opt_vel_Parameter);//20
}

//���������˲��ںϼ��ٶȼ�
void mini_flow_Fix(void)
{

	////////////////////////*����λ�ƴ���*////////////////////////////
	//λ��һ�װ�����˹�˲�
	mini_flow.fix_x_i =LPButterworth(mini_flow.x_i,&Buffer_OpticalFlow[0],&OpticalFlow_Parameter);
	mini_flow.fix_y_i =LPButterworth(mini_flow.y_i,&Buffer_OpticalFlow[1],&OpticalFlow_Parameter);
//	//����λ�ƺʹ���������ں�
	mini_flow.ang_x += (-100.0f*tan(LIMIT(imu_data.rol,-15.0f,15.0f)*angle_to_rad1) - mini_flow.ang_x) *0.2f;
	mini_flow.ang_y += (-100.0f*tan(LIMIT(imu_data.pit,-15.0f,15.0f)*angle_to_rad1) - mini_flow.ang_y) *0.2f;

	//λ����ǶȻ�����ʵ��λ�Ʊ仯��Χ+-150��
	mini_flow.out_x_i = mini_flow.fix_x_i - mini_flow.ang_x;
	mini_flow.out_y_i = mini_flow.fix_y_i - mini_flow.ang_y;
	
	////////////////////////*΢���ٶȴ���*///////////////////////////////
	
	//΢�����ٶȣ�ʵ���ٶ�ֵ+-500��
	mini_flow.x = (mini_flow.out_x_i - mini_flow.out_x_i_o)/0.01f;	
	mini_flow.out_x_i_o = mini_flow.out_x_i;
	mini_flow.y = (mini_flow.out_y_i - mini_flow.out_y_i_o)/0.01f;	
	mini_flow.out_y_i_o = mini_flow.out_y_i;

	//��ͨ�˲�
	mini_flow.fix_x += ( mini_flow.x - mini_flow.fix_x ) * 0.2f;
	mini_flow.fix_y += ( mini_flow.y - mini_flow.fix_y ) * 0.2f;

	//mini_flow_reset();
}
void mini_flow_reset(void)
{

	mini_flow.x = 0;
	mini_flow.y = 0;
	mini_flow.x_i = 0;
	mini_flow.y_i = 0;
	mini_flow.ang_x = 0;
	mini_flow.ang_y = 0;
	mini_flow.fix_x_i = 0;
	mini_flow.fix_y_i = 0;
	mini_flow.fix_x = 0;
	mini_flow.fix_y = 0;
//	ct_loc_out[X]=ct_loc_out[Y]= 0;
}
#define M_PI_F 3.141592653589793f
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF)
{
  float fr = sample_frequent / cutoff_frequent;
  float ohm = tanf(M_PI_F / fr);
  float c = 1.0f + 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm;
  if (cutoff_frequent <= 0.0f) {
    // no filtering
    return;
  }
  LPF->b[0] = ohm * ohm / c;
  LPF->b[1] = 2.0f * LPF->b[0];
  LPF->b[2] = LPF->b[0];
  LPF->a[0]=1.0f;
  LPF->a[1] = 2.0f * (ohm * ohm - 1.0f) / c;
  LPF->a[2] = (1.0f - 2.0f * cosf(M_PI_F / 4.0f) * ohm + ohm * ohm) / c;
}

/*************************************************
������:	float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
˵��:	���ٶȼƵ�ͨ�˲���
���:	float curr_input ��ǰ������ٶȼ�,�˲����������˲�������
����:	��
��ע:	2��Butterworth��ͨ�˲���
*************************************************/
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter)
{
  /* ���ٶȼ�Butterworth�˲� */
  /* ��ȡ����x(n) */
  static int LPF_Cnt=0;
  Buffer->Input_Butter[2]=curr_input;
  if(LPF_Cnt>=100)
  {
    /* Butterworth�˲� */
    Buffer->Output_Butter[2]=
      Parameter->b[0] * Buffer->Input_Butter[2]
        +Parameter->b[1] * Buffer->Input_Butter[1]
          +Parameter->b[2] * Buffer->Input_Butter[0]
            -Parameter->a[1] * Buffer->Output_Butter[1]
              -Parameter->a[2] * Buffer->Output_Butter[0];
  }
  else
  {
    Buffer->Output_Butter[2]=Buffer->Input_Butter[2];
    LPF_Cnt++;
  }
  /* x(n) ���б��� */
  Buffer->Input_Butter[0]=Buffer->Input_Butter[1];
  Buffer->Input_Butter[1]=Buffer->Input_Butter[2];
  /* y(n) ���б��� */
  Buffer->Output_Butter[0]=Buffer->Output_Butter[1];
  Buffer->Output_Butter[1]=Buffer->Output_Butter[2];
  return Buffer->Output_Butter[2];
}
float Flow_Speed_Filter(float ImuSpeed,float FlowSpeed)
{     
    static double Q=0.01f; 
    static double R=1.5f; 

    double dt=0.008f; 
   
    static double Speed=0.0f;
    static double Q_bias=0.0, Speed_err=0;
    static double PCt_0=0, PCt_1=0, E=0;
    static double K_0=0, K_1=0, t_0=0.004944f, t_1=-0.00000089f; 
    double Pdot[4] ={0,0,0,0}; 
    static double PP[2][2] = { { 0.0004868f, -0.000001f},{ -0.000001f, 0.000002f } };
    
    //��һ��loopֱ�Ӹ�����ѹֵ
    static u8 first_flag=1;
    if(first_flag)
    {
        first_flag=0;
        Speed=FlowSpeed;
    }
    
    Speed += (ImuSpeed - Q_bias); 
    
    Pdot[0]=Q - PP[0][1] - PP[1][0]; 
    Pdot[1] = -PP[1][1]; 
    Pdot[2] = -PP[1][1]; 
    Pdot[3] = 0; 

    PP[0][0] += Pdot[0] * dt; 
    PP[0][1] += Pdot[1] * dt; 
    PP[1][0] += Pdot[2] * dt; 
    PP[1][1] += Pdot[3] * dt; 

    PCt_0 = PP[0][0];
    PCt_1 = PP[1][0]; 
    E = R + PCt_0;
    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E; 

    Speed_err = FlowSpeed - Speed; 
    Speed += K_0 * Speed_err; 
    Q_bias += K_1 * Speed_err; 

    t_0 = PCt_0; 
    t_1 = PP[0][1];
    PP[0][0] -= K_0 * t_0; 
    PP[0][1] -= K_0 * t_1; 
    PP[1][0] -= K_1 * t_0; 
    PP[1][1] -= K_1 * t_1; 
    
    return Speed; 
} 
