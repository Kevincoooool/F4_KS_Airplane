#include "Face_track.h"
static uint16_t Face_Err = 1;
uint8_t Face_SSI_CNT = 0;
static u8 RxBuffer_face[10];
void Face_Data_Receive_Prepare(u8 data)
{
	static u8 _data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0x88)
	{
		state=1;
		RxBuffer_face[0]=data;
	}

	else if(state==1)
	{
		RxBuffer_face[++_data_cnt]=data;
	}
	
	if(_data_cnt==10)
	{
		state = 0;
		_data_cnt = 0;
		if(data == 0x22)
			Face_Err=0;
		//Face_Data_Receive_Anl(RxBuffer_face);
	}
	

}

struct _face_track_ face_track;

void Face_Data_Receive_Anl(u8 *data_buf)
{
	//人脸位置数据
	face_track.x = ( (s16)(*(data_buf+2)<<8)|*(data_buf+1) );//人脸X
	face_track.y = ( (s16)(*(data_buf+4)<<8)|*(data_buf+3) );//人脸Y
	face_track.w_f = ( (s16)(*(data_buf+6)<<8)|*(data_buf+5) );//人脸大小
	face_track.h_f = ( (s16)(*(data_buf+8)<<8)|*(data_buf+7) );
}

float ct_face_out[VEC_XYZ];
void face_x_control(void)
{
	static float kp = 0.0005,kd = 0.0003;
	static float err = 0,last_err = 0;
	ct_face_out[X] = kp * face_track.x + (last_err - err)*kd;
	last_err = face_track.x;
}
void face_y_control(void)
{
	static float kp = 0.01,kd = 0.001;
	static float err = 0,last_err = 0;
	ct_face_out[Y] = kp * face_track.y + (last_err - err)*kd;
	last_err = face_track.y;
}
void face_h_control(void)
{
	static float kp = 0.01,kd = 0.001;
	static float err = 0,last_err = 0;
	ct_face_out[Z] = kp * (face_track.w_f - 50) + (last_err - err)*kd;
	last_err = face_track.w_f;
	
}
void face_loc_control(void)
{
	face_x_control();
	face_y_control();
	face_h_control();
}
void face_data_clear(void)
{
	face_track.x = 0;
	face_track.y = 0;
	face_track.w_f = 50;
	face_track.h_f = 50;
	ct_face_out[X]= 0;
	ct_face_out[Y]= 0;
	ct_face_out[Z]= 0;

}
u8 Face_Connect(void)
{
	static u8 Connect_flag;
	
	Face_Err ++;
	if(Face_Err==1)
	{
		Face_Data_Receive_Anl(RxBuffer_face);
		Face_SSI_CNT++;
		Connect_flag = 1;
	}
	if(Face_Err>=60)
	{
		Face_Err = 1;
		Connect_flag = 0;
	}
	return Connect_flag;
}
