#ifndef _wifi_ufo_h_
#define _wifi_ufo_h_
#include "stm32f4xx.h"
#include "WIFI_UFO.h"
#include "RC.h"
#include "KS_Data_Transfer.h"
#include "Control.h"
#include "Loc_Ctrl.h"
#include "IMU.h"
struct _face_track_
{
	float x;			//x�����ԭʼֵ
	float y;			//y�����ԭʼֵ
	
	float w_f;		//x���ٶ��ں�ֵ
	float h_f;		//y���ٶ��ں�ֵ
};
void face_loc_control(void);
extern float ct_face_out[VEC_XYZ];
extern struct _face_track_ face_track;
void Face_Data_Receive_Prepare(u8 data);
void Face_Data_Receive_Anl(u8 *data_buf);
void face_data_clear(void);
u8 Face_Connect(void);
extern uint16_t Face_Err;
extern uint8_t Face_SSI,Face_SSI_CNT;
#endif
