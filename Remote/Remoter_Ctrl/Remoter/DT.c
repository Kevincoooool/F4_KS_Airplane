#include "DT.h"

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	
u8 data_to_send[50];
int16_t PLANE_YAW,PLANE_ROL,PLANE_PIT;

void DT_Send_Data(u8 *dataToSend , u8 length)
{
	NRF_TxPacket(dataToSend,length);
}

void DT_Send_RCData(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	data_to_send[_cnt++]=BYTE1(Rc.THR);
	data_to_send[_cnt++]=BYTE0(Rc.THR);
	data_to_send[_cnt++]=BYTE1(Rc.YAW);
	data_to_send[_cnt++]=BYTE0(Rc.YAW);
	data_to_send[_cnt++]=BYTE1(Rc.ROL);
	data_to_send[_cnt++]=BYTE0(Rc.ROL);
	data_to_send[_cnt++]=BYTE1(Rc.PIT);
	data_to_send[_cnt++]=BYTE0(Rc.PIT);

	data_to_send[_cnt++]=BYTE1(Rc.AUX1);
	data_to_send[_cnt++]=BYTE0(Rc.AUX1);
	data_to_send[_cnt++]=BYTE1(Rc.AUX2);
	data_to_send[_cnt++]=BYTE0(Rc.AUX2);
	data_to_send[_cnt++]=BYTE1(Rc.AUX3);
	data_to_send[_cnt++]=BYTE0(Rc.AUX3);
	data_to_send[_cnt++]=BYTE1(Rc.AUX4);
	data_to_send[_cnt++]=BYTE0(Rc.AUX4);
	data_to_send[_cnt++]=BYTE1(Rc.AUX5);
	data_to_send[_cnt++]=BYTE0(Rc.AUX5);
	data_to_send[_cnt++]=BYTE1(Rc.AUX6);
	data_to_send[_cnt++]=BYTE0(Rc.AUX6);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	DT_Send_Data(data_to_send, _cnt);
}

void DT_Send_RCData_To_Pc(void)
{
	u8 _cnt=0;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0x03;
	data_to_send[_cnt++]=0;
	
	data_to_send[_cnt++]=BYTE1(Rc.THR);
	data_to_send[_cnt++]=BYTE0(Rc.THR);
	data_to_send[_cnt++]=BYTE1(Rc.YAW);
	data_to_send[_cnt++]=BYTE0(Rc.YAW);
	data_to_send[_cnt++]=BYTE1(Rc.ROL);
	data_to_send[_cnt++]=BYTE0(Rc.ROL);
	data_to_send[_cnt++]=BYTE1(Rc.PIT);
	data_to_send[_cnt++]=BYTE0(Rc.PIT);

	data_to_send[_cnt++]=BYTE1(Rc.AUX1);
	data_to_send[_cnt++]=BYTE0(Rc.AUX1);
	data_to_send[_cnt++]=BYTE1(Rc.AUX2);
	data_to_send[_cnt++]=BYTE0(Rc.AUX2);
	data_to_send[_cnt++]=BYTE1(Rc.AUX3);
	data_to_send[_cnt++]=BYTE0(Rc.AUX3);
	data_to_send[_cnt++]=BYTE1(Rc.AUX4);
	data_to_send[_cnt++]=BYTE0(Rc.AUX4);
	data_to_send[_cnt++]=BYTE1(Rc.AUX5);
	data_to_send[_cnt++]=BYTE0(Rc.AUX5);
	data_to_send[_cnt++]=BYTE1(Rc.AUX6);
	data_to_send[_cnt++]=BYTE0(Rc.AUX6);
	
	data_to_send[3] = _cnt-4;
	
	u8 sum = 0;
	for(u8 i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	Usb_Hid_Adddata(data_to_send, _cnt);
	Usb_Hid_Send();
}

void DT_NrfData_Anl(u8 *data_buf , u8 num)
{
	u8 sum = 0;
	
	for(u8 i=0;i<(num-1);i++)
		sum += *(data_buf+i);
	if(!(sum==*(data_buf+num-1)))		return;		//判断sum
	if(!(*(data_buf)==0xAA && *(data_buf+1)==0x05))		return;		//判断帧头
	
		if(*(data_buf+3)==0x01)//读取姿态角
		{
			Show.X = ( (int16_t)(*(data_buf+5)<<8)|*(data_buf+6) )*0.1;
			Show.Y = ( (int16_t)(*(data_buf+7)<<8)|*(data_buf+8) )*0.1;
			Show.Z = ( (int16_t)(*(data_buf+9)<<8)|*(data_buf+10) )*0.1;
			Show.H = ( (int32_t)(*(data_buf+11)<<24)|(*(data_buf+12)<<16)|(*(data_buf+13)<<8)|*(data_buf+14) )*0.1;
		}
		if(*(data_buf+2)==0x05)//读取飞控电压等
		{
			Show.Battery_Fly = ( (vs16)(*(data_buf+4)<<8)|*(data_buf+5) );
			//Show.Battery_Rc  = ( (vs16)(*(data_buf+6)<<8)|*(data_buf+7) );
			Show.Rc_num = *(data_buf+9);
			Show.test_flag = ( (vs16)(*(data_buf+10)<<8)|*(data_buf+11) );
			Show.set_flag  = ( (vs16)(*(data_buf+12)<<8)|*(data_buf+13) );
			
			if(Show.set_flag==set_temp) send_flag = 0;
		}
}

void DT_Send_Flag_To_Fly(u16 flag0, u16 flag1)
{
	u8 _cnt=0,i;
	u8 sum = 0;
	u16 temp;
	
	data_to_send[_cnt++]=0xAA;
	data_to_send[_cnt++]=0xAF;
	data_to_send[_cnt++]=0xF1;
	data_to_send[_cnt++]=0;
	
	temp = flag0;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	temp = flag1;
	data_to_send[_cnt++]=BYTE1(temp);
	data_to_send[_cnt++]=BYTE0(temp);
	
	data_to_send[3] = _cnt-4;
	
	for(i=0;i<_cnt;i++)
		sum += data_to_send[i];
	
	data_to_send[_cnt++]=sum;
	
	DT_Send_Data(data_to_send, 32);
}
