
#include "WIFI_UFO.h"
#include "include.h"
#include "Data.h"
////////////////////////////////////////////////////
//以下是WIFI_UFO通信协议代码
////////////////////////////////////////////////////
static u8 RxBuffer[8];
static uint16_t WIFI_UFO_Err=1;
//u8 WIFI_SSI,WIFI_SSI_CNT;//WIFI信号
//WIFI_UFO数据接收函数
void WIFI_UFO_Data_Receive_Prepare(u8 data)
{
	static u8 _data_cnt = 0;
	static u8 state = 0;
	
	if(state==0&&data==0xcc)
	{
		state=1;
		RxBuffer[0]=data;
	}
	else if(state==1)
	{
		RxBuffer[++_data_cnt]=data;
	}
	
	if(_data_cnt==7)
	{
		state = 0;
		_data_cnt = 0;
		if(data == 0x33)
		WIFI_UFO_Err=0;
	}

}
static u8 last_key=0;
static void Key_Function(u8 key)
{

	if(key != last_key)//标志位判断
	{
		last_key = key;
			if (key & 0x01 && (fly_ready == 0))//一键起飞
			{
				fly_ready = 1;
				high_start = 90;
			}
			else if (fly_ready==1) auto_landing = 1;
			
			if ((key & 0x02) >> 1)//
			{
					No_Head_Mode = 1;
			}
			else No_Head_Mode = 0;
			
			if ((key & 0x04) >> 2)//
			{
					roll_en = 1;
			}
			else roll_en = 0;
			
			if ((((key & 0x20) >> 5) + ((key & 0x10) >> 4 ))== 0)//
			{
				
					//Speed_state = 1;
			}
			else if ((((key & 0x20) >> 5) + ((key & 0x10) >> 4 ))== 1)//
			{
					//Speed_state = 2;
			}
			else if ((((key & 0x20) >> 5) + ((key & 0x10) >> 4 ))== 2)//
			{
					//Speed_state = 3;
			}
			
			
			
	}
}
//WIFI_UFO数据处理函数
void WIFI_UFO_Data_Receive_Anl(u8 *data_buf,u8 num)
{
	flag.NS = 2;//遥控数据来源（1：遥控器。2：WiFi图传模块。3：蓝牙模块）

	//遥控数据
	RX_CH[CH_PIT] = ((float)*(data_buf+2)/256) * 1000 + 1000;
	RX_CH[CH_THR] = ((float)*(data_buf+3)/256) * 1000 + 1000;
	RX_CH[CH_YAW] = ((float)*(data_buf+4)/256) * 1000 + 1000;
	RX_CH[CH_ROL] = ((float)*(data_buf+1)/256) * 1000 + 1000;

	//标志位
	RX_CH[AUX1] = *(data_buf+5);
	Key_Function(RX_CH[AUX1]);
}
//WIFI_UFO连接函数
u8 WIFI_UFO_Connect(void)
{
	static u8 Connect_flag;
	
	WIFI_UFO_Err ++;
	if(WIFI_UFO_Err==1)
	{
		WIFI_UFO_Data_Receive_Anl(RxBuffer,8);
		WIFI_SSI_CNT++;
		Connect_flag = 1;
	}
	if(WIFI_UFO_Err>=500)
	{
		WIFI_UFO_Err = 1;
		Connect_flag = 0;
	}
	return Connect_flag;
}
