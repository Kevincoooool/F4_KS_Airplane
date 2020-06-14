
#include "Scheduler.h"
#include "sysconfig.h"

uint16_t S_cnt_2ms=0,S_cnt_4ms=0,S_cnt_10ms=0,S_cnt_20ms=0,S_cnt_50ms=0,S_cnt_1000ms=0;

void Loop_check(void)
{
	S_cnt_2ms++;
	S_cnt_4ms++;
	S_cnt_10ms++;
	S_cnt_20ms++;
	S_cnt_50ms++;
	S_cnt_1000ms++;
	
	Loop();
}

static void Loop_500Hz(void)	//2ms执行一次
{	
	NRF_Check_Event();//检查是否收到数据
	if(NRF_Evnet)//如果收到数据就把它发到上位机
	{
		Usb_Hid_Adddata(NRF24L01_2_RXDATA , RX_LEN);
		Usb_Hid_Send();
		NRF_Evnet = 0;
	}
	Stick_Scan();//扫描遥杆等数据
}

static void Loop_250Hz(void)	//4ms执行一次
{
	//如果收到上位机数据就发送到飞机，否则发送遥控数据
	if (USB_ReceiveFlg == TRUE)
	{
		if(Hid_RxData[0] < 33)
			NRF_TxPacket(&(Hid_RxData[1]),Hid_RxData[0]);
		USB_ReceiveFlg = 0x00;	
	}
	else
	{
		DT_Send_RCData();
	}
}

static void Loop_100Hz(void)	//10ms执行一次
{
	key_function();//按键检测
	Gesture_Check();//手势检测
	
	//如果与飞机连接失败就发送遥控数据到电脑
	if(!Show.Connect_Succeed)
	{
		DT_Send_RCData_To_Pc();
	}
}

static void Loop_50Hz(void)	//20ms执行一次
{
	Breath_LED(0.04);//呼吸灯控制
}

static void Loop_20Hz(void)	//50ms执行一次
{
	Show.oled_delay = 1;//屏幕显示延时
	NRF_Check_Ch();//自动对频检测
	
	//发送设置数据到飞机
	if(send_flag)	DT_Send_Flag_To_Fly(set_temp,0);
}

static void Loop_1Hz(void)
{
	//更新电压值
	Show.Battery_Rc = Rc.AUX5;
	
	//计算收到飞机数据的帧率
	NRF_SSI = NRF_SSI_CNT;
	Rc.AUX6 = NRF_SSI;
	NRF_SSI_CNT = 0;
	
	/*如果帧率为0标记为失联状态*/
	if(NRF_SSI==0)
	{
		LED_0_OFF();
		Show.Rc_num = 0;
		Show.Connect_Succeed = 0;
	}
	else
	{
		LED_0_ON();
		Show.Connect_Succeed = 1;
	}
}

void Loop(void)
{
	if(S_cnt_2ms >= 1){
		Loop_500Hz();
		S_cnt_2ms = 0;
	}		
	if(S_cnt_4ms >= 2){	
		Loop_250Hz();
		S_cnt_4ms = 0;
	}
	if(S_cnt_10ms >= 5){		
		Loop_100Hz();
		S_cnt_10ms = 0;
	}
	if(S_cnt_20ms >= 10){		
		Loop_50Hz();
		S_cnt_20ms = 0;
	}	
	if(S_cnt_50ms >= 25){		
		Loop_20Hz();
		S_cnt_50ms = 0;
	}	
	if(S_cnt_1000ms >= 500){		
		Loop_1Hz();
		S_cnt_1000ms = 0;
	}
}
/******************* (C) COPYRIGHT 2014 KS TECH *****END OF FILE************/
