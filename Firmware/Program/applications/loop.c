
#include "loop.h"
#include "IMU.h"
#include "KS_Data_Transfer.h"
#include "RC.h"
#include "Control.h"
#include "Voltage.h"
#include "height_ctrl.h"
#include "Face_track.h"
#include "MotionCal.h"
#include "WIFI_UFO.h"
#include "kalman3.h"
#include "kalmanVel.h"
float test_time[14],time_sum;
u16 test_time_cnt;

void Fc_Sensor_Get()//1ms
{
	static u8 cnt;
	if(sys_init_ok)
	{
		/*读取陀螺仪加速度计数据*/
		Drv_Icm20689_Read();
		cnt ++;
		cnt %= 10;
		if(cnt==0)
		{
			Height_Get(0.01f);
	
		}
	}	
	
}
void Duty_1ms()// 794us
{
	test_time[0] = GetSysTime_us();
	
	
	//Roll_3D();           //3D翻滚控制
	/*  传感器读取 陀螺仪 气压计*/
	Fc_Sensor_Get();
	/* 加速度角速度处理 */
	Sensor_Data_Prepare(1);
	
	Flight_State_Task(1,CH_N);
	/* 姿态解算      */
	IMU_Update_Task(1);
	WCZ_Acc_Get_Task();
	/* 控制*/
	CTRL_Duty((float)CH_N[CH_ROL],(float)CH_N[CH_PIT],(float)CH_N[CH_THR],(float)CH_N[CH_YAW]);//自稳控制
	NRF_Check_Event();//检测遥控数据
	if(NRF_Connect() == 0)
	{
		flag.nrf_ok  = 0;
	}
	KS_DT_Data_Exchange(); 			//数据交换
	
	VelocityEstimate();
	PositionEstimate();
	test_time[1] = GetSysTime_us()- test_time[0];	
}


void Duty_2ms()//406US
{
	
	test_time[2] = GetSysTime_us();	

	RC_duty(0.002f); 	//遥控数据处理
	Strapdown_INS_High(baro_height);
	Motor_Ctrl_Task(2);	
	
	test_time[3] = GetSysTime_us() - test_time[2];
}
void Duty_4ms()//406US
{
	
	test_time[4] = GetSysTime_us();	
	
	calculate_RPY();

	test_time[5] = GetSysTime_us() - test_time[4];
}
void Duty_10ms() //848US
{
	test_time[6] = GetSysTime_us();

	WCZ_Data_Calc(10);//高度数据融合
	High_Ctrl(CH_N[CH_THR],wcz_hei_fus.out,wcz_spe_fus.out);//定高控制
	
	Throw_Fly_Check();//抛飞检测
	mini_flow_Fix();
	
	test_time[7] = GetSysTime_us() - test_time[6];
}

void Duty_20ms()//18us
{
	test_time[8] = GetSysTime_us();

	/*位置控制*/
	Fixed_Point_Ctrl((float)CH_N[CH_ROL],(float)CH_N[CH_PIT]);
	LED_Duty(0.02f);	
	test_time[9] = GetSysTime_us() - test_time[8];
	
}

void Duty_50ms()//11us
{
	test_time[10] = GetSysTime_us();
	
	Static_Check();
//	voltage_check();
	
	test_time[11] = GetSysTime_us() - test_time[10];
}
u8 WIFI_SSI,Face_SSI;
int WIFI_SSI_CNT;//WIFI信号
void Duty_500ms()
{
	test_time[12] = GetSysTime_us();
	PID_Save_Overtime(1500,500);
//	WIFI_SSI = WIFI_SSI_CNT;//WIFI信号频率
//	WIFI_SSI_CNT = 0;
//	
//	if(!WIFI_SSI) flag.NS = 0;//失控保护
	Flag_Check();
//	Face_SSI = Face_SSI_CNT;//人脸信号频率
//	Face_SSI_CNT = 0;
	NRF_SSI = NRF_SSI_CNT;//NRF信号频率
	NRF_SSI_CNT = 0;
//	if(!Face_SSI ) Speed_state =1;
//	else Speed_state =1;
	test_time[13] = GetSysTime_us() - test_time[12];
	time_sum = 0;
	for(u8 i=0;i<7;i++)	time_sum += test_time[i];
}

//系统任务配置，创建不同执行周期的“线程”
static sched_task_t sched_tasks[] = 
{
	{Duty_500ms ,  500, 0},
	{Duty_50ms  ,   50, 0},
	{Duty_20ms  ,   20, 0},
	{Duty_10ms  ,   10, 0},
	{Duty_4ms   ,    5, 0},
	{Duty_2ms   ,    2, 0},
	{Duty_1ms   ,    1, 0},
	
};
//根据数组长度，判断线程数量
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))

//这个函数放到main函数的while(1)中，不停判断是否有线程应该执行
void Scheduler_Run(void)
{
	uint8_t index = 0;
	
	//循环判断所有线程，是否应该执行
	for(index=0;index < TASK_NUM;index++)
	{
		//获取系统当前时间，单位MS
		uint32_t tnow = sysTickUptime;
		//进行判断，如果当前时间减去上一次执行的时间，大于等于该线程的执行周期，则执行线程
		if(tnow - sched_tasks[index].last_run >= sched_tasks[index].interval_ticks)
		{
			//更新线程的执行时间，用于下一次判断
			sched_tasks[index].last_run = tnow;
			//执行线程函数，使用的是函数指针
			sched_tasks[index].task_func();
		}
	}
}


