
#include "Init.h"
#include "include.h"

u8 sys_init_ok = 0;
u32 CpuID;

//获取CPU唯一ID
void GetLockCode(void)
{
	CpuID = *(vu32*)(0x1FFF7A10);//低字节
}

void sys_init()
{
	//中断优先级组别设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	//初始化系统滴答定时器
	cycleCounterInit();
	Usb_Hid_Init();					//飞控usb接口的hid初始化
	Delay_ms(100);					//延时
	GetLockCode();
	USART1_Init(19200);
//	USART2_Init(19200);
//	USART3_Init(19200);
//	OpticalFlow_Init();
	//led初始化
	LED_Init();
	//初始化SPI
	Spi_GPIO_Init();
	Spi_Open(GYRO_SPI|BARO_SPI);
	Spi_Open(NRF2401_SPI);
	/*  NRF2401检测 */
	sens_hd_check.nrf_ok = NRF_Check();
	/* 如果连接正常，则将NRF初始化为RX2模式（高级接收） */
	if(sens_hd_check.nrf_ok)	NRF_Init(MODEL_RX2,51);
	Delay_ms(100);
	/*  陀螺仪检测  */
	sens_hd_check.gyro_ok = sens_hd_check.acc_ok = ICM20689_Detect();
	Drv_Icm20689Reg_Init();
	Delay_ms(100);
	/*  气压计检测  */
	sens_hd_check.baro_ok = Drv_Spl0601_Init();	
	/*  参数读取    */
	Param_Init();
	Param_Read();
	/*  输出PWM定时器*/
	pwm_out_init();
	/*  PID参数初始化*/
	All_PID_Init();
	pid_init();
	my_mem_init();
	NavigationInit();
	OpticalFlow_Init();
	/*  电池电量检测 */
//	Drv_AdcInit();	

	sys_init_ok = 1;	
}


