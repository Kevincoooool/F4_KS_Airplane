/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "sysconfig.h"
#include "Param.h"
#include "Drv_Flash.h"
uint8_t 	NRF_ENABLE = 0;
uint8_t 	UART1_ENABLE = 0;

int main(void)
{
	//中断优先级组别设置
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	//参数初始化
	Param_READ();
	
	//初始化USB
	USB_HID_Init();
	
	//初始化LED
	LED_Init();
	LED_0_FLASH();
	
	//初始化NRF所用SPI
	MY_SPI_Init();
	//检查NRF连接是否正常
	NRF_ENABLE = NRF_Check();
	
	//如果连接正常，则将NRF初始化为主发送模式
	if(NRF_ENABLE)	NRF_Init(MODEL_TX2,51);

	//初始化ADC采样
	ADC1_Init();
	
	KEY_Init();
	OLED_Init();
	
	Show.windows = 0;
	
	//初始化系统滴答定时器,2ms中断
	SysTick_Config(SystemCoreClock / 500);	
	
	while (1)
	{			 
		if(Show.oled_delay)
		{
			Show_Duty();
			Show.oled_delay=0;
		}
	}
}
