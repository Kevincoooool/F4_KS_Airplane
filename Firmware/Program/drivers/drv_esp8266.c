#include "drv_esp8266.h"

void Esp8266_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_15;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Esp8266_SetGpio0(uint8_t enable)
{
	if(enable)
		GPIO_SetBits(GPIOA, GPIO_Pin_15);
	else
		GPIO_ResetBits(GPIOA, GPIO_Pin_15);	
}

void Esp8266_SetEnable(uint8_t enable)
{
	if(enable)
		GPIO_SetBits(GPIOB, GPIO_Pin_9);
	else
		GPIO_ResetBits(GPIOB, GPIO_Pin_9);	
}
