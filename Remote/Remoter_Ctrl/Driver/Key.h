#ifndef _KEY_H_
#define _KEY_H_
#include "stm32f10x.h"

/******************************************************************************
							全局变量声明
*******************************************************************************/ 
extern uint8_t Mode,Fun;

#define	ROLL_Read		(GPIOA->IDR  & GPIO_Pin_1) //Mode读数据
#define	Mode_Read	(GPIOB->IDR  & GPIO_Pin_1) //Mode读数据
#define	Fun_Read	(GPIOB->IDR  & GPIO_Pin_9) //Mode读数据
/******************************************************************************
							全局函数声明
*******************************************************************************/ 
void KEY_Init(void);
void key_function(void);

#endif
