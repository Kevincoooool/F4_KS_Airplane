#ifndef _KEY_H_
#define _KEY_H_
#include "stm32f10x.h"

/******************************************************************************
							ȫ�ֱ�������
*******************************************************************************/ 
extern uint8_t Mode,Fun;

#define	ROLL_Read		(GPIOA->IDR  & GPIO_Pin_1) //Mode������
#define	Mode_Read	(GPIOB->IDR  & GPIO_Pin_1) //Mode������
#define	Fun_Read	(GPIOB->IDR  & GPIO_Pin_9) //Mode������
/******************************************************************************
							ȫ�ֺ�������
*******************************************************************************/ 
void KEY_Init(void);
void key_function(void);

#endif
