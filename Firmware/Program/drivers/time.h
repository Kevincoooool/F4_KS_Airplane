#ifndef _TIME_H
#define	_TIME_H

#include "stm32f4xx.h"

void TIM3_INIT(void);
void sys_time(void);
u16 Get_Time(u8,u16,u16);
extern s16 time_1h,time_1m,time_1s,time_1ms;
#endif



