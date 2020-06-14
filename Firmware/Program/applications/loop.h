#ifndef FUNCTION_H
#define FUNCTION_H

#include "stm32f4xx.h"

typedef struct
{
	void(*task_func)(void);
	uint16_t interval_ticks;//ִ��ʱ����
	uint32_t last_run;//���һ������ʱ��
}sched_task_t;

void Scheduler_Run(void);
#endif
