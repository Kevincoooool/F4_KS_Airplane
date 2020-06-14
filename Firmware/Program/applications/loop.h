#ifndef FUNCTION_H
#define FUNCTION_H

#include "stm32f4xx.h"

typedef struct
{
	void(*task_func)(void);
	uint16_t interval_ticks;//执行时间间隔
	uint32_t last_run;//最后一次运行时间
}sched_task_t;

void Scheduler_Run(void);
#endif
