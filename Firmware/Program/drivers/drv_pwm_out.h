#ifndef _KS_MOTOR_H
#define	_KS_MOTOR_H

#include "stm32f4xx.h"
#include "Config.h"
void pwm_out_init(void);
void motor_out(s16 pwm[]);

#endif



