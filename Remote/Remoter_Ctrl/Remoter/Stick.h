#ifndef __STICK_H__
#define __STICK_H__

#include "sysconfig.h"

struct _Rc
{
	int16_t THR;
	int16_t YAW;
	int16_t PIT;
	int16_t ROL;
	
	int16_t AUX1;
	int16_t AUX2;
	int16_t AUX3;
	int16_t AUX4;
	int16_t AUX5;
	int16_t AUX6;
};

#define Filter_Num 10

struct _Filter
{
	uint32_t sum;
	uint16_t old[Filter_Num];
};

extern struct _Filter Filter;
extern struct _Rc Rc;
extern uint8_t offset;

void Stick_Init(void);
void Stick_Scan(void);

#endif
