#ifndef __KS_RC_H
#define __KS_RC_H

#include "stm32f4xx.h"
#include "include.h"

#define BIT0	0x0001
#define BIT1	0x0002
#define BIT2	0x0004
#define BIT3	0x0008
#define BIT4	0x0010
#define BIT5	0x0020
#define BIT6	0x0040
#define BIT7	0x0080
#define BIT8	0x0100
#define BIT9	0x0200
#define BIT10 0x0400
#define BIT11 0x0800
#define BIT15 0x8000

extern u8 fly_ready,NS;
extern float turn_head;
extern u16 RX_CH[CH_NUM];
extern s16 CH_N[CH_NUM];
extern u16 test_flag,set_flag;

void Rc_Connect(void);
void RC_duty(float dT);
void Flag_Check(void);
void key_function(u8 key);
void fail_safe_check(void);
	
#endif

