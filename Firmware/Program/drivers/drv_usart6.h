#ifndef __DRV_USART6_H__
#define __DRV_USART6_H__

#include "include.h"

extern u8 RxState6;

void USART6_Init(u32 br_num);
void USART6_DeInit(void);
void USART6_IRQ(void);

void KS_USART6_Put_Char(unsigned char DataToSend);
void KS_USART6_Put_String(unsigned char *Str);
void KS_USART6_Put_Buf(unsigned char *DataToSend , u8 data_num);

#endif
