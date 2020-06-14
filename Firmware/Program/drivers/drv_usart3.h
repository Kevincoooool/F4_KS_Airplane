#ifndef __DRV_USART3_H__
#define __DRV_USART3_H__

#include "include.h"

extern u8 RxState3;

void USART3_Init(u32 br_num);
void USART3_DeInit(void);
void USART3_IRQ(void);

void USART3_Put_Char(unsigned char DataToSend);
void USART3_Put_String(unsigned char *Str);
void USART3_Put_Buf(unsigned char *DataToSend , u8 data_num);

#endif
