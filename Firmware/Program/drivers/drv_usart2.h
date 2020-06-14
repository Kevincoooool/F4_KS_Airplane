#ifndef __KS_DRV_USART2_H__
#define __KS_DRV_USART2_H__

#include "include.h"

void USART2_Init(u32 br_num);
void USART2_DeInit(void);
void USART2_IRQ(void);
void USART2_Put_Char(unsigned char DataToSend);
void USART2_Put_String(unsigned char *Str);
void USART2_Put_Buf(unsigned char *DataToSend , u8 data_num);

#endif
