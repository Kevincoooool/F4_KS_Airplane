#ifndef __KS_DRV_UART_H__
#define __KS_DRV_UART_H__

#include "include.h"

void USART1_Init(u32 br_num);
void USART1_DeInit(void);
void USART1_IRQ(void);
void USART1_Put_Char(unsigned char DataToSend);
void USART1_Put_String(unsigned char *Str);
void USART1_Put_Buf(unsigned char *DataToSend , u8 data_num);

#endif
