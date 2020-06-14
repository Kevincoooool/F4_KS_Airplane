#ifndef __DRV_UART5_H__
#define __DRV_UART5_H__

#include "include.h"

extern u8 RxState5;

void UART5_Init(u32 br_num);
void UART5_DeInit(void);
void UART5_IRQ(void);

void KS_UART5_Put_Char(unsigned char DataToSend);
void KS_UART5_Put_String(unsigned char *Str);
void KS_UART5_Put_Buf(unsigned char *DataToSend , u8 data_num);

#endif
