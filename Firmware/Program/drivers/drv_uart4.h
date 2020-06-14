#ifndef __KS_DRV_UART4_H__
#define __KS_DRV_UART4_H__

#include "include.h"

void UART4_Init(u32 br_num);
void KS_UART4_DeInit(void);
void UART4_IRQ(void);
void KS_UART4_Put_Char(unsigned char DataToSend);
void KS_UART4_Put_String(unsigned char *Str);
void KS_UART4_Put_Buf(unsigned char *DataToSend , u8 data_num);

#endif
