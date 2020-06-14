#ifndef __DRV_UART3_H__
#define __DRV_UART3_H__

#include "board.h"

extern u8 RxState3;

void UART3_Init(u32 br_num);
void UART3_IRQ(void);

void UART3_Put_Char(unsigned char DataToSend);
void UART3_Put_String(unsigned char *Str);
void UART3_Put_Buf(unsigned char *DataToSend , u8 data_num);

#endif
