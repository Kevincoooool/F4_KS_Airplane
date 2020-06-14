#ifndef _wifi_ufo_h_
#define _wifi_ufo_h_
#include "stm32f4xx.h"

extern u8 WIFI_SSI;
extern int WIFI_SSI_CNT;//WIFIÐÅºÅ;
void WIFI_UFO_Data_Receive_Prepare(u8 data);
void WIFI_UFO_Data_Receive_Anl(u8 *data_buf,u8 num);
extern u8 WIFI_UFO_Connect(void);

#endif
