#ifndef __DT_H__
#define __DT_H__
#include "stm32f10x.h"
#include "Stick.h"

extern int16_t PLANE_YAW,PLANE_ROL,PLANE_PIT;

void DT_Send_RCData(void);
void DT_Send_RCData_To_Pc(void);
void DT_NrfData_Anl(u8 *data_buf , u8 num);
void DT_Send_Flag_To_Fly(u16 flag0, u16 flag1);
#endif
