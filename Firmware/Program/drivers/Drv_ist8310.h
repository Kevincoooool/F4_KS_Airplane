#ifndef __IST8310_H
#define	__IST8310_H

#include "mathTool.h"
#include "include.h"
u8 IST8310_Detect(void);
void IST8310_Init(void);
void IST8310_Update(void);
void IST8310_Read(Vector3f_t* mag);
uint8_t IST8310_ReadReg(u8 REG_Address);
void Drv_IST8310_Read(void);
void Mag_Get(s16 mag_val[3]);
#endif


