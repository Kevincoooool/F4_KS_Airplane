#ifndef _VL53L0X_H_
#define _VL53L0X_H_
#include "stm32f4xx.h"

extern u8 VL53L0X_LINKOK;
extern u16 tof_height_mm;

void Drv_Vl53_Init(void);
void Drv_Vl53_RunTask(void);

#endif
