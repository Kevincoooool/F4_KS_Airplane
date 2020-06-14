#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

#include "stm32f10x.h"

void MY_SPI_Init(void);
u8 SPI_RW(u8 dat);
void SPI_CE_H(void);
void SPI_CE_L(void);
void SPI_CSN_H(void);
void SPI_CSN_L(void);

#endif










