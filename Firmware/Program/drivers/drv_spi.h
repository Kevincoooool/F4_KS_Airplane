#ifndef __KS_DRV_SPI_H__
#define __KS_DRV_SPI_H__

#include "stm32f4xx.h"


void SPI_CE_H(void);
void SPI_CE_L(void);
void SPI_CSN_H(void);
void SPI_CSN_L(void);

void Spi_GPIO_Init(void);
void Spi_Open(uint8_t deviceNum);
uint8_t Spi_SingleWirteAndRead(uint8_t deviceNum, uint8_t dat);
void SPI_MultiWriteAndRead(uint8_t deviceNum, uint8_t *out, uint8_t *in, int len);

void Spi_GyroSingleWrite(uint8_t reg, uint8_t value);
void Spi_GyroMultiRead(uint8_t reg,uint8_t *data, uint8_t length);
void Spi_BaroSingleWrite(uint8_t reg, uint8_t value);
void Spi_BaroMultiRead(uint8_t reg,uint8_t *data, uint8_t length);
void Spi_BaroEnable(void);
void Spi_BaroDisable(void);
void Spi_GyroDisable(void);
#endif










