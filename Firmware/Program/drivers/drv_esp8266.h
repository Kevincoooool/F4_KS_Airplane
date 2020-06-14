#ifndef _ESP8266_H_

#include "stm32f4xx.h"

void Esp8266_Init(void);
void Esp8266_SetGpio0(uint8_t enable);
void Esp8266_SetEnable(uint8_t enable);

#endif
