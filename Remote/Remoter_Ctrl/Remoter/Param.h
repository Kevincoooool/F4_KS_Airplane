#ifndef __PARAM_H
#define __PARAM_H

#include "stm32f10x.h"
#include "Drv_Flash.h"

struct param{
	uint8_t  NRF_Channel;
	uint16_t OffSet_Rol;
	uint16_t OffSet_Pit;
	uint16_t OffSet_Thr;
	uint16_t OffSet_Yaw;
	uint16_t FirstInitFlag;
};
extern struct param Param;

void Param_READ(void);
void Param_SAVE(void);

#endif

