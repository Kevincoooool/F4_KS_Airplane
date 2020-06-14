#ifndef __DRV_ADC_H
#define	__DRV_ADC_H

#include "include.h"

extern __IO uint16_t AdcValue;

extern __IO uint16_t ADC_ConvertedValue[];
void Drv_AdcInit(void);
#endif /* __ADC_H */


