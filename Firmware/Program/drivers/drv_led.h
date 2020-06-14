#ifndef __KS_DRV_LED_H__
#define __KS_DRV_LED_H__

#include "include.h"

void LED_Init(void);
#define LED_B_OFF         GPIO_LED->BSRRH = LED_B          //BSRRL   LEVEL_H
#define LED_B_ON          GPIO_LED->BSRRL = LED_B		//L
#define LED_G_ON          GPIO_LED->BSRRH = LED_G
#define LED_G_OFF         GPIO_LED->BSRRL = LED_G
#define LED_R_ON          GPIO_LED->BSRRH = LED_R
#define LED_R_OFF         GPIO_LED->BSRRL = LED_R

/***************LED GPIO∂®“Â******************/
#define RCC_LED			RCC_AHB1Periph_GPIOB
#define GPIO_LED		GPIOB
#define LED_R		GPIO_Pin_14
#define LED_G		GPIO_Pin_13
#define LED_B		GPIO_Pin_12

void LED_1ms_DRV(void);
void LED_Duty(float );

u8 led_breath(float,u16 T);
u8 led_flash(float dT,u16 group_n,u16 on_ms,u16 off_ms,u16 group_dT_ms);

extern u8 LED_warn;
#endif

