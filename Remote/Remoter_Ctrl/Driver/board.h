#ifndef __BOARD_H__
#define __BOARD_H__
#include "stm32f10x.h"
#include "Drv_Uart.h"
#include "Drv_Uart3.h"
#include "Drv_SPI.h"
#include "Drv_Nrf24l01.h"
#include "Drv_LED.h"
#include "Drv_hid.h"
#include "Drv_ADC.h"
#include "DT.h"

//#define GPIO_Remap_SWJ_JTAGDisable  ((uint32_t)0x00300200)  /*!< JTAG-DP Disabled and SW-DP Enabled */

#define HW_TYPE	1
#define HW_VER	3
#define BL_VER	100
#define PT_VER	400
/***************LED GPIO定义******************/
#define RCC_LED_0					RCC_APB2Periph_GPIOB
#define GPIO_LED_0				GPIOB
#define Pin_LED_0					GPIO_Pin_12
#define LED_Red_GPIO		      GPIOB
#define LED_Red_Pin		        GPIO_Pin_11
/*********************************************/
/***************I2C GPIO定义******************/
#define GPIO_I2C	GPIOB
#define I2C_Pin_SCL		GPIO_Pin_6
#define I2C_Pin_SDA		GPIO_Pin_7
#define RCC_I2C		RCC_APB2Periph_GPIOB
/*********************************************/
/***************UART1 GPIO定义******************/
#define RCC_UART1			RCC_APB2Periph_GPIOA
#define GPIO_UART1		GPIOA
#define UART1_Pin_TX	GPIO_Pin_9
#define UART1_Pin_RX	GPIO_Pin_10
/*********************************************/
/***************SPI GPIO定义******************/
#define GPIO_SPI		GPIOB
#define RCC_GPIO_SPI		RCC_APB2Periph_GPIOB
#define SPI_Pin_SCK			GPIO_Pin_13
#define SPI_Pin_MISO		GPIO_Pin_14
#define SPI_Pin_MOSI		GPIO_Pin_15

#define NRF_CE_GPIO		GPIOC		
#define NRF_CE_Pin		GPIO_Pin_14	
#define NRF_CSN_GPIO	GPIOC		
#define NRF_CSN_Pin		GPIO_Pin_13	
#define NRF_IRQ_GPIO	GPIOC
#define NRF_IRQ_Pin		GPIO_Pin_15

/*********************************************/
/***************硬件中断优先级******************/
#define NVIC_UART_P	5
#define NVIC_UART_S	1
/***********************************************/

void Delay(vu32 nCount);
void cycleCounterInit(void);
void SysTick_IRQ(void);

#endif 
