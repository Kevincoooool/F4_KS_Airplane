#ifndef _include_H_
#define _include_H_

#include "math.h"
#include "stm32f4xx.h"
#include "mymath.h"
#include "stdint.h"
#include "loop.h"
#include "drv_usart1.h"
#include "drv_usart2.h"
#include "drv_usart3.h"
#include "drv_uart4.h"
#include "drv_uart5.h"
#include "drv_usart6.h"
#include "drv_spi.h"
#include "drv_nrf24l01.h"
#include "drv_led.h"
#include "drv_spl06.h"
#include "drv_icm20689.h"
//#include "drv_mpu6050.h"
#include "drv_adc.h"
#include "drv_esp8266.h"
#include "bsp_SysTick.h"
#include "malloc.h"
#include "MotionCal.h"
#include "RC.h"
#include "IMU.h"
#include "Control.h"
#include "Param.h"
#include "drv_pwm_out.h"
#include "filter.h"
#include "drv_i2c_soft.h"
#include "drv_spl06.h"
#include "opticalflow.h"
#include "ADRC.h"
#include "PID.h"
#include "RC.h"
#include "Data.h"
#include "height_ctrl.h"
#include "mymath.h"
#include "MotionCal.h"
#include "Alt_Ctrl.h"
#include "Loc_Ctrl.h"
#include "Face_track.h"
#include "mathTool.h"
#include "WIFI_UFO.h"
#include "Voltage.h"
#include "my_usb.h"
#include "AttCtrl.h"
#include "MotorCtrl.h"
#include "Navigate.h"
extern u8 sys_init_ok;

#define HW_TYPE	1
#define HW_VER	3
#define BL_VER	100
#define PT_VER	400

/***************LED GPIO定义******************/
#define RCC_LED_R				RCC_APB2Periph_GPIOB
#define GPIO_LED_R				GPIOB
#define Pin_LED_R				GPIO_Pin_14
#define RCC_LED_G				RCC_APB2Periph_GPIOB
#define GPIO_LED_G				GPIOB
#define Pin_LED_G				GPIO_Pin_13
#define RCC_LED_B				RCC_APB2Periph_GPIOB
#define GPIO_LED_B				GPIOB
#define Pin_LED_B				GPIO_Pin_12
/*********************************************/
#define GYRO_SPI             1              //陀螺仪SPI配置
#define GYRO_CS_GPIO         GPIOC
#define GYRO_CS_PIN          GPIO_Pin_5

#define BARO_SPI             1              //气压计SPI配置
#define BARO_CS_GPIO         GPIOC
#define BARO_CS_PIN          GPIO_Pin_4

#define NRF2401_SPI          3              //NRF SPI配置
/**********************************************************************************************************
*SPI引脚及参数配置
**********************************************************************************************************/
#define SPI1_GPIO_MOSI          GPIOA
#define SPI1_GPIO_MISO          GPIOA
#define SPI1_GPIO_SCK           GPIOA
#define SPI1_PINSOURCE_MOSI     GPIO_PinSource7
#define SPI1_PINSOURCE_MISO     GPIO_PinSource6
#define SPI1_PINSOURCE_SCK      GPIO_PinSource5
#define SPI1_PIN_MOSI           GPIO_Pin_7
#define SPI1_PIN_MISO           GPIO_Pin_6
#define SPI1_PIN_SCK            GPIO_Pin_5
#define SPI1_CLOCKDIV           SPI_BaudRatePrescaler_8

#define SPI2_GPIO_MOSI          GPIOC
#define SPI2_GPIO_MISO          GPIOC
#define SPI2_GPIO_SCK           GPIOC 
#define SPI2_PINSOURCE_MOSI     GPIO_PinSource12
#define SPI2_PINSOURCE_MISO     GPIO_PinSource11
#define SPI2_PINSOURCE_SCK      GPIO_PinSource10
#define SPI2_PIN_MOSI           GPIO_Pin_12
#define SPI2_PIN_MISO           GPIO_Pin_11
#define SPI2_PIN_SCK            GPIO_Pin_10
#define SPI2_CLOCKDIV           SPI_BaudRatePrescaler_8

#define SPI3_GPIO_MOSI          GPIOC
#define SPI3_GPIO_MISO          GPIOC
#define SPI3_GPIO_SCK           GPIOC 
#define SPI3_PINSOURCE_MOSI     GPIO_PinSource12
#define SPI3_PINSOURCE_MISO     GPIO_PinSource11
#define SPI3_PINSOURCE_SCK      GPIO_PinSource10
#define SPI3_PIN_MOSI           GPIO_Pin_12
#define SPI3_PIN_MISO           GPIO_Pin_11
#define SPI3_PIN_SCK            GPIO_Pin_10
#define SPI3_CLOCKDIV           SPI_BaudRatePrescaler_2
/***************UART1 GPIO定义******************/
#define RCC_USART1		RCC_AHB1Periph_GPIOA
#define GPIO_USART1		GPIOA
#define USART1_Pin_TX	GPIO_Pin_9
#define USART1_Pin_RX	GPIO_Pin_10
/*********************************************/
/***************UART1 GPIO定义******************/
#define RCC_USART2		RCC_AHB1Periph_GPIOA
#define GPIO_USART2		GPIOA
#define USART2_Pin_TX	GPIO_Pin_2
#define USART2_Pin_RX	GPIO_Pin_3
/*********************************************/
/***************UART3 GPIO定义******************/
#define RCC_USART3		RCC_APB1Periph_GPIOB
#define GPIO_USART3		GPIOB
#define USART3_Pin_TX	GPIO_Pin_10
#define USART3_Pin_RX	GPIO_Pin_11
/*********************************************/
/***************硬件中断优先级******************/
#define NVIC_USART1_P	1
#define NVIC_USART1_S	1

#define NVIC_USART2_P	2
#define NVIC_USART2_S	1

#define NVIC_USART3_P	3
#define NVIC_USART3_S	1

#define NVIC_UART4_P	3
#define NVIC_UART4_S	1

#define NVIC_UART5_P	3
#define NVIC_UART5_S	1

#define NVIC_USART6_P	4
#define NVIC_USART6_S	1


/***********************************************/


#endif


