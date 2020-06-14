#include "stm32f4xx_it.h"
#include "include.h"
//void USART1_IRQHandler(void)  //串口中断函数
//{
//	USART1_IRQ();
//}

//void USART2_IRQHandler(void)  //串口中断函数
//{
//	USART2_IRQ();
//}
//void USART3_IRQHandler(void)  //串口中断函数
//{
//	USART3_IRQ();
//}
//void UART4_IRQHandler(void)  //串口中断函数
//{
//	UART4_IRQ();
//}
//void UART5_IRQHandler(void)  //串口中断函数
//{
//	UART5_IRQ();
//}

//void USART6_IRQHandler(void)  //串口中断函数
//{
//	USART6_IRQ();
//}
void SysTick_Handler(void)
{
	SysTick_IRQ();
}

void HardFault_Handler(void)
{
	while(1)
	{
	
	}

}


