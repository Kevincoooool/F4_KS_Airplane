
#include "drv_usart1.h"
#include "KS_Data_Transfer.h"
#include "WIFI_UFO.h"
#include "Face_track.h"
void USART1_Init(u32 br_num)
{
   USART_InitTypeDef USART_InitStructure;
    USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_USART1, ENABLE ); //开启USART1时钟
    RCC_AHB1PeriphClockCmd ( RCC_USART1, ENABLE );

    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_USART1_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_USART1_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );	
	
	
    GPIO_PinAFConfig ( GPIO_USART1, GPIO_PinSource9, GPIO_AF_USART1 );
    GPIO_PinAFConfig ( GPIO_USART1, GPIO_PinSource10, GPIO_AF_USART1 );

    //配置PD5作为USART1　Tx
    GPIO_InitStructure.GPIO_Pin = USART1_Pin_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIO_USART1, &GPIO_InitStructure );
    //配置PD6作为USART1　Rx
    GPIO_InitStructure.GPIO_Pin = USART1_Pin_RX ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIO_USART1, &GPIO_InitStructure );

    //配置USART1
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    //配置USART1时钟
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;  //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;  //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;  //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init ( USART1, &USART_InitStructure );
    USART_ClockInit ( USART1, &USART_ClockInitStruct );

    //使能USART1接收中断
    USART_ITConfig ( USART1, USART_IT_RXNE, ENABLE );
    //使能USART1
    USART_Cmd ( USART1, ENABLE );
}
void USART1_DeInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	USART_DeInit(USART1);
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}
u8 Tx1Buffer[256];
u8 Tx1Counter=0;
u8 count1=0;
void USART1_IRQHandler(void)
{
    u8 com_data;

    if ( USART1->SR & USART_SR_ORE ) //ORE中断
    {
       com_data = USART1->DR;
    }

    //接收中断
    if ( USART_GetITStatus ( USART1, USART_IT_RXNE ) )
    {
        
        com_data = USART1->DR;
        Mini_Flow_Receive ( com_data );
		USART_ClearITPendingBit ( USART1, USART_IT_RXNE ); //清除中断标志

    }
    //发送（进入移位）中断
    if ( USART_GetITStatus ( USART1, USART_IT_TXE ) )
    {
        USART1->DR = Tx1Buffer[Tx1Counter++]; //写DR清除中断标志
        if ( Tx1Counter == count1 )
        {
            USART1->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
        }
        USART_ClearITPendingBit(USART1,USART_IT_TXE);
    }



}
void USART1_Put_Char(unsigned char DataToSend)
{
	Tx1Buffer[count1++] = DataToSend;  
	//if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}
void USART1_Put_String(unsigned char *Str)
{
	//判断Str指向的数据是否有效.
	while(*Str)
	{
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='\r')USART1_Put_Char(0x0d);
		else if(*Str=='\n')USART1_Put_Char(0x0a);
			else USART1_Put_Char(*Str);
	//指针++ 指向下一个字节.
	Str++;
	}
}
void USART1_Put_Buf(unsigned char *DataToSend , u8 data_num)
{
	for(u8 i=0;i<data_num;i++)
		Tx1Buffer[count1++] = *(DataToSend+i);
	if(!(USART1->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE); 
}


