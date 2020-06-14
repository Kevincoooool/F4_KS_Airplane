#include "KS_Data_Transfer.h"
#include "WIFI_UFO.h"

#include "include.h"
void USART6_Init(u32 br_num)
{
   USART_InitTypeDef USART_InitStructure;
    //USART_ClockInitTypeDef USART_ClockInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd ( RCC_APB2Periph_USART6, ENABLE ); //开启USART2时钟
    RCC_AHB1PeriphClockCmd ( RCC_AHB1Periph_GPIOA, ENABLE );
   
    //串口中断优先级
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_USART6_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_USART6_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init ( &NVIC_InitStructure );


    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource11, GPIO_AF_USART6 );
    GPIO_PinAFConfig ( GPIOA, GPIO_PinSource12, GPIO_AF_USART6 );

    //配置PC12作为USART6　Tx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );
    //配置PD2作为USART6　Rx
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init ( GPIOA, &GPIO_InitStructure );

    //配置USART6
    //中断被屏蔽了
    USART_InitStructure.USART_BaudRate = br_num;       //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;   //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;    //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送、接收使能
    USART_Init ( USART6, &USART_InitStructure );



    //使能USART6接收中断
    USART_ITConfig ( USART6, USART_IT_RXNE, ENABLE );
    //使能USART5
    USART_Cmd ( USART6, ENABLE );
}
void USART6_DeInit(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	
	USART_DeInit(USART6);
	USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
}
u8 Tx6Buffer[256];
u8 Tx6Counter=0;
u8 count6=0;

void USART6_IRQ(void)
{
	//u8 com_data;

    //接收中断
    if ( USART_GetITStatus ( USART6, USART_IT_RXNE ) )
    {
        USART_ClearITPendingBit ( USART6, USART_IT_RXNE ); //清除中断标志

        //com_data = USART6->DR;
		//Mini_Flow_Receive(com_data);
       // WIFI_Data_Receive_Prepare ( com_data );
    }

    //发送（进入移位）中断
    if ((USART6->SR & (1<<7))&&(USART6->CR1 & USART_CR1_TXEIE) )
    {

        USART6->DR = Tx6Buffer[Tx6Counter++]; //写DR清除中断标志

        if ( Tx6Counter == count6 )
        {
            USART6->CR1 &= ~USART_CR1_TXEIE;		//关闭TXE（发送中断）中断
        }


        //USART_ClearITPendingBit(USART6,USART_IT_TXE);
    }

}

void KS_USART6_Put_Char(unsigned char DataToSend)
{
	Tx6Buffer[count6++] = DataToSend;
  //if(!(USART3->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART6, USART_IT_TXE, ENABLE); 
}
void KS_USART6_Put_String(unsigned char *Str)
{
	//判断Str指向的数据是否有效.
	while(*Str)
	{
	//是否是回车字符 如果是,则发送相应的回车 0x0d 0x0a
	if(*Str=='\r')KS_USART6_Put_Char(0x0d);
		else if(*Str=='\n')KS_USART6_Put_Char(0x0a);
			else KS_USART6_Put_Char(*Str);
	//指针++ 指向下一个字节.
	Str++;
	}
}
void KS_USART6_Put_Buf(unsigned char *DataToSend , u8 data_num)
{
	for(u8 i=0;i<data_num;i++)
		Tx6Buffer[count6++] = *(DataToSend+i);
	if(!(USART6->CR1 & USART_CR1_TXEIE))
		USART_ITConfig(USART6, USART_IT_TXE, ENABLE);  
}


