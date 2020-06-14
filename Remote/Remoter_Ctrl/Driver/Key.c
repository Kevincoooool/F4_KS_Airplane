
#include "sysconfig.h"

uint8_t Mode=1,Fun=0;//默认锁尾模式、非夜间模式
/******************************************************************************
函数原型:	void KEY_Init(void)
功　　能:	按键初始化
*******************************************************************************/ 
void KEY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA ,ENABLE);
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;	//MODE按键
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//上拉输入
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;	//MODE1按键
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//上拉输入
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;	//FUN按键
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;//上拉输入
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}


void key_function(void)
{
	static u8 count0=0,count1=0;
	
	Rc.AUX4 = 0;
	if(!Mode_Read)	Rc.AUX4 |= 0x01;//左边按键
	if(!Fun_Read)		Rc.AUX4 |= 0x02;//右边按键
	if(!ROLL_Read)		Rc.AUX4 |= 0x04;//右边按键

	
	///////////////////////////////////////////////////////
	if(Rc.AUX4&0x01)//右边按键
	{
		count0++;
		if(count0==200)//长按2s
		{
			Check_Ch = 1;
			LED_0_FLASH();
		}
		if(count0>=250)	count0=250;
	}
	else
	{
		if( count0>=2 && count0<=100 )
		{
			Show.windows++;
			if(Show.windows>2) Show.windows=0;
		}
		count0=0;
	}
	///////////////////////////////////////////////////////		
	if(Rc.AUX4&0x02)//左边按键
	{
		count1++;
		if(count1==200)//长按2s，摇杆中位校准
		{
			offset = 1;
			LED_0_FLASH();
		}
		if(count1>=250)	count1=250;
	}
	else
	{
		if( count1>=2 && count1<=100 ) 
		{

		}
		count1=0;
	}

}
