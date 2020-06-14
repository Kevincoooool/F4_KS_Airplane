
#include "include.h"
#include "time.h"
#include "loop.h"
//初始化系统滴答定时器

volatile uint32_t usTicks = 0;
// 滴答定时器计数变量 ,49天后溢出
volatile uint32_t sysTickUptime = 0;
RCC_ClocksTypeDef rcc_clocks;

void cycleCounterInit(void)
{
    uint32_t         cnts;

    RCC_GetClocksFreq ( &rcc_clocks );

    cnts = ( uint32_t ) rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
    cnts = cnts / 8;

    SysTick_Config ( cnts );
    SysTick_CLKSourceConfig ( SysTick_CLKSource_HCLK_Div8 );
}

void SysTick_IRQ(void)
{
	sysTickUptime++;
	LED_1ms_DRV();
}

uint32_t GetSysTime_us(void) //两次获取若大于u32/1000(us),则两次差值溢出，不可取
{
    register uint32_t ms;
    u32 value;
	do
	{
		ms = sysTickUptime;
		value = ms * TICK_US + ( SysTick->LOAD - SysTick->VAL ) * TICK_US / SysTick->LOAD;
	}
	while(ms != sysTickUptime);
	return value;
}

void Delay_us(__IO u32 nTime)
{ 
	
	u32 now = GetSysTime_us();
	while(GetSysTime_us()-now<nTime);
}

void Delay_ms(__IO u32 nTime)
{ 
    while (nTime--)
        Delay_us(1000);
}

void Delay(vu32 nCount)
{
  for(; nCount!= 0;nCount--);
}

