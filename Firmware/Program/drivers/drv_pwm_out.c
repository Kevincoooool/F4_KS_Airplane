
#include "drv_pwm_out.h"
#include "mymath.h"

//21分频到 84000000/21 = 4M   0.25us

/*初始化高电平时间1000us（4000份）*/
#define INIT_DUTY 4000 //u16(1000/0.25)
/*频率400hz*/
#define HZ        18000
/*精度1000，每份0.25us*/
#define ACCURACY 1000 //u16(2500/0.25) //accuracy
/*设置飞控控制信号转换比例为4*/
#define PWM_RADIO 4//(8000 - 4000)/1000.0

void TIM_MOTO_GpioConfig(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOC, ENABLE); //GPIOA GPIOB 时钟使能 

	//GPIOB 配置: TIM3 CH1 (PA6), TIM3 CH2 (PA7), TIM3 CH3 (PB0) and TIM3 CH4 (PB2) 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; //复用模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //速度100M
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推完输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉输入
	//PA6 PA7 初始化 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	//PB0 PB1 初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
	GPIO_Init(GPIOA, &GPIO_InitStructure); 

	//连接 TIM3 的通道到 AF2 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5); 
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3); 
}

void pwm_out_init()
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	uint16_t PrescalerValue = 0;
	u32 hz_set = ACCURACY * HZ;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3|RCC_APB1Periph_TIM5, ENABLE); //TIM3 时钟使能

	TIM_MOTO_GpioConfig();

	hz_set = LIMIT ( hz_set, 1, 84000000 );
	/* Compute the prescaler value */
	PrescalerValue = ( uint16_t ) ( ( SystemCoreClock / 2 ) / hz_set ) - 1;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = ACCURACY;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	//PWM1 模式配置 通道1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//PWM1 模式配置 通道2
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_Cmd(TIM3,ENABLE); //使能TIM3


	TIM_TimeBaseStructure.TIM_Period = ACCURACY;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	//PWM1 模式配置 通道1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//PWM1 模式配置 通道3
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

	//PWM1 模式配置 通道4
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
	//ARR 寄存器预装载
	TIM_ARRPreloadConfig(TIM5, ENABLE);
	TIM_Cmd(TIM5,ENABLE); //使能TIM3
}

void motor_out(s16 pwm[MOTOR_NUM])
{
	u8 i;
	for(i=0;i<MOTOR_NUM;i++)
	{
		pwm[i] = LIMIT(pwm[i],0,1000);
	}
	
	TIM3->CCR1 = (u16)pwm[0] ; 
	TIM5->CCR2 = (u16)pwm[1] ;	
	TIM5->CCR1 = (u16)pwm[2] ; 
	TIM3->CCR2 = (u16)pwm[3] ;
	
}


