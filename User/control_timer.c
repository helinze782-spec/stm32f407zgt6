#include "stm32f4xx.h"                  // Device header
#include "sys.h"

/**************************************************************************
Function: TIM7 init, 10ms interrupt
Input   : none
Output  : none
函数功能：TIM7初始化，10ms中断一次，给速度环提供节拍
入口参数：无
返回  值：无
**************************************************************************/
void Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_ClocksTypeDef RCC_Clocks;
	u32 Timer_Clock;
	u16 Prescaler;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);

	/* 获取TIM7实际定时器时钟，APB1预分频不为1时乘2 */
	RCC_GetClocksFreq(&RCC_Clocks);
	if(RCC_Clocks.PCLK1_Frequency==RCC_Clocks.HCLK_Frequency)
		Timer_Clock=RCC_Clocks.PCLK1_Frequency;
	else
		Timer_Clock=RCC_Clocks.PCLK1_Frequency*2;

	/* 先分频到1MHz，再数10000个数，就是10ms */
	Prescaler=(u16)(Timer_Clock/1000000-1);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler=Prescaler;
	TIM_TimeBaseStructure.TIM_Period=10000-1;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM7,ENABLE);
}
