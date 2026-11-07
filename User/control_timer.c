#include "stm32f4xx.h"
#include "control_timer.h"
#include "encoder.h"
#include "speed_control.h"

void Control_Timer_Init(void)
{
	TIM_TimeBaseInitTypeDef timer_init;
	NVIC_InitTypeDef nvic_init;
	RCC_ClocksTypeDef clocks;
	uint32_t timer_clock;
	uint16_t prescaler;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);

	/* 获取TIM7实际定时器时钟。APB1预分频不为1时，定时器时钟乘以2。 */
	RCC_GetClocksFreq(&clocks);
	if(clocks.PCLK1_Frequency==clocks.HCLK_Frequency)
		timer_clock=clocks.PCLK1_Frequency;
	else
		timer_clock=clocks.PCLK1_Frequency*2U;

	/* 先分频到1MHz，再每10000个计数产生一次中断，即10ms。 */
	prescaler=(uint16_t)(timer_clock/1000000U-1U);

	TIM_TimeBaseStructInit(&timer_init);
	timer_init.TIM_Prescaler=prescaler;
	timer_init.TIM_Period=10000U-1U;
	timer_init.TIM_ClockDivision=TIM_CKD_DIV1;
	timer_init.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7,&timer_init);

	TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);

	nvic_init.NVIC_IRQChannel=TIM7_IRQn;
	nvic_init.NVIC_IRQChannelPreemptionPriority=1;
	nvic_init.NVIC_IRQChannelSubPriority=1;
	nvic_init.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&nvic_init);

	TIM_Cmd(TIM7,ENABLE);
}

/* TIM7每10ms执行一次：先读编码器，再执行速度PI。 */
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);

		Encoder_Update(MOTOR_CONTROL_PERIOD_MS);
		SpeedControl_Update((float)MOTOR_CONTROL_PERIOD_MS/1000.0f);
	}
}
