#include "main.h"
#include "encoder.h"
#include "motor.h"
#include "speed_control.h"
#include "control_timer.h"

static __IO uint32_t timing_delay;

static void DelayMs(uint32_t milliseconds);

int main(void)
{
	/* 统一中断优先级分组，方便后续添加MPU6050等中断。 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* SysTick每1ms中断一次，用于10ms控制周期。 */
	SysTick_Config(SystemCoreClock/1000U);
	DelayMs(100);

	/* 初始化电机、编码器和速度环。 */
	Motor_Init();
	Encoder_Init();
	SpeedControl_Init();

	/* 设置两路电机的目标转速。 */
	SpeedControl_SetTargetRpm(MOTOR_ID_1,SPEED_CONTROL_TARGET_RPM_1);
	SpeedControl_SetTargetRpm(MOTOR_ID_2,SPEED_CONTROL_TARGET_RPM_2);

	/* TIM7中断负责10ms速度控制，主循环留给OLED等慢任务。 */
	Control_Timer_Init();

	while(1)
	{
		/*
		 * 后续在这里添加OLED、按键、串口等任务。
		 * 编码器和速度环已经在TIM7中断中执行。
		 */
	}
}

void TimingDelay_Decrement(void)
{
	if(timing_delay!=0)
		timing_delay--;
}

static void DelayMs(uint32_t milliseconds)
{
	timing_delay=milliseconds;
	while(timing_delay!=0);
}
