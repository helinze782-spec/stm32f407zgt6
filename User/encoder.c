#include "encoder.h"

typedef struct
{
	TIM_TypeDef *timer;		/* 编码器定时器 */
	GPIO_TypeDef *port_a;	/* A相端口 */
	uint16_t pin_a;			/* A相引脚 */
	uint8_t pin_a_source;
	GPIO_TypeDef *port_b;	/* B相端口 */
	uint16_t pin_b;			/* B相引脚 */
	uint8_t pin_b_source;
	uint8_t af;				/* 编码器复用功能 */
}EncoderHardwareTypeDef;

/*
 * 左轮编码器：PA0=A相，PA1=B相，TIM2。
 * 右轮编码器：PA6=A相，PA7=B相，TIM3。
 */
static const EncoderHardwareTypeDef encoder_hardware[MOTOR_COUNT]=
{
	{TIM2,GPIOA,GPIO_Pin_0,GPIO_PinSource0,GPIOA,GPIO_Pin_1,GPIO_PinSource1,GPIO_AF_TIM2},
	{TIM3,GPIOA,GPIO_Pin_6,GPIO_PinSource6,GPIOA,GPIO_Pin_7,GPIO_PinSource7,GPIO_AF_TIM3}
};

static uint16_t encoder_last_count[MOTOR_COUNT];
static float encoder_speed_rpm[MOTOR_COUNT];

static void Encoder_TimerInit(MotorId motor)
{
	TIM_TimeBaseInitTypeDef timer_init;
	TIM_ICInitTypeDef ic_init;
	const EncoderHardwareTypeDef *hardware=&encoder_hardware[motor];

	TIM_TimeBaseStructInit(&timer_init);
	timer_init.TIM_Prescaler=0;
	timer_init.TIM_CounterMode=TIM_CounterMode_Up;
	timer_init.TIM_Period=0xFFFFU;
	timer_init.TIM_ClockDivision=TIM_CKD_DIV1;
	timer_init.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(hardware->timer,&timer_init);

	/*
	 * A/B两相正交解码，两相都检测上升沿。
	 * 因此电机轴每转一圈得到13*4个计数。
	 */
	TIM_EncoderInterfaceConfig(hardware->timer,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);

	TIM_ICStructInit(&ic_init);
	ic_init.TIM_Channel=TIM_Channel_1;
	ic_init.TIM_ICPolarity=TIM_ICPolarity_Rising;
	ic_init.TIM_ICSelection=TIM_ICSelection_DirectTI;
	ic_init.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	ic_init.TIM_ICFilter=0x03U;
	TIM_ICInit(hardware->timer,&ic_init);

	ic_init.TIM_Channel=TIM_Channel_2;
	ic_init.TIM_ICPolarity=TIM_ICPolarity_Rising;
	ic_init.TIM_ICSelection=TIM_ICSelection_DirectTI;
	ic_init.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	ic_init.TIM_ICFilter=0x03U;
	TIM_ICInit(hardware->timer,&ic_init);

	TIM_SetCounter(hardware->timer,0);
	TIM_Cmd(hardware->timer,ENABLE);
}

void Encoder_Init(void)
{
	GPIO_InitTypeDef gpio_init;
	MotorId motor;

	/* 初始化两路编码器的GPIO和定时器。 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2|RCC_APB1Periph_TIM3,ENABLE);

	/* 初始化两个A/B相编码器输入。 */
	for(motor=MOTOR_ID_1;motor<MOTOR_COUNT;motor++)
	{
		const EncoderHardwareTypeDef *hardware=&encoder_hardware[motor];

		GPIO_StructInit(&gpio_init);
		gpio_init.GPIO_Pin=hardware->pin_a|hardware->pin_b;
		gpio_init.GPIO_Mode=GPIO_Mode_AF;
		gpio_init.GPIO_Speed=GPIO_Speed_100MHz;
		gpio_init.GPIO_OType=GPIO_OType_PP;
		gpio_init.GPIO_PuPd=GPIO_PuPd_UP;
		GPIO_Init(hardware->port_a,&gpio_init);
		GPIO_PinAFConfig(hardware->port_a,hardware->pin_a_source,hardware->af);
		GPIO_PinAFConfig(hardware->port_b,hardware->pin_b_source,hardware->af);

		/* 清空编码器累计状态。 */
		encoder_last_count[motor]=0;
		encoder_speed_rpm[motor]=0;
		Encoder_TimerInit(motor);
	}
}

void Encoder_Update(uint32_t elapsed_ms)
{
	MotorId motor;

	for(motor=MOTOR_ID_1;motor<MOTOR_COUNT;motor++)
	{
		const EncoderHardwareTypeDef *hardware=&encoder_hardware[motor];
		uint16_t current_count=TIM_GetCounter(hardware->timer);
		int32_t delta;

		/*
		 * 计算相对上次采样的计数差。
		 * 使用 int32_t 先做减法，再按 16 位回绕规则还原方向信息，避免
		 * 无符号->有符号转换和 0xFFFF/0x0001 这类边界情况的实现定义行为。
		 */
		delta=(int32_t)current_count-(int32_t)encoder_last_count[motor];
		if(delta>32767)
			delta-=65536;
		else if(delta<-32768)
			delta+=65536;

		encoder_last_count[motor]=current_count;

		/*
		 * RPM = 计数差 * 60000 / (采样时间ms * 输出轴一圈计数)。
		 * 输出轴一圈计数 = 13 * 4 * 60 = 3120。
		 */
		if(elapsed_ms!=0)
		{
			float counts_per_revolution=(float)ENCODER_PPR*(float)ENCODER_QUADRATURE_FACTOR*(float)ENCODER_GEAR_RATIO;
			encoder_speed_rpm[motor]=((float)delta*60000.0f)/(counts_per_revolution*(float)elapsed_ms);
		}
	}
}

float Encoder_GetSpeedRpm(MotorId motor)
{
	if(motor>=MOTOR_COUNT)
		return 0;
	return encoder_speed_rpm[motor];
}

