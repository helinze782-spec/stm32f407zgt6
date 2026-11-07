#include "motor.h"

typedef struct
{
	TIM_TypeDef *timer;			/* PWM定时器 */
	GPIO_TypeDef *pwm_port;		/* PWM引脚端口 */
	uint16_t pwm_pin;			/* PWM引脚 */
	uint8_t pwm_pin_source;		/* PWM复用功能编号 */
	uint8_t pwm_af;				/* PWM复用功能 */
	GPIO_TypeDef *in1_port;		/* L298N方向脚1 */
	uint16_t in1_pin;
	GPIO_TypeDef *in2_port;		/* L298N方向脚2 */
	uint16_t in2_pin;
}MotorHardwareTypeDef;

/*
 * M1：PA2输出PWM，PD0/PD1控制方向。
 * M2：PF6输出PWM，PD2/PD3控制方向。
 */
static const MotorHardwareTypeDef motor_hardware[MOTOR_COUNT]=
{
	{TIM9,GPIOA,GPIO_Pin_2,GPIO_PinSource2,GPIO_AF_TIM9,GPIOD,GPIO_Pin_0,GPIOD,GPIO_Pin_1},
	{TIM10,GPIOF,GPIO_Pin_6,GPIO_PinSource6,GPIO_AF_TIM10,GPIOD,GPIO_Pin_2,GPIOD,GPIO_Pin_3}
};

static uint16_t motor_pwm_period[MOTOR_COUNT];
static int16_t motor_command[MOTOR_COUNT];

/* 获取定时器实际时钟。APB预分频不为1时，定时器时钟需要乘以2。 */
static uint32_t Motor_GetTimerClock(TIM_TypeDef *timer)
{
	RCC_ClocksTypeDef clocks;
	uint32_t pclk;

	RCC_GetClocksFreq(&clocks);

	if((timer==TIM9)||(timer==TIM10))
	{
		pclk=clocks.PCLK2_Frequency;
		if(clocks.PCLK2_Frequency==clocks.HCLK_Frequency)
			return pclk;
		return pclk*2U;
	}

	pclk=clocks.PCLK1_Frequency;
	if(clocks.PCLK1_Frequency==clocks.HCLK_Frequency)
		return pclk;
	return pclk*2U;
}

static void Motor_SetCompare(MotorId motor,uint16_t compare)
{
	/* 把比较值写入对应的PWM定时器，直接改变占空比。 */
	if(motor>=MOTOR_COUNT)
		return;

	switch(motor)
	{
		case MOTOR_ID_1: TIM_SetCompare1(TIM9,compare); break;
		case MOTOR_ID_2: TIM_SetCompare1(TIM10,compare); break;
		default: break;
	}
}

static void Motor_SetDirection(MotorId motor,uint8_t forward)
{
	const MotorHardwareTypeDef *hardware;

	if(motor>=MOTOR_COUNT)
		return;

	hardware=&motor_hardware[motor];

	/* L298N方向：IN1=1、IN2=0正转；IN1=0、IN2=1反转。 */
	if(forward)
	{
		GPIO_SetBits(hardware->in1_port,hardware->in1_pin);
		GPIO_ResetBits(hardware->in2_port,hardware->in2_pin);
	}
	else
	{
		GPIO_ResetBits(hardware->in1_port,hardware->in1_pin);
		GPIO_SetBits(hardware->in2_port,hardware->in2_pin);
	}
}

static void Motor_PwmTimerInit(MotorId motor)
{
	TIM_TimeBaseInitTypeDef timer_init;
	TIM_OCInitTypeDef oc_init;
	const MotorHardwareTypeDef *hardware;
	uint32_t timer_clock;

	if(motor>=MOTOR_COUNT)
		return;

	hardware=&motor_hardware[motor];
	timer_clock=Motor_GetTimerClock(hardware->timer);

	/*
	 * ARR = 定时器时钟 / PWM频率 - 1。
	 * F407主频168MHz时，TIM9/TIM10的PWM周期约为25kHz。
	 */
	motor_pwm_period[motor]=(uint16_t)((timer_clock/MOTOR_PWM_FREQUENCY_HZ)-1U);

	TIM_TimeBaseStructInit(&timer_init);
	timer_init.TIM_Prescaler=0;
	timer_init.TIM_CounterMode=TIM_CounterMode_Up;
	timer_init.TIM_Period=motor_pwm_period[motor];
	timer_init.TIM_ClockDivision=TIM_CKD_DIV1;
	timer_init.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(hardware->timer,&timer_init);

	/* PWM1模式，计数小于比较值时输出高电平。 */
	TIM_OCStructInit(&oc_init);
	oc_init.TIM_OCMode=TIM_OCMode_PWM1;
	oc_init.TIM_OutputState=TIM_OutputState_Enable;
	oc_init.TIM_OCPolarity=TIM_OCPolarity_High;
	oc_init.TIM_Pulse=0;
	TIM_OC1Init(hardware->timer,&oc_init);
	TIM_OC1PreloadConfig(hardware->timer,TIM_OCPreload_Enable);
	TIM_ARRPreloadConfig(hardware->timer,ENABLE);
	TIM_SetCounter(hardware->timer,0);
	TIM_Cmd(hardware->timer,ENABLE);
}

void Motor_Init(void)
{
	GPIO_InitTypeDef gpio_init;
	MotorId motor;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOF,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9|RCC_APB2Periph_TIM10,ENABLE);

	/* 逐个初始化PWM复用脚和L298N方向脚。 */
	for(motor=MOTOR_ID_1;motor<MOTOR_COUNT;motor++)
	{
		const MotorHardwareTypeDef *hardware=&motor_hardware[motor];

		GPIO_StructInit(&gpio_init);
		gpio_init.GPIO_Pin=hardware->pwm_pin;
		gpio_init.GPIO_Mode=GPIO_Mode_AF;
		gpio_init.GPIO_Speed=GPIO_Speed_100MHz;
		gpio_init.GPIO_OType=GPIO_OType_PP;
		gpio_init.GPIO_PuPd=GPIO_PuPd_DOWN;
		GPIO_Init(hardware->pwm_port,&gpio_init);
		GPIO_PinAFConfig(hardware->pwm_port,hardware->pwm_pin_source,hardware->pwm_af);

		gpio_init.GPIO_Pin=hardware->in1_pin|hardware->in2_pin;
		gpio_init.GPIO_Mode=GPIO_Mode_OUT;
		gpio_init.GPIO_Speed=GPIO_Speed_2MHz;
		gpio_init.GPIO_OType=GPIO_OType_PP;
		gpio_init.GPIO_PuPd=GPIO_PuPd_DOWN;
		GPIO_Init(hardware->in1_port,&gpio_init);
		GPIO_ResetBits(hardware->in1_port,hardware->in1_pin);
		GPIO_ResetBits(hardware->in2_port,hardware->in2_pin);

		/* 上电默认停止，避免电机立即转动。 */
		motor_command[motor]=0;
		Motor_PwmTimerInit(motor);
	}
}

void Motor_SetSpeed(MotorId motor,int16_t speed_permille)
{
	uint16_t compare,magnitude;
	int16_t previous_command;

	if(motor>=MOTOR_COUNT)
		return;

	/* 速度指令范围限制为-1000~1000。 */
	if(speed_permille>1000)
		speed_permille=1000;
	else if(speed_permille<-1000)
		speed_permille=-1000;

	previous_command=motor_command[motor];
	motor_command[motor]=speed_permille;

	/* 速度为0时关闭输出，电机自由滑行。 */
	if(speed_permille==0)
	{
		Motor_Coast(motor);
		return;
	}

	/* 正数正转，负数反转。换向前先撤掉PWM，减小反向冲击。 */
	if(speed_permille>0)
	{
		magnitude=(uint16_t)speed_permille;
		if(previous_command<0)
			Motor_SetCompare(motor,0);
		Motor_SetDirection(motor,1);
	}
	else
	{
		magnitude=(uint16_t)(-speed_permille);
		if(previous_command>0)
			Motor_SetCompare(motor,0);
		Motor_SetDirection(motor,0);
	}

	/* 把0~1000千分比换算成定时器比较值。 */
	compare=(uint16_t)(((uint32_t)magnitude*((uint32_t)motor_pwm_period[motor]+1U)+999U)/1000U);
	Motor_SetCompare(motor,compare);
}

void Motor_Coast(MotorId motor)
{
	const MotorHardwareTypeDef *hardware;

	if(motor>=MOTOR_COUNT)
		return;

	/* 滑行：PWM为0且两个方向输入都为低。 */
	hardware=&motor_hardware[motor];
	Motor_SetCompare(motor,0);
	GPIO_ResetBits(hardware->in1_port,hardware->in1_pin);
	GPIO_ResetBits(hardware->in2_port,hardware->in2_pin);
	motor_command[motor]=0;
}

void Motor_StopAll(void)
{
	MotorId motor;

	for(motor=MOTOR_ID_1;motor<MOTOR_COUNT;motor++)
		Motor_Coast(motor);
}

