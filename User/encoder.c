#include "stm32f4xx.h"                  // Device header
#include "sys.h"

/**************************************************************************
Function: Encoder interface init (TIM2: PA0=A, PA1=B)
Input   : none
Output  : none
函数功能：左轮编码器接口初始化，TIM2的PA0=A相、PA1=B相
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;		//复用
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;		//上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM2);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler=0;			//预分频器
	TIM_TimeBaseStructure.TIM_Period=ENCODER_TIM_PERIOD;	//自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);

	/* A/B两相都检测上升沿，四倍频 */
	TIM_EncoderInterfaceConfig(TIM2,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter=0x03;			//输入滤波
	TIM_ICInit(TIM2,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;
	TIM_ICInit(TIM2,&TIM_ICInitStructure);

	TIM_SetCounter(TIM2,0);
	TIM_Cmd(TIM2,ENABLE);
}

/**************************************************************************
Function: Encoder interface init (TIM3: PA6=A, PA7=B)
Input   : none
Output  : none
函数功能：右轮编码器接口初始化，TIM3的PA6=A相、PA7=B相
入口参数：无
返回  值：无
**************************************************************************/
void Encoder_Init_TIM3(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_TIM3);

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler=0;
	TIM_TimeBaseStructure.TIM_Period=ENCODER_TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);

	/* A/B两相都检测上升沿，四倍频 */
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity=TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection=TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter=0x03;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);

	TIM_ICInitStructure.TIM_Channel=TIM_Channel_2;
	TIM_ICInit(TIM3,&TIM_ICInitStructure);

	TIM_SetCounter(TIM3,0);
	TIM_Cmd(TIM3,ENABLE);
}

/**************************************************************************
Function: Read and clear encoder counter
Input   : TIMX: 2 or 3
Output  : counts in this period
函数功能：读编码器计数并清零，前进为正，后退为负
入口参数：TIMX：2（左轮）或3（右轮）
返回  值：本周期计数差
**************************************************************************/
int Read_Encoder(u8 TIMX)
{
	int Encoder_TIM;
	switch(TIMX)
	{
		case 2:  Encoder_TIM=(s16)TIM2->CNT;  TIM2->CNT=0; break;
		case 3:  Encoder_TIM=(s16)TIM3->CNT;  TIM3->CNT=0; break;
		default: Encoder_TIM=0;
	}
	return Encoder_TIM;
}

/**************************************************************************
Function: Convert encoder counts to RPM
Input   : encoder_left, encoder_right
Output  : none (result in Encoder_Speed[])
函数功能：编码器读数换算成转速
入口参数：encoder_left：左轮计数  encoder_right：右轮计数
返回  值：无，结果存在Encoder_Speed[]里
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{
	/* 转速 = 计数差*60000/(每圈计数*采样时间ms) */
	Encoder_Speed[0]=(float)encoder_left*60000.0f/(Counts_Per_Rev*(float)MOTOR_CONTROL_PERIOD_MS);
	Encoder_Speed[1]=(float)encoder_right*60000.0f/(Counts_Per_Rev*(float)MOTOR_CONTROL_PERIOD_MS);
}
