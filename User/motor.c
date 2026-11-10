#include "stm32f4xx.h"                  // Device header
#include "sys.h"

static u16 PWM_Period[MOTOR_COUNT];		//两个定时器的ARR
static int Motor_Command[MOTOR_COUNT];		//上一次的PWM，用来判断换向

/**************************************************************************
Function: Get the real clock of the timer
Input   : TIMx: TIM9 or TIM10
Output  : Timer clock frequency
函数功能：获取定时器实际时钟，APB预分频不为1时定时器时钟要乘2
入口参数：TIMx：TIM9/TIM10
返回  值：定时器时钟频率
**************************************************************************/
static u32 Get_Timer_Clock(TIM_TypeDef* TIMx)
{
	RCC_ClocksTypeDef RCC_Clocks;
	u32 pclk;

	RCC_GetClocksFreq(&RCC_Clocks);

	if((TIMx==TIM9)||(TIMx==TIM10))
	{
		pclk=RCC_Clocks.PCLK2_Frequency;
		if(RCC_Clocks.PCLK2_Frequency==RCC_Clocks.HCLK_Frequency)
			return pclk;
		return pclk*2;
	}

	pclk=RCC_Clocks.PCLK1_Frequency;
	if(RCC_Clocks.PCLK1_Frequency==RCC_Clocks.HCLK_Frequency)
		return pclk;
	return pclk*2;
}

/**************************************************************************
Function: PWM timer init
Input   : TIMx: TIM9/TIM10, Period: auto reload value
Output  : none
函数功能：PWM定时器初始化
入口参数：TIMx：TIM9/TIM10  Period：自动重装值
返回  值：无
**************************************************************************/
static void PWM_Init(TIM_TypeDef* TIMx,u16 Period)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler=0;			//预分频器，不分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;	//向上计数
	TIM_TimeBaseStructure.TIM_Period=Period;		//自动重装值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	//时钟分割
	TIM_TimeBaseStructure.TIM_RepetitionCounter=0;
	TIM_TimeBaseInit(TIMx,&TIM_TimeBaseStructure);

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;		//PWM模式1
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;	//比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;	//输出极性高
	TIM_OCInitStructure.TIM_Pulse=0;
	TIM_OC1Init(TIMx,&TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIMx,TIM_OCPreload_Enable);	//CH1预装载使能
	TIM_ARRPreloadConfig(TIMx,ENABLE);			//ARR预装载使能
	TIM_SetCounter(TIMx,0);
	TIM_Cmd(TIMx,ENABLE);					//使能定时器
}

/**************************************************************************
Function: Motor init (direction pins, PWM pins, PWM timers)
Input   : none
Output  : none
函数功能：电机初始化，方向脚、PWM脚、PWM定时器
入口参数：无
返回  值：无
**************************************************************************/
void Motor_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	u16 Period;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOF,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9|RCC_APB2Periph_TIM10,ENABLE);

	/* 方向脚 PD0~PD3 推挽输出，上电默认全低 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	AIN1=0; AIN2=0; BIN1=0; BIN2=0;

	/* 左轮PWM：PA2 = TIM9_CH1 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM9);

	/* 右轮PWM：PF6 = TIM10_CH1 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_TIM10);

	/* 左轮定时器：ARR = 定时器时钟/PWM频率 - 1 */
	Period=(u16)(Get_Timer_Clock(TIM9)/MOTOR_PWM_FREQUENCY_HZ-1);
	PWM_Period[0]=Period;
	PWM_Init(TIM9,Period);

	/* 右轮定时器 */
	Period=(u16)(Get_Timer_Clock(TIM10)/MOTOR_PWM_FREQUENCY_HZ-1);
	PWM_Period[1]=Period;
	PWM_Init(TIM10,Period);
}

/**************************************************************************
Function: Output PWM to motor driver
Input   : Motor_left: left PWM, Motor_right: right PWM (-1000~1000)
Output  : none
函数功能：把左右轮PWM（千分比）写到定时器和方向脚
入口参数：Motor_left：左轮PWM  Motor_right：右轮PWM
返回  值：无
**************************************************************************/
void Set_PWM(int Motor_left,int Motor_right)
{
	if(Motor_left>1000)  Motor_left=1000;
	if(Motor_left<-1000) Motor_left=-1000;
	if(Motor_right>1000) Motor_right=1000;
	if(Motor_right<-1000)Motor_right=-1000;

	/* 左轮 */
	if(Motor_left==0)
	{
		AIN1=0; AIN2=0; PWMA=0;				//滑行
	}
	else
	{
		if(Motor_left*Motor_Command[0]<0) PWMA=0;	//换向前先撤掉PWM
		if(Motor_left>0) {AIN1=1; AIN2=0;}		//正转
		else		 {AIN1=0; AIN2=1;}		//反转
		PWMA=(u16)(((u32)myabs(Motor_left)*((u32)PWM_Period[0]+1U)+999U)/1000U);
	}
	Motor_Command[0]=Motor_left;

	/* 右轮 */
	if(Motor_right==0)
	{
		BIN1=0; BIN2=0; PWMB=0;
	}
	else
	{
		if(Motor_right*Motor_Command[1]<0) PWMB=0;
		if(Motor_right>0) {BIN1=1; BIN2=0;}
		else		  {BIN1=0; BIN2=1;}
		PWMB=(u16)(((u32)myabs(Motor_right)*((u32)PWM_Period[1]+1U)+999U)/1000U);
	}
	Motor_Command[1]=Motor_right;
}

/**************************************************************************
Function: Stop both motors
Input   : none
Output  : none
函数功能：两路电机滑行停车
入口参数：无
返回  值：无
**************************************************************************/
void Motor_StopAll(void)
{
	Set_PWM(0,0);
}

/**************************************************************************
Function: Absolute value
Input   : a
Output  : |a|
函数功能：取绝对值
入口参数：a：输入值
返回  值：绝对值
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0) {temp=-a;}
	else    {temp=a;}
	return temp;
}
