#include "stm32f4xx.h"                  // Device header
#include "sys.h"

/* 全局变量，调试时可以在Watch窗口直接看 */
int Motor_Left,Motor_Right;			//左右轮PWM（千分比，-1000~1000）
int Encoder_Left,Encoder_Right;			//左右轮编码器读数（每10ms）
float Encoder_Speed[MOTOR_COUNT];		//左右轮转速RPM
float Speed_Target[MOTOR_COUNT];		//目标转速RPM
float Speed_Measured[MOTOR_COUNT];		//滤波后的实际转速RPM
float Speed_Integral[MOTOR_COUNT];		//PI积分项
float Speed_Output[MOTOR_COUNT];		//速度环输出（千分比）
u8 Speed_Filter_Flag[MOTOR_COUNT];		//滤波初值标志
u8 Speed_Startup_Flag[MOTOR_COUNT];		//启动补偿状态
u16 Speed_Stall_Time[MOTOR_COUNT];		//连续低速计时ms
u8 Speed_Stall_Fault[MOTOR_COUNT];		//堵转保护锁存

static volatile u32 timing_delay;		//延时用的计数，中断和主循环都要访问

/**************************************************************************
Function: SysTick delay decrement
Input   : none
Output  : none
函数功能：SysTick中断里调用，延时计数减1
入口参数：无
返回  值：无
**************************************************************************/
void TimingDelay_Decrement(void)
{
	if(timing_delay!=0)
		timing_delay--;
}

/**************************************************************************
Function: Delay in ms
Input   : milliseconds
Output  : none
函数功能：毫秒级延时，靠SysTick中断计数
入口参数：milliseconds：延时毫秒数
返回  值：无
**************************************************************************/
void Delay_ms(u32 milliseconds)
{
	timing_delay=milliseconds;
	while(timing_delay!=0);
}

/**************************************************************************
Function: Main
Input   : none
Output  : none
函数功能：主函数，初始化电机、编码器和速度环，然后进死循环
入口参数：无
返回  值：无
**************************************************************************/
int main(void)
{
	/* 统一中断优先级分组，方便后续添加MPU6050等中断 */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* SysTick每1ms中断一次 */
	SysTick_Config(SystemCoreClock/1000);
	Delay_ms(100);

	/* 初始化电机、编码器和速度环 */
	Motor_Init();
	Encoder_Init_TIM2();				//左轮编码器
	Encoder_Init_TIM3();				//右轮编码器
	Speed_Init();

#if Motor_Test_Mode
	/* ---------- 开路测试：不跑速度环，直接给固定PWM ---------- */
#if   Motor_Test_Mode==1
	Set_PWM(Test_PWM,Test_PWM);			//双轮正转
#elif Motor_Test_Mode==2
	Set_PWM(-Test_PWM,-Test_PWM);			//双轮反转
#elif Motor_Test_Mode==3
	Set_PWM(Test_PWM,0);				//只左轮正转
#elif Motor_Test_Mode==4
	Set_PWM(0,Test_PWM);				//只右轮正转
#elif Motor_Test_Mode==5
	/* PWM每1秒加100‰，看第几秒轮子开始转，起步占空比≈秒数×100‰ */
	{
		u16 pwm=0,ms=0;

		while(1)
		{
			Set_PWM(pwm,pwm);
			Encoder_Left=Encoder_Sign_1*Read_Encoder(2);	//左轮
			Encoder_Right=Encoder_Sign_2*Read_Encoder(3);	//右轮
			Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);
			Delay_ms(10);
			if(++ms>=100)
			{
				ms=0;
				if(pwm<Test_PWM) pwm+=100;
			}
		}
	}
#elif Motor_Test_Mode==6
	/*
	 * 编码器自检（不用调试器）：
	 * 电机停着，每轮给你1秒时间用手转轮子；
	 * 左编码器转到数 -> 左轮点动一下提示；右编码器转到数 -> 右轮点动一下提示。
	 * 手转了但对应轮子不点动，就说明那一路编码器没读数。
	 */
	{
		int cnt1,cnt2;

		while(1)
		{
			Set_PWM(0,0);
			Delay_ms(1500);					//等轮子停稳
			Read_Encoder(2);				//清掉旧计数
			Read_Encoder(3);
			Delay_ms(1000);					//这1秒内用手转轮子
			cnt1=Encoder_Sign_1*Read_Encoder(2);
			cnt2=Encoder_Sign_2*Read_Encoder(3);
			if((cnt1>3)||(cnt1<-3))				//左编码器有数
			{
				Set_PWM(Test_PWM,0);			//左轮点动提示
				Delay_ms(400);
				Set_PWM(0,0);
			}
			if((cnt2>3)||(cnt2<-3))				//右编码器有数
			{
				Set_PWM(0,Test_PWM);			//右轮点动提示
				Delay_ms(400);
				Set_PWM(0,0);
			}
		}
	}
#endif

#if (Motor_Test_Mode!=5)&&(Motor_Test_Mode!=6)
	while(1)
	{
		/* 每10ms读一次编码器，方便在Watch窗口看有没有计数 */
		Encoder_Left=Encoder_Sign_1*Read_Encoder(2);	//左轮
		Encoder_Right=Encoder_Sign_2*Read_Encoder(3);	//右轮
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);
		Delay_ms(10);
	}
#endif
#else
	/* ---------- 正常运行：速度环 ---------- */
	/* 设置两路电机的目标转速 */
	Set_Target_Speed(0,Target_RPM_1);		//左轮
	Set_Target_Speed(1,Target_RPM_2);		//右轮

	/* TIM7中断负责10ms速度环，主循环留给OLED等慢任务 */
	Timer_Init();

	while(1)
	{
		/* 以后在这里添加OLED、按键、串口等任务 */
	}
#endif
}
