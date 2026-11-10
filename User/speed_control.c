#include "stm32f4xx.h"                  // Device header
#include "sys.h"

/* 两个电机各自的启动补偿PWM */
static const float Start_PWM[MOTOR_COUNT]=
{
	Start_PWM_1,
	Start_PWM_2
};

/**************************************************************************
Function: Float limit
Input   : IN: input, max: upper, min: lower
Output  : limited value
函数功能：浮点限幅
入口参数：IN：输入值  max：上限  min：下限
返回  值：限幅后的值
**************************************************************************/
static float Speed_Limit(float IN,float max,float min)
{
	float OUT=IN;
	if(OUT>max) OUT=max;
	if(OUT<min) OUT=min;
	return OUT;
}

/**************************************************************************
Function: Absolute value
Input   : value
Output  : |value|
函数功能：取绝对值
入口参数：value：输入值
返回  值：绝对值
**************************************************************************/
static float Speed_Abs(float value)
{
	if(value<0) return -value;
	return value;
}

/**************************************************************************
Function: TIM7 interrupt, speed loop every 10ms
Input   : none
Output  : none
函数功能：TIM7中断，10ms一次，读编码器并做两路速度环
入口参数：无
返回  值：无
**************************************************************************/
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)!=RESET)
	{
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);

		/* 读两路编码器，前进为正 */
		Encoder_Left=Encoder_Sign_1*Read_Encoder(2);		//左轮
		Encoder_Right=Encoder_Sign_2*Read_Encoder(3);		//右轮
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);

		/* 两路速度环 */
		Motor_Left=Speed_Control(0);
		Motor_Right=Speed_Control(1);
		Motor_Left=PWM_Limit(Motor_Left,1000,-1000);
		Motor_Right=PWM_Limit(Motor_Right,1000,-1000);

		Set_PWM(Motor_Left,Motor_Right);
	}
}

/**************************************************************************
Function: Speed loop init
Input   : none
Output  : none
函数功能：速度环初始化，清状态并让电机停下
入口参数：无
返回  值：无
**************************************************************************/
void Speed_Init(void)
{
	u8 i;

	for(i=0;i<MOTOR_COUNT;i++)
	{
		Speed_Target[i]=0;
		Speed_Measured[i]=0;
		Speed_Integral[i]=0;
		Speed_Output[i]=0;
		Speed_Filter_Flag[i]=0;
		Speed_Startup_Flag[i]=0;
		Speed_Stall_Time[i]=0;
		Speed_Stall_Fault[i]=0;
	}

	Motor_StopAll();
}

/**************************************************************************
Function: Set target speed
Input   : motor: 0 left 1 right, rpm: target speed
Output  : none
函数功能：设置目标转速
入口参数：motor：0左轮 1右轮  rpm：目标转速
返回  值：无
**************************************************************************/
void Set_Target_Speed(u8 motor,float rpm)
{
	float old_target,new_target;

	if(motor>=MOTOR_COUNT)
		return;

	/* 目标转速限制在配置的上限内 */
	new_target=Speed_Limit(rpm,Target_Limit_RPM,-Target_Limit_RPM);

	/* 重新设置目标时解除堵转锁存 */
	if(Speed_Stall_Fault[motor]!=0)
	{
		Speed_Stall_Fault[motor]=0;
		Speed_Stall_Time[motor]=0;
		Speed_Startup_Flag[motor]=(new_target!=0);
		Speed_Integral[motor]=0;
	}

	/* 换向或者目标归零时清空积分，不清的话旧积分会把电机往原来方向拖 */
	old_target=Speed_Target[motor];
	if(((old_target>0)&&(new_target<0))||((old_target<0)&&(new_target>0))||(new_target==0))
	{
		Speed_Integral[motor]=0;
		Speed_Startup_Flag[motor]=(new_target!=0);
		Speed_Stall_Time[motor]=0;
		Speed_Stall_Fault[motor]=0;
	}
	else if((old_target==0)&&(new_target!=0))
	{
		Speed_Startup_Flag[motor]=1;
		Speed_Stall_Time[motor]=0;
		Speed_Stall_Fault[motor]=0;
	}

	Speed_Target[motor]=new_target;
}

/**************************************************************************
Function: Speed PI control
Input   : motor: 0 left 1 right
Output  : PWM (per mille, -1000~1000)
函数功能：速度环PI控制，10ms算一次
入口参数：motor：0左轮 1右轮
返回  值：PWM输出（千分比，-1000~1000）
**************************************************************************/
int Speed_Control(u8 motor)
{
	float raw_rpm;			//编码器直接算出来的转速
	float error_rpm;		//目标转速-实际转速
	float pi_output;		//PI输出，还没做启动补偿和限幅
	float output;			//最终输出
	u8 starting_up;			//启动补偿状态

	if(motor>=MOTOR_COUNT)
		return 0;

	/* 1.读编码器转速，一阶低通滤波压掉量化抖动 */
	raw_rpm=Encoder_Speed[motor];
	if(Speed_Filter_Flag[motor]==0)
	{
		Speed_Measured[motor]=raw_rpm;
		Speed_Filter_Flag[motor]=1;
	}
	else
	{
		Speed_Measured[motor]=Filter_Alpha*raw_rpm
		                     +(1.0f-Filter_Alpha)*Speed_Measured[motor];
	}

	/*
	 * 堵转保护：先滑行Stall_Retry_MS，然后自动清故障重试。
	 * 这样既能限制堵转时的持续电流，又不会因为一次误判就永久停机。
	 */
	if(Speed_Stall_Fault[motor]!=0)
	{
		Speed_Stall_Time[motor]+=(u16)(Control_Period*1000.0f);
		if(Speed_Stall_Time[motor]>=Stall_Retry_MS)
		{
			Speed_Stall_Fault[motor]=0;
			Speed_Stall_Time[motor]=0;
			Speed_Startup_Flag[motor]=(Speed_Target[motor]!=0);
			Speed_Integral[motor]=0;
		}
		Speed_Output[motor]=0;
		return 0;
	}

	/* 2.目标为0而且已经停下：滑行，本拍结束 */
	if((Speed_Target[motor]==0)&&(Speed_Abs(Speed_Measured[motor])<=Zero_Band_RPM))
	{
		Speed_Integral[motor]=0;
		Speed_Output[motor]=0;
		return 0;
	}

	/* 3.算误差 */
	error_rpm=Speed_Target[motor]-Speed_Measured[motor];

	/* 4.起步状态：目标不为0而且电机几乎没有转 */
	if(Speed_Target[motor]==0)
		Speed_Startup_Flag[motor]=0;
	else if(Speed_Startup_Flag[motor]==0)
	{
		if(Speed_Abs(Speed_Measured[motor])<Start_RPM)
			Speed_Startup_Flag[motor]=1;
	}
	starting_up=Speed_Startup_Flag[motor];

	/* 5.累加积分，起步阶段也积，让PI逐步接管启动PWM下限 */
	Speed_Integral[motor]+=Speed_Ki*error_rpm*Control_Period;
	Speed_Integral[motor]=Speed_Limit(Speed_Integral[motor],Integral_Limit,-Integral_Limit);

	/* 6.PI输出 */
	pi_output=(Speed_Kp*error_rpm)+Speed_Integral[motor];

	/* 7.启动补偿：起步阶段给PWM一个下限，帮它冲过静摩擦死区 */
	output=pi_output;
	if(starting_up!=0)
	{
		if((Speed_Target[motor]>0)&&(output<Start_PWM[motor]))
			output=Start_PWM[motor];
		else if((Speed_Target[motor]<0)&&(output>-Start_PWM[motor]))
			output=-Start_PWM[motor];

		/*
		 * 转速超过退出阈值就退出启动补偿。
		 * 退出这一拍把积分设成"刚好复现保底输出"的值，输出不跳变，
		 * 之后由PI自然接管（无扰切换）。
		 */
		if(Speed_Abs(Speed_Measured[motor])>=Start_Exit_RPM)
		{
			if(Speed_Target[motor]>0)
				Speed_Integral[motor]=Start_PWM[motor]-(Speed_Kp*error_rpm);
			else
				Speed_Integral[motor]=-Start_PWM[motor]-(Speed_Kp*error_rpm);
			Speed_Integral[motor]=Speed_Limit(Speed_Integral[motor],Integral_Limit,-Integral_Limit);
			Speed_Startup_Flag[motor]=0;
		}
	}

	/* 8.堵转保护：连续低速达到阈值后停机，避免堵转时持续大电流 */
	if((Speed_Abs(Speed_Target[motor])>=Stall_Target_RPM)
	   &&(Speed_Abs(Speed_Measured[motor])<Stall_Speed_RPM))
	{
		u32 stall_ms=(u32)Speed_Stall_Time[motor]+(u32)(Control_Period*1000.0f);
		if(stall_ms>=Stall_Time_MS)
		{
			Speed_Stall_Fault[motor]=1;
			Speed_Stall_Time[motor]=0;		//重新计时，用于自动重试
			Speed_Integral[motor]=0;
			Speed_Output[motor]=0;
			return 0;
		}
		Speed_Stall_Time[motor]=(u16)stall_ms;
	}
	else
		Speed_Stall_Time[motor]=0;

	/* 9.抗饱和：PI输出顶到限幅后把积分拉回来，判断用pi_output而不是output */
	if((pi_output>Output_Limit)&&(error_rpm>0))
		Speed_Integral[motor]=Output_Limit-(Speed_Kp*error_rpm);
	else if((pi_output<-Output_Limit)&&(error_rpm<0))
		Speed_Integral[motor]=-Output_Limit-(Speed_Kp*error_rpm);

	Speed_Integral[motor]=Speed_Limit(Speed_Integral[motor],Integral_Limit,-Integral_Limit);

	/* 目标为0但电机还在转时，限制制动幅度，保护L298N和电源 */
	if(Speed_Target[motor]==0)
		output=Speed_Limit(output,Brake_Limit,-Brake_Limit);

	/* 10.限幅输出 */
	output=Speed_Limit(output,Output_Limit,-Output_Limit);
	Speed_Output[motor]=output;

	/* 千分比四舍五入 */
	if(output>=0) return (int)(output+0.5f);
	return (int)(output-0.5f);
}

/**************************************************************************
Function: Integer limit
Input   : IN: input, max: upper, min: lower
Output  : limited value
函数功能：整数限幅
入口参数：IN：输入值  max：上限  min：下限
返回  值：限幅后的值
**************************************************************************/
int PWM_Limit(int IN,int max,int min)
{
	int OUT=IN;
	if(OUT>max) OUT=max;
	if(OUT<min) OUT=min;
	return OUT;
}
