/*
 * speed_control.c
 * 速度环：编码器测速 + PI + 启动补偿，由TIM7每10ms调用一次，两个电机各算一遍。
 *
 * 单位约定：
 *   转速  RPM
 *   PWM   千分比，1000 表示 100%
 *   时间  秒
 *
 * SpeedControl_Update里一次计算的顺序（从上往下就是执行顺序）：
 *   1. 读编码器转速，做方向修正和低通滤波，得到实际转速
 *   2. 目标为0而且电机已经停下 -> 直接滑行，本拍结束
 *   3. 算误差：目标转速 - 实际转速
 *   4. 累加积分（起步阶段先不积，避免电机还没转起来积分就积大了）
 *   5. 算PI输出：Kp*误差 + 积分
 *   6. 启动补偿：电机几乎静止时，PWM至少给到START_PWM，帮它冲过静摩擦死区
 *   7. 抗饱和，然后限幅，最后把PWM交给电机驱动
 */

#include "speed_control.h"
#include "encoder.h"
#include "motor.h"

typedef struct
{
	float target_rpm;			/* 目标转速 */
	float measured_rpm;			/* 实际转速（低通滤波后） */
	float integral;				/* PI积分项 */
	float output;				/* 最终PWM输出，1000 = 100% */
	uint8_t filter_initialized;	/* 第一次采样直接使用实测值，不做滤波 */
}SpeedControlStateTypeDef;

/*
 * 两个电机分别保存一套PI状态。
 * 故意不加static，方便在Keil的Watch窗口里直接看speed_state[0]、speed_state[1]。
 */
SpeedControlStateTypeDef speed_state[MOTOR_COUNT];

/* 编码器方向修正：正PWM时转速读出来是负的，就把对应的值改成-1.0f。 */
static const float encoder_sign[MOTOR_COUNT]=
{
	SPEED_CONTROL_ENCODER_SIGN_1,
	SPEED_CONTROL_ENCODER_SIGN_2
};

/* 两个电机各自的启动补偿PWM，左轮和右轮可以分别设置。 */
static const float start_pwm[MOTOR_COUNT]=
{
	SPEED_CONTROL_START_PWM_1,
	SPEED_CONTROL_START_PWM_2
};

/* 把value限制在minimum~maximum之间。 */
static float SpeedControl_Clamp(float value,float minimum,float maximum)
{
	if(value>maximum)
		return maximum;
	if(value<minimum)
		return minimum;
	return value;
}

/* 取绝对值。 */
static float SpeedControl_Abs(float value)
{
	if(value<0)
		return -value;
	return value;
}

/* 把浮点PWM转换成-1000~1000的整数（四舍五入），电机驱动只认整数千分比。 */
static int16_t SpeedControl_ToPermille(float value)
{
	if(value>=0)
		return (int16_t)(value+0.5f);
	return (int16_t)(value-0.5f);
}

void SpeedControl_Init(void)
{
	MotorId motor;

	/* 清空PI状态并让电机停下。 */
	for(motor=MOTOR_ID_1;motor<MOTOR_COUNT;motor++)
	{
		speed_state[motor].target_rpm=0;
		speed_state[motor].measured_rpm=0;
		speed_state[motor].integral=0;
		speed_state[motor].output=0;
		speed_state[motor].filter_initialized=0;
	}

	Motor_StopAll();
}

void SpeedControl_SetTargetRpm(MotorId motor,float target_rpm)
{
	float old_target;
	float new_target;

	if(motor>=MOTOR_COUNT)
		return;

	/* 目标转速同样限制在配置的上限内。 */
	new_target=SpeedControl_Clamp(target_rpm,-SPEED_CONTROL_TARGET_LIMIT_RPM,SPEED_CONTROL_TARGET_LIMIT_RPM);

	/*
	 * 换向或者目标归零时清空积分。
	 * 不清的话，旧积分会一直把电机往原来的方向拖，反向起步会明显发冲。
	 */
	old_target=speed_state[motor].target_rpm;
	if(((old_target>0)&&(new_target<0))||((old_target<0)&&(new_target>0))||(new_target==0))
	{
		speed_state[motor].integral=0;
	}

	speed_state[motor].target_rpm=new_target;
}

void SpeedControl_Update(float elapsed_seconds)
{
	MotorId motor;

	if(elapsed_seconds<=0)
		return;

	/* 两个电机用同一套流程，各算各的。 */
	for(motor=MOTOR_ID_1;motor<MOTOR_COUNT;motor++)
	{
		SpeedControlStateTypeDef *state=&speed_state[motor];
		float raw_rpm;			/* 编码器直接算出来的转速 */
		float error_rpm;		/* 目标转速 - 实际转速 */
		float pi_output;		/* PI算出来的PWM，还没做启动补偿和限幅 */
		float output;			/* 最终发给电机的PWM */
		uint8_t starting_up;	/* 1 = 正在起步，这一拍用启动补偿、不积分 */

		/* ---------- 1. 读编码器转速 ---------- */
		raw_rpm=Encoder_GetSpeedRpm(motor)*encoder_sign[motor];

		/* 一阶低通滤波，压掉编码器量化带来的抖动。 */
		if(state->filter_initialized==0)
		{
			state->measured_rpm=raw_rpm;
			state->filter_initialized=1;
		}
		else
		{
			state->measured_rpm=SPEED_CONTROL_FILTER_ALPHA*raw_rpm
			                   +(1.0f-SPEED_CONTROL_FILTER_ALPHA)*state->measured_rpm;
		}

		/* ---------- 2. 目标为0而且已经停下：滑行，本拍结束 ---------- */
		if((state->target_rpm==0)&&(SpeedControl_Abs(state->measured_rpm)<=SPEED_CONTROL_ZERO_BAND_RPM))
		{
			state->integral=0;
			state->output=0;
			Motor_Coast(motor);
			continue;
		}

		/* ---------- 3. 算误差 ---------- */
		/* 误差 > 0：实际转速不够，要加大PWM；误差 < 0：实际转速过快，要减小PWM。 */
		error_rpm=state->target_rpm-state->measured_rpm;

		/* 判断是不是"正在起步"：目标不为0，而且电机几乎没有转。 */
		starting_up=0;
		if((state->target_rpm!=0)&&(SpeedControl_Abs(state->measured_rpm)<SPEED_CONTROL_START_RPM))
			starting_up=1;

		/* ---------- 4. 累加积分 ---------- */
		/*
		 * 起步阶段先不积分：这时候电机还没转起来，误差一直是满的，
		 * 积分会一口气积得很大，等电机转起来以后就会冲过头。
		 */
		if(starting_up==0)
		{
			state->integral+=SPEED_CONTROL_KI*error_rpm*elapsed_seconds;
			state->integral=SpeedControl_Clamp(state->integral,
			                                   -SPEED_CONTROL_INTEGRAL_LIMIT,
			                                    SPEED_CONTROL_INTEGRAL_LIMIT);
		}

		/* ---------- 5. 算PI输出 ---------- */
		pi_output=(SPEED_CONTROL_KP*error_rpm)+state->integral;

		/* ---------- 6. 启动补偿 ---------- */
		/*
		 * 电机几乎静止时，PI输出往往低于L298N和齿轮的静摩擦死区，电机必须用手转一下才动。
		 * 所以起步阶段给PWM设一个下限START_PWM。
		 * 到了正常调速阶段（starting_up=0）这一条不生效，不会影响转速精度。
		 */
		output=pi_output;
		if(starting_up!=0)
		{
			if((state->target_rpm>0)&&(output<start_pwm[motor]))
				output=start_pwm[motor];
			else if((state->target_rpm<0)&&(output>-start_pwm[motor]))
				output=-start_pwm[motor];
		}

		/* ---------- 7. 抗饱和 ---------- */
		/*
		 * PI输出顶到限幅以后，就不要再让积分继续涨了，否则积分会"饱和"：
		 * 转速已经超过目标想往回收时，积分要慢慢退很久，电机会冲出去。
		 * 这里把积分拉回到"刚好让PI输出等于限幅"的位置，下一拍接着正常算。
		 *
		 * 注意判断的是pi_output（PI自己的输出），不是叠加启动补偿后的output：
		 * 用output判断的话，启动补偿一把PWM抬到限幅，积分就会被误改成很大的值。
		 * 原来START_PWM=1000的时候，上电第一拍积分就从0跳到940，电机直接满速冲出去。
		 */
		if((pi_output>SPEED_CONTROL_OUTPUT_LIMIT)&&(error_rpm>0))
			state->integral=SPEED_CONTROL_OUTPUT_LIMIT-(SPEED_CONTROL_KP*error_rpm);
		else if((pi_output<-SPEED_CONTROL_OUTPUT_LIMIT)&&(error_rpm<0))
			state->integral=-(SPEED_CONTROL_OUTPUT_LIMIT)-(SPEED_CONTROL_KP*error_rpm);

		state->integral=SpeedControl_Clamp(state->integral,
		                                   -SPEED_CONTROL_INTEGRAL_LIMIT,
		                                    SPEED_CONTROL_INTEGRAL_LIMIT);

		/* 目标为0但电机还在转时，PI会输出反向PWM做制动，这里限制制动幅度，保护驱动和电源。 */
		if(state->target_rpm==0)
			output=SpeedControl_Clamp(output,-SPEED_CONTROL_BRAKE_LIMIT,SPEED_CONTROL_BRAKE_LIMIT);

		/* 最后统一按输出上限收一次，再交给电机驱动。 */
		output=SpeedControl_Clamp(output,-SPEED_CONTROL_OUTPUT_LIMIT,SPEED_CONTROL_OUTPUT_LIMIT);
		state->output=output;
		Motor_SetSpeed(motor,SpeedControl_ToPermille(output));
	}
}
