#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

#include "stm32f4xx.h"

/*
 * 开路测试开关（不跑速度环，直接给固定PWM，用来查硬件/标定）：
 *   0 = 正常运行速度环（双轮闭环）
 *   1 = 双轮正转 100%
 *   2 = 双轮反转 100%
 *   3 = 只左轮正转 100%
 *   4 = 只右轮正转 100%
 *   5 = 标定模式：PWM从0开始每1秒加100‰，看第几秒轮子开始转，
 *       起步占空比≈秒数×100‰（用来定Start_PWM）
 *   6 = 编码器自检：上电后电机停着，用手转某个轮子，若该路编码器有计数，
 *       对应电机会点动一下（不用调试器就能判断编码器好坏）
 * 正常跑车记得改回0！
 */
#define Motor_Test_Mode					0
#define Test_PWM						1000

/* 当前工程两路电机、两路编码器 */
#define MOTOR_COUNT					2

/* 电机参数：WHEELTEC MG513P60_12V，13线编码器，60:1减速箱，A/B四倍频 */
#define Encoder_PPR					13.0f		//编码器线数
#define Reduction_Ratio					60.0f		//减速比
#define Encoder_Multiples				4.0f		//四倍频
#define Counts_Per_Rev					(Encoder_PPR*Encoder_Multiples*Reduction_Ratio)	//输出轴一圈3120个计数

/* 速度环10ms执行一次，PWM频率25kHz */
#define Control_Frequency				100.0f		//速度环100Hz
#define Control_Period					(1.0f/Control_Frequency)	//0.01s
#define MOTOR_CONTROL_PERIOD_MS			10U
#define MOTOR_PWM_FREQUENCY_HZ			25000U

/* 速度环PI参数 */
#define Speed_Kp						1.2f		//比例项，误差越大输出越大
#define Speed_Ki						4.0f		//积分项，用来消除稳态转速误差
#define Filter_Alpha					0.65f		//转速低通滤波系数，越大响应越快

/* PWM输出和积分项限幅，1000表示100%PWM */
#define Output_Limit					1000.0f
#define Integral_Limit					1000.0f

/* 启动补偿：电机几乎静止时PWM至少给到Start_PWM，帮它越过静摩擦
 * 太小起不来（只听嗡嗡响），太大起步会窜；用Motor_Test_Mode=5标定出起步值再+10~20% */
#define Start_PWM_1					500.0f		//左轮
#define Start_PWM_2					500.0f		//右轮
#define Start_RPM						8.0f		//进入起步状态的转速
#define Start_Exit_RPM					20.0f		//退出起步状态的转速（迟滞）

/* 堵转保护：目标有效但转速长期过低时关闭该路电机 */
#define Stall_Target_RPM				10.0f
#define Stall_Speed_RPM					5.0f
#define Stall_Time_MS					1000
#define Stall_Retry_MS					1000		//停机后滑行这么久再自动重试

/* 目标为0但电机还在转时的制动PWM上限，保护L298N和电源 */
#define Brake_Limit					300.0f

/* 目标转速上限和停车判断 */
#define Target_Limit_RPM				500.0f
#define Zero_Band_RPM					2.0f

/* 编码器方向修正：正PWM时读数应为正，反了就把对应值改成-1 */
#define Encoder_Sign_1					(-1)
#define Encoder_Sign_2					(-1)

/* 上电后两路电机的目标转速，单位RPM
 * 两轮设成一样才是直线行驶；左右差1~2RPM用来微调跑偏 */
#define Target_RPM_1					100.0f		//左轮
#define Target_RPM_2					100.0f		//右轮

#endif
