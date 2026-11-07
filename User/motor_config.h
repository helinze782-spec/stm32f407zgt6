#ifndef __MOTOR_CONFIG_H
#define __MOTOR_CONFIG_H

#include "stm32f4xx.h"

/* 当前工程控制两个电机、读取两个编码器。 */
#define MOTOR_COUNT					2

/* WHEELTEC MG513P60_12V：13线编码器、60:1减速箱、A/B四倍频。 */
#define ENCODER_PPR					13U
#define ENCODER_GEAR_RATIO				60U
#define ENCODER_QUADRATURE_FACTOR			4U

/*
 * 输出轴转一圈的编码器计数：
 * 13线 * 4倍频 * 60减速比 = 3120计数/圈。
 */

/* 速度环每10ms执行一次，PWM频率25kHz。 */
#define MOTOR_CONTROL_PERIOD_MS			10U
#define MOTOR_PWM_FREQUENCY_HZ			25000U

/*
 * 速度环PI参数。
 * Kp：比例项，误差越大输出越大，主要决定响应。
 * Ki：积分项，用来消除稳态转速误差。
 * FILTER_ALPHA：编码器转速低通滤波，越大响应越快，越小越平滑。
 */
#define SPEED_CONTROL_KP				1.2f
#define SPEED_CONTROL_KI				4.0f
#define SPEED_CONTROL_FILTER_ALPHA			0.65f

/* PWM输出和积分项限制，1000表示100%PWM。 */
#define SPEED_CONTROL_OUTPUT_LIMIT			1000.0f
#define SPEED_CONTROL_INTEGRAL_LIMIT		1000.0f

/*
 * 启动死区补偿：
 * 电机几乎静止时，PWM至少给到 START_PWM，帮它越过L298N和齿轮的静摩擦。
 * 300表示30%PWM。如果电机仍然不能自己起步，可以改成400或500。
 * 注意：不要给到1000（=100%PWM）。给到1000的话，上电第一拍就是满占空比，
 * 电机直接窜出去；而且补偿会把PWM顶到限幅，抗饱和那一步容易把积分改坏。
 */
#define SPEED_CONTROL_START_PWM_1			300.0f
#define SPEED_CONTROL_START_PWM_2			300.0f
#define SPEED_CONTROL_START_RPM			8.0f

/*
 * 目标速度设为0、但电机还在转时的制动PWM上限。
 * 反向制动电流比正常驱动大，限制幅度可以保护L298N和电源，避免掉电复位。
 * 300表示30%PWM；想要更猛地停车可以调大，最大1000（等于不限制）。
 */
#define SPEED_CONTROL_BRAKE_LIMIT			300.0f

/* 目标速度和停止判断。 */
#define SPEED_CONTROL_TARGET_LIMIT_RPM		500.0f
#define SPEED_CONTROL_ZERO_BAND_RPM			2.0f

/*
 * 编码器方向修正：
 * 如果正PWM时对应RPM为负，就把对应值改为-1.0f。
 */
#define SPEED_CONTROL_ENCODER_SIGN_1		-1.0f
#define SPEED_CONTROL_ENCODER_SIGN_2		-1.0f

/*
 * 上电后两路电机的闭环目标转速，单位RPM。
 * 修改后必须重新Rebuild并下载程序。
 *
 * M1假设是左轮，M2假设是右轮。
 * 左轮明显偏快时，适当降低M1或提高M2。
 */
#define SPEED_CONTROL_TARGET_RPM_1			50.0f
#define SPEED_CONTROL_TARGET_RPM_2			100.0f

typedef enum
{
	MOTOR_ID_1=0,
	MOTOR_ID_2
}MotorId;

#endif
