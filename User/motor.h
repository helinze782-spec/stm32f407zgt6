#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "motor_config.h"

/* 左轮：PA2(TIM9_CH1)输出PWM，PD0/PD1控制方向 */
#define PWMA				TIM9->CCR1
#define AIN1				PDout(0)
#define AIN2				PDout(1)

/* 右轮：PF6(TIM10_CH1)输出PWM，PD2/PD3控制方向 */
#define PWMB				TIM10->CCR1
#define BIN1				PDout(2)
#define BIN2				PDout(3)

void Motor_Init(void);
void Set_PWM(int Motor_left,int Motor_right);
void Motor_StopAll(void);
int myabs(int a);

#endif
