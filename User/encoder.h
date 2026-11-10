#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "motor_config.h"

#define ENCODER_TIM_PERIOD		0xFFFF		//F4的TIM2是32位，这里按16位用

void Encoder_Init_TIM2(void);
void Encoder_Init_TIM3(void);
int Read_Encoder(u8 TIMX);
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);

#endif
