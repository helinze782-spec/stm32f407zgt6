#ifndef __SPEED_CONTROL_H__
#define __SPEED_CONTROL_H__

#include "motor_config.h"

void Speed_Init(void);
void Set_Target_Speed(u8 motor,float rpm);
int Speed_Control(u8 motor);
int PWM_Limit(int IN,int max,int min);

#endif
