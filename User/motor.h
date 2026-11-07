#ifndef __MOTOR_H
#define __MOTOR_H

#include "motor_config.h"

void Motor_Init(void);
void Motor_SetSpeed(MotorId motor,int16_t speed_permille);
void Motor_Coast(MotorId motor);
void Motor_StopAll(void);

#endif
