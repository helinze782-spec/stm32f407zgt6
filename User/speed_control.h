#ifndef __SPEED_CONTROL_H
#define __SPEED_CONTROL_H

#include "motor_config.h"

void SpeedControl_Init(void);
void SpeedControl_SetTargetRpm(MotorId motor,float target_rpm);
void SpeedControl_Update(float elapsed_seconds);

#endif
