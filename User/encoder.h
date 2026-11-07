#ifndef __ENCODER_H
#define __ENCODER_H

#include "motor_config.h"

void Encoder_Init(void);
void Encoder_Update(uint32_t elapsed_ms);
float Encoder_GetSpeedRpm(MotorId motor);

#endif
