#ifndef SERVO_H
#define SERVO_H

#include "main.h"

void servo_init(TIM_HandleTypeDef* htim, uint32_t channel);
void setServoAngle(TIM_HandleTypeDef* htim, uint32_t channel, uint8_t angle);

#endif // DS3231_H
