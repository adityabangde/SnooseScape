#include "servo.h"

void servo_init(TIM_HandleTypeDef* htim, uint32_t channel){
	HAL_TIM_PWM_Start(htim, channel);
}

void setServoAngle(TIM_HandleTypeDef* htim, uint32_t channel, uint8_t angle)
{
    // Limit angle to 0-180 degrees
    if(angle > 180) angle = 180;

    // Convert angle to pulse width (200-400 counts)
    // 200 counts = 0 degrees, 400 counts = 180 degrees
    uint32_t pulse = (angle * 230) / 180 +170;

    // Update PWM duty cycle
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

