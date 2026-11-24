#ifndef _MOTOR_H
#define _MOTOR_H
#include "main.h"
#include <stdint.h>
#define MOTOR_PWM_MAX 4000
void Motor_Init(void);
void Motor_LeftSetPWM(int16_t pwm);
void Motor_RightSetPWM(int16_t pwm);
void Motor_SetVehiclePWM(int16_t left_pwm,int16_t right_pwm);
void Motor_Stop(void);

#endif