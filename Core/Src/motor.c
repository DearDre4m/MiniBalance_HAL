#include "motor.h"
#include "tim.h"

static int16_t Motor_LimitPWM(int16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX)
    {
        pwm = MOTOR_PWM_MAX;
    }
    else if (pwm < -MOTOR_PWM_MAX)
    {
        pwm = -MOTOR_PWM_MAX;
    }

    return pwm;
}


void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

    Motor_Stop();
}

/* 
从车屁股看向车，当pwm>0的时候左轮子带动车向前
*/
void Motor_LeftSetPWM(int16_t pwm)
{
    pwm = Motor_LimitPWM(pwm);
    
    if (pwm > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm);//左轮逆时针转动
    }
    else if(pwm < 0)
    {
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,-pwm);//左轮顺时针转动
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);//左轮停止转动
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,0);

    }
}

void Motor_RightSetPWM(int16_t pwm)
{
    pwm = Motor_LimitPWM(pwm);
    
    if (pwm > 0)
    {
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm);
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);//右轮顺时针转动
    }
    else if (pwm <0)
   {
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);//右轮逆时针转动
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,-pwm);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,0);//右轮停止转动
        __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,0);

    }
   
}

void Motor_SetVehiclePWM(int16_t left_pwm,int16_t right_pwm)
{
    Motor_LeftSetPWM(left_pwm);
    Motor_RightSetPWM(right_pwm);
}

void Motor_Stop(void)
{
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
}




