#include "encoder.h"
#include "tim.h"

static int16_t Encoder_ReadAndClear(TIM_HandleTypeDef *htim)
{
    int16_t count;
    count = -(int16_t)__HAL_TIM_GET_COUNTER(htim);

    __HAL_TIM_SET_COUNTER(htim,0);

    return count;
}

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim2,0);
    __HAL_TIM_SET_COUNTER(&htim3,0);
}

int16_t Encoder_LeftRead()
{
    return Encoder_ReadAndClear(&htim3);
}

int16_t Encoder_RightRead()
{
    return Encoder_ReadAndClear(&htim2);
}

