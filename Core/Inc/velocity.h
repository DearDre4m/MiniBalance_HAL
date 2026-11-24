#ifndef _VELOCITY_H
#define _VELOCITY_H

#include <stdint.h>
/* =========================
 * 速度采样参数
 * ========================= */
#define V_SAMPLE_PERIOD_S 0.02f //速度采样周期20ms
#define V_SAMPLE_FREQUENCY_HZ 50.0f //采样频率50Hz
/* =========================
 * 轮子参数
 * ========================= */
#define WHEEL_DIAMETER_CM 6.7f //轮子直径6.7厘米

#define ENCODER_MULTIPLIER 4.0f //编码器倍频系数4倍
								//STM32 的定时器在编码器模式下，可以捕捉 A 相和 B 相上升沿和下降沿的全部 4 个边沿。这意味着，每“线”能变成 4 个计数。
								
#define ENCODER_PRECISION 13.0f //编码器物理线数13线，precision 精度
                                //减速器齿轮转一圈，定时器累计的计数值 = 13 × 4 = 52 个

#define MOTOR_REDUCTION_RATIO 30.0f //减速比30:1
                              //减速齿轮转30圈带动车轮转一圈

#define PI 3.1415926f

#define ENCODER_COUNTER_PRE_WHEEL (ENCODER_MULTIPLIER * ENCODER_PRECISION * MOTOR_REDUCTION_RATIO)//理论值上，车轮转一圈需要的计数 13 × 4 × 30 = 1560 个脉冲计数

#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * PI)//周长 = π × 直径
/* =========================
 * 编码器加速度低通滤波
 * =========================
 *
 * a_filter = a_filter + alpha * (a_raw - a_filter)
 *
 * alpha越大：
 * 响应越快，但是噪声越大
 *
 * alpha越小：
 * 更平滑，但是延迟更明显
 */
#define ACC_LPF_ALPHA           0.30f


typedef struct 
{   /* 特定时间内左右轮编码器计数 */
    int16_t left_count;
    int16_t right_count;
    /* 左右轮转速 rev/s */
    float left_rps;
    float right_rps;
    /* 左右轮线速度 cm/s */
    float left_cm_s;
    float right_cm_s;
    /* 小车平均速度 cm/s */
    float average_cm_s;
     /*
     * 小车平均速度 m/s
     * 后面EKF全部使用SI单位
     */
    float average_m_s;
    /*
     * 速度直接差分得到的原始线加速度
     * 单位 m/s^2
     *
     * 用于调试看噪声
     */
    float linear_acc_raw_m_s2;
    /*
     * 低通滤波后的线加速度
     * 单位 m/s^2
     *
     * 用于EKF
     */
    float linear_acc_m_s2;
}velocityData_t;

extern volatile velocityData_t g_velocity;

void Velocity_Update(int16_t left_count,int16_t right_count);
void Velocity_Reset(void);

#endif