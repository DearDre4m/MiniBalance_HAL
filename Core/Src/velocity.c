#include "velocity.h"

volatile velocityData_t g_velocity;

/* 上一次20ms采样得到的速度 */
static float last_velocity_m_s = 0.0f;

/* 防止第一次采样时错误求导 */
static uint8_t acceleration_initialized = 0;

void Velocity_Update(int16_t left_count,int16_t right_count)
{
    float current_velocity_m_s;
    float raw_acc;
    /* 保存编码器计数 */
    g_velocity.left_count = left_count;
    g_velocity.right_count = right_count;
    /* 计算左右轮转速 rev/s */
    g_velocity.left_rps = ((float)left_count / ENCODER_COUNTER_PRE_WHEEL)/V_SAMPLE_PERIOD_S;
    g_velocity.right_rps = ((float)right_count / ENCODER_COUNTER_PRE_WHEEL)/V_SAMPLE_PERIOD_S;
    /* 转换成 cm/s */
    g_velocity.left_cm_s = g_velocity.left_rps * WHEEL_CIRCUMFERENCE_CM;
    g_velocity.right_cm_s = g_velocity.right_rps * WHEEL_CIRCUMFERENCE_CM;
    /* 车辆平均速度 */
    g_velocity.average_cm_s = (g_velocity.left_cm_s + g_velocity.right_cm_s)/2.0f;
     /* cm/s -> m/s */
    current_velocity_m_s = g_velocity.average_cm_s * 0.01f;
    g_velocity.average_m_s = current_velocity_m_s;

    /*
     * 速度差分得到加速度
     *
     * a = (v[k] - v[k-1]) / dt
     */
    if (acceleration_initialized == 0)
    {
        last_velocity_m_s = current_velocity_m_s;

        g_velocity.linear_acc_raw_m_s2 = 0.0f;
        g_velocity.linear_acc_m_s2 = 0.0f;

        acceleration_initialized = 1;
    }
    else
    {
        raw_acc = (current_velocity_m_s - last_velocity_m_s)/ V_SAMPLE_PERIOD_S;
        last_velocity_m_s = current_velocity_m_s;
        /* 原始加速度 */
        g_velocity.linear_acc_raw_m_s2 = raw_acc;

        /*
         * 一阶低通滤波
         *
         * a_f[k] =
         * a_f[k-1] +
         * alpha * (a_raw - a_f[k-1])
         */
        g_velocity.linear_acc_m_s2 = g_velocity.linear_acc_m_s2 + ACC_LPF_ALPHA * (raw_acc - g_velocity.linear_acc_m_s2);
    }
}

void Velocity_Reset(void)
{
    g_velocity.left_count = 0;
    g_velocity.right_count = 0;

    g_velocity.left_rps = 0.0f;
    g_velocity.right_rps = 0.0f;

    g_velocity.left_cm_s = 0.0f;
    g_velocity.right_cm_s = 0.0f;

    g_velocity.average_cm_s = 0.0f;
    
}