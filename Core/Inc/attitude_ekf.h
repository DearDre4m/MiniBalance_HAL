#ifndef ATTITUDE_EKF_H
#define ATTITUDE_EKF_H

typedef struct 
{
    /* 状态 */
    float theta_rad; // 小车机械倾角 rad
    float bias_rad_s;// gyro bias rad/s
    /* 协方差矩阵 P */
    float p00;
    float p01;
    float p10;
    float p11;

    /* 过程噪声 Q */
    float q_theta;
    float q_bias;

    /* 加速度计观测噪声 R */
    float r_ay;
    float r_az;

    /* IMU 相对于机械直立位置的固定安装角 */
    float mount_offset_rad;

}AttitudeEKF_t;

void AttitudeEKF_Init(AttitudeEKF_t *ekf,float mount_offset_deg,float initial_bias_rad_s);
void AttitudeEKF_Update(AttitudeEKF_t *ekf, float gyro_x_rad_s, float acc_y_mps2, float acc_z_mps2, float linear_acc_mps2, float dt);
float AttitudeEKF_GetAngleDeg(const AttitudeEKF_t *ekf);
float AttitudeEKF_GetBiasDps(const AttitudeEKF_t *ekf);
#endif