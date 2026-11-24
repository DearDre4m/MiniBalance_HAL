#include "attitude_ekf.h"
#include <math.h>
#include <stddef.h>

#define EKF_GRAVITY 9.80665f
#define EKF_DEG_TO_RAD 0.01745329252f
#define EKF_RAD_TO_DEG  57.2957795f

static float EKF_WrapPi(float angle)
{
    while (angle > 3.14159265f)
    {
        angle -= 6.28318531f;
    }

    while (angle < -3.14159265f)
    {
        angle +=  6.28318531f;
    }
    
    return angle;
}

void AttitudeEKF_Init(AttitudeEKF_t *ekf,float mount_offset_deg,float initial_bias_rad_s)
{
    float theta_std;
    float bias_std;

    if(ekf == NULL)
    {
        return;
    }

    /*
     * 状态初值
     *
     * 校准时小车处于机械直立：
     * theta = 0
     *
     * gyro bias 则用静态校准结果初始化
     */
    ekf->theta_rad = 0 * EKF_DEG_TO_RAD;
    ekf->bias_rad_s = initial_bias_rad_s;

    /*
     * 固定安装偏角
     *
     * 你现在大约：
     * angle_zero = +5.10 deg
     */
    ekf->mount_offset_rad =  mount_offset_deg * EKF_DEG_TO_RAD;

    /*
     * 初始协方差 P
     *
     * 先假设：
     * angle 初始不确定度约 5 deg
     * bias  初始不确定度约 2 deg/s
     */
    theta_std = 5.0f * EKF_DEG_TO_RAD;
    bias_std = 2.0f * EKF_DEG_TO_RAD;

    ekf->p00 = theta_std * theta_std;
    ekf->p01 = 0.0f;
    ekf->p10 = 0.0f;
    ekf->p11 = bias_std * bias_std;

    /*过程噪声协方差Q的两个对角元素*/
    ekf->q_theta = 1.0e-6f;
    ekf->q_bias = 1.0e-8f;

    /*
     * R 是测量噪声方差。
     *
     * 这里先认为 ay / az 的标准差大约 0.5 m/s^2
     *
     * variance = sigma^2 = 0.25
     */
    ekf->r_ay = 0.25f;
    ekf->r_az = 0.25f;
}

void AttitudeEKF_Update(AttitudeEKF_t *ekf, float gyro_x_rad_s, float acc_y_mps2, float acc_z_mps2, float linear_acc_mps2, float dt)
{
    float theta_pred;
    float bias_pred;

    float p00_pred;//P[k|k-1]
    float p01_pred;
    float p10_pred;
    float p11_pred;

    float phi;//phi是IMU 坐标系下的角度，theta 是机械坐标系下的角度，mount_offset 是安装偏角。
    float sin_phi;
    float cos_phi;

    float h_y;
    float h_z;

    float H0;
    float H1;

    float innovation_y;//新息
    float innovation_z;

    float s00;//S = H P- H' + R
    float s01;
    float s11;

    float det_s;//矩阵 S 的行列式

    float inv_s00;//矩阵 S 的逆矩阵 S⁻¹ 的元素
    float inv_s01;
    float inv_s11;
    
    float ph00;//(P·Hᵀ) 的第 0 行第 0 列
    float ph01;
    float ph10;
    float ph11;

    float k00;
    float k01;
    float k10;
    float k11;

    float a00;// Joseph 形式协方差更新里的矩阵 A = I - K·H 的元素。
    float a10;

    float ap00;//A·P[k|k-1] 的元素，是 Joseph 形式协方差更新的中间变量
    float ap01;
    float ap10;
    float ap11;

    float new_p00;
    float new_p01;
    float new_p10;
    float new_p11;

    float p_cross;//强制协方差矩阵 P 对称的中间变量

    if((ekf == NULL)||(dt<= 0.0f))
    {
        return;
    }

    /* =========================================================
     * 1. EKF 状态预测
     *
     * theta(k) = theta(k-1) + [gyro(角速度) - bias] * dt
     * bias(k) = bias(k-1)
     * =========================================================
     */

    theta_pred = ekf->theta_rad + (gyro_x_rad_s - ekf->bias_rad_s) * dt;
    theta_pred = EKF_WrapPi(theta_pred);
    bias_pred =ekf->bias_rad_s;

    /*
     * F = [ 1   -dt ]
     *     [ 0     1 ]
     * P[k|k-1] = F P F' + Q
     */
    p00_pred = ekf->p00 - dt*ekf->p10 - dt*ekf->p01 + dt * dt * ekf->p11 + ekf->q_theta;
    p01_pred = ekf->p01 - dt*ekf->p11;
    p10_pred = ekf->p10 - dt*ekf->p11;
    p11_pred = ekf->p11 + ekf->q_bias;

    /* =========================================================
     * 2. 预测加速度计观测
     * =========================================================
     *
     * theta_rad 是小车机械倾角。
     *
     * IMU 本身存在安装偏角：
     *
     * phi = theta + theta0
     */
    phi = theta_pred + ekf->mount_offset_rad;

    sin_phi = sinf(phi);
    cos_phi = cosf(phi);
    /*
     * 最终确定的观测模型：
     *
     * ay = -a*cos(phi) + g*sin(phi)
     * az =  a*sin(phi) + g*cos(phi)
     */

    h_y = -linear_acc_mps2 *cos_phi + EKF_GRAVITY * sin_phi;
    h_z = linear_acc_mps2 * sin_phi + EKF_GRAVITY *cos_phi;

    /* =========================================================
     * 3. 计算观测 Jacobian H
     * =========================================================
     *
     * 对 theta 求偏导：
     * dh_y/dtheta = a*sin(phi) + g*cos(phi)
     *
     * dh_z/dtheta = a*cos(phi) - g*sin(phi)
     *
     * 对 bias 的偏导都是 0
     *
     *
     * H = [ H0   0 ]
     *     [ H1   0 ]
     */

    H0 = linear_acc_mps2 * sin_phi + EKF_GRAVITY * cos_phi;
    H1 = linear_acc_mps2 * cos_phi - EKF_GRAVITY * sin_phi;

    /* =========================================================
     * 4. Innovation
     *
     * y = z - h(x)
     * =========================================================
     */

    innovation_y = acc_y_mps2 - h_y;
    innovation_z = acc_z_mps2 - h_z;

    /* =========================================================
     * 5. S = H P[k|k-1] H' + R
     * S 是 2x2
     * =========================================================
     */
    s00 = H0 * p00_pred * H0 + ekf->r_ay;
    s01 = H0 * H1 * p00_pred;//s10 = s01
    s11 = H1 * H1 * p00_pred + ekf->r_az;

    /*
     * 2x2 矩阵求逆
     *
     * S^-1 =
     *
     * 1/det *
     * [ s11   -s01 ]
     * [ -s01   s00 ]
     */
    det_s = s00 * s11 - s01 * s01;

    if(fabsf(det_s) < 1.0e-12f)
    {
        /*
         * 极端情况下 S 无法求逆。
         * 只保留预测结果。
         */
        ekf->theta_rad = theta_pred;
        ekf->bias_rad_s = bias_pred;

        ekf->p00 = p00_pred;
        ekf->p01 = p01_pred;
        ekf->p10 = p10_pred;
        ekf->p11 = p11_pred;

        return;
    }

    inv_s00 = s11/det_s;
    inv_s01 = -s01/det_s;
    inv_s11 = s00/det_s;

    /* =========================================================
     * 6. K = P- H' S^-1
     * =========================================================
     */

    /*
     * P H'
     */
    
    ph00 = p00_pred * H0;
    ph01 = p00_pred * H1;
    ph10 = p10_pred * H0;
    ph11 = p10_pred * H1;

    /*
     * K = PH' * S^-1
     */

     k00 = ph00 * inv_s00 + ph01 * inv_s01;
     k01 = ph00 * inv_s01 + ph01 * inv_s11;
     k10 = ph10 * inv_s00 + ph11 * inv_s01;
     k11 = ph10 * inv_s01 + ph11 * inv_s11;

    /* =========================================================
     * 7. 状态更新
     *
     * X[k|k] = X[k|k-1] + K * innovation
     * =========================================================
     */

    ekf->theta_rad =  theta_pred + k00 * innovation_y + k01 * innovation_z;
    ekf->theta_rad = EKF_WrapPi(ekf->theta_rad);
    ekf->bias_rad_s =   bias_pred + k10 * innovation_y + k11 * innovation_z;

    /* =========================================================
     * 8. 协方差更新
     *
     * 使用 Joseph form：
     *
     * P =
     * (I-KH) P- (I-KH)' + K R K'
     *
     * 数值稳定性比简单的
     * P=(I-KH)P
     * 更好。
     * =========================================================
     */

    /*
     * A = I - KH
     *
     * 由于 H 第二列为0：
     *
     * A =
     *
     * [ a00   0 ]
     * [ a10   1 ]
     */
    a00 = 1.0f - k00 * H0 - k01 * H1;
    a10 = -k10 * H0 - k11 * H1;

    /*
     * A * P[k|k-1]
     */
    ap00 = a00 * p00_pred;

    ap01 = a00 * p01_pred;

    ap10 = a10 * p00_pred + p10_pred;

    ap11 = a10 * p01_pred + p11_pred;

    /*
     * A P[k|k-1] A'
     */
    new_p00 = ap00 * a00;

    new_p01 = ap00 * a10 + ap01;

    new_p10 = ap10 * a00;

    new_p11 = ap10 * a10 + ap11;

    /*
     * + K R K'
     *
     * R = [ r_ay   0    ]
     *     [ 0      r_az ]
     */

    new_p00 +=k00 * k00 * ekf->r_ay + k01 * k01 * ekf->r_az;

    new_p01 += k00 * k10 * ekf->r_ay + k01 * k11 * ekf->r_az;

    new_p10 += k10 * k00 * ekf->r_ay + k11 * k01 * ekf->r_az;

    new_p11 += k10 * k10 * ekf->r_ay + k11 * k11 * ekf->r_az;
    /*
     * 消除浮点误差造成的轻微不对称
     */
    p_cross = 0.5f * (new_p01 + new_p10);

    ekf->p00 = new_p00;
    ekf->p01 = p_cross;
    ekf->p10 = p_cross;
    ekf->p11 = new_p11;

}

float AttitudeEKF_GetAngleDeg(const AttitudeEKF_t *ekf)
{
    if (ekf == NULL)
    {
        return 0.0f;
    }

    return ekf->theta_rad * EKF_RAD_TO_DEG;
}

float AttitudeEKF_GetBiasDps(const AttitudeEKF_t *ekf)
{
    if (ekf == NULL)
    {
        return 0.0f;
    }

    return ekf->bias_rad_s * EKF_RAD_TO_DEG;
}