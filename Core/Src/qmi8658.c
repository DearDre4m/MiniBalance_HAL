#include "qmi8658.h"
#include "spi.h"
#include "main.h"
#include <math.h>

float gyro_x_offset = 0.0f;
float gyro_z_offset = 0.0f;
float angle_zero = 0.0f;
float g_zero = 0.0f;
float g_phys = 0.0f;

/*
QMI8658 的数据手册规定，SPI 通信的第一个字节格式是：
bit7   bit6 ~ bit0
  ↓        ↓
读/写    寄存器地址
QMI8658 的 SPI 协议规定，读寄存器时，第一个字节的最高位（bit7）要置 1 表示“读”
所以读的时候的地址要 | 0x80 = 1000 0000
*/
static void QMI8658_CS_Low(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,IMU_CS_Pin,GPIO_PIN_RESET);
}

static void QMI8658_CS_High(void)
{
    HAL_GPIO_WritePin(IMU_CS_GPIO_Port,IMU_CS_Pin,GPIO_PIN_SET);
}

void QMI8658_Init(void)
{
    uint8_t ctrl1;
    /* SPI空闲时先取消片选 */
    QMI8658_CS_High(); 
    HAL_Delay(10);
    /* 软件复位 QMI8658 */
    QMI8658_WriteReg(QMI8658_REG_RESET, 0xB0U);//里面有使能片选
    //向RESET 寄存器（0x60）写 0xB0，触发软件复位。
    
    HAL_Delay(50);//延时50ms：官方最大复位时间是 15ms，等 50ms 留足余量。

    /*读-改-写
     * CTRL1
     * bit6 = 0：每次读都从同一个地址读，连续读 12 字节会读到 12 次同一个寄存器的值。
     * bit6 = 1：每读一个字节，地址自动 +1，才能连续读出加速度计 6 个字节、陀螺仪 6 个字节。
     * bit6 = 1：连续读取时地址自动递增
     * 0x40 = 0100 0000
     */
    ctrl1 = QMI8658_ReadReg(QMI8658_REG_CTRL1);
    ctrl1 |= 0x40;
    QMI8658_WriteReg(QMI8658_REG_CTRL1,ctrl1);
    HAL_Delay(2);

    /* 加速度计配置 
        0x25 = 0010 0101
               ↑↑↑↑ ↑↑↑↑
               aFS  aODR
    aFS[7:4]	0010	加速度量程 ±4g
    aODR[3:0]	0101	输出数据率 250Hz（具体看数据手册）
    */
    QMI8658_WriteReg(QMI8658_REG_CTRL2,0x25);
    HAL_Delay(2);

    /* 陀螺仪配置 
    0x55 = 0101 0101
           ↑↑↑↑ ↑↑↑↑
           gFS  gODR
    gFS[7:4]	0101	陀螺仪量程 ±512°/s（具体看数据手册）
    gODR[3:0]	0101	输出数据率 250Hz
    */
    QMI8658_WriteReg(QMI8658_REG_CTRL3,0x55);
    HAL_Delay(2);

    /* 暂时关闭可配置低通滤波 
    为什么关闭：因为你在软件里会用卡尔曼滤波或互补滤波做姿态融合，
    硬件滤波会引入额外延迟，干扰软件滤波的效果。让原始数据尽可能“生”，
    软件处理更灵活。
    */
    QMI8658_WriteReg(QMI8658_REG_CTRL5,0x00);
    HAL_Delay(2);

    /* Acc + Gyro Enable 使能传感器
    0x03 = 0000 0011
                  ↑↑
               bit1 bit0
    bit0 = 1	使能加速度计
    bit1 = 1	使能陀螺仪  
    */
    QMI8658_WriteReg(QMI8658_REG_CTRL7, 0x03);
    HAL_Delay(200);//陀螺仪启动后需要时间稳定。数据手册建议至少等100ms，你等200ms更保险。

}

uint8_t QMI8658_ReadReg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2] = {0};

    tx[0] = reg | 0x80;
    tx[1] = 0x00;//空头:dummy byte

    QMI8658_CS_Low();//使能QMI8658

    HAL_SPI_TransmitReceive(&hspi3,tx,rx,2,100);

    QMI8658_CS_High();//关闭QMI8658

    /*
     * rx[0] 是发送寄存器地址时收到的无效数据
     * rx[1] 才是真正寄存器内容
     */
    return rx[1];
}

void QMI8658_ReadRegs(uint8_t start_reg,uint8_t *buffer,uint16_t length)
{
    uint8_t address;
    if((buffer == NULL) || (length == 0))
    {
        return;
    }

    address = start_reg | 0x80U;

    QMI8658_CS_Low();//使能QMI8658

    HAL_SPI_Transmit(&hspi3,&address,1,100);//HAL_SPI_Transmit 要的是“地址”，不是“值”

    HAL_SPI_Receive(&hspi3,buffer,length,100);//buffer是指针本身存的就是地址

    QMI8658_CS_High();//关闭QMI8658


}


void QMI8658_ReadRaw(QMI8658_RawData_t *data)
{
    uint8_t buff[12];
    if (data == 0)
    {
        return;
    }
    /*
     * 从 AX_L 开始连续读取12字节
     *
     * AX_L AX_H
     * AY_L AY_H
     * AZ_L AZ_H
     * GX_L GX_H
     * GY_L GY_H
     * GZ_L GZ_H
     */
    QMI8658_ReadRegs(QMI8658_REG_AX_L,buff,12);

    data->acc_x = (int16_t)(((uint16_t)buff[1]<<8)|buff[0]);
    data->acc_y = (int16_t)(((uint16_t)buff[3]<<8)|buff[2]);
    data->acc_z = (int16_t)(((uint16_t)buff[5]<<8)|buff[4]);

    data->gyro_x = (int16_t)(((uint16_t)buff[7]<<8)|buff[6]);
    data->gyro_y = (int16_t)(((uint16_t)buff[9]<<8)|buff[8]);
    data->gyro_z = (int16_t)(((uint16_t)buff[11]<<8)|buff[10]);

}


uint8_t QMI8658_ReadWhoAmI(void)
{
    return QMI8658_ReadReg(QMI8658_WHO_AM_I_REG);
}


void QMI8658_WriteReg(uint8_t reg,uint8_t data)
{
    uint8_t tx[2];
    //0x7F = 0111 1111
    tx[0] = reg & 0x7FU;
    tx[1] = data;

    QMI8658_CS_Low();//使能QMI8658

    HAL_SPI_Transmit(&hspi3,tx,2,100);

    QMI8658_CS_High();//关闭QMI8658

}


void QMI8658_Calibrate(void)
{
    uint16_t i;

    int32_t gyro_x_sum = 0;
    int32_t gyro_z_sum = 0;

    int32_t acc_y_sum = 0;
    int32_t acc_z_sum = 0;

    float acc_y_average;
    float acc_z_average;

    QMI8658_RawData_t imu;

    /*
     * 给你一点时间把小车扶直、保持静止
     */

    HAL_Delay(1000);

    for ( i = 0; i < IMU_CALIBRATION_COUNT; i++)
    {

        QMI8658_ReadRaw(&imu);
        gyro_x_sum += imu.gyro_x;
        gyro_z_sum += imu.gyro_z;

        acc_y_sum += imu.acc_y;//反转Y轴方向是的Y正方向为车头
        acc_z_sum += imu.acc_z;

        HAL_Delay(5);
    }
    
    gyro_x_offset = (float) gyro_x_sum/ (float) IMU_CALIBRATION_COUNT;
    gyro_z_offset = (float)gyro_z_sum / (float)IMU_CALIBRATION_COUNT;
    acc_y_average = (float)acc_y_sum / (float)IMU_CALIBRATION_COUNT;
    acc_z_average = (float)acc_z_sum / (float)IMU_CALIBRATION_COUNT;
    
    g_zero = sqrtf(acc_y_average * acc_y_average + acc_z_average * acc_z_average);

    g_phys = g_zero / 4096.0f;
    /*
     * 小车机械直立位置对应的加速度倾角
     */
    angle_zero = atan2f(acc_y_average, acc_z_average) * 57.2957795f;

}

float Angle_Normalize180(float angle)
{
    while(angle > 180.0f)
    {
        angle -= 360.0f;
    }

    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    
    return angle;
}

float QMI8658_GetAccAngle(QMI8658_RawData_t *data)
{
    float angle;

    if(data == NULL)
    {
        return 0.0f;
    }

    angle = atan2f((float)data->acc_y,(float)data->acc_z) * RAD_TO_DEG;

    angle -= angle_zero;//所以翻转 Y 后，安装零偏也要翻转
    
    return angle;
}

float QMI8658_AccRawToMps2(float raw)
{
    return raw / 4096.0f * 9.80665f;
}

float QMI8658_GyroRawToRadS(float raw)
{
    return raw / 64.0f * 0.01745329252f;
}