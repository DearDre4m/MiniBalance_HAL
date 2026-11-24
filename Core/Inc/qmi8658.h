#ifndef _QMI8658_H
#define _QMI8658_H
#include <stdint.h>
#include <stddef.h>

#define QMI8658_WHO_AM_I_REG 0X00
#define QMI8658_REG_CTRL1 0X02
#define QMI8658_REG_CTRL2 0X03
#define QMI8658_REG_CTRL3 0X04
#define QMI8658_REG_CTRL5 0X06
#define QMI8658_REG_CTRL7 0X08
#define QMI8658_REG_STATUS0 0X2E

#define QMI8658_REG_AX_L 0X35

#define QMI8658_REG_RESET 0X60

#define QMI8658_REG_RESET_RESULT 0x4D

#define QMI8658_WHO_AM_I_VALUE 0X05

#define IMU_CALIBRATION_COUNT 200U

#define RAD_TO_DEG 57.2957795

extern float gyro_x_offset;

extern float gyro_z_offset;

extern float g_zero;
extern float g_phys;

extern float angle_zero;


typedef struct
{
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

}QMI8658_RawData_t;

void QMI8658_Init(void);
uint8_t QMI8658_ReadReg(uint8_t reg);
void QMI8658_ReadRegs(uint8_t start_reg,uint8_t *buffer,uint16_t length);
void QMI8658_ReadRaw(QMI8658_RawData_t *data);
uint8_t QMI8658_ReadWhoAmI(void);
void QMI8658_WriteReg(uint8_t reg,uint8_t data);

void QMI8658_Calibrate(void);
float Angle_Normalize180(float angle);
float QMI8658_GetAccAngle(QMI8658_RawData_t *data);

float QMI8658_AccRawToMps2(float raw);
float QMI8658_GyroRawToRadS(float raw);
#endif