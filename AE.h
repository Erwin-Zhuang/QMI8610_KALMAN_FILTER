#ifndef _AE_H
#define _AE_H

#include "stm32f4xx_hal.h"
#include "QMI8610.h"
#include "math.h"

typedef struct three_angle
{
	double yaw;
	double pitch;
	double roll;
}Euler_Angle;
//定义欧拉角

/******/
//观测函数，从原始数据和上一次卡尔曼滤波结果观测得到欧拉角，
void Altitude_Observe(Sensor_Data* RAW_DATA,
											Euler_Angle* Observe_Euler_Angle,
											Euler_Angle* Forecast_Euler_Angle,
											Euler_Angle* Kalman_Euler_Angle,
											double time,
											double* e_pitch,
											double* k_pitch,
											double* e_roll,
											double* k_roll,
											double* Q_pitch,
											const double* R_pitch,
											double* Q_roll,
											const double* R_roll,
											double* droll,
											double* dpitch);

#endif
