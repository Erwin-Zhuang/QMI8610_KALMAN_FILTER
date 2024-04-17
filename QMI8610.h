#ifndef _QMI8610_H
#define _QMI8610_H

#include "stm32f4xx_hal.h"
#include "math.h"

#define DEV_ADD 0X6A

#define CTRL1 0X02
#define CTRL2 0X03
#define CTRL3 0X04
#define CTRL4 0X05
#define CTRL5 0X06
#define CTRL6 0X07
#define CTRL7 0X08
#define CTRL8 0X09
#define CTRL9 0X0A


#define data70 0x00//关闭所有传感器
#define data71 0x03//启动六轴
#define data72 0x0b//启动六轴和运动引擎

#define data21 0x00//加速度±2g
#define data22 0x08//加速度±4g
#define data23 0x10//加速度±8g

#define data31 0x00//角速度±32dps
#define data32 0x10//角速度±128dps
#define data33 0x20//角速度±512dps

#define data60 0x00//启动运动引擎，20Hz输出速度

typedef struct six_data
{
	double Acce_X;
	double Acce_Y;
	double Acce_Z;
	double Gyro_X;
	double Gyro_Y;
	double Gyro_Z;
}Sensor_Data;
//定义欧拉角

/******/
//QMI8610检测函数，找到QMI8610则返回 HAL_OK
HAL_StatusTypeDef QMI8610_DETECT(I2C_HandleTypeDef *iic);

/******/
//QMI8610写寄存器
HAL_StatusTypeDef QMI8610_SET_Register(I2C_HandleTypeDef *iic, uint8_t Reg_ADD, uint8_t Value);

/******/
//QMI8610读寄存器
uint8_t QMI8610_Read_Register(I2C_HandleTypeDef *iic, uint8_t Reg_ADD);

/***
//QMI8610设置函数
***/
HAL_StatusTypeDef QMI8610_SEETING(I2C_HandleTypeDef *iic);
																	
/******/
//QMI8610读数据
void QMI8610_Read_Data(I2C_HandleTypeDef *iic,Sensor_Data* pDATA);


#endif
