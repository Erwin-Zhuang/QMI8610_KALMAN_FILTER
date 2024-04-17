#include "AE.h"

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
											double* dpitch)
{
		Observe_Euler_Angle->pitch=atan(RAW_DATA->Acce_X/RAW_DATA->Acce_Z);
		Observe_Euler_Angle->roll=asin(RAW_DATA->Acce_Y);
		Observe_Euler_Angle->yaw=Kalman_Euler_Angle->yaw;
		//根据三周加速度来观测欧拉角
		*droll					=	RAW_DATA->Gyro_X+
										sin(Kalman_Euler_Angle->pitch)*sin(Kalman_Euler_Angle->roll)/cos(Kalman_Euler_Angle->pitch)*RAW_DATA->Gyro_Y+
										cos(Kalman_Euler_Angle->roll)*sin(Kalman_Euler_Angle->pitch)/cos(Kalman_Euler_Angle->pitch)*RAW_DATA->Gyro_Z;
		
		*dpitch				=	cos(Kalman_Euler_Angle->roll)*RAW_DATA->Gyro_Y-
										sin(Kalman_Euler_Angle->roll)*RAW_DATA->Gyro_Z;
		
		double dyaw					=	sin(Kalman_Euler_Angle->roll)/cos(Kalman_Euler_Angle->pitch)*RAW_DATA->Gyro_Y+
										cos(Kalman_Euler_Angle->roll)/cos(Kalman_Euler_Angle->pitch)*RAW_DATA->Gyro_Z;
		//按照上一时刻的姿态反向计算欧拉角角速度
		Forecast_Euler_Angle->pitch=(Kalman_Euler_Angle->pitch*57.29578-time**dpitch)/57.29578;
		Forecast_Euler_Angle->roll=(Kalman_Euler_Angle->roll*57.29578+time**droll)/57.29578;
		Forecast_Euler_Angle->yaw=(Kalman_Euler_Angle->yaw*57.29578+time*dyaw)/57.29578;
		//按照角速度和时间计算当前时刻的预测欧拉角
		*Q_pitch=2*sqrt((Observe_Euler_Angle->pitch-Kalman_Euler_Angle->pitch)*(Observe_Euler_Angle->pitch-Kalman_Euler_Angle->pitch));
		*Q_roll=2*sqrt((Observe_Euler_Angle->roll-Kalman_Euler_Angle->roll)*(Observe_Euler_Angle->roll-Kalman_Euler_Angle->roll));
		//估算观测噪声
		*e_pitch=*e_pitch+time+*Q_pitch;
		*k_pitch=*e_pitch/(*e_pitch+*R_pitch);
		
		*e_roll=*e_roll+time+*Q_roll;
		*k_roll=*e_roll/(*e_roll+*R_roll);
		//计算卡尔曼增益
		Kalman_Euler_Angle->pitch=Observe_Euler_Angle->pitch+*k_pitch*(Forecast_Euler_Angle->pitch-Observe_Euler_Angle->pitch);
		Kalman_Euler_Angle->roll=Observe_Euler_Angle->roll+*k_roll*(Forecast_Euler_Angle->roll-Observe_Euler_Angle->roll);
		Kalman_Euler_Angle->yaw=Forecast_Euler_Angle->yaw;
		//更新卡尔曼最优姿态
		*e_pitch=(1.0f-*k_pitch)*(*e_pitch);
		*e_roll=(1.0f-*k_roll)*(*e_roll);
		//更新误差
}

