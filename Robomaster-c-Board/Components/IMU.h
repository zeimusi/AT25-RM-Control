#ifndef __IMU_H
#define __IMU_H
typedef struct{
			float Gyro_Yaw;
			float Gyro_Pitch;
			float Gyro_Roll;
			float Angle_Yaw;
			float Angle_Yawcontinuous;
			float Angle_Pitch;    
			float Angle_Roll;
	    float q[4];
	    int r;
}IMU_t;
extern IMU_t IMU;
#endif
