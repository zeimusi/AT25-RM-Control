#ifndef __GIMBAL_H
#define __GIMBAL_H

#include "Variate.h"
#include "Function.h"


/* 云台模式 */
typedef enum{
    INIT = 0,
    MECH = 1,
    GYRO = 2,
    AIM = 3,
    GIMBAL_MODE = 4,
}eGimbalPidMode;
extern eGimbalPidMode GimbalPidMode;
typedef enum {
	gNormal    = 0,
	gAim       = 1,
	gFllow     = 2,
	gTest      = 3
}eGimbalCtrl;
extern eGimbalCtrl GimbalCtrl;
/**
 * @brief 云台角度结构体
 */
typedef struct {
    float Pitch;               //!<@brief Pitch轴角度
    float Yaw;                 //!<@brief Yaw轴角度
} PTZAngle_Ref_t;
typedef struct 
{
  float Pitch;
	float Roll;
	float Yaw;
	int16_t r;
	float Last;
	float ContinuousYaw;
}eAngle;
typedef struct{
enum {
    Gyro = 0,
	  Mech = 1,
	  ModeSum = 2
}Mode;
int LastCtrl;
PTZAngle_Ref_t Ref[ModeSum];
eAngle Angle[ModeSum];
struct{
		float Yaw;
	  float Pitch;
} Speed[ModeSum];  //!<@brief 转速
float YawInit,PitchInit;
float increase[GIMBAL_SUM];
uint8_t MidMode;
}eGimbal;
extern PID_Smis Gimbal_Place_PIDS[GIMBAL_SUM][GIMBAL_MODE];
extern PID Gimbal_Speed_PID[GIMBAL_SUM][GIMBAL_MODE];
extern PID_TypeDef Gimbal_Speed_pid_Pitch[GIMBAL_MODE], Gimbal_Place_pid_Pitch[GIMBAL_MODE],
	                 Gimbal_Speed_pid_Yaw[GIMBAL_MODE], Gimbal_Place_pid_Yaw[GIMBAL_MODE];																						

extern eGimbal Gimbal;
extern void MedianInit();
extern void Gimbal_RC_Ctrl();
extern void Gimbal_Key_Ctrl();
extern void Gimbal_Stop();
extern void Gimbal_Close();

extern void GimbalMode_Decide();
extern void GimbalCtrl_Decide();
extern void GimbalRef_Update();
extern void GimbalReal_Update();
extern void Gimbal_Pid();
extern void Gimbal_Send();

#endif
