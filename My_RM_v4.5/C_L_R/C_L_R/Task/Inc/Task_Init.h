#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#include "stm32f4xx_hal.h"
#include "can.h"
#include "gpio.h"
#include "stm32f4xx_it.h"
#include "math.h"
#include "tim.h"
#include "stdbool.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "stack_macros.h"

#include "RMLibHead.h"
#include "CANDrive.h"
#include "ramp.h"
#include "Chassis.h"
#include "remote.h"
#include "motor.h"
#include "WatchDog.h"
#include "CRC.h"
#include "RMQueue.h"
#include "PID.h"

#include "Attribute_Typedef.h"

#include "WatchDog_Task.h"
#include "Send_Task.h"
#include "Receive_Task.h"
#include "Usb_Task.h"
#include "Vision_Task.h"
#include "Gimbal_Task.h"
#include "Shoot_Task.h"
#include "Double_Gimbal_Task.h"

#define single_double 1 //单头作战为1，双头作战为0 
#define L_R           1
//左头为1，右头为0 

#define pi 3.1415926f

#if L_R
#define Pitch_Median 4000
#define Yaw_Median 6153

#define  Pitch_Limit_Max  5020
#define  Pitch_Limit_Min  3400
#define  Yaw_Limit_Max    10300
#define  Yaw_Limit_Min    5500
#else
#define Pitch_Median 7100
#define Yaw_Median 2070

#define  Pitch_Limit_Max  7600
#define  Pitch_Limit_Min  6150
#define  Yaw_Limit_Max    2880
#define  Yaw_Limit_Min    -2160
#endif

/* 反馈值 */
typedef struct{
	GM6020_TypeDef Gimbal_Yaw_Back;
	GM6020_TypeDef Gimbal_Pitch_Back;
	
	RM3508_TypeDef R_Friction_Back;
	RM3508_TypeDef L_Friction_Back;

	M2006_TypeDef Pluck_Back;
}Back_t;
extern Back_t Back;

/* 期望值 */
typedef struct{
	float Gimbal_Yaw_Exp_imu;
	int16_t Gimbal_Yaw_Exp;
	
	float Gimbal_Pitch_Exp_imu;
	int16_t Gimbal_Pitch_Exp;
	
	int16_t R_Friction_Exp;
	int16_t L_Friction_Exp;	

	int16_t Pluck_Exp;	
}Exp_t;
extern Exp_t Exp;


/* PID */
typedef struct{
	PID_Smis Gimbal_Yaw_Place_PID;
	PID Gimbal_Yaw_Speed_PID;

	PID_Smis Gimbal_Pitch_Place_PID;
	PID Gimbal_Pitch_Speed_PID;
    
	PID_Smis Gimbal_Yaw_Place_PID_imu;
	PID Gimbal_Yaw_Speed_PID_imu;

	PID_Smis Gimbal_Pitch_Place_PID_imu;
	PID Gimbal_Pitch_Speed_PID_imu;
    
	PID R_Friction_PID;
	PID L_Friction_PID;
	
	PID Pluck_PID;	
}Pid_t;
extern Pid_t Pid;

void Start_Task(void);

#endif

