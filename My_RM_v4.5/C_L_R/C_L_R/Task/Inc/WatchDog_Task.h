#ifndef _WATCHDOG_TASK_H_
#define _WATCHDOG_TASK_H_

#include "Task_Init.h"

/* 看门狗返回状态 */
typedef enum
{
    Device_Error = 1,
    Device_Right = 0
}Device_Status;

/* 看门狗: */
typedef struct{
	WatchDog_TypeDef Gimbal_Yaw_WatchDog;
	WatchDog_TypeDef Gimbal_Pitch_WatchDog;
    
	WatchDog_TypeDef L_Friction_WatchDog;
	WatchDog_TypeDef R_Friction_WatchDog;
	WatchDog_TypeDef Pluck_WatchDog;
    
    WatchDog_TypeDef Remote_WatchDog;    
    WatchDog_TypeDef Judge_WatchDog; 
    
    WatchDog_TypeDef PC_WatchDog;
}WatchDog_t;
extern WatchDog_t WatchDog;

typedef struct
{
	Device_Status Gimbal_Pitch;
	Device_Status Gimbal_Yaw;
    
    Device_Status L_Friction;
    Device_Status R_Friction;
    
    Device_Status Pluck;
    
    Device_Status Remote;
    Device_Status Judge;
    
    Device_Status PC_State;
    
}DeviceStatus_;
extern DeviceStatus_ DeviceStatus;

extern TaskHandle_t WatchDog_Task_Handle;
void WatchDog_Task(void *pvParameters);

#endif

