#ifndef __WatchDog_Task_H__
#define __WatchDog_Task_H__

#include "Task_Init.h"

typedef enum{
    Device_Error = 1,
    Device_Right = 0,
}Device_Status;

typedef struct
{
	Device_Status Classis1;
	Device_Status Classis2;
    Device_Status Classis3;
    Device_Status Classis4;
    
    Device_Status Main_Yaw;
    
    Device_Status AimAngle_L;
    Device_Status AimAngle_R;
    
    Device_Status SuperCap;
    Device_Status Judge;
    Device_Status Remote;
}DeviceStatus_;
extern DeviceStatus_ DeviceStatus;

/* ø¥√≈π∑: */
typedef struct{
    WatchDog_TypeDef Main_Yaw_WatchDog;
    WatchDog_TypeDef Classis1_WatchDog;
    WatchDog_TypeDef Classis2_WatchDog;
    WatchDog_TypeDef Classis3_WatchDog;
    WatchDog_TypeDef Classis4_WatchDog;
    
    WatchDog_TypeDef AimAngle_L_WatchDog;
    WatchDog_TypeDef AimAngle_R_WatchDog;
    
    WatchDog_TypeDef SuperCap_WatchDog;
    WatchDog_TypeDef Judge_WatchDog;
    WatchDog_TypeDef Remote_WatchDog;
}WatchDog_t;
extern WatchDog_t WatchDog;

extern TaskHandle_t WatchDog_Task_Handle;
void WatchDog_Task(void *pvParameters);

#endif

