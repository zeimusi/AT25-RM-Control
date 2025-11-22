#include "WatchDog_Task.h"

DeviceStatus_ DeviceStatus = {.AimAngle_L = Device_Error, .AimAngle_R = Device_Error};
WatchDog_t WatchDog;

TaskHandle_t WatchDog_Task_Handle;
void WatchDog_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
        WatchDog_Polling();
       
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

/**
 * 亮红灯：主Yaw有问题
 * 亮蓝灯：轮子有问题
 * 亮绿灯：通信有问题（遥控器，裁判系统，超电）
 */
void FeedDog_CallBack(WatchDogp handle)
{    
    /* 主Yaw  4310温度过高代表掉线，pid归0冷静一下 */
    if(IS_Dog(handle, WatchDog.Main_Yaw_WatchDog) && DM4310_Motor_Temp(&Back.Main_Yaw_Back) == 1)
    {
        if(DeviceStatus.Main_Yaw == Device_Error)
            Main_Yaw_PidInit();
        DeviceStatus.Main_Yaw = Device_Right;
        HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
    }    
    
    /* 底盘 */
    if(IS_Dog(handle, WatchDog.Classis1_WatchDog))
    {
        if(DeviceStatus.Classis1 == Device_Error)
            Pid_Init(NULL, &Pid.Classis1_PID,0,0,0,0,20,0,0,0);
        DeviceStatus.Classis1 = Device_Right;
    }
    if(IS_Dog(handle, WatchDog.Classis2_WatchDog))
    {
        if(DeviceStatus.Classis2 == Device_Error)
            Pid_Init(NULL, &Pid.Classis2_PID,0,0,0,0,20,0,0,0);
        DeviceStatus.Classis2 = Device_Right;
    }    
    if(IS_Dog(handle, WatchDog.Classis3_WatchDog))
    {
        if(DeviceStatus.Classis3 == Device_Error)
            Pid_Init(NULL, &Pid.Classis3_PID,0,0,0,0,20,0,0,0);
        DeviceStatus.Classis3 = Device_Right;
    } 
    if(IS_Dog(handle, WatchDog.Classis4_WatchDog))
    {
        if(DeviceStatus.Classis4 == Device_Error)
            Pid_Init(NULL, &Pid.Classis4_PID,0,0,0,0,20,0,0,0);
        DeviceStatus.Classis4 = Device_Right;
    }     
    if(DeviceStatus.Classis1 == Device_Right && DeviceStatus.Classis2 == Device_Right && DeviceStatus.Classis3 == Device_Right && DeviceStatus.Classis4 == Device_Right)
        HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);

    /* 通信 */
    if(IS_Dog(handle, WatchDog.SuperCap_WatchDog))
    {
        DeviceStatus.SuperCap = Device_Right;
    }      
    if(IS_Dog(handle, WatchDog.Judge_WatchDog))
    {
        DeviceStatus.Judge = Device_Right;
    }    
    if(IS_Dog(handle, WatchDog.Remote_WatchDog))
    {
        DeviceStatus.Remote = Device_Right;
    }
    if(DeviceStatus.SuperCap == Device_Right && DeviceStatus.Judge == Device_Right && DeviceStatus.Remote == Device_Right)
        HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
    
    /* 两头和雷达通信 不亮灯 */
    if(IS_Dog(handle, WatchDog.AimAngle_L_WatchDog))
    {
        DeviceStatus.AimAngle_L = Device_Right;
    }  
    if(IS_Dog(handle, WatchDog.AimAngle_R_WatchDog))
    {
        DeviceStatus.AimAngle_R = Device_Right;
    }  
}

void WatchDog_CallBack(WatchDogp handle)
{
    /* 主Yaw */
    if(IS_Dog(handle, WatchDog.Main_Yaw_WatchDog) || DM4310_Motor_Temp(&Back.Main_Yaw_Back) == 0)
    {
        DeviceStatus.Main_Yaw = Device_Error;
        Pid_Init(&Pid.Main_Yaw_Place_PID, &Pid.Main_Yaw_Speed_PID,0,0,0,0,0,0,0,0);
        HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
    }   
    
    /* 底盘 */
    if(IS_Dog(handle, WatchDog.Classis1_WatchDog))
    {
        DeviceStatus.Classis1 = Device_Error;
        HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
        Pid_Init(NULL, &Pid.Classis1_PID,0,0,0,0,0,0,0,0);
    }
    if(IS_Dog(handle, WatchDog.Classis2_WatchDog))
    {
        DeviceStatus.Classis2 = Device_Error;
        HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
        Pid_Init(NULL, &Pid.Classis2_PID,0,0,0,0,0,0,0,0);
    }    
    if(IS_Dog(handle, WatchDog.Classis3_WatchDog))
    {
        DeviceStatus.Classis3 = Device_Error;
        HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
        Pid_Init(NULL, &Pid.Classis3_PID,0,0,0,0,0,0,0,0);
    }
    if(IS_Dog(handle, WatchDog.Classis4_WatchDog))
    {
        DeviceStatus.Classis4 = Device_Error;
        HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
        Pid_Init(NULL, &Pid.Classis4_PID,0,0,0,0,0,0,0,0);
    }   

    /* 通信 */
    if(IS_Dog(handle, WatchDog.SuperCap_WatchDog))
    {
        DeviceStatus.SuperCap = Device_Error;
        HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
    }
    if(IS_Dog(handle, WatchDog.Judge_WatchDog))
    {
        DeviceStatus.Judge = Device_Error;
        HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
    }
    if(IS_Dog(handle, WatchDog.Remote_WatchDog))
    {
        DeviceStatus.Remote = Device_Error;
        HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
    }

    /* 与两头和导航通信 */
    if(IS_Dog(handle, WatchDog.AimAngle_L_WatchDog))
    {
        DeviceStatus.AimAngle_L = Device_Error;
    }       
    if(IS_Dog(handle, WatchDog.AimAngle_R_WatchDog))
    {
        DeviceStatus.AimAngle_R = Device_Error;
    }
}















