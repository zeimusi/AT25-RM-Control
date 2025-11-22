#include "WatchDog_Task.h"

WatchDog_t WatchDog;
DeviceStatus_ DeviceStatus = {.PC_State = Device_Error};

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
 * 亮红灯：射击结构有问题
 * 亮蓝灯：云台结构有问题
 * 亮绿灯：板间通信有问题(遥控器，裁判系统)
 */

void FeedDog_CallBack(WatchDogp handle)
{    
    /*****************        云台         ******************/
    if(IS_Dog(handle, WatchDog.Gimbal_Pitch_WatchDog))
    {
        if(DeviceStatus.Gimbal_Pitch == Device_Error)
            Gimbal_PidInit();
        DeviceStatus.Gimbal_Pitch = Device_Right;
    }
    if(IS_Dog(handle, WatchDog.Gimbal_Yaw_WatchDog))
    {
        if(DeviceStatus.Gimbal_Yaw == Device_Error)
            Gimbal_PidInit();
        DeviceStatus.Gimbal_Yaw = Device_Right;
    }
    if(DeviceStatus.Gimbal_Pitch == Device_Right && DeviceStatus.Gimbal_Yaw == Device_Right)
        HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_RESET);
    
    /*****************        射击         ******************/
    if(IS_Dog(handle, WatchDog.R_Friction_WatchDog))
    {
        if(DeviceStatus.R_Friction == Device_Error)
            Pid_Init(NULL, &Pid.R_Friction_PID,0,0,0,0,7,0,0,0);
        DeviceStatus.R_Friction = Device_Right;
    }
    if(IS_Dog(handle, WatchDog.L_Friction_WatchDog))
    {
        if(DeviceStatus.L_Friction == Device_Error)
            Pid_Init(NULL, &Pid.L_Friction_PID,0,0,0,0,7,0,0,0);
        DeviceStatus.L_Friction = Device_Right;
    }    
    if(IS_Dog(handle, WatchDog.Pluck_WatchDog))
    {
        if(DeviceStatus.Pluck == Device_Error)
            Pid_Init(NULL, &Pid.Pluck_PID,0,0,0,0,7,0,0,0);
        DeviceStatus.Pluck = Device_Right;
    }    
    if(DeviceStatus.L_Friction == Device_Right && DeviceStatus.R_Friction == Device_Right && DeviceStatus.Pluck == Device_Right)
        HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_RESET);
    
    /*****************        通信         ******************/
    if(IS_Dog(handle, WatchDog.Remote_WatchDog))
    {
        DeviceStatus.Remote = Device_Right;
    }        
    if(IS_Dog(handle, WatchDog.Judge_WatchDog))
    {
        DeviceStatus.Judge = Device_Right;
    } 
    if(DeviceStatus.Remote == Device_Right && DeviceStatus.Judge == Device_Right)
        HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
    
    if(IS_Dog(handle, WatchDog.PC_WatchDog))
        DeviceStatus.PC_State = Device_Right;      
}

void WatchDog_CallBack(WatchDogp handle)
{
    /*****************        云台         ******************/
    if(IS_Dog(handle, WatchDog.Gimbal_Pitch_WatchDog))
    {
        DeviceStatus.Gimbal_Pitch = Device_Error;
        Pid_Init(&Pid.Gimbal_Pitch_Place_PID, &Pid.Gimbal_Pitch_Speed_PID,0,0,0,0,0,0,0,0);
        Pid_Init(&Pid.Gimbal_Pitch_Place_PID_imu, &Pid.Gimbal_Pitch_Speed_PID_imu,0,0,0,0,0,0,0,0);
        HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
    }
    if(IS_Dog(handle, WatchDog.Gimbal_Yaw_WatchDog))
    {
        DeviceStatus.Gimbal_Yaw = Device_Error;
        Pid_Init(&Pid.Gimbal_Yaw_Place_PID, &Pid.Gimbal_Yaw_Speed_PID,0,0,0,0,0,0,0,0);
        Pid_Init(&Pid.Gimbal_Yaw_Place_PID_imu, &Pid.Gimbal_Yaw_Speed_PID_imu,0,0,0,0,0,0,0,0);
        HAL_GPIO_WritePin(LED_B_GPIO_Port,LED_B_Pin,GPIO_PIN_SET);
    }
   
    /*****************        射击         ******************/
    if(IS_Dog(handle, WatchDog.R_Friction_WatchDog))
    {
        DeviceStatus.R_Friction = Device_Error;
        HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
        Pid_Init(NULL, &Pid.R_Friction_PID,0,0,0,0,0,0,0,0);
    }
    if(IS_Dog(handle, WatchDog.L_Friction_WatchDog))
    {
        DeviceStatus.L_Friction = Device_Error;
        HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
        Pid_Init(NULL, &Pid.L_Friction_PID,0,0,0,0,0,0,0,0);
    }    
    if(IS_Dog(handle, WatchDog.Pluck_WatchDog))
    {
        DeviceStatus.Pluck = Device_Error;
        HAL_GPIO_WritePin(LED_R_GPIO_Port,LED_R_Pin,GPIO_PIN_SET);
        Pid_Init(NULL, &Pid.Pluck_PID,0,0,0,0,0,0,0,0);
    }
    
    /*****************        通信         ******************/
    if(IS_Dog(handle, WatchDog.Remote_WatchDog))
    {
        DeviceStatus.Remote = Device_Error;
        HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
    }    
    if(IS_Dog(handle, WatchDog.Judge_WatchDog))
    {
        DeviceStatus.Judge = Device_Error;
        HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
    }    

    if(IS_Dog(handle, WatchDog.PC_WatchDog))
        DeviceStatus.PC_State = Device_Error;
}





