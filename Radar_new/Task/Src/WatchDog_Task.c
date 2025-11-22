#include "WatchDog_Task.h"

DeviceStatus_ DeviceStatus;
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
 * 亮绿灯：通信有问题（裁判系统）
 */
void FeedDog_CallBack(WatchDogp handle)
{    
  
    if(IS_Dog(handle, WatchDog.Judge_WatchDog))
    {
        DeviceStatus.Judge = Device_Right;
    }    
    if(DeviceStatus.Judge == Device_Right)
        HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_RESET);
}

void WatchDog_CallBack(WatchDogp handle)
{
    /* 通信 */
    if(IS_Dog(handle, WatchDog.Judge_WatchDog))
    {
        DeviceStatus.Judge = Device_Error;
        HAL_GPIO_WritePin(LED_G_GPIO_Port,LED_G_Pin,GPIO_PIN_SET);
    }
}















