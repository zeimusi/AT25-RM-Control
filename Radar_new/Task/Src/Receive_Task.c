#include "Receive_Task.h"

/*******************************************************************************************************************************************
 * @brief 处理裁判系统信息
 */
uint8_t judge_system_buff[USART6_LEN*2]; 
/* 将接收到的数据转存到缓存数组里 */
uint8_t unpack_judge_system_data(RMQueue_Handle *Queue) {
    uint8_t *judge_ptr = RMQueueTop(Queue);//referee_ptr与队列头是同一个数组
    if (judge_ptr){
        /* referee_system_buff缓存区里可以存两包数据，每来一包新数据就把缓存区里的数据往前顶一包 */
        memcpy(judge_system_buff, judge_system_buff + USART6_LEN, USART6_LEN);
        memcpy(judge_system_buff + USART6_LEN, judge_ptr, USART6_LEN);
        
        RMQueuePop(Queue);//将队列头删除并将指向队列头的指针后移一位
        
        for (uint8_t i = 0; i < USART6_LEN; i++) {
            if (judge_system_buff[i] == 0xA5)
                i += judge_sensor.update(&judge_system_buff[i]);
        }
        return 1;
    }
    return 0;
}

TaskHandle_t Judge_Task_Handle;
void Judge_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
        unpack_judge_system_data(&judge_queue);
        Feed_Dog(&WatchDog.Judge_WatchDog);        

        vTaskDelayUntil(&xLastWakeTime,2);
    }
}
