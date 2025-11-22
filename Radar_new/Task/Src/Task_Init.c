#include "Task_Init.h"

/*******************************************************************************************************************************************
 * @brief 初始化
 * @note 在初始任务中调用
 * @warning 由于裁判系统信息量过大，这里不使用dma失能，空闲中断等影响通信的手段，并采用了环形缓冲区
 */

uint8_t usart6_dma_buf[USART6_LEN];
RMQueue_Handle judge_queue;


void Start_Task()
{
    /* 创建裁判系统信息队列,这里相当于创建了一个15x50的二维数组，队列里每一个成员是一个50位的数组 */
	RMQueueInit(&judge_queue,USART6_LEN,15);      
    
    /* 以DMA模式接收数据 */
    HAL_UART_Receive_DMA(&huart6, RMQueueGetEndPtr(&judge_queue), USART6_LEN);
    
    /* 开启PWM（蜂鸣器） */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    
    /* 看门狗初始化 */ 
    WatchDog_Init(&WatchDog.Judge_WatchDog, 100);    
   
    xTaskCreate(WatchDog_Task        , "WatchDog_Task"        , 128 , NULL, osPriorityNormal     , &WatchDog_Task_Handle);
    xTaskCreate(Judge_Task           , "Judge_Task"           , 128 , NULL, osPriorityAboveNormal, &Judge_Task_Handle);    
    xTaskCreate(Send_Task            , "Send_Task"            , 128 , NULL, osPriorityNormal     , &Send_Task_handle);
    xTaskCreate(usb_task             , "usb_task"             , 128 , NULL, osPriorityNormal     , &usb_task_handle);
	xTaskCreate(send,"send",128,NULL,osPriorityNormal,&send_handle);
}








