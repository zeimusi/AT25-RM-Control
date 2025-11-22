#include "Init_Task.h"
#include "BMI088driver.h"
#include "bsp_dwt.h"
#include "spi.h"

void Init_Task(){
    taskENTER_CRITICAL(); // 进入临界区
	/* CAN1初始化（拨弹盘电机2006、两个摩擦轮电机3508) */
    CanFilter_Init(&hcan1);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	
    /* CAN2初始化（上下板通信（速度，状态、裁判系统）、Yaw轴6020、Pitch轴电机6020) */
    CanFilter_Init(&hcan2);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);

	/* 串口3初始化（遥控器） */
    remote_control_init();
    /* 看门狗初始化（初始化顺序为看门狗ID顺序（ID从1开始）） */
    WatchDog_Init(&Remote_Dog, 20);
    WatchDog_Init(&IMU_Dog, 20);
    WatchDog_Init(&Gimbal_Dog[YAW], 10);
    WatchDog_Init(&Gimbal_Dog[PITCH], 10);
    WatchDog_Init(&Shoot_Dog[LEFT], 10);
    WatchDog_Init(&Shoot_Dog[RIGHT], 10);
    WatchDog_Init(&Pluck_Dog, 10);
    WatchDog_Init(&Down_Dog, 50);
    WatchDog_Init(&PC_Dog, 100);
    /* 定时器初始化tim3 1000hz */
    HAL_TIM_Base_Start_IT(&htim3); 
		

		xTaskCreate((TaskFunction_t)Serialport_Task,	 "Serialport_Task",   256*2,   NULL, 2, &Serialport_Task_handle);
    xTaskCreate((TaskFunction_t)Aim_Task,          "Aim_Task",          256*4, NULL, 4, &Aim_Task_handle);
    xTaskCreate((TaskFunction_t)Shoot_Task,			   "Shoot_Task",        256,   NULL, 4, &Shoot_Task_handle);		
    xTaskCreate((TaskFunction_t)Gimbal_Task,       "Gimbal_Task",       256,   NULL, 4, &Gimbal_Task_handle);
    xTaskCreate((TaskFunction_t)MainCtrl_Task,     "MainCtrl_Task",     256,   NULL, 7, &MainCtrl_Task_handle);

	  taskEXIT_CRITICAL(); // 退出临界区 
    vTaskDelete(NULL);   // 删除开始空闲任务	
}

