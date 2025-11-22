#include "Task_Init.h"

/*******************************************************************************************************************************************
 * @brief 初始化
 * @note 在初始任务中调用
 * @warning 由于裁判系统信息量过大，这里不使用dma失能，空闲中断等影响通信的手段，并采用了环形缓冲区
 */
Pid_t Pid;

Exp_t Exp;    

Back_t Back;

uint8_t usart3_dma_buf[USART3_LEN];
uint8_t usart6_dma_buf[USART6_LEN];
RMQueue_Handle judge_queue;

SemaphoreHandle_t Remote_DMA_Finish_Semaphore; 
//SemaphoreHandle_t Judge_DMA_Finish_Semaphore; 

TaskHandle_t GoMiddle_Task_Handle;
void GoMiddle_Task(void *pvParameters);
void Go_To_Middle(void);

void Start_Task()
{
    Remote_DMA_Finish_Semaphore = xSemaphoreCreateBinary();
//    Judge_DMA_Finish_Semaphore = xSemaphoreCreateBinary();
    
    /* 初始化CAN滤波器 */
    CanFilter_Init(&hcan1);
    CanFilter_Init(&hcan2);

    /* 启动CAN模块 */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    
    /* 启动CAN中断 */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
    
    /* 失能DMA中断 */
    HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
    
    /* 创建裁判系统信息队列,这里相当于创建了一个15x50的二维数组，队列里每一个成员是一个50位的数组 */
	RMQueueInit(&judge_queue,USART6_LEN,15);      
    
    /* 启动串口中断中断源为IDLE */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    /* 以DMA模式接收数据 */
    HAL_UART_Receive_DMA(&huart3, usart3_dma_buf, USART3_LEN);
    HAL_UART_Receive_DMA(&huart6, RMQueueGetEndPtr(&judge_queue), USART6_LEN);
    
    /* 开启PWM（蜂鸣器） */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    
    /* 看门狗初始化 */
    WatchDog_Init(&WatchDog.Classis1_WatchDog, 20);
    WatchDog_Init(&WatchDog.Classis2_WatchDog, 20);    
    WatchDog_Init(&WatchDog.Classis3_WatchDog, 20);
    WatchDog_Init(&WatchDog.Classis4_WatchDog, 20);
    WatchDog_Init(&WatchDog.Main_Yaw_WatchDog, 20);
    WatchDog_Init(&WatchDog.AimAngle_L_WatchDog, 200);
    WatchDog_Init(&WatchDog.AimAngle_R_WatchDog, 200);
    WatchDog_Init(&WatchDog.SuperCap_WatchDog, 100);    
    WatchDog_Init(&WatchDog.Judge_WatchDog, 100);    
    WatchDog_Init(&WatchDog.Remote_WatchDog, 100);    
    
    /* PID参数初始化(机械和imu都用同一套pid) */
    Pid_Init(&Pid.Main_Yaw_Place_PID, &Pid.Main_Yaw_Speed_PID, 5,0.2,-30,20,1,0,0,0);
    Pid_Init(NULL, &Pid.Classis1_PID, 0,0,0,0,20,0,0,0);
    Pid_Init(NULL, &Pid.Classis2_PID, 0,0,0,0,20,0,0,0);
    Pid_Init(NULL, &Pid.Classis3_PID, 0,0,0,0,20,0,0,0);
    Pid_Init(NULL, &Pid.Classis4_PID, 0,0,0,0,20,0,0,0);
//    Pid_Init(&Pid.Main_Yaw_Place_PID, &Pid.Main_Yaw_Speed_PID, 5,0.2,-30,20,1,0,0,1);

    xTaskCreate(WatchDog_Task        , "WatchDog_Task"        , 128 , NULL, osPriorityNormal     , &WatchDog_Task_Handle);
    xTaskCreate(StartINS_Task        , "StartINS_Task"        , 1024, NULL, osPriorityHigh       , &StartINS_Task_Handle);
    xTaskCreate(Remote_Task          , "Remote_Task"          , 128 , NULL, osPriorityAboveNormal, &Remote_Task_Handle);
    xTaskCreate(SuperCap_Task        , "SuperCap_Task"        , 128 , NULL, osPriorityNormal     , &SuperCap_Task_Handle);
    xTaskCreate(Judge_Task           , "Judge_Task"           , 128 , NULL, osPriorityAboveNormal, &Judge_Task_Handle);    
    xTaskCreate(Send_Task            , "Send_Task"            , 128 , NULL, osPriorityNormal     , &Send_Task_handle);
    xTaskCreate(usb_task             , "usb_task"             , 128 , NULL, osPriorityNormal     , &usb_task_handle);
    xTaskCreate(Double_Gimble_Task   , "Double_Gimble_Task"   , 128 , NULL, osPriorityNormal     , &Double_Gimble_Task_handle);        
    xTaskCreate(GoMiddle_Task        , "GoMiddle_Task"        , 128 , NULL, osPriorityNormal     , &GoMiddle_Task_Handle);
}

/*******************************************************************************************************************************************
 * @brief 归中
 * @warning 若看门狗疯，则对应设备pid归零，归中仍能正常进行
 */
Ramp_ Ramp = {.Main_Yaw_Ramp.RampTime = 1000};

void GoMiddle_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
	
    for(;;)
    {
        /* 电机在不在线，无法完成归中，后续无法开始，这是由于板子连pc供电，上电之后pc供电口最先有电 */
        if(Slope(&Ramp.Main_Yaw_Ramp) == 1)
        {
            portENTER_CRITICAL();
                        
            /* 如果归中的时候经过了0点，由于归中时用的反馈值为机械角度，那么此时期望值 = 机械角度，但遥控器控制时用的反馈值为陀螺仪，与期望值偏差过大，会疯转，所以这里将期望值矫正一下 */
            Exp.Main_Yaw_Exp = IMU.EulerAngler.ContinuousYaw;
            
            /* 如果归中的时候经过了0点，由于归中时用的反馈值为机械角度，那么此时期望值 = 机械角度，但控制时用的反馈值为连续机械角度，连续机械角度 = 机械角度+-8191，与期望值偏差过大，会疯转，所以这里将连续机械角度矫正一下 */
            Back.Main_Yaw_Back.flag = 0;
            Back.Main_Yaw_Back.r = 0;
            Back.Main_Yaw_Back.Angle = Back.Main_Yaw_Back.MchanicalAngle;            
            
            Main_Yaw_PidInit();
            
            xTaskCreate(Main_Yaw_Task, "Main_Yaw_Task", 128, NULL, osPriorityNormal, &Main_Yaw_Task_Handle);
            xTaskCreate(Chassis_Task, "Chassis_Task", 128, NULL, osPriorityNormal, &Chassis_Task_Handle);
            
            ResetSlope(&Ramp.Main_Yaw_Ramp);
                        
            portEXIT_CRITICAL();
            vTaskDelete(NULL);        
        }else{
            Go_To_Middle();
        }

        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

void Go_To_Middle(void)
{
    if(DeviceStatus.Main_Yaw == Device_Right)
    {
        Ramp.Main_Yaw_Diff = QuickCentering(Back.Main_Yaw_Back.MchanicalAngle, Main_Yaw_Median) - Back.Main_Yaw_Back.MchanicalAngle;
        
        Exp.Main_Yaw_Exp = (QuickCentering(Back.Main_Yaw_Back.MchanicalAngle, Main_Yaw_Median) - Ramp.Main_Yaw_Diff*(1 - Slope(&Ramp.Main_Yaw_Ramp)))*360/8191;           

        PID_Control_Smis(Back.Main_Yaw_Back.MchanicalAngle_DEG, Exp.Main_Yaw_Exp, &Pid.Main_Yaw_Place_PID, IMU.AngularVelocity.z);
        PID_Control(IMU.AngularVelocity.z, Pid.Main_Yaw_Place_PID.pid_out, &Pid.Main_Yaw_Speed_PID);
        limit(Pid.Main_Yaw_Speed_PID.pid_out, DM4310_LIMIT, -DM4310_LIMIT);
        Can_Msg_Tx.yaw_msg_tx[0] = Pid.Main_Yaw_Speed_PID.pid_out;
        
        /* 将pid.out发送 */
        MotorSend (&hcan2, 0x3FE, Can_Msg_Tx.yaw_msg_tx);        
    }else{
        ResetSlope(&Ramp.Main_Yaw_Ramp);
    }
}



