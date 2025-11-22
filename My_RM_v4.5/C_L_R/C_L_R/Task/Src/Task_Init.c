#include "Task_Init.h"
/*******************************************************************************************************************************************
 * @brief 初始化
 * @note 在初始任务中调用
 */
Pid_t Pid;

Exp_t Exp;    

Back_t Back;

TaskHandle_t GoMiddle_Task_Handle;
void GoMiddle_Task(void *pvParameters);
void Go_To_Middle(void);

void Start_Task()
{
    /* 初始化CAN滤波器 */
    CanFilter_Init(&hcan1);
    CanFilter_Init(&hcan2);

    /* 启动CAN模块 */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
    
    /* 启动CAN中断 */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
    
    /* 开启PWM（蜂鸣器） */
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    
    /* 看门狗初始化 */
    WatchDog_Init(&WatchDog.Gimbal_Pitch_WatchDog, 20);
    WatchDog_Init(&WatchDog.Gimbal_Yaw_WatchDog, 20);    
    WatchDog_Init(&WatchDog.L_Friction_WatchDog, 20);
    WatchDog_Init(&WatchDog.R_Friction_WatchDog, 20);
    WatchDog_Init(&WatchDog.Pluck_WatchDog, 20);
    WatchDog_Init(&WatchDog.Remote_WatchDog, 100);
    WatchDog_Init(&WatchDog.Judge_WatchDog, 100);
    WatchDog_Init(&WatchDog.PC_WatchDog, 100);//视觉1s内没给我发数据证明狗就疯，证明现在没有目标装甲板
      
    /* PID参数初始化 */
#if L_R    
    Gimbal_PidInit();
    Pid_Init(NULL, &Pid.L_Friction_PID, 0,0,0,0,7,0,0,0);
    Pid_Init(NULL, &Pid.R_Friction_PID, 0,0,0,0,7,0,0,0);
    Pid_Init(NULL, &Pid.Pluck_PID, 0,0,0,0,7,0,0,0);
#else 
    Gimbal_PidInit();
    Pid_Init(NULL, &Pid.L_Friction_PID, 0,0,0,0,7,0,0,0);
    Pid_Init(NULL, &Pid.R_Friction_PID, 0,0,0,0,7,0,0,0);
    Pid_Init(NULL, &Pid.Pluck_PID, 0,0,0,0,7,0,0,0);
#endif

    xTaskCreate(WatchDog_Task, "WatchDog_Task", 128, NULL, osPriorityNormal, &WatchDog_Task_Handle);
    xTaskCreate(Send_Task    , "Send_Task"    , 256, NULL, osPriorityNormal, &Send_Task_handle);
    xTaskCreate(Remote_Task  , "Remote_Task"  , 128, NULL, osPriorityNormal, &Remote_Task_Handle);
    xTaskCreate(StartINS_Task, "StartINS_Task", 1024, NULL,osPriorityAboveNormal  , &StartINS_Task_Handle);
    xTaskCreate(Vision_Task  , "Vision_Task"  , 256, NULL, osPriorityNormal, &Vision_Task_Handle);
    xTaskCreate(GoMiddle_Task, "GoMiddle_Task", 128, NULL, osPriorityNormal, &GoMiddle_Task_Handle);
}

/*******************************************************************************************************************************************
 * @brief 归中
 * @warning 若看门狗疯，则对应设备pid归零，归中仍能正常进行
 */

typedef struct{
    Ramp_Typedef Gimbal_Yaw_Ramp;
    Ramp_Typedef Gimbal_Pitch_Ramp;
    int16_t Gimbal_Yaw_diff;
    int16_t Gimbal_Pitch_diff;
}Ramp_;
Ramp_ Ramp = {.Gimbal_Yaw_Ramp.RampTime = 1000, .Gimbal_Pitch_Ramp.RampTime = 1000};

void GoMiddle_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
	
    for(;;)
    {
        /* 无论电机在不在线，时间一到就代表完成归中，如果电机不在线，在看门狗那里会将电机pid置0， 相应机构不会动 */
        if(Slope(&Ramp.Gimbal_Pitch_Ramp) == 1 && Slope(&Ramp.Gimbal_Yaw_Ramp) == 1)
        {
            portENTER_CRITICAL();          
            
            /* 如果归中的时候经过了0点，由于归中时用的反馈值为机械角度，那么此时期望值 = 机械角度，但控制时用的反馈值为连续机械角度，连续机械角度 = 机械角度+-8191，与期望值偏差过大，会疯转，所以这里将连续机械角度矫正一下 */
            Back.Gimbal_Yaw_Back.flag = 0;
            Back.Gimbal_Yaw_Back.r = 0;
            Back.Gimbal_Yaw_Back.Angle = Back.Gimbal_Yaw_Back.MchanicalAngle;
            Exp.Gimbal_Pitch_Exp_imu = IMU.EulerAngler.Pitch;
            Exp.Gimbal_Yaw_Exp_imu = IMU.EulerAngler.ContinuousYaw;
            Exp.Gimbal_Pitch_Exp = Back.Gimbal_Pitch_Back.Angle;
            Exp.Gimbal_Yaw_Exp = Back.Gimbal_Yaw_Back.Angle;
            
            ResetSlope(&Ramp.Gimbal_Pitch_Ramp);
            ResetSlope(&Ramp.Gimbal_Yaw_Ramp);

            xTaskCreate(Gimbal_Task      , "Gimbal_Task"      , 128, NULL, osPriorityNormal, &Gimbal_Task_Handle);
            xTaskCreate(Shoot_Task       , "Shoot_Task"       , 128, NULL, osPriorityNormal, &Shoot_Task_Handle);
            xTaskCreate(Shoot_Detect_Task, "Shoot_Detect_Task", 128, NULL, osPriorityNormal, &Shoot_Detect_Task_Handle);
            xTaskCreate(usb_task         , "usb_task"         , 256, NULL, osPriorityNormal, &usb_task_handle);
            
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
    if(DeviceStatus.Gimbal_Pitch == Device_Right && DeviceStatus.Gimbal_Yaw == Device_Right)
    {
        Ramp.Gimbal_Pitch_diff = QuickCentering(Back.Gimbal_Pitch_Back.MchanicalAngle, Pitch_Median) - Back.Gimbal_Pitch_Back.MchanicalAngle;
        Ramp.Gimbal_Yaw_diff = QuickCentering(Back.Gimbal_Yaw_Back.MchanicalAngle, Yaw_Median) - Back.Gimbal_Yaw_Back.MchanicalAngle;
        
        Exp.Gimbal_Pitch_Exp = QuickCentering(Back.Gimbal_Pitch_Back.MchanicalAngle, Pitch_Median) - Ramp.Gimbal_Pitch_diff*(1 - Slope(&Ramp.Gimbal_Pitch_Ramp));           
        Exp.Gimbal_Yaw_Exp = QuickCentering(Back.Gimbal_Yaw_Back.MchanicalAngle, Yaw_Median) - Ramp.Gimbal_Yaw_diff*(1 - Slope(&Ramp.Gimbal_Yaw_Ramp));           

        PID_Control_Smis(Back.Gimbal_Pitch_Back.MchanicalAngle, Exp.Gimbal_Pitch_Exp, &Pid.Gimbal_Pitch_Place_PID, Back.Gimbal_Pitch_Back.Speed);
        PID_Control(Back.Gimbal_Pitch_Back.Speed, Pid.Gimbal_Pitch_Place_PID.pid_out, &Pid.Gimbal_Pitch_Speed_PID);
        limit(Pid.Gimbal_Pitch_Speed_PID.pid_out, GM6020_LIMIT, -GM6020_LIMIT);
        
        PID_Control_Smis(Back.Gimbal_Yaw_Back.MchanicalAngle, Exp.Gimbal_Yaw_Exp, &Pid.Gimbal_Yaw_Place_PID, Back.Gimbal_Yaw_Back.Speed);
        PID_Control(Back.Gimbal_Yaw_Back.Speed, Pid.Gimbal_Yaw_Place_PID.pid_out, &Pid.Gimbal_Yaw_Speed_PID);
        limit(Pid.Gimbal_Yaw_Speed_PID.pid_out, GM6020_LIMIT, -GM6020_LIMIT);
 
#if L_R        
        Can_Msg_Tx.gimble_msg_tx[0] = Pid.Gimbal_Pitch_Speed_PID.pid_out;
        Can_Msg_Tx.gimble_msg_tx[1] = Pid.Gimbal_Yaw_Speed_PID.pid_out;
#else
        Can_Msg_Tx.gimble_msg_tx[2] = Pid.Gimbal_Pitch_Speed_PID.pid_out;
        Can_Msg_Tx.gimble_msg_tx[3] = Pid.Gimbal_Yaw_Speed_PID.pid_out;
#endif
        /* 将pid.out发送 */
        MotorSend (&hcan1, 0x1FF, Can_Msg_Tx.gimble_msg_tx);        
    }else{
        ResetSlope(&Ramp.Gimbal_Pitch_Ramp);
        ResetSlope(&Ramp.Gimbal_Yaw_Ramp);
    }
}



