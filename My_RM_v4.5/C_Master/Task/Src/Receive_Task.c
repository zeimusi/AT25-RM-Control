#include "Receive_Task.h"

/*******************************************************************************************************************************************
 * @brief 处理遥控器发过来的数据
 */
Control_Mode_t Control_Mode = Remote_Mode;
uint8_t remote_flag = 0;

extern SemaphoreHandle_t Remote_DMA_Finish_Semaphore; 
TaskHandle_t Remote_Task_Handle;
void Remote_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
	/* 判断是否成功获取信号量的变量 */
    BaseType_t Msg_return_data = pdFALSE;
    
    for(;;)
    {
        Msg_return_data = xSemaphoreTake(Remote_DMA_Finish_Semaphore, portMAX_DELAY);
        if(Msg_return_data == pdTRUE)
        {
            Remote_Rx(usart3_dma_buf);
            Feed_Dog(&WatchDog.Remote_WatchDog);
			remote_flag = 1;
        }
        else if(Msg_return_data == pdFALSE)
        {
            Remote_Rx(NULL);
        }        
        
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

void RemoteControlProcess(Remote *rc) {
    /* 软件复位 */
    if(Control_Mode == Stop_Mode)
    {
	   register uint32_t __regFaultMask       __ASM("faultmask");
	   __regFaultMask = (1 & (uint32_t)1U);
	   HAL_NVIC_SystemReset();    
    }    
    if(Control_Mode != Remote_Mode)
    {
        Control_Mode = Remote_Mode;
        /* 导航、小陀螺模式、遥控模式与底盘跟随模式来回切换时，陀螺仪反馈值，电机角度反馈值与期望值有偏差，要在这里纠正一下 */
        Exp.Main_Yaw_Exp = IMU.EulerAngler.ContinuousYaw;
    }
}

void MouseKeyControlProcess(Mouse *mouse, Key_t key, Key_t Lastkey) {
    /* 软件复位 */
    if(Control_Mode == Stop_Mode)
    {
	   register uint32_t __regFaultMask       __ASM("faultmask");
	   __regFaultMask = (1 & (uint32_t)1U);
	   HAL_NVIC_SystemReset();    
    }
    
    switch(RC_CtrlData.rc.s1)
    {
        case 1:
            if(Control_Mode != Classis_Follow_Mode)
            {
                ResetSlope(&Ramp.Main_Yaw_Ramp);
                Control_Mode = Classis_Follow_Mode;
            }
            break;
        case 3:
            if(Control_Mode != Rotate_Mode)
            {
                Control_Mode = Rotate_Mode;
                /* 导航、小陀螺模式、遥控模式与底盘跟随模式来回切换时，陀螺仪反馈值，电机角度反馈值与期望值有偏差，要在这里纠正一下 */
                Exp.Main_Yaw_Exp = IMU.EulerAngler.ContinuousYaw;
            }
            break;
        case 2:
            if(Control_Mode != Radar_Mode)
            {
                Control_Mode = Radar_Mode;
                /* 导航、小陀螺模式、遥控模式与底盘跟随模式来回切换时，陀螺仪反馈值，电机角度反馈值与期望值有偏差，要在这里纠正一下 */
                Exp.Main_Yaw_Exp = IMU.EulerAngler.ContinuousYaw;
                main_yaw_calculation_exp = IMU.EulerAngler.ContinuousYaw;
            }
            break;
        default: break;
    }
}

void STOPControlProcess(void) {
    Control_Mode = Stop_Mode;    
    Main_Yaw_PidInit();
    Pid_Init(NULL, &Pid.Classis1_PID, 0,0,0,0,0,0,0,0);
    Pid_Init(NULL, &Pid.Classis2_PID, 0,0,0,0,0,0,0,0);
    Pid_Init(NULL, &Pid.Classis3_PID, 0,0,0,0,0,0,0,0);
    Pid_Init(NULL, &Pid.Classis4_PID, 0,0,0,0,0,0,0,0);
    
    memset(&Can_Msg_Tx.yaw_msg_tx, 0, sizeof(Can_Msg_Tx.yaw_msg_tx));
    MotorSend (&hcan2, 0x3FE, Can_Msg_Tx.yaw_msg_tx);  
    
    memset(&Can_Msg_Tx.chassis_msg_tx, 0, sizeof(Can_Msg_Tx.chassis_msg_tx));
    MotorSend(&hcan1, 0x200, Can_Msg_Tx.chassis_msg_tx);    
}

/*******************************************************************************************************************************************
 * @brief 陀螺仪
 */

IMU_Typedef IMU;
void INS_Task(void);

TaskHandle_t StartINS_Task_Handle;
void StartINS_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
        INS_Task();   
        
        vTaskDelayUntil(&xLastWakeTime,1);
    }
}
/*******************************************************************************************************************************************
 * @brief 超电数据接收函数
 */

#define chassis_power_limit 100

Supercap_TypeDef SuperCap_Msg;

TaskHandle_t SuperCap_Task_Handle;
void SuperCap_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
        static uint16_t cnt = 0;
        
//        if(judge_sensor.info->RobotStatus.current_HP == 0)
//        {
//            SuperCAP_Stop(&hcan2, 0x210);
//        }else{
            SuperCAP_Send(&hcan2, 0x210,chassis_power_limit-10,judge_sensor.info->PowerHeatData.buffer_energy);
           
            if(SuperCap_Msg.cap_percent == 0)
            {
                cnt++;
                if(cnt >= 200)
                {
                    SuperCAP_Force2Restart(&hcan2, 0x210);
                    cnt = 0;
                }
            }
//        }
        
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}


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
//extern SemaphoreHandle_t Judge_DMA_Finish_Semaphore; 
void Judge_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
//	/* 判断是否成功获取信号量的变量 */
//    BaseType_t Msg_return_data = pdFALSE;
    
    for(;;)
    {
//        Msg_return_data = xSemaphoreTake(Judge_DMA_Finish_Semaphore, portMAX_DELAY);
//        if(Msg_return_data == pdTRUE)
//        {
            unpack_judge_system_data(&judge_queue);
            Feed_Dog(&WatchDog.Judge_WatchDog);        
//        }
        
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}



/*******************************************************************************************************************************************
 * @brief can回调
 */

/* CAN1中断回调函数：底盘 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t ID = CAN_Receive_DataFrame(&hcan1, CAN1_buff);
     if (hcan->Instance == CAN1)
    {
        switch(ID)
        {
			/* 接受轮子反馈值 */
			case 0x203:
				RM3508_Receive(&Back.Classis1_Back,CAN1_buff);
                Feed_Dog(&WatchDog.Classis1_WatchDog);
				break;
			case 0x202:
				RM3508_Receive(&Back.Classis2_Back,CAN1_buff);
                Feed_Dog(&WatchDog.Classis2_WatchDog);
				break;
			case 0x201:
				RM3508_Receive(&Back.Classis3_Back,CAN1_buff);
                Feed_Dog(&WatchDog.Classis3_WatchDog);
				break;
			case 0x204:
				RM3508_Receive(&Back.Classis4_Back,CAN1_buff);
                Feed_Dog(&WatchDog.Classis4_WatchDog);
				break;
            default: break;
        }
    }
}

/* CAN2中断回调函数:大云台,超电，两头数据（角度，距离，是否识别到目标（用看门狗）） */
AutoAimMsg_t AutoAimMsg_L;
AutoAimMsg_t AutoAimMsg_R;
AutoAimMsg_rx_u AutoAimMsg_rx_L;
AutoAimMsg_rx_u AutoAimMsg_rx_R;
AutoAimXY_t AutoAimXY_L;
AutoAimXY_t AutoAimXY_R;
AutoAimZ_t AutoAimZ_L;
AutoAimZ_t AutoAimZ_R;
GimbleDiff_t GimbleDiff_L;
GimbleDiff_t GimbleDiff_R;
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t ID = CAN_Receive_DataFrame(&hcan2, CAN2_buff);

    if (hcan->Instance == CAN2)
    {
        switch(ID)
        {
            case 0x301:
                DM4310_Receive(&Back.Main_Yaw_Back, CAN2_buff);
			    Feed_Dog(&WatchDog.Main_Yaw_WatchDog);
                break;
            case 0x211:
                SuperCAP_Receive(&SuperCap_Msg, CAN2_buff);
                Feed_Dog(&WatchDog.SuperCap_WatchDog);
                break;
            case 0x411:
                memcpy(&AutoAimMsg_rx_L.can_buff, CAN2_buff, sizeof(CAN2_buff));
            
                if(AutoAimMsg_R.flag == 0)
                    AutoAimMsg_L.flag = 1;
                if(AutoAimMsg_R.flag == 1 && AutoAimMsg_L.flag == 1)//防止两个同时变成1
                    AutoAimMsg_L.flag = 0;
                Feed_Dog(&WatchDog.AimAngle_L_WatchDog);
                break;
            case 0x412:
                memcpy(&AutoAimMsg_rx_R.can_buff, CAN2_buff, sizeof(CAN2_buff));

                if(AutoAimMsg_L.flag == 0)
                    AutoAimMsg_R.flag = 1;
                if(AutoAimMsg_R.flag == 1 && AutoAimMsg_L.flag == 1)//防止两个同时变成1
                    AutoAimMsg_L.flag = 0;
                Feed_Dog(&WatchDog.AimAngle_R_WatchDog);             
                break;        
            case 0x421:
                memcpy(&AutoAimXY_L, CAN2_buff, sizeof(CAN2_buff));
                break;
            case 0x422:
                memcpy(&AutoAimXY_R, CAN2_buff, sizeof(CAN2_buff));
                break;
			case 0x423:
				memcpy(&AutoAimZ_L, CAN2_buff,	sizeof(CAN2_buff));
				break;
			case 0x424:
				memcpy(&AutoAimZ_R, CAN2_buff,	sizeof(CAN2_buff));
				break;
			case 0x431:
                memcpy(&GimbleDiff_L, CAN2_buff, sizeof(CAN2_buff));
                break;
			case 0x432:
                memcpy(&GimbleDiff_R, CAN2_buff, sizeof(CAN2_buff));
                break;
			default: break;
        }
    }
}


