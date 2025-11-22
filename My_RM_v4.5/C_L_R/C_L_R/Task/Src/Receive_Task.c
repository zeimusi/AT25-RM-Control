#include "Receive_Task.h"

/*******************************************************************************************************************************************
 * @brief 处理主控板发过来的数据
 * @note 包括遥控器信息
 */
Control_Mode_t Control_Mode = Remote_Mode;
uint8_t Remote_Msg_rx[8];
TaskHandle_t Remote_Task_Handle;
void Remote_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
	    
    for(;;)
    {
        if(Remote_Msg_rx[0] == 0xa5)
        {
            Remote_Rx(&Remote_Msg_rx[1]);
        }else{
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

        /* 自瞄模式与遥控模式来回切换时，陀螺仪反馈值，电机角度反馈值与期望值有偏差，要在这里纠正一下 */
        Exp.Gimbal_Pitch_Exp = Back.Gimbal_Pitch_Back.Angle;
        Exp.Gimbal_Yaw_Exp = Back.Gimbal_Yaw_Back.Angle;
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
    
#if single_double
    /* 视觉不发数据代表pc掉线，开启巡逻模式 */
    if(Aim_Rx.aim_runing == 0 && Control_Mode != Patrol_Mode)
    {
        Control_Mode = Patrol_Mode;
        /* 自瞄模式与遥控巡逻模式来回切换时，陀螺仪反馈值，电机角度反馈值与期望值有偏差，要在这里纠正一下 */
        Exp.Gimbal_Pitch_Exp = Back.Gimbal_Pitch_Back.Angle;
        Exp.Gimbal_Yaw_Exp = Back.Gimbal_Yaw_Back.Angle;
    }else if(Aim_Rx.aim_runing == 1 && Control_Mode != AutoAim_Mode)
    {
        Control_Mode = AutoAim_Mode;    
        /* 自瞄模式与遥控巡逻模式来回切换时，陀螺仪反馈值，电机角度反馈值与期望值有偏差，要在这里纠正一下 */
        Exp.Gimbal_Pitch_Exp_imu = IMU.EulerAngler.Pitch;
        Exp.Gimbal_Yaw_Exp_imu = IMU.EulerAngler.ContinuousYaw;
    }
#else
        Double_Guns_Combat();
#endif
}

void STOPControlProcess(void) {
    Control_Mode = Stop_Mode;    
    Gimbal_PidInit();
    Pid_Init(NULL, &Pid.L_Friction_PID, 0,0,0,0,0,0,0,0);
    Pid_Init(NULL, &Pid.R_Friction_PID, 0,0,0,0,0,0,0,0);
    Pid_Init(NULL, &Pid.Pluck_PID, 0,0,0,0,0,0,0,0);

    memset(&Can_Msg_Tx.gimble_msg_tx, 0, sizeof(Can_Msg_Tx.gimble_msg_tx));
    MotorSend(&hcan1, 0x1FF, Can_Msg_Tx.gimble_msg_tx);
    
    memset(&Can_Msg_Tx.shoot_msg_tx, 0, sizeof(Can_Msg_Tx.shoot_msg_tx));
    MotorSend(&hcan1, 0x200, Can_Msg_Tx.shoot_msg_tx);
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
 * @brief can回调
 */

/* CAN1中断回调函数：射弹云台 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t ID = CAN_Receive_DataFrame(&hcan1, CAN1_buff);
     if (hcan->Instance == CAN1)
    {
        switch(ID)
        {
            case 0x203:
                M2006_Receive(&Back.Pluck_Back, CAN1_buff);
                Feed_Dog(&WatchDog.Pluck_WatchDog);
                break;			
            case 0x202:
                RM3508_Receive(&Back.R_Friction_Back, CAN1_buff);
                Feed_Dog(&WatchDog.R_Friction_WatchDog);  
                break;
            case 0x201:
                RM3508_Receive(&Back.L_Friction_Back, CAN1_buff);
                Feed_Dog(&WatchDog.L_Friction_WatchDog);
                break;
#if L_R            
            case 0x205:
                GM6020_Receive(&Back.Gimbal_Pitch_Back, CAN1_buff);
				Feed_Dog(&WatchDog.Gimbal_Pitch_WatchDog);
                break;
            case 0x206:
                GM6020_Receive(&Back.Gimbal_Yaw_Back, CAN1_buff);
				Feed_Dog(&WatchDog.Gimbal_Yaw_WatchDog);
                break;
#else            
            case 0x207:
                GM6020_Receive(&Back.Gimbal_Pitch_Back, CAN1_buff);
				Feed_Dog(&WatchDog.Gimbal_Pitch_WatchDog);
                break;
            case 0x208:
                GM6020_Receive(&Back.Gimbal_Yaw_Back, CAN1_buff);
				Feed_Dog(&WatchDog.Gimbal_Yaw_WatchDog);
                break;            
#endif            
			default: break;
        }
    }
}

/* CAN2中断回调函数: 通信:遥控器，裁判系统（颜色，比赛状态，枪口热量及上限，伤害状态，射击数据，允许发弹量）,双头合作信息*/

Judge_Msg_t Judge_Msg;
DoubleGimble_Msg_t DoubleGimble_Msg;
Is_Fortress_Msg_t Is_Fortress_Msg;
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t ID = CAN_Receive_DataFrame(&hcan2, CAN2_buff);

    if (hcan->Instance == CAN2)
    {
        switch(ID)
        {
            case 0x404:
                memcpy(&Remote_Msg_rx, CAN2_buff, sizeof(Remote_Msg_rx));
                Feed_Dog(&WatchDog.Remote_WatchDog);
                break;
            case 0x405:
                Judge_Msg.head = CAN2_buff[0];
                if(Judge_Msg.head == 0xa5)
                {
                    memcpy(&Judge_Msg, CAN2_buff, sizeof(Judge_Msg_t));
                    Feed_Dog(&WatchDog.Judge_WatchDog);
                }
                break;
            case 0x401:
                DoubleGimble_Msg.MainYaw_Arrive_flag = (MainYaw_Arrive_flag_t)CAN2_buff[0];
                break;
#if L_R                
            case 0x402:
                memcpy(&DoubleGimble_Msg, CAN2_buff, sizeof(CAN2_buff));
                break;
#else
            case 0x403:
                memcpy(&DoubleGimble_Msg, CAN2_buff, sizeof(CAN2_buff));
                break;
#endif            			
			case 0x406:
				memcpy(&Is_Fortress_Msg, CAN2_buff, sizeof(CAN2_buff));				
				break;			
            default: break;
        }
    }
}







