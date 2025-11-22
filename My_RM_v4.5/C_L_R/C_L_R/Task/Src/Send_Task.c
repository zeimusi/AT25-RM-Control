#include "Send_Task.h"

Can_Msg_Tx_t Can_Msg_Tx;
DouAutoAim_Status_t DouAutoAim_Status;

void AutoAimAngle_Msg_Send(void);
void AutoAimXYZ_Msg_Send(void);

/*************************      发布数据任务     *******************************/
TaskHandle_t Send_Task_handle;
void Send_Task(void *pvParameters)
{    
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
        if(Control_Mode == AutoAim_Mode)
        {
            AutoAimAngle_Msg_Send();
//            AutoAimXYZ_Msg_Send();
			GimbleDiff_Msg_Send();
        }
        
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}




/***********************       发布数据函数         *******************************/
/**
 * @brief 发布自瞄角度数据（双头合作需要） 
 */
void AutoAimAngle_Msg_Send()
{
    if(Aim_Rx.aim_runing == 1)
    {
        Can_Msg_Tx.AutoAimAngle_Msg_tx.AutoAimAngle_Msg_tx.DouAutoAim_Status = DouAutoAim_Status;

        Can_Msg_Tx.AutoAimAngle_Msg_tx.AutoAimAngle_Msg_tx.Pitch = Aim_Ref.Pitch*10;
        Can_Msg_Tx.AutoAimAngle_Msg_tx.AutoAimAngle_Msg_tx.HorizontalDistance = Aim_Ref.HorizontalDistance*10;
        
#if L_R    
        if(fabsf(Aim_Ref.Yaw-IMU.EulerAngler.ContinuousYaw)>3)                                                                                                             
            Can_Msg_Tx.AutoAimAngle_Msg_tx.AutoAimAngle_Msg_tx.Yaw = (((Aim_Ref.Yaw-IMU.EulerAngler.ContinuousYaw)*8191/360+Back.Gimbal_Yaw_Back.Angle) - Yaw_Median)*360/8191*10;
        else
            Can_Msg_Tx.AutoAimAngle_Msg_tx.AutoAimAngle_Msg_tx.Yaw = (Back.Gimbal_Yaw_Back.Angle - Yaw_Median)*360/8191*10;
                   
        if(Back.Gimbal_Yaw_Back.Angle < Yaw_Median-200 || Back.Gimbal_Yaw_Back.Angle > Yaw_Median+200)
            Can_Msg_Tx.AutoAimAngle_Msg_tx.AutoAimAngle_Msg_tx.IsMainYawMove = 1;
        else
            Can_Msg_Tx.AutoAimAngle_Msg_tx.AutoAimAngle_Msg_tx.IsMainYawMove = 0; 
        
        CAN_Send_StdDataFrame(&hcan2, 0x411,Can_Msg_Tx.AutoAimAngle_Msg_tx.can_buff);         
#else
        if(fabsf(Aim_Ref.Yaw-IMU.EulerAngler.ContinuousYaw)>3)                                                                                                             
            Can_Msg_Tx.AutoAimAngle_Msg_tx.AutoAimAngle_Msg_tx.Yaw = (Yaw_Median - ((Aim_Ref.Yaw-IMU.EulerAngler.ContinuousYaw)*8191/360+Back.Gimbal_Yaw_Back.Angle))*360/8191*10;
        else
            Can_Msg_Tx.AutoAimAngle_Msg_tx.AutoAimAngle_Msg_tx.Yaw = (Yaw_Median - Back.Gimbal_Yaw_Back.Angle)*360/8191*10;
                   
        if(Back.Gimbal_Yaw_Back.Angle < Yaw_Median-200 || Back.Gimbal_Yaw_Back.Angle > Yaw_Median+200)
            Can_Msg_Tx.AutoAimAngle_Msg_tx.AutoAimAngle_Msg_tx.IsMainYawMove = 1;
        else
            Can_Msg_Tx.AutoAimAngle_Msg_tx.AutoAimAngle_Msg_tx.IsMainYawMove = 0; 
        
        CAN_Send_StdDataFrame(&hcan2, 0x412,Can_Msg_Tx.AutoAimAngle_Msg_tx.can_buff);         
#endif    
    }
}


/**
 * @brief 发布自瞄坐标数据（导航需要）
 */
void AutoAimXYZ_Msg_Send()
{
    if(Aim_Rx.aim_runing == 1)
    {
		Can_Msg_Tx.AutoAimXY_Msg_tx.AutoAimXY_Msg_tx.x = pointGun[0];
		Can_Msg_Tx.AutoAimXY_Msg_tx.AutoAimXY_Msg_tx.y = pointGun[1];  
		
#if L_R    
		Can_Msg_Tx.AutoAimZ_Msg_tx.AutoAimZ_Msg_tx.L_z = pointGun[2];
        CAN_Send_StdDataFrame(&hcan2, 0x421,Can_Msg_Tx.AutoAimXY_Msg_tx.can_buff);       
		CAN_Send_StdDataFrame(&hcan2, 0x423,Can_Msg_Tx.AutoAimZ_Msg_tx.can_buff);
#else
		Can_Msg_Tx.AutoAimZ_Msg_tx.AutoAimZ_Msg_tx.R_z = pointGun[2];
        CAN_Send_StdDataFrame(&hcan2, 0x422,Can_Msg_Tx.AutoAimXY_Msg_tx.can_buff);       
		CAN_Send_StdDataFrame(&hcan2, 0x424,Can_Msg_Tx.AutoAimZ_Msg_tx.can_buff);
#endif   
    }        
}



void GimbleDiff_Msg_Send()
{  
	Can_Msg_Tx.GimbelDiff_Msg_t.GimbelDiff_Msg_tx.Pitch_diff = (Back.Gimbal_Pitch_Back.Angle - Pitch_Median)*360*1.0/8192;
	Can_Msg_Tx.GimbelDiff_Msg_t.GimbelDiff_Msg_tx.Yaw_diff = (Back.Gimbal_Yaw_Back.Angle - Yaw_Median)*360*1.0/8192;
	
#if L_R    
	CAN_Send_StdDataFrame(&hcan2, 0x431,Can_Msg_Tx.GimbelDiff_Msg_t.can_buff);       
#else
	CAN_Send_StdDataFrame(&hcan2, 0x432,Can_Msg_Tx.GimbelDiff_Msg_t.can_buff);       
#endif   
}




