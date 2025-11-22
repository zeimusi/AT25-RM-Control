#include "Send_Task.h"

Can_Msg_Tx_t Can_Msg_Tx;

void Remote_Msg_Send(void);
void Judge_Msg_Send(void);
void DoubleGimble_Msg_Send(void);
void Is_Fortress_Msg_Send(void);

extern uint8_t remote_flag;
/*************************      发布数据任务     *******************************/
TaskHandle_t Send_Task_handle;
void Send_Task(void *pvParameters)
{    
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
		if(remote_flag == 1)
			Remote_Msg_Send();        
        Judge_Msg_Send();
        Judge_Send();
		Is_Fortress_Msg_Send();
#if single_double == 0        
        DoubleGimble_Msg_Send();
#endif
        
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

/***********************       发布数据函数         *******************************/
/**
 * @brief 发布遥控器数据 
 */
void Remote_Msg_Send()
{
    Can_Msg_Tx.remote_msg_tx[0] = 0xa5;
    memcpy(&Can_Msg_Tx.remote_msg_tx[1], &usart3_dma_buf[0], sizeof(Can_Msg_Tx.remote_msg_tx)-1);
    CAN_Send_StdDataFrame(&hcan2, 0x404,Can_Msg_Tx.remote_msg_tx); 
} 

/**
 * @brief 发布裁判系统数据 
 */
void Judge_Msg_Send()
{
    Can_Msg_Tx.judge_msg_tx.Judge_Msg.head = 0xa5;
    
    if(judge_sensor.info->RobotStatus.robot_id == 7)
        Can_Msg_Tx.judge_msg_tx.Judge_Msg.Color = 1;
    else if(judge_sensor.info->RobotStatus.robot_id == 107)
        Can_Msg_Tx.judge_msg_tx.Judge_Msg.Color = 0;
    
    if(judge_sensor.info->GameStatus.stage_remain_time < 240)
        Can_Msg_Tx.judge_msg_tx.Judge_Msg.IsAttack_2 = 1;
    else
        Can_Msg_Tx.judge_msg_tx.Judge_Msg.IsAttack_2 = 0;
    
    Can_Msg_Tx.judge_msg_tx.Judge_Msg.GameStatus.game_progress = judge_sensor.info->GameStatus.game_progress;
    
    Can_Msg_Tx.judge_msg_tx.Judge_Msg.HeatLimit_Heat.shooter_17mm_1_barrel_heat = judge_sensor.info->PowerHeatData.shooter_17mm_1_barrel_heat;
    Can_Msg_Tx.judge_msg_tx.Judge_Msg.HeatLimit_Heat.shooter_17mm_2_barrel_heat = judge_sensor.info->PowerHeatData.shooter_17mm_2_barrel_heat;
    Can_Msg_Tx.judge_msg_tx.Judge_Msg.ShootAbout.initial_speed = (uint8_t)(judge_sensor.info->ShootData.initial_speed*10);
 
    CAN_Send_StdDataFrame(&hcan2, 0x405,Can_Msg_Tx.judge_msg_tx.can_buff); 
}

/**
 * @brief 发布双头合作数据 
 */
void DoubleGimble_Msg_Send()
{
    if(AutoAimMsg_L.flag == 1)
    {
        if((AutoAimMsg_L.DouAutoAim_Status == Aim_1 && Calculation_flag == 1)||AutoAimMsg_L.DouAutoAim_Status == Aim_Gap)
        {
            CAN_Send_StdDataFrame(&hcan2, 0x401,Can_Msg_Tx.mainyaw_flag_msg_tx); 
            CAN_Send_StdDataFrame(&hcan2, 0x402,Can_Msg_Tx.lgimble_msg_tx.can_buff); 
            CAN_Send_StdDataFrame(&hcan2, 0x403,Can_Msg_Tx.rgimble_msg_tx.can_buff); 
        }
    }else if(AutoAimMsg_R.flag == 1){
        if((AutoAimMsg_R.DouAutoAim_Status == Aim_1 && Calculation_flag == 1)||AutoAimMsg_R.DouAutoAim_Status == Aim_Gap)
        {
            CAN_Send_StdDataFrame(&hcan2, 0x401,Can_Msg_Tx.mainyaw_flag_msg_tx); 
            CAN_Send_StdDataFrame(&hcan2, 0x402,Can_Msg_Tx.lgimble_msg_tx.can_buff); 
            CAN_Send_StdDataFrame(&hcan2, 0x403,Can_Msg_Tx.rgimble_msg_tx.can_buff); 
        }
    }
    
}

void Is_Fortress_Msg_Send()
{
	if(IsFortress.data.IsFortress_Flag == 1)
	{
		Can_Msg_Tx.Is_Fortress[0] = 1;
		CAN_Send_StdDataFrame(&hcan2, 0x406,Can_Msg_Tx.Is_Fortress);
	}
	else
	{
		Can_Msg_Tx.Is_Fortress[0] = 0;
	}
}

uint8_t seq  ; 

uint8_t Judge_BUFF[19];

void Judge_Send()
{
	Judge_Packhead     framehead;
    Judge_Data_Operate datahead;    
    
	framehead.SOF            = 0xA5;
    framehead.Data_Length    = 10;
    framehead.Seq            = seq;
    framehead.CRC8           = get_CRC8_check_sum((uint8_t *)&framehead, 4, 0xff);
    framehead.CMD_ID         = 0x0301;
	
    datahead.Data_ID     = 0x0120;
    datahead.Sender_ID   = judge_sensor.info->RobotStatus.robot_id;
    datahead.Receiver_ID = 0x8080;
	
	memset (Judge_BUFF,      0,                   sizeof(Judge_BUFF));
	memcpy (Judge_BUFF,     (uint8_t*)&framehead, sizeof(framehead));
    memcpy (Judge_BUFF + 7, (uint8_t*)&datahead,  sizeof(datahead));
    
	if(((judge_sensor.info->SentryInfo.sentry_info>>19)&0x01)==1)
	{
		Judge_BUFF[13]=1;
	}
	else 
	{
		Judge_BUFF[13]=0;
	}
    
	*(uint16_t *)(Judge_BUFF + 7 + 10) = get_CRC16_check_sum(Judge_BUFF, 7 + 10, 0xffff);

	HAL_UART_Transmit_DMA (&huart6, Judge_BUFF, 7 + 10 + 2); 
	
	seq++;
}




