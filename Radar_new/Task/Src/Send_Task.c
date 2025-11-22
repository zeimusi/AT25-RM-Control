#include "Send_Task.h"

uint8_t Radar_Vulnerable_Chance = 1;		//雷达获得双倍易伤的机会
/*************************      发布数据任务     *******************************/
TaskHandle_t Send_Task_handle;
void Send_Task(void *pvParameters)
{    
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
		Judge_Position_Send();
		
		vTaskDelayUntil(&xLastWakeTime,50);
	}
}
TaskHandle_t send_handle;
void send(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();	
    for(;;)
    {
		if(judge_sensor.info->GameStatus.stage_remain_time >= 240)
		{
			Radar_Vulnerable_Chance = 1;
		}
		else
		{
			Radar_Vulnerable_Chance = 2;
		
		}
		Judge_Radar_Send();
		
		vTaskDelayUntil(&xLastWakeTime,520);
	}
}




/***********************       发布数据函数         *******************************/

uint8_t Seq; 

uint8_t Judge_Position_BUFF[33];
uint8_t Judge_Radar_BUFF[16]; 

void Judge_Position_Send()
{
	Judge_Packhead     framehead;
	
	framehead.SOF            = 0xA5;
    framehead.Data_Length    = 24;
    framehead.Seq            = Seq;
    framehead.CRC8           = get_CRC8_check_sum((uint8_t *)&framehead, 4, 0xff);
    framehead.CMD_ID         = 0x0305;
	
	memset (Judge_Position_BUFF,      0,                   sizeof(Judge_Position_BUFF));
	memcpy (Judge_Position_BUFF,     (uint8_t*)&framehead, sizeof(framehead));
	for(int i=1;i<=6;i++)
	{
		switch(i){
			case hero_id:
				Judge_Position_BUFF[7 + (i-1) * 4] 				= ReceiveAdvancedData.hero_position_x % 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1] 			= ReceiveAdvancedData.hero_position_x / 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1 + 1] 		= ReceiveAdvancedData.hero_position_y % 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1 + 1 + 1] 	= ReceiveAdvancedData.hero_position_y / 256;
				break;
			case engineer_id:
				Judge_Position_BUFF[7 + (i-1) * 4] 				= ReceiveAdvancedData.engineer_position_x % 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1] 			= ReceiveAdvancedData.engineer_position_x / 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1 + 1] 		= ReceiveAdvancedData.engineer_position_y % 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1 + 1 + 1] 	= ReceiveAdvancedData.engineer_position_y / 256;		
				break;
			case infantry_3_id:
				Judge_Position_BUFF[7 + (i-1) * 4] 				= ReceiveAdvancedData.infantry_3_position_x % 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1] 			= ReceiveAdvancedData.infantry_3_position_x / 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1 + 1] 		= ReceiveAdvancedData.infantry_3_position_y % 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1 + 1 + 1] 	= ReceiveAdvancedData.infantry_3_position_y / 256;		
				break;	
			case infantry_4_id:
				Judge_Position_BUFF[7 + (i-1) * 4] 				= ReceiveAdvancedData.infantry_4_position_x % 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1] 			= ReceiveAdvancedData.infantry_4_position_x / 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1 + 1] 		= ReceiveAdvancedData.infantry_4_position_y % 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1 + 1 + 1] 	= ReceiveAdvancedData.infantry_4_position_y / 256;		
				break;	
			case infantry_5_id:
				Judge_Position_BUFF[7 + (i-1) * 4] 				= ReceiveAdvancedData.infantry_5_position_x % 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1] 			= ReceiveAdvancedData.infantry_5_position_x / 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1 + 1] 		= ReceiveAdvancedData.infantry_5_position_y % 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1 + 1 + 1] 	= ReceiveAdvancedData.infantry_5_position_y / 256;		
				break;	
			case sentry_id:
				Judge_Position_BUFF[7 + (i-1) * 4] 				= ReceiveAdvancedData.sentry_position_x % 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1] 			= ReceiveAdvancedData.sentry_position_x / 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1 + 1] 		= ReceiveAdvancedData.sentry_position_y % 256;
				Judge_Position_BUFF[7 + (i-1) * 4 + 1 + 1 + 1] 	= ReceiveAdvancedData.sentry_position_y / 256;		
				break;			
		}
	}
	*(uint16_t *)(Judge_Position_BUFF + 7 + 24) = get_CRC16_check_sum(Judge_Position_BUFF, 7 + 24, 0xffff);

	HAL_UART_Transmit_DMA (&huart6, Judge_Position_BUFF, 7 + 24 + 2); 

	Seq++;
}

void Judge_Radar_Send()
{
//	Judge_Packhead     framehead;
//    Judge_Data_Operate datahead;    
//    
//	framehead.SOF            = 0xA5;
//    framehead.Data_Length    = 7;
//    framehead.Seq            = Seq;
//    framehead.CRC8           = 0xb9;//get_CRC8_check_sum((uint8_t *)&framehead, 4, 0xff);
//    framehead.CMD_ID         = 0x0301;
//	
//    datahead.Data_ID     = 0x0121;
//    datahead.Sender_ID   = judge_sensor.info->RobotStatus.robot_id;
//    datahead.Receiver_ID = 0x8080;
//	
//	memset (Judge_Radar_BUFF,      0,                   sizeof(Judge_Radar_BUFF));
//	memcpy (Judge_Radar_BUFF,     (uint8_t*)&framehead, sizeof(framehead));
//    memcpy (Judge_Radar_BUFF + 7, (uint8_t*)&datahead,  sizeof(datahead));
//    
	Judge_Radar_BUFF[0] = 0xa5;
	Judge_Radar_BUFF[1] = 0x07;
	Judge_Radar_BUFF[2] = 0x00;
	Judge_Radar_BUFF[3] = 0x00;
	Judge_Radar_BUFF[4] = 0xb9;
	Judge_Radar_BUFF[5] = 0x01;
	Judge_Radar_BUFF[6] = 0x03;
	Judge_Radar_BUFF[7] = 0x21;
	Judge_Radar_BUFF[8] = 0x01;
	Judge_Radar_BUFF[9] = 9;
	Judge_Radar_BUFF[10] = 0x00;
	Judge_Radar_BUFF[11] = 0x80;
	Judge_Radar_BUFF[12] = 0x80;
	Judge_Radar_BUFF[13] = Radar_Vulnerable_Chance;
	if(Radar_Vulnerable_Chance == 1)
	{
		Judge_Radar_BUFF[14] = 'f';
		Judge_Radar_BUFF[15] = 0x86;	
	}
	else
	{
		Judge_Radar_BUFF[14] = 0xfd;
		Judge_Radar_BUFF[15] = 0xb4;		
	}

	
//	*(uint16_t *)(Judge_Radar_BUFF + 7 + 7) = get_CRC16_check_sum(Judge_Radar_BUFF, 7 + 7, 0xffff);

	HAL_UART_Transmit_DMA (&huart6, Judge_Radar_BUFF, 7 + 7 + 2); 

//	Seq++;
}



