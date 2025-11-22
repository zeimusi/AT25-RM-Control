#ifndef USB_TASK_H
#define USB_TASK_H

#include "Task_Init.h"

#define SEND_SOF    ((uint8_t)0x5A)
#define RECEIVE_SOF ((uint8_t)0x5A)

/******************************* 发送id *********************************/
/* 视觉用的 */
#define DATA_SEND_ID_Imu           ((uint8_t)0x01)  // ms
#define DATA_SEND_ID_AutoAim       ((uint8_t)0x09)  // ms

#define ALL_ROBOT_HP_SEND_ID       ((uint8_t)0x05)
#define GAME_STATUS_SEND_ID        ((uint8_t)0x06)
#define DATA_SEND_ID_GameBegin     ((uint8_t)0x12)  // ms
#define RADAR_MARK_SEND_ID 		   ((uint8_t)0x07)	// ms
#define Vulnerable_SEND_ID		   ((uint8_t)0x08)
/******************************* 接收id *********************************/
#define ReceiveAdvanced_DATA_RECEIVE_ID ((uint8_t)0x06)

typedef struct
{
    uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
    uint8_t len;  // 数据段长度
    uint8_t id;   // 数据段id
    uint8_t crc;  // 数据帧头的 CRC8 校验
} __packed__ FrameHeader_t;
/*-------------------- Send --------------------*/
// 全场机器人hp信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x05
    uint32_t time_stamp;
    struct
    {
        uint16_t red_1_robot_hp;
        uint16_t red_2_robot_hp;
        uint16_t red_3_robot_hp;
        uint16_t red_4_robot_hp;
        uint16_t red_5_robot_hp;
        uint16_t red_7_robot_hp;
        uint16_t red_outpost_hp;
        uint16_t red_base_hp;
        uint16_t blue_1_robot_hp;
        uint16_t blue_2_robot_hp;
        uint16_t blue_3_robot_hp;
        uint16_t blue_4_robot_hp;
        uint16_t blue_5_robot_hp;
        uint16_t blue_7_robot_hp;
        uint16_t blue_outpost_hp;
        uint16_t blue_base_hp;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataAllRobotHp_s;


// 雷达标记进度数据包
typedef struct
{
	FrameHeader_t frame_header;	 // 数据段id = 0x07
	uint32_t time_stamp;
	struct
	{
		uint8_t mark_progress;
	} __packed__ data;
	uint16_t crc;
} __packed__ SendDataRadarMark_s;



// 比赛信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x06
    uint32_t time_stamp;
    struct
    {
        uint8_t game_progress;
        uint16_t stage_remain_time;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataGameStatus_s;


typedef struct
{
	FrameHeader_t frame_header;	 // 数据段id = 0x08
	uint32_t time_stamp;
	struct
	{
		uint8_t vulnerable;
	} __packed__ data;
	uint16_t crc;
} __packed__ SendDataVulnerable_s;


// 比赛是否开始标志位
typedef struct 
{
    FrameHeader_t frame_header;  // 数据段id = 0x12
    uint32_t time_stamp;

    uint8_t IsGameBegin_Flag;
    
    uint16_t crc;
} __packed__ SendDataGameBegin_s;


/*-------------------- Receive --------------------*/

typedef struct{
	FrameHeader_t frame_header;
	
	uint32_t time_stamp;
	
	uint8_t 	hero_id;
	uint16_t 	hero_position_x;
	uint16_t 	hero_position_y;
	
	uint8_t 	engineer_id;
	uint16_t 	engineer_position_x;
	uint16_t 	engineer_position_y;
	
	uint8_t 	infantry_3_id;
	uint16_t 	infantry_3_position_x;
	uint16_t 	infantry_3_position_y;
	
	uint8_t 	infantry_4_id;
	uint16_t 	infantry_4_position_x;
	uint16_t 	infantry_4_position_y;
	
	uint8_t 	infantry_5_id;
	uint16_t 	infantry_5_position_x;
	uint16_t 	infantry_5_position_y;
	
	uint8_t 	sentry_id;
	uint16_t 	sentry_position_x;
	uint16_t 	sentry_position_y;
		
	uint16_t crc16;
}__packed__ ReceiveAdvancedData_t;
extern ReceiveAdvancedData_t ReceiveAdvancedData;

typedef struct{

	uint16_t 	hero_lastposition_x;
	uint16_t 	hero_lastposition_y;

	uint16_t 	engineer_lastposition_x;
	uint16_t 	engineer_lastposition_y;

	uint16_t 	infantry_3_lastposition_x;
	uint16_t 	infantry_3_lastposition_y;

	uint16_t 	infantry_4_lastposition_x;
	uint16_t 	infantry_4_lastposition_y;
	
	uint16_t 	infantry_5_lastposition_x;
	uint16_t 	infantry_5_lastposition_y;
	
	uint16_t 	sentry_lastposition_x;
	uint16_t 	sentry_lastposition_y;	
}__packed__ LastPosition_t;
extern LastPosition_t LastPosition;

typedef enum
{
	hero_id = 1,
	engineer_id = 2,
	infantry_3_id = 3,
	infantry_4_id = 4,
	infantry_5_id = 5,
	sentry_id = 6,
}robot_id_t;

extern TaskHandle_t usb_task_handle;
void usb_task(void *pvParameters);
void Advanced_Process(ReceiveAdvancedData_t* ReceiveAdvancedData, uint8_t * sof_address, uint8_t cnt);


#endif /* USB_TASK_H */


