/**
  * @file       judge_potocol.c/h
  * @brief      RM2024赛季裁判系统信息读取，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数
  * @history
  */
#include "judge.h"

#include "crc.h"

#define ODDLINE_MAXCNT 200
#define JUDGE_FRAME_HEADER		(0xA5)

extern uint8_t judge_update(uint8_t* rxBuf);            //数据更新：在这里解包数据
static void judge_heart_beat(judge_sensor_t* judge); //心跳：判断裁判系统是否离线,离线计数值会在数据更新函数里归零

judge_info_t 	judge_sensor_info; 
judge_sensor_t	judge_sensor =
{
    .info = &judge_sensor_info,
    .update = judge_update,
    .heart_beat = judge_heart_beat,
    .work_state = DEV_OFFLINE,
    .offline_max_cnt = ODDLINE_MAXCNT,
    .offline_cnt = ODDLINE_MAXCNT+1,
    .data_valid = true,
};

static void judge_heart_beat(judge_sensor_t* judge_sen)
{
    judge_sen->offline_cnt++;
    if(judge_sen->offline_cnt > judge_sen->offline_max_cnt)
    {
        judge_sen->offline_cnt = judge_sen->offline_max_cnt;
        judge_sen->work_state = DEV_OFFLINE;
    }
    else
    {
        judge_sen->work_state = DEV_ONLINE;
    }
}

/* 数据包各部分位置（包头，id，数据） */
typedef enum
{
    FRAME_HEADER	  = 0,
    CMD_ID			  = 5,
    DATA_SEG		  = 7
} Judge_Frame_Offset_t;

/* 包头各部分位置 */
typedef enum
{
    J_SOF		= 0,
    DATA_LENGTH	= 1,
    SEQ			= 3,
    J_CRC8		= 4
} Judge_Frame_Header_Offset_t;

/* 各类数据id */
typedef enum
{
    ID_GAME_STATUS 			 = 0x0001,	// 比赛状态
    ID_GAME_RESULT 			 = 0x0002,	// 比赛结果
    ID_GAME_ROBOT_HP 		 = 0x0003,	// 机器人血量数据
	ID_ROBOT_STATUS 		 = 0x0201,	// 比赛机器人状态
	ID_radar_mark_data		 = 0x020C,	// 雷达标记进度数据
	ID_radar_info			 = 0x020E,	// 雷达自主决策信息同步数据
} Judge_Cmd_ID_t;

/* 各数据长度 */
typedef enum
{
    LEN_FRAME_HEAD 	                = 5,
    LEN_CMD_ID 		                = 2,
    LEN_FRAME_TAIL 	                = 2,
    
    LEN_GAME_STATUS 				= 11,
    LEN_GAME_RESULT 				= 1,
    LEN_GAME_ROBOT_HP 			    = 32,
    LEN_ROBOT_STATUS			    = 13,	
	LEN_radar_mark_data				= 1,
	LEN_radar_info					= 1,
} Judge_Data_Length_t;


uint8_t judge_update(uint8_t* rxBuf)
{
    uint8_t  res = false; //用来判断本次crc校验是否成功
    static uint16_t frame_length; //整包数据长度
    uint16_t cmd_id;              //本次数据id
    judge_info_t* judge_info = judge_sensor.info;
    memcpy(&judge_info->FrameHeader, rxBuf, LEN_FRAME_HEAD); //包头数据转移

    if(rxBuf[J_SOF] == 0xa5)
    {
        if(verify_CRC8_check_sum(rxBuf, LEN_FRAME_HEAD) == true)
        {
            frame_length = LEN_FRAME_HEAD + LEN_CMD_ID + judge_info->FrameHeader.data_length + LEN_FRAME_TAIL;
            judge_info->frame_length = frame_length;

            if(verify_CRC16_check_sum(rxBuf, frame_length) == true)
            {
                res = true;  //crc成功
                judge_sensor.offline_cnt = 0;  //离线时间归零
                cmd_id = (rxBuf[CMD_ID + 1] << 8 | rxBuf[CMD_ID]);  //获取id
                judge_info->cmd_id = cmd_id; 

                switch(cmd_id)
                {
                    case ID_GAME_STATUS:
                        memcpy(&judge_info->GameStatus, (rxBuf + DATA_SEG), LEN_GAME_STATUS);
                        break;
                    case ID_GAME_RESULT:
                        memcpy(&judge_info->GameResult, (rxBuf + DATA_SEG), LEN_GAME_RESULT);
                        break;
                    case ID_ROBOT_STATUS:
                        memcpy(&judge_info->RobotStatus, (rxBuf + DATA_SEG), LEN_ROBOT_STATUS);
                        break;					
                    case ID_GAME_ROBOT_HP:
                        memcpy(&judge_info->GameRobotHP, (rxBuf + DATA_SEG), LEN_GAME_ROBOT_HP);
                        break;
					case ID_radar_mark_data:
						memcpy(&judge_info->radar_mark_data, (rxBuf + DATA_SEG), LEN_radar_mark_data);
                        break;
					case ID_radar_info:
						memcpy(&judge_info->radar_info,	(rxBuf + DATA_SEG),	LEN_radar_info);
						break;
                    default:break;
                    }
            }
        }
    }  
    judge_sensor.data_valid = res;
    
    judge_sensor.data_valid = res;
    if(res == true)
        return frame_length;
    else
        return 0;    
}





