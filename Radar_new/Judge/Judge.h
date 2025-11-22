/**
  * @file       judge_potocol.c/h
  * @brief      RM20242季裁判系统信息读取，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数
  * @history
  */
#ifndef __JUDGE_POTOCOL_H
#define __JUDGE_POTOCOL_H

#if defined(__ARMCC_VERSION) || defined(__GNUC__)
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx) || defined(STM32F439xx) || \
    defined(STM32F401xC) || defined(STM32F401xE) || defined(STM32F410Tx) || defined(STM32F410Cx) || \
    defined(STM32F410Rx) || defined(STM32F411xE) || defined(STM32F446xx) || defined(STM32F469xx) || \
    defined(STM32F479xx) || defined(STM32F412Cx) || defined(STM32F412Zx) || defined(STM32F412Rx) || \
    defined(STM32F412Vx) || defined(STM32F413xx) || defined(STM32F423xx)
#include <stm32f4xx.h>
#elif defined(STM32F030x6) || defined(STM32F030x8) || defined(STM32F031x6) || defined(STM32F038xx) || \
    defined(STM32F042x6) || defined(STM32F048xx) || defined(STM32F051x8) || defined(STM32F058xx) ||   \
    defined(STM32F070x6) || defined(STM32F070xB) || defined(STM32F071xB) || defined(STM32F072xB) ||   \
    defined(STM32F078xx) || defined(STM32F091xC) || defined(STM32F098xx) || defined(STM32F030xC)
#include <stm32f0xx.h>
#elif defined(STM32F100xB) || defined(STM32F100xE) || defined(STM32F101x6) || defined(STM32F101xB) || \
    defined(STM32F101xE) || defined(STM32F101xG) || defined(STM32F102x6) || defined(STM32F102xB) ||   \
    defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || defined(STM32F103xG) ||   \
    defined(STM32F105xC) || defined(STM32F107xC)
#include <stm32f1xx.h>
#elif defined(STM32F301x8) || defined(STM32F302x8) || defined(STM32F302xC) || defined(STM32F302xE) || \
    defined(STM32F303x8) || defined(STM32F303xC) || defined(STM32F303xE) || defined(STM32F373xC) ||   \
    defined(STM32F334x8) || defined(STM32F318xx) || defined(STM32F328xx) || defined(STM32F358xx) ||   \
    defined(STM32F378xx) || defined(STM32F398xx)
#include <stm32f3xx.h>
#elif defined(STM32H743xx) || defined(STM32H753xx) || defined(STM32H750xx) || defined(STM32H742xx) || \
    defined(STM32H745xx) || defined(STM32H755xx) || defined(STM32H747xx) || defined(STM32H757xx) ||   \
    defined(STM32H7B0xx) || defined(STM32H7B0xxQ) || defined(STM32H7A3xx) || defined(STM32H7B3xx) ||  \
    defined(STM32H7A3xxQ) || defined(STM32H7B3xxQ) || defined(STM32H735xx) || defined(STM32H733xx) || \
    defined(STM32H730xx) || defined(STM32H730xxQ) || defined(STM32H725xx) || defined(STM32H723xx)
#include <stm32h7xx.h>
#endif
#endif

#include "stdbool.h"
#include "string.h"

/*******************************************************裁判系统数据*************************************************/
/* ID: 0x0001   Byte:   1  比赛状态数据 */
typedef __packed struct
{
 uint8_t game_type : 4;      //比赛类型
 uint8_t game_progress : 4;  //当前比赛阶段
 uint16_t stage_remain_time; //当前阶段剩余时间，单位：秒
 uint64_t SyncTimeStamp;     //UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
}game_status_t;

/* ID: 0x0002	Byte:	1	比赛结果数据 */
typedef __packed struct
{
 uint8_t winner;
}game_result_t;

/* ID: 0x0003	Byte:	36	机器人血量数据数据 */
typedef __packed struct
{
 uint16_t red_1_robot_HP;	// 红1英雄机器人血量(未上场及罚下血量为0)
 uint16_t red_2_robot_HP;	// 红2工程机器人血量
 uint16_t red_3_robot_HP;	// 红3步兵机器人血量
 uint16_t red_4_robot_HP;	// 红4步兵机器人血量
 uint16_t red_5_robot_HP;	// 红5步兵机器人血量
 uint16_t red_7_robot_HP;	// 红7哨兵机器人血量
 uint16_t red_outpost_HP;	// 红方前哨站血量
 uint16_t red_base_HP;		// 红方基地血量
 uint16_t blue_1_robot_HP;	// 蓝1英雄机器人血量
 uint16_t blue_2_robot_HP;	// 蓝2工程机器人血量
 uint16_t blue_3_robot_HP;	// 蓝3步兵机器人血量
 uint16_t blue_4_robot_HP;	// 蓝4步兵机器人血量
 uint16_t blue_5_robot_HP;	// 蓝5步兵机器人血量
 uint16_t blue_7_robot_HP;	// 蓝7哨兵机器人血量
 uint16_t blue_outpost_HP;	// 蓝方前哨站血量
 uint16_t blue_base_HP;		// 蓝方基地血量
}game_robot_HP_t;

/* ID: 0x0201   Byte:   13 机器性能人体系*/
typedef __packed struct
{
 uint8_t robot_id;            //本机器人 ID
 uint8_t robot_level;         //机器人等级
 uint16_t current_HP;         //机器人当前血量
 uint16_t maximum_HP;         //机器人血量上限
 uint16_t shooter_barrel_cooling_value;   //机器人枪口热量每秒冷却值
 uint16_t shooter_barrel_heat_limit;      //机器人枪口热量上限
 uint16_t chassis_power_limit;            //机器人底盘功率上限
 uint8_t power_management_gimbal_output : 1; //gimbal 口输出
uint8_t power_management_chassis_output : 1; //chassis 口输出
 uint8_t power_management_shooter_output : 1;//shooter 口输出
}robot_status_t;

/* ID:0x020C    Byte：1   雷达标记进度数据*/
typedef __packed struct
{
	uint8_t mark_progress;
}radar_mark_data_t;

/* ID:0x020E    Byte：1   雷达自主决策信息同步数据*/
typedef __packed struct
{
	uint8_t radar_info;
}radar_info_t;
/******************************************^^^^^^裁判系统数据^^^^^^*******************************************/

typedef enum
{
    DEV_ONLINE,
    DEV_OFFLINE,
} dev_work_state_t;

typedef __packed struct
{
    uint8_t  sof;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc8;
} std_frame_header_t;

/* judge_info_t内包括：数据长度，数据id，各数据*/
typedef struct
{
    uint16_t frame_length;  
    uint16_t cmd_id;
    std_frame_header_t				FrameHeader;
    game_status_t 				    GameStatus;					// 0x0001
    game_result_t 				    GameResult;					// 0x0002
    game_robot_HP_t 			    GameRobotHP;		 		// 0x0003
    robot_status_t                  RobotStatus;                // 0x0201	
	radar_mark_data_t				radar_mark_data;			// 0x020C
	radar_info_t					radar_info;					// 0x020E
} judge_info_t;

/* judge_sensor_t内包括info，两个函数，离线相关，crc校验是否成功 */
typedef struct judge_sensor_struct
{
    judge_info_t*	info;
    uint8_t	(*update)(uint8_t* rxBuf);
    void	(*heart_beat)(struct judge_sensor_struct* self);
    dev_work_state_t	work_state;
    int16_t		offline_cnt;
    int16_t		offline_max_cnt;
    bool	   data_valid;    
} judge_sensor_t;
extern judge_sensor_t judge_sensor;


typedef __packed struct{
   uint8_t SOF;                    
   uint16_t Data_Length;           
   uint8_t Seq;                   
   uint8_t CRC8;                  
   uint16_t CMD_ID;               
} Judge_Packhead;

typedef struct{
   uint16_t Data_ID;               
   uint16_t Sender_ID;             
   uint16_t Receiver_ID;           
} Judge_Data_Operate; 

#endif

