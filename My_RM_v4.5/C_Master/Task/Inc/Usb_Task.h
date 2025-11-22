#ifndef USB_TASK_H
#define USB_TASK_H

#include "Task_Init.h"

#define SEND_SOF    ((uint8_t)0x5A)
#define RECEIVE_SOF ((uint8_t)0x5A)

/******************************* 发送id *********************************/
/* 视觉用的 */
#define DATA_SEND_ID_Imu           ((uint8_t)0x01)  // ms
#define DATA_SEND_ID_AutoAim       ((uint8_t)0x09)  // ms
/* 导航要的（前三个是开源里的，2用了，13没用；后面是自己加的的） */
#define ROBOT_INFO_DATA_SEND_ID    ((uint8_t)0x03)
#define ALL_ROBOT_HP_SEND_ID       ((uint8_t)0x05)
#define GAME_STATUS_SEND_ID        ((uint8_t)0x06)
#define DATA_SEND_ID_AutoAim_L     ((uint8_t)0x10)  // ms
#define DATA_SEND_ID_AutoAim_R     ((uint8_t)0x11)  // ms
#define DATA_SEND_ID_GameBegin     ((uint8_t)0x12)  // ms
#define DATA_SEND_ID_MapCommand    ((uint8_t)0x13)  // ms
#define DATA_SEND_ID_AllowCapasity ((uint8_t)0x15)  // ms
#define DATA_SEND_ID_AllLocation   ((uint8_t)0x16)  // ms
#define DATA_SEND_ID_GimbleDiff	   ((uint8_t)0x17)
#define DATA_SEND_ID_Color         ((uint8_t)0x18)

/*导航没要，但我觉得有用*/
#define DATA_SEND_ID_IsRFID        ((uint8_t)0x14)  // ms
/******************************* 接收id *********************************/
#define ROBOT_CMD_DATA_RECEIVE_ID       ((uint8_t)0x01)
#define IsUP_DATA_RECEIVE_ID            ((uint8_t)0x02)
#define IsFortress_DATA_RECEIVE_ID		((uint8_t)0x03)
typedef struct
{
    uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
    uint8_t len;  // 数据段长度
    uint8_t id;   // 数据段id
    uint8_t crc;  // 数据帧头的 CRC8 校验
} __packed__ FrameHeader_t;
/*-------------------- Send --------------------*/
/*----- 视觉用的 -----*/
// IMU 数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
    uint32_t time_stamp;
    struct
    {
        float yaw;    // rad
        float pitch;  // rad
        float roll;   // rad

        float yaw_vel;    // rad/s
        float pitch_vel;  // rad/s
        float roll_vel;   // rad/s
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataImu_s;

// 发给自瞄
typedef struct 
{
    FrameHeader_t frame_header;  // id = 0x09
    uint32_t time_stamp;

    struct
    {
        uint8_t detect_color;
        uint8_t reset_tracker;
        float roll;
        float pitch;
        float yaw;
        float aim_x;
        float aim_y;
        float aim_z;
    } __attribute__((packed)) data;

    uint16_t crc;
} __packed__ SendDataAutoAim_s;

/*----- 导航要的（前三个是开源里的，2用了，13没用；后面是自己加的的） -----*/
// 机器人信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x03
    uint32_t time_stamp;
    struct
    {
        /// @brief 机器人部位类型 2 bytes
        struct
        {
            uint16_t chassis : 3;
            uint16_t gimbal : 3;
            uint16_t shoot : 3;
            uint16_t arm : 3;
            uint16_t custom_controller : 3;
            uint16_t reserve : 1;
        } __packed__ type;
        /// @brief 机器人部位状态 1 byte
        /// @note 0: 正常，1: 错误
        struct
        {
            uint8_t chassis : 1;
            uint8_t gimbal : 1;
            uint8_t shoot : 1;
            uint8_t arm : 1;
            uint8_t custom_controller : 1;
            uint8_t reserve : 3;
        } __packed__ state;
        /// @brief 机器人裁判系统信息 7 bytes
        struct
        {
            uint8_t id;
            uint8_t color;  // 0-red 1-blue 2-unknown
            bool attacked;
            uint16_t hp;
            uint16_t heat;
        } __packed__ referee;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRobotInfo_s;

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

//左头数据包
typedef struct{
  FrameHeader_t frame_header;  // 数据段id = 0x10

  uint32_t time_stamp;

  struct 
  {
    float x;
    float y;
    float z;

  } __packed__ data;

  uint16_t crc;
} __packed__ SendDataAutoAim_L_s;

//右头数据包
typedef struct{
  FrameHeader_t frame_header;  // 数据段id = 0x11

  uint32_t time_stamp;

  struct 
  {
    float x;
    float y;
    float z;

  } __packed__ data;

  uint16_t crc;
} __packed__ SendDataAutoAim_R_s;

//比赛是否开始标志位
typedef struct 
{
    FrameHeader_t frame_header;  // 数据段id = 0x12
    uint32_t time_stamp;

    uint8_t IsGameBegin_Flag;
    
    uint16_t crc;
} __packed__ SendDataGameBegin_s;

// 允许发弹量
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x08
    uint32_t time_stamp;
    struct
    {
        uint16_t projectile_allowance;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataAllowCapasity_s;

// 云台手下发数据
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x13
    uint32_t time_stamp;
    struct
    {
        float target_position_x;
        float target_position_y;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataMapCommand_s;

typedef struct
{
	FrameHeader_t frame_header;
	uint32_t time_stamp;
	struct
	{
		float Pitch_Diff_L;
		float Yaw_Diff_L;
		float Pitch_Diff_R;
		float Yaw_Diff_R;
		float L_x;
		float L_y;
		float L_z;
		float R_x;
		float R_y;
		float R_z;
	} __packed__ data;
	uint16_t crc;
} __packed__ SendDataGimbleDiff_s;

//
typedef struct{
  FrameHeader_t frame_header;  // 数据段id = 0x18

  uint32_t time_stamp;

  struct 
  {
	    uint8_t color;

  } __packed__ data;

  uint16_t crc;
} __packed__ SendDataColor_s;

/*———              导航没要，但我觉得有用               ———*/
// 机器人运动数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x13
    uint32_t time_stamp;
    struct
    {
          float hero_x;  
          float hero_y;  
          float engineer_x;  
          float engineer_y;  
          float standard_3_x;  
          float standard_3_y;  
          float standard_4_x;  
          float standard_4_y;  
          float standard_5_x;  
          float standard_5_y; 
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataAllRobotPos_s;

// 是否检测到各地区的RFID
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x06
    uint32_t time_stamp;
    struct
    {
          uint32_t rfid_status;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataRFIDstatus_s;


/*-------------------- Receive --------------------*/
typedef struct{
  FrameHeader_t frame_header;

  uint32_t time_stamp;

  struct 
  {
    uint8_t tracking;
    uint8_t id;
    uint8_t armors_num;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
  } __attribute__((packed)) data;

  uint16_t checksum;
} __packed__ ReceiveAutoaimData_t;
extern ReceiveAutoaimData_t ReceiveAutoaimData;

typedef struct 
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector;  // 底盘速度期望值
        
        struct
        {
            float roll;
            float pitch;
            float yaw;
            float leg_length;
        }__packed__ chassis;

        struct
        {
            float pitch;
            float yaw;
        }__packed__ gimbal;

        struct
        {
            bool fire;
            bool fric_on;
        }__packed__ shoot;
    } __packed__ data;
    uint16_t checksum;
} __packed__ RobotCmdData_t;
extern RobotCmdData_t ROBOT_CMD_DATA ;
extern RobotCmdData_t ROBOT_CMD_LASTDATA ;

//判断是否上坡
typedef struct 
{
    FrameHeader_t frame_header;  // 数据段id = 0x02
    uint32_t time_stamp;
    struct
    {
        int8_t IsUP_Flag;
    } __packed__ data;
    uint16_t checksum;
} __packed__ IsUP_t;
extern IsUP_t IsUP ;

//判断敌人是否在堡垒上
typedef struct 
{
    FrameHeader_t frame_header;  // 数据段id = 0x03
    uint32_t time_stamp;
    struct
    {
        int8_t IsFortress_Flag;
    } __packed__ data;
    uint16_t checksum;
} __packed__ IsFortress_t;
extern IsFortress_t IsFortress;

extern TaskHandle_t usb_task_handle;
void usb_task(void *pvParameters);

#endif /* USB_TASK_H */


