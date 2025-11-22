#ifndef USB_TASK_H
#define USB_TASK_H

#include "Variate.h"
#include "Function.h"

#define SEND_SOF    ((uint8_t)0x5A)
#define RECEIVE_SOF ((uint8_t)0x5A)

#define DATA_SEND_ID_Imu           ((uint8_t)0x01)  // ms
#define DATA_SEND_ID_AllRobotHp    ((uint8_t)0x02)  // ms
#define DATA_SEND_ID_GameStatus    ((uint8_t)0x03)  // ms
#define DATA_SEND_ID_MyLocation    ((uint8_t)0x04)  // ms
#define DATA_SEND_ID_AllLocation   ((uint8_t)0x05)  // ms
#define DATA_SEND_ID_IsRFID        ((uint8_t)0x06)  // ms
#define DATA_SEND_ID_AutoAimMsg    ((uint8_t)0x07)  // ms
#define DATA_SEND_ID_AllowCapasity ((uint8_t)0x08)  // ms

#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x03)
//#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x01)

typedef struct
{
    uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
    uint8_t len;  // 数据段长度
    uint8_t id;   // 数据段id
    uint8_t crc;  // 数据帧头的 CRC8 校验
} __packed__ FrameHeader_t;
/*-------------------- Send --------------------*/

// IMU 数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
    uint32_t time_stamp;
	  uint8_t self_color ; //0-blud  1-red
	  uint8_t aim_opon ; 

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

// 全场机器人hp信息数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x02
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
    FrameHeader_t frame_header;  // 数据段id = 0x03
    uint32_t time_stamp;
    struct
    {
        uint8_t game_progress;
        uint16_t stage_remain_time;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataGameStatus_s;

// 本机器人位置
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x04
    uint32_t time_stamp;
    struct
    {
         float x;       //本机器人位置 x 坐标，单位：m
         float y;       //本机器人位置 y 坐标，单位：m
         float angle;
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataMyRobotPos_t;
 
// 机器人运动数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x05
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

// 自瞄信息
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x07
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float Yaw;
            float Pitch;
            float HorizontalDistance;    
        }__packed__ L_AutoAimMsg;

        struct
        {
            float Yaw;
            float Pitch;
            float HorizontalDistance;       
        }__packed__ R_AutoAimMsg;      
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataAutoAimMsg_s;

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

/*-------------------- Receive --------------------*/
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
            float vx;
            float vy;
            float wz;
        } __packed__ speed_vector_last;  // 上一次底盘速度期望值
    } __packed__ data;
    uint16_t checksum;
} __packed__ RobotCmdData_t;

extern RobotCmdData_t ROBOT_CMD_DATA ;
extern RobotCmdData_t ROBOT_CMD_LASTDATA ;

typedef struct {
	uint8_t header;
	uint8_t detect_color : 1; //0-red  1-blue
	bool reset_tracker : 1;
	uint8_t reserved :6;
	float roll;
	float pitch;
	float yaw;
	float aim_x;
	float aim_y;
	float aim_z;
  uint16_t checksum;
}SendVision_t;
extern SendVision_t SendVision;


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
} __packed__ ReceiveVisionData_t;
extern ReceiveVisionData_t ReceiveVisionData;
typedef struct {
	 uint8_t header;
	 float linear_x;
	 float linear_y;
	 float linear_z;
	 float angular_x;
	 float angular_y;
	 float angular_z;
	 uint16_t checksum;
}ReceiveTwist_t;
extern ReceiveTwist_t ReceiveTwist;

#endif /* USB_TASK_H */


