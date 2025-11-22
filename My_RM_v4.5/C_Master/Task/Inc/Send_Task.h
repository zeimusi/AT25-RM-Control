#ifndef __Send_Task_H__
#define __Send_Task_H__

#include "Task_Init.h"

typedef struct{
    uint8_t head;
    
    uint8_t Color:1;
    uint8_t IsAttack_2:1;//是否可以开始攻击工程
    struct
    {
        uint8_t game_progress;//当前比赛阶段
    } __packed__ GameStatus;
    struct
    {
        uint16_t shooter_17mm_1_barrel_heat;//第 1 个 17mm 发射机构的枪口热量
        uint16_t shooter_17mm_2_barrel_heat;//第 2 个 17mm 发射机构的枪口热量
    } __packed__ HeatLimit_Heat;  

    struct{
        uint8_t initial_speed;  //弹丸初速度（单位：m/s）
    }__packed__ ShootAbout;
     
}Judge_Msg_t;
typedef union
{
    uint8_t can_buff[8];
    Judge_Msg_t Judge_Msg;
} Judge_Msg_Tx_t;


typedef struct
{
    float Angle_Pitch;
    float Angle_Yaw;
}GimbleAngle_t;
typedef union
{
    uint8_t can_buff[8];
    GimbleAngle_t GimbleAngle;
}Gimble_Msg_Tx_t;

/* can发送信息 */
typedef struct{
    int16_t yaw_msg_tx[4];
    int16_t chassis_msg_tx[4];
    uint8_t remote_msg_tx[8];
    Judge_Msg_Tx_t judge_msg_tx;
    Gimble_Msg_Tx_t lgimble_msg_tx;
    Gimble_Msg_Tx_t rgimble_msg_tx;
    uint8_t mainyaw_flag_msg_tx[8];
	uint8_t Is_Fortress[1];
}Can_Msg_Tx_t;
extern Can_Msg_Tx_t Can_Msg_Tx;

extern TaskHandle_t Send_Task_handle;
void Send_Task(void *pvParameters);

void Judge_Send(void);

#endif

