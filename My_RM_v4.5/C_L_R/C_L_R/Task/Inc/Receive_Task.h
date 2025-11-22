#ifndef __RECEIVE_TASK_H__
#define __RECEIVE_TASK_H__

#include "Task_Init.h"

typedef enum{
    Remote_Mode,
    AutoAim_Mode,
    Patrol_Mode,
    Stop_Mode,
}Control_Mode_t;
extern Control_Mode_t Control_Mode;

typedef struct
{
    struct {
		float Pitch;
		float Roll;
		float Yaw;
		int16_t r;
        uint8_t flag;
		float LsatAngle;
		float ContinuousYaw;
	}EulerAngler;////欧拉角

    struct {
		float Pitch;
		float Roll;
		float Yaw;
		int16_t r;
        uint8_t flag;
		float LsatAngle;
		float ContinuousYaw;
	}EulerAngler_Init;////初始欧拉角    
  
    struct {
		float Pitch;
		float Roll;
		float Yaw;
		int16_t r;
        uint8_t flag;
		float LsatAngle;
		float ContinuousYaw;
	}EulerAngler_RAW;////原始欧拉角    
    
	struct {
		float W;
		float x;
		float y;
		float z;
	}Quaternions;//四元数   
    
    struct {
        float x;
        float y;
        float z;
	}AngularVelocity;//角速度    
}IMU_Typedef;
extern IMU_Typedef IMU;

typedef struct{
    uint8_t head;
    
    uint8_t Color:1;
    uint8_t IsAttack_2:1;
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
extern Judge_Msg_t Judge_Msg;


typedef enum{
    Unknown = 0,
    OK = 2,
    NO = 1,
}MainYaw_Arrive_flag_t;
typedef struct
{
    float Angle_Pitch;
    float Angle_Yaw;
    MainYaw_Arrive_flag_t MainYaw_Arrive_flag;
}DoubleGimble_Msg_t;
extern DoubleGimble_Msg_t DoubleGimble_Msg;

typedef struct
{
	uint8_t Is_Fortress_Flag;
}Is_Fortress_Msg_t;
extern Is_Fortress_Msg_t Is_Fortress_Msg;

extern TaskHandle_t Remote_Task_Handle;
void Remote_Task(void *pvParameters);

extern TaskHandle_t StartINS_Task_Handle;
void StartINS_Task(void *pvParameters);

#endif

