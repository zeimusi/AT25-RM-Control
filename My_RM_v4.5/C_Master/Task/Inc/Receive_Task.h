#ifndef __RECEIVE_TASK_H__
#define __RECEIVE_TASK_H__

#include "Task_Init.h"

typedef enum{
    Remote_Mode,
    Classis_Follow_Mode,
    Rotate_Mode,
    Radar_Mode,
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

extern Supercap_TypeDef SuperCap_Msg;

typedef enum
{
    NoAim,    //未瞄准
    Aim_1,    //一次瞄准
    Aim_Gap,  //一二次瞄准间隙
    Aim_2,    //二次瞄准
}DouAutoAim_Status_t;
typedef struct
{
    int16_t Yaw;
    int16_t Pitch;
    int16_t HorizontalDistance;
    uint8_t IsMainYawMove;
    DouAutoAim_Status_t DouAutoAim_Status;
}AutoAimMsg_rx_t;
typedef union{
    uint8_t can_buff[8];
    AutoAimMsg_rx_t AutoAimMsg_rx;
}AutoAimMsg_rx_u;
extern AutoAimMsg_rx_u AutoAimMsg_rx_L;
extern AutoAimMsg_rx_u AutoAimMsg_rx_R;
typedef struct
{
    float Yaw;
    float Pitch;
    float HorizontalDistance;
    uint8_t IsMainYawMove;
    DouAutoAim_Status_t DouAutoAim_Status;
    uint8_t flag;//是否收到信息
}AutoAimMsg_t;
extern AutoAimMsg_t AutoAimMsg_L;
extern AutoAimMsg_t AutoAimMsg_R;

typedef struct
{
    float x;
    float y;
}AutoAimXY_t;
extern AutoAimXY_t AutoAimXY_L;
extern AutoAimXY_t AutoAimXY_R;

typedef struct
{
	float L_z;
	float R_z;
}AutoAimZ_t;
extern AutoAimZ_t AutoAimZ_L;
extern AutoAimZ_t AutoAimZ_R;


typedef struct
{
	float Yaw_Diff;
	float Pitch_Diff;
}GimbleDiff_t;

extern GimbleDiff_t GimbleDiff_L;
extern GimbleDiff_t GimbleDiff_R;

extern TaskHandle_t Remote_Task_Handle;
void Remote_Task(void *pvParameters);

extern TaskHandle_t StartINS_Task_Handle;
void StartINS_Task(void *pvParameters);

extern TaskHandle_t SuperCap_Task_Handle;
void SuperCap_Task(void *pvParameters);

extern TaskHandle_t Judge_Task_Handle;
void Judge_Task(void *pvParameters);

#endif

