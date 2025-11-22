#ifndef __SEND_TASK_H__
#define __SEND_TASK_H__

#include "Task_Init.h"

typedef enum
{
    NoAim,    //未瞄准
    Aim_1,    //一次瞄准
    Aim_Gap,  //一二次瞄准间隙
    Aim_2,    //二次瞄准
}DouAutoAim_Status_t;
extern DouAutoAim_Status_t DouAutoAim_Status;

typedef struct
{
    int16_t Yaw;
    int16_t Pitch;
    int16_t HorizontalDistance;
    uint8_t IsMainYawMove;
    DouAutoAim_Status_t DouAutoAim_Status;
}AutoAimAngle_Msg_tx_t;
typedef union{
    uint8_t can_buff[8];
    AutoAimAngle_Msg_tx_t AutoAimAngle_Msg_tx;
}AutoAimAngle_Msg_tx_u;

typedef struct
{
    float x;
    float y;
}AutoAimXY_Msg_tx_t;
typedef union{
    uint8_t can_buff[8];
    AutoAimXY_Msg_tx_t AutoAimXY_Msg_tx;
}AutoAimXY_Msg_tx_u;

typedef struct{
	float L_z;
	float R_z;
}AutoAimZ_Msg_tx_t;

typedef union{
	uint8_t can_buff[8];
	AutoAimZ_Msg_tx_t AutoAimZ_Msg_tx;
}AutoAimZ_Msg_tx_u;

typedef struct{
	float Yaw_diff;
	float Pitch_diff;
}GimbelDiff_Msg_tx_t;
typedef union{
    uint8_t can_buff[8];
    GimbelDiff_Msg_tx_t GimbelDiff_Msg_tx;
}GimbelDiff_Msg_tx_u;

/* can发送信息 */
typedef struct{
    int16_t gimble_msg_tx[4];
    int16_t shoot_msg_tx[4];
    AutoAimAngle_Msg_tx_u AutoAimAngle_Msg_tx;
    AutoAimXY_Msg_tx_u AutoAimXY_Msg_tx;
	AutoAimZ_Msg_tx_u AutoAimZ_Msg_tx;
	GimbelDiff_Msg_tx_u GimbelDiff_Msg_t;
}Can_Msg_Tx_t;
extern Can_Msg_Tx_t Can_Msg_Tx;

extern TaskHandle_t Send_Task_handle;
void Send_Task(void *pvParameters);

void GimbleDiff_Msg_Send(void);
void AutoAimXYZ_Msg_Send(void);


#endif

