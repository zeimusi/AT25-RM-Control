#include "Variate.h"
int AimAllow = 0;

float Serialport [SerialChannel];

uint8_t NormalModeFlag = 0,GyroscopeModeFlag = 0;
/* 卡弹标志位 */
int StuckFlag = 0;
/* 云台初始化标志位 */
uint8_t GimbalInitFlag = 0;
enum {
    InsStop     = 0,
	  InsStart    = 1,
	  InsFinished = 2
}Ins_Flag;
/* 看门狗结构体定义 */
WatchDog_TypeDef Remote_Dog, Down_Dog, Gimbal_Dog[GIMBAL_SUM], Shoot_Dog[FRIC_SUM], Pluck_Dog, IMU_Dog, PC_Dog,Referee_Dog;
/* 设备状态 */
eSystemState SystemState;
DeviceStates DeviceState;
/* 电机数据结构体 */
GM6020_TypeDef Gimbal_Motor[GIMBAL_SUM];
DM4310_TypeDef DM4310_Pitch;

/* 遥控器处理值 */
float Key_ch[4]   = {0};
float Mouse_ch[3] = {0};
const RC_ctrl_t *local_rc_ctrl;
InputMode_e RemoteMode;
/* 机构运行状态 */
eAimAction AimAction         = AIM_STOP;
eMidMode MidMode             = FRONT;
/****** 对视觉数据进行处理 ******/
AIM_Typedef Aim_Data;


/* 裁判系统 */
Chassis_board_send_t Referee_data_Rx;      // 上下板通信接收
/* 上下板通信发送 */
Gimbal_data_t Gimbal_data;
Gimbal_action_t Gimbal_action;

/* 弹频 */
int16_t pluck_speed;
