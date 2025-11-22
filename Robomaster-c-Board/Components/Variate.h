#ifndef __VARIATE_H
#define __VARIATE_H
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "usart.h"
#include "tim.h"
//#include "iwdg.h"
#include "remote_control.h"
#include "remote.h"
#include "motor.h"
#include "CANDrive.h"
#include "WatchDog.h"
#include "PID.h"
#include "IMU.h"
#include "Serialport.h"
#include "Data_Exchange.h"


#include "CRC.h"
#include "stdbool.h"
#include "Attribute_Typedef.h"

/* 便于单独测试各机构，如果启动的机构电机不全在线，需注释对应任务里的电机在线判断 */
/** @brief    是否启动底盘电机驱动 */
#define CHASSIS_RUN 1
/** @brief    是否启动云台电机驱动 */
#define GIMBAL_RUN  1
/** @brief    是否启动发射机构电机驱动 */
#define SHOOT_RUN   1

/** @brief  3号老步兵(麦轮)，4号步兵（DM4310） */
#define ROBOT_ID 3

/** @brief 串口助手通道数 */
#define SerialChannel 4
enum SerialType{
FireWater = 0,
JustFloat = 1,
RawData   = 2
};
extern float Serialport [SerialChannel];
/** @brief 不同ROBOT_ID不同参数 */
#define Yaw_Mid_Front 3235    //!< @brief Yaw轴电机机器人前方中值
#define Pitch_Mid 6645        //!< @brief Pitch轴电机云台水平值
#define P_ADD_limit 22	     //!< @brief Pitch轴电机上限位与水平位差值
#define P_LOSE_limit 16     //!< @brief Pitch轴电机下限位与水平位差值
#define Pimu_ADD_limit 10   //!< @brief Pitch轴电机上限位与水平位差值
#define Pimu_LOSE_limit 34     //!< @brief Pitch轴电机下限位与水平位差值

#define SHOOT_SPEED  6000    //!< @brief 摩擦轮电机速度环PID的期望值（射速）
#define PLUCK_SPEED 8000     //!< @brief 拨弹盘电机连发时速度环PID的期望值（弹频）
#define PLUCK_MOTOR_ONE 1310 //!< @brief 一发弹丸拨弹盘电机转过的机械角度

#define Pi 3.1415926535
enum {
        YAW = 0,
        PITCH = 1,
        GIMBAL_SUM = 2,
};  
enum{
    LEFT = 0,
    RIGHT= 1,
    FRIC_SUM = 2,
};



/* 根据机器人正前方Yaw轴机械角度算出机器人剩下三个方向Yaw轴机械角度（6020正装，反装将左右相反） */
#if (Yaw_Mid_Front + 2048) > 8191
#define Yaw_Mid_Left Yaw_Mid_Front - 6143
#else
#define Yaw_Mid_Left Yaw_Mid_Front + 2048
#endif

#if (Yaw_Mid_Left + 2048) > 8191
#define Yaw_Mid_Back Yaw_Mid_Left - 6143
#else
#define Yaw_Mid_Back Yaw_Mid_Left + 2048
#endif

#if (Yaw_Mid_Back + 2048) > 8191
#define Yaw_Mid_Right Yaw_Mid_Back - 6143
#else
#define Yaw_Mid_Right Yaw_Mid_Back + 2048
#endif


/** @brief 电机参考速度 */
#define STD_Speed 2406    // 标准速度1m/s
#define STD_Omega 6142    // 标准速度1rpm/s
#define STD_Angle 0.36f	  // 角度制1rpm/s
#define STD_MAngle 8.192  // 机械角度制1rpm/s

// 数据名称宏
#define IMU_NAME "imu_data"
#define CHASSIS_FDB_SPEED_NAME "chassis_fdb_speed"
#define ROBOT_CMD_DATA_NAME "ROBOT_CMD_DATA"
#define USB_OFFLINE_NAME "usb_offline"
#define AIMREF "AimRef"
#define CALI_BUZZER_STATE_NAME "CaliBuzzerState"




/* 卡弹标志位 */
extern int StuckFlag;
/* 云台初始化标志位 */
extern uint8_t GimbalInitFlag;
enum Ins_Flag;

/* 系统状态 */
typedef enum{
	SYSTEM_STARTING = 0,     //!< @brief 正在启动
	SYSTEM_RUNNING  = 1,	 //!< @brief 正在运行
} eSystemState;
/*设备状态 */
typedef enum{
	Device_Offline = 0,     //!< @brief 设备离线
	Device_Online  = 1,	    //!< @brief 设备在线
	Device_Error   = 2	    //!< @brief 设备错误
} eDeviceState;
typedef struct {
	eDeviceState Remote_State, IMU_State, Gimbal_State[GIMBAL_SUM], Shoot_State[FRIC_SUM], Pluck_State, Down_State, PC_State,Referee_State;
}DeviceStates;
extern DeviceStates DeviceState;
extern eSystemState SystemState;

/* 电机 */
extern GM6020_TypeDef Gimbal_Motor[GIMBAL_SUM];
extern DM4310_TypeDef DM4310_Pitch;
extern RM3508_TypeDef Shoot_Motor[FRIC_SUM];
extern M2006_TypeDef  Pluck_Motor;

/* 云台归中位置 */
typedef enum{
	FRONT = 0,       //!< @brief   前方
	BACK  = 1,       //!< @brief   后方
} eMidMode;
extern eMidMode MidMode;

/* 自瞄状态 */
typedef enum{
	AIM_STOP = 0,     //!< @brief   关闭自瞄
	AIM_AID  = 1,     //!< @brief   自瞄不自动射击
	AIM_AUTO = 2,     //!< @brief   自瞄+自动射击
} eAimAction;
extern uint8_t Opon_aim ;
extern eAimAction AimAction;
/* 视觉 */
typedef struct{
	  uint8_t Flag;
		float Ref_Yaw;
	  float Ref_Pitch;
		float HorizontalDistance;
	  float id;
	enum {
	 AimStop  = 0,  
	 AimReady = 1,
	 AimFire  = 2,
	}AimShoot;
}AIM_Typedef;
extern AIM_Typedef Aim_Data;
extern int AimAllow;
/* 裁判系统 */
typedef struct  {
//		int8_t  robot_color;         // 机器人颜色
    uint16_t heat_limit_remain;  // 剩余热量
    uint16_t heat_limit_recover; // 冷却速率
	  int16_t ChassisSpeed; //底盘角速度
	  int8_t game_state_robot_color;//比赛状态    --0 未开始 --1 开始
} Chassis_board_send_t;
/* 发送底盘数据 */
typedef enum {
	Gimbal_offline = 0,    
	Gimbal_online  = 1,     
}Gimbal_status_e;

typedef enum {
	shoot_offline = 0,     
	shoot_online  = 1,      
}shoot_status_e;

typedef enum {
	stop = 0,    
	normal  = 1,   
  rotate = 2,
  fllow = 3,
  grasp = 4
}move_status_e;

typedef enum {
	vision_offline = 0,   
	vision_online  = 1,    
}vision_status_e;


typedef enum {
	shoot_mode_stop = 0,   
	shoot_mode_ready  = 1,   
  shoot_mode_fire = 2,
  shoot_mode_follow = 3,
  shoot_mode_stucking = 4,

}shoot_mode_e;

typedef struct  {
    Gimbal_status_e Pitch: 1;
    Gimbal_status_e Yaw : 1;
} Gimbal_status_t;

typedef struct  {
	uint16_t vision_distance;  
	int16_t Pitch_angle;
	int16_t Yaw_angle;
	uint16_t Offset_Angle;
} Gimbal_data_t;

typedef struct{
    Gimbal_status_t Gimbal_status;         
    shoot_status_e shoot_status : 1;          
    move_status_e move_status : 4;     
    vision_status_e vision_status : 1;          
    shoot_mode_e shoot_mode;				           
	uint8_t Key; 
	uint8_t vision_number; 
}Gimbal_action_t;
/* 上下板通信发送 */
extern Gimbal_data_t Gimbal_data;
extern Gimbal_action_t Gimbal_action;
/** @briefn */
/* 遥控器 */
extern float Key_ch[4], Mouse_ch[3];
extern InputMode_e RemoteMode;
/* 任务句柄 */
extern TaskHandle_t  Chassis_Task_handle, Gimbal_Task_handle, Shoot_Task_handle, Aim_Task_handle, Ins_Task_handle,Music_Task_handle,usb_task_handle;
/* 看门狗 */
extern WatchDog_TypeDef Remote_Dog, IMU_Dog, Gimbal_Dog[GIMBAL_SUM], Shoot_Dog[FRIC_SUM], Pluck_Dog, Down_Dog, PC_Dog,Referee_Dog;
/* 裁判系统 */
extern Chassis_board_send_t Referee_data_Rx;      // 上下板通信发送

extern uint8_t NormalModeFlag,GyroscopeModeFlag;
/* 弹频 */
extern int16_t pluck_speed;

#endif
