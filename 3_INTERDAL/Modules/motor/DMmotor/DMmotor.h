#ifndef __DMMOTOR_H
#define __DMMOTOR_H
/**
  * 1 传统MIT模式
  * 2 达妙电机一拖四模式(仿大疆电机)
 **/
#define DMMOTORCTRLMODE 2

#include "motor_def.h"
/* 电机最大值 */
#define DM_MOTOR_CNT 4
/* 达妙电机参数设置 */
#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f
// 一圈编码器刻度
#define Encoder_Num_Per_Round 8192;
// 电流到输出的转化系数
#define Current_To_Out = 16384.0f 10.261194f;
// 理论最大输出电流
#define Theoretical_Output_Current_Max 10.261194f;

/* 滤波系数设置为1的时候即关闭滤波 */  
// TODO 给不同的电机设置不同的低通滤波器惯性系数而不是统一使用宏
#define DMMOTOR_SPEED_SMOOTH_COEF 0.85f      // 最好大于0.85
#define DMMOTOR_CURRENT_SMOOTH_COEF 0.9f     // 必须大于0.9
#define ECD_ANGLE_COEF_DM 0.0439453125f // (360/8192),将编码器值转化为角度制

#pragma pack(1)
#if DMMOTORCTRLMODE == 1
typedef struct {
    uint8_t id;
    uint8_t state;
    float Speed;      // 速率
    float last_position; 
    float position;      // 位置
    float torque;        // 力矩
    float T_Mos;
    float T_Rotor;       // 
    int32_t total_round;
} DMMotor_Measure;
#elif DMMOTORCTRLMODE == 2
typedef struct {
    float Speed;             // 速率
    uint16_t LastMchanicalAngle;   // 上次位置
    uint16_t MchanicalAngle;       // 位置
    int16_t Angle;                // 连续化角度制角度
    int16_t LastAngle;            // 连续化角度制角度
    float Angle_DEG;            // 连续化角度制角度
    float TorqueCurrent; 		// 力矩电流
    int16_t Mos_temperature;      // 线圈温度
    int16_t ERROR_Handle;      // error
	
	float SpeedFilter;        // 滤波后转速 /rpm
    float CurrentFilter;      // 滤波后转矩电流
    
    int16_t round;              // 圈数
	uint16_t MotorCenter;
} DMMotor_Measure;
#endif
#if DMMOTORCTRLMODE
typedef struct {
	uint16_t position_des;
	uint16_t velocity_des;
	uint16_t torque_des;
	uint16_t kp;
	uint16_t kd;
} DMMotor_Send_s;

typedef struct {
	DMMotor_Measure measure;
   	WatchDog_TypeDef* watchdog;        // 电机看门狗
    CANInstance* motor_can_instance;   // 电机CAN实例

   	Controller_s motor_controller;
   	Motor_Working_Type_e stop_flag;  // 电机启停标志
	
    /* 分组发送设置 */
    uint8_t sender_group;
    uint8_t message_num;
    float *other_angle_feedback_ptr;   // 角度反馈数据指针,注意电机使用total_angle
    float *other_speed_feedback_ptr;   //速度反馈数据指针,单位为angle per sec
//    uint16_t lost_cnt;
}DMMotor_Instance;
#pragma pack()

#pragma pack(1)
typedef enum {  // MIT模式使用
	DM_CMD_MOTOR_MODE = 0xfc, //使能，会响应指令
	DM_CMD_RESET_MODE = 0xfd, //停止
	DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
	DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;
#pragma pack()

DMMotor_Instance *DMMotorInit(Motor_Init_config_s *_config);

void DMMotorSetRef(DMMotor_Instance *motor, float ref);
/* 电流只能通过电机自带传感器监测*/
void DMMotorChangeFeed(DMMotor_Instance *motor, Feedback_Source_e type);
void DMMotorOuterLoop(DMMotor_Instance *motor, Closeloop_Type_e outer_loop);
void DMMotorEnable(DMMotor_Instance *motor);
void DMMotorStop(void *id);
void DMMotorReceive(DMMotor_Instance *motor, uint8_t buf[8]);
//void DMMotorCailEncoder(DMMotor_Instance *motor);
//void DMMotorControlInit();
void DMMmotor_Update();
#endif

#endif
