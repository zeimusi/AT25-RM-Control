/*!
 * @file Task_Vision.h
 * @date 2024-1-1
 * @brief 视觉通信（自瞄）任务头文件
 */
#ifndef __TASK_VISION_H
#define __TASK_VISION_H


#include "Task_Init.h"

#define AM02_ARMOR_X 0.14f
#define AM02_ARMOR_Y 0.125f
#define AM12_ARMOR_X 0.26f
#define AM12_ARMOR_Y 0.15f

extern float pointGun[3];


typedef struct {
    float Pitch;               //!<@brief Pitch轴角度
    float Yaw;                 //!<@brief Yaw轴角度
    float HorizontalDistance;
} Aim_t; 
extern Aim_t Aim_Ref;

/* 目标位姿 */
typedef struct{
    float X;
    float Y;
    float Z;
    float HorizontalDistance;
    float theta;
}Pose_t;

/* 储存自瞄信息 */
typedef struct{
    enum {
        TIMESTAMP = 0,
        COLOUR = 1,
        UPDATE = 2,
        UPDATING = 3,
    }Rx_State; //!< @brief 接收状态

    Pose_t Predicted_Center_Pose; //!< @brief 惯性系下的目标中心点位姿
    float Predicted_Center_time; //!< @brief 预测时间
    uint16_t Fixed_Center_time; //!< @brief 静态预测时间(拨弹盘转动，通信延迟等)
    float Predicted_Center_Yaw; //!< @brief 预测中心的Y轴角度
    float Predicted_Center_Pitch; //!< @brief 预测中心的P轴角度

    Pose_t Predicted_Armor_Pose[4]; //!< @brief 目标装甲板位姿（最多四块）
    float Predicted_Armor_time;
    uint16_t Fixed_Armor_time;
    float Predicted_Armor_Yaw;
    float Predicted_Armor_Pitch;

    float K_thre; //!< @brief 自动打弹角度阈值比例
    float P_thre; //!< @brief 自动打弹角度阈值
    float Y_thre;
    float K; //!< @brief 空气阻力系数
    float PitchOffset; //!< @brief 弹道补偿
    uint8_t aim_runing; //!< @brief 自瞄启动
    int64_t TimeStamp_setoff; //!< @brief 补偿单片机时间戳
}Aim_Rx_t;
extern Aim_Rx_t Aim_Rx;

typedef struct {
       float r[3][3];
} RotationMatrix_t;   //!< @brief 旋转矩阵（B->N）


extern TaskHandle_t Vision_Task_Handle;
void Vision_Task(void *pvParameters);

///*陀螺仪到转轴坐标转换*/
//void Coordinate_Transform(ReceiveAutoaimData_t *AutoaimData,Pose_t *pose, float theta,float x_offset,float y_offset,float z_offset);

/**
* @brief 自瞄控制主程序
*/
void Aim_Control(void);

/**
* @brief 坐标转换(N->B)
*/
//void Coordinate_Transformation (RotationMatrix_t R, const Pose_t* PoseN, Pose_t* PoseB);

/**
* @brief 坐标点到原点的水平距离
*/
float DistanceHorizontal(Pose_t pose);

///**
//* @brief 坐标点到原点的距离
//*/
//float DistanceToOrigin(Pose_t pose);

//float pitchTrajectoryCompensation(float s, float z, float v);

/**
* @brief 空气阻力模型
*/
float monoAirResistance_Model(float horizontal, float bullet_speed, float angle_pitch);

/**
* @brief 获得飞行时间
*/
float Get_bullet_fly_time( float horizontal, float bullet_speed, float angle_pitch);

///**
//* @brief 完全空气阻力模型
//*/
//double ballisticSolver(float horizontal, float vertical, double bullet_speed, double k);

/**
* @brief Pitch轴弹道补偿
*/
float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed);

#endif
