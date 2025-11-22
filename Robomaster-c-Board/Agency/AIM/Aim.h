#ifndef  __AIM_H
#define __AIM_H
#include "Variate.h"
#include "Function.h"
#include "VCOMCOMM.h"
#include "usbd_def.h"
#include "SolveTrajectory.h"
#define AM02_ARMOR_X  0.14
#define AM02_ARMOR_Y  0.125f
#define AM12_ARMOR_X  0.235f
#define AM12_ARMOR_Y  0.127f

/* 目标位姿 */
typedef struct{
        float x;
        float y;
        float z;
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
    }Rx_State;                        //!< @brief 接收状态 
    
    Pose_t Predicted_Center_Pose;     //!< @brief 惯性系下的目标中心点位姿
    float Predicted_Center_time;      //!< @brief 预测时间
    uint16_t  Fixed_Center_time;      //!< @brief 静态预测时间(拨弹盘转动，通信延迟等)
    float Predicted_Center_Yaw;       //!< @brief 预测中心的Y轴角度
    float Predicted_Center_Pitch;     //!< @brief 预测中心的P轴角度
    
    Pose_t Predicted_Armor_Pose[4];   //!< @brief 目标装甲板位姿（最多四块）
    float Predicted_Armor_time;
    uint16_t  Fixed_Armor_time;
    float Predicted_Armor_Yaw[4];
    float Predicted_Armor_Pitch; 
    
    float K_thre;                      //!< @brief 自动打弹角度阈值比例
    float P_thre;                      //!< @brief 自动打弹角度阈值
    float Y_thre; 
    float K;                           //!< @brief 空气阻力系数
    float PitchOffset;                 //!< @brief 弹道补偿
    uint8_t aim_runing;                //!< @brief 自瞄启动
    int64_t TimeStamp_setoff;          //!< @brief 补偿单片机时间戳
}Aim_Rx_t;
extern Aim_Rx_t Aim_Rx;

/* 发送给视觉的结构体 */
typedef struct{
    uint64_t TimeStamp;      //!< @brief 对齐后的时间戳
    struct{
        float w;       
        float x;
        float y;
        float z;
    }Quaternions;            //!< @brief IMU四元数(云台位姿）
    uint16_t Time_Gap;
    struct{
    float x;
    float y;
    float z;
    }Tar_Pose;              //!< @brief 预测击打点位姿
} Aim_Tx_t;
extern Aim_Tx_t Aim_Tx;

/* 接收视觉数据的结构体（内存对齐） */
#pragma pack(1)
typedef struct{
    struct{
			  float x;                         //!< @brief 全局坐标系下车辆中心点的坐标
        float y;
        float z;
        float Vx;                        //!< @brief 全局坐标系下车辆中心点的线速度
        float Vy;
        float Vz;
        float theta;                     //!< @brief 目标装甲板朝向角
        float omega;                     //!< @brief 目标装甲板朝向角的角速度
        float r1;                        //!< @brief 目标中心到前后装甲板的距离
        float r2;                        //!< @brief 目标中心到左右装甲板的距离
        float dz;                        //!< @brief 另一对装甲板的相对于被跟踪装甲板的高度差
        uint8_t armor_number;            //!< @brief 装甲板的数量
        uint8_t armor_id;                //!< @brief 目标车辆ID
    }pose;                               //!< @brief 目标车辆位姿
    uint8_t delay;                       //!< @brief 视觉程序延迟
    uint8_t tracker_status;              //!< @brief 相机追踪状态
} Aim_Rx_info;
#pragma pack()
extern Aim_Rx_info Aim_Rx_infopack;
extern float Predicted_Gap ;

extern uint8_t Opon_aim ;


/**
* @brief 自瞄打弹
*/
void Aim_Shoot();

/**
* @brief 自瞄补偿
*/
void Aim_Offset(float roll,float pitch,float yaw);
/**
* @brief 自瞄控制主程序
*/
void Aim_Control(void);
/**
* 自瞄数据计算
*/
void Aim_Calc();



#endif
