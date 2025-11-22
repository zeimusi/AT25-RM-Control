#ifndef __DATA_EXCHANGE_H
#define __DATA_EXCHANGE_H
#include "struct_typedef.h"
//#include "custom_typedef.h"
#include <WatchDog.h>

// 数据名称宏
#define SHOOT_MOTOR_R  "SHOOT_MOTOR_R"
#define SHOOT_MOTOR_L  "SHOOT_MOTOR_L"
#define GIMBAL_MOTOR_Y "GIMBAL_MOTOR_Y"
#define GIMBAL_MOTOR_P "GIMBAL_MOTOR_P"
#define PLUCK_MOTOR  "PLUCK_MOTOR"
#define ACTION_NAME  "Action_name"
#define RC_Ctrl_Name "RC_CtrlData"
#define IMU_NAME "imu_data"

/* 看门狗 */
extern WatchDog_TypeDef Remote_Dog, IMU_Dog, Gimbal_Dog[2], Shoot_Dog[2], Pluck_Dog, Down_Dog, PC_Dog;

typedef enum __DataExchangeIndex {
    TEST_DATA = 0,
    YAW_ANGLE,
    Data_Exchange_INDEX_NUM
} DataExchangeIndex_e;

typedef enum __Data_Type {
    DE_INT8 = 0,
    DE_UINT8,
    DE_INT16,
    DE_UINT16,
    DE_INT32,
    DE_UINT32,
    DE_FLOAT,
    Data_Type_NUM
} DataType_e;

typedef enum DataPublishStatus {
    PUBLISH_FAIL = 0,
    PUBLISH_OK,
    PUBLISH_ALREADY_EXIST,
    PUBLISH_ALREADY_FULL
} DataPublishStatus_e;

typedef enum DataSubscribeStatus { SUBSCRIBE_FAIL = 0, SUBSCRIBE_OK } DataSubscribeStatus_e;

extern uint8_t Publish(void * address, char * name);
extern const void * Subscribe(char * name);

#endif  // __DATA_EXCHANGE_H
