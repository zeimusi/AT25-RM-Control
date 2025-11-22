#ifndef USB_TASK_H
#define USB_TASK_H

#include "Task_Init.h"

#define SEND_SOF    ((uint8_t)0x5A)
#define RECEIVE_SOF ((uint8_t)0x5A)

/******************************* 发送id *********************************/
/* 视觉用的 */
#define DATA_SEND_ID_Imu           ((uint8_t)0x01)  // ms
#define DATA_SEND_ID_AutoAim       ((uint8_t)0x09)  // ms

/******************************* 接收id *********************************/
#define AUTOAIM_DATA_RECEIVE_ID         ((uint8_t)0x03)

typedef struct
{
    uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
    uint8_t len;  // 数据段长度
    uint8_t id;   // 数据段id
    uint8_t crc;  // 数据帧头的 CRC8 校验
} __packed__ FrameHeader_t;
/*-------------------- Send --------------------*/
/*----- 视觉用的 -----*/
// IMU 数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x01
    uint32_t time_stamp;
    struct
    {
        float yaw;    // rad
        float pitch;  // rad
        float roll;   // rad

        float yaw_vel;    // rad/s
        float pitch_vel;  // rad/s
        float roll_vel;   // rad/s
    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataImu_s;

// 发给自瞄
typedef struct 
{
    FrameHeader_t frame_header;  // id = 0x09
    uint32_t time_stamp;

    struct
    {
        uint8_t detect_color;
        uint8_t reset_tracker;
        float roll;
        float pitch;
        float yaw;
        float aim_x;
        float aim_y;
        float aim_z;
    } __attribute__((packed)) data;

    uint16_t crc;
} __packed__ SendDataAutoAim_s;

/*-------------------- Receive --------------------*/
typedef struct{
  FrameHeader_t frame_header;

  uint32_t time_stamp;

  struct 
  {
    uint8_t tracking;
    uint8_t id;
    uint8_t armors_num;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
  } __attribute__((packed)) data;

  uint16_t checksum;
} __packed__ ReceiveAutoaimData_t;
extern ReceiveAutoaimData_t ReceiveAutoaimData;


extern TaskHandle_t usb_task_handle;
void usb_task(void *pvParameters);

#endif /* USB_TASK_H */


