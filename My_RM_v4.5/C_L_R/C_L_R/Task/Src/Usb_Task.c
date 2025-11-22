/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      通过USB串口与上位机通信
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/
#include "usb_task.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"

#define USB_OFFLINE_THRESHOLD 1000  // ms USB离线时间阈值
#define USB_CONNECT_CNT 10

// 发送时间间隔，每次发送经过以下时间后发送消息
#define SEND_DURATION_Imu           5   // ms
#define SEND_DURATION_AutoAim       10  // ms 

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte

// Variable Declarations
uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

// ##send_name 是一个预处理器操作符，用于将 send_name 的值与 字符 连接
// 检查是否已经过了一定的时间间隔， 超过时间间隔后，发送消息
#define CheckDurationAndSend(send_name)                                                  \
    do {                                                                                 \
        if ((HAL_GetTick() - LAST_SEND_TIME.##send_name) >= SEND_DURATION_##send_name) { \
            LAST_SEND_TIME.##send_name = HAL_GetTick();                                  \
            UsbSend##send_name##Data();                                                  \
        }                                                                                \
    } while (0)

// 判断USB连接状态用到的一些变量
bool USB_OFFLINE = true;
uint32_t RECEIVE_TIME = 0;
uint32_t LATEST_RX_TIMESTAMP = 0;
uint32_t CONTINUE_RECEIVE_CNT = 0;

// 数据发送结构体
static SendDataImu_s            SEND_DATA_IMU;             // 发送IMU数据
       SendDataAutoAim_s        SEND_DATA_AutoAim;         // 发给视觉的数据  
    
// 机器人控制指令数据（视觉导航发来的x,y,z）
ReceiveAutoaimData_t ReceiveAutoaimData;

// 发送数据间隔时间
typedef struct
{
    uint32_t Imu           ;
    uint32_t AutoAim       ;
} LastSendTime_t;
static LastSendTime_t LAST_SEND_TIME;

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/
static void UsbSendData(void);
static void UsbReceiveData(void);
static void UsbInit(void);

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/
static void UsbSendImuData(void);
static void UsbSendAutoAimData(void);

/******************************************************************/
/* Task                                                           */
/******************************************************************/
/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
 
TaskHandle_t usb_task_handle;
void usb_task(void *pvParameters)
{
    UsbInit();
    
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
   
        UsbSendData();
        UsbReceiveData();

        if (HAL_GetTick() - RECEIVE_TIME < USB_OFFLINE_THRESHOLD) {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
        } else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT) {
            USB_OFFLINE = false;
        } else {
            CONTINUE_RECEIVE_CNT++;
        }
        
        if(USB_OFFLINE == false)
        {
            memset(&USB_RX_BUF, 0, sizeof(USB_RX_BUF));
        }
                
        
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}


/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/
/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    // 数据置零
    memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));
    memset(&ReceiveAutoaimData, 0, sizeof(ReceiveAutoaimData_t));

    // 1.初始化IMU数据包
    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    SEND_DATA_IMU.frame_header.id = DATA_SEND_ID_Imu;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));

    // 2.初始化自瞄代码
    SEND_DATA_AutoAim.frame_header.sof = SEND_SOF;
    SEND_DATA_AutoAim.frame_header.len = (uint8_t)(sizeof(SendDataAutoAim_s) - 6);
    SEND_DATA_AutoAim.frame_header.id = DATA_SEND_ID_AutoAim;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_AutoAim.frame_header),sizeof(SEND_DATA_AutoAim.frame_header));    
    
}

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
    UsbSendImuData();
    UsbSendAutoAimData();
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
uint8_t ignore_id = 0;
uint16_t c = 0;

uint8_t * sof_address;
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;
    static uint8_t * rx_data_start_address = USB_RX_BUF;  // 接收数据包时存放于缓存区的起始位置
    static uint8_t * rx_data_end_address;  // 接收数据包时存放于缓存区的结束位置
    sof_address = USB_RX_BUF;

    // 计算数据包的结束位置
    rx_data_end_address = rx_data_start_address + USB_RECEIVE_LEN;
    // 读取数据
    USB_Receive(rx_data_start_address, &len);  // Read data into the buffer

    while (sof_address <= rx_data_end_address) {  // 解析缓冲区中的所有数据包
        // 寻找帧头位置
        while (*(sof_address) != RECEIVE_SOF && (sof_address <= rx_data_end_address)) {
            sof_address++;
        }
        // 判断是否超出接收数据范围
        if (sof_address > rx_data_end_address) {
            break;  // 退出循环
        }
        // 检查CRC8校验
        bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
        if (crc8_ok) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            // 检查整包CRC16校验 4: header size, 2: crc16 size
            bool crc16_ok = verify_CRC16_check_sum(sof_address, 4 + data_len + 2);
            if (crc16_ok) {
                switch (data_id) {
                    case AUTOAIM_DATA_RECEIVE_ID:{
                        ReceiveAutoaimData.data.id = sof_address[9];
                        if(ReceiveAutoaimData.data.id != 0)
                        {
                            if((Judge_Msg.IsAttack_2 == 0 && ReceiveAutoaimData.data.id != 2) || Judge_Msg.IsAttack_2 == 1){
								if((Is_Fortress_Msg.Is_Fortress_Flag == 1 && ignore_id != ReceiveAutoaimData.data.id)||Is_Fortress_Msg.Is_Fortress_Flag == 0)
								{
									memcpy(&ReceiveAutoaimData,sof_address, sizeof(ReceiveAutoaimData_t));     
									Aim_Rx.Rx_State = UPDATING; 
									Feed_Dog(&WatchDog.PC_WatchDog);
								}
                            }
                        }
                    }
                    break;
                    
                    default:break;
                }
                if (*((uint32_t *)(&sof_address[4])) > LATEST_RX_TIMESTAMP) {
                    LATEST_RX_TIMESTAMP = *((uint32_t *)(&sof_address[4]));
                    RECEIVE_TIME = HAL_GetTick();
                }
            }
            sof_address += (data_len + HEADER_SIZE + 2);
        } else {  //CRC8校验失败，移动到下一个字节
            sof_address++;
        }
    }
    // 更新下一次接收数据的起始位置
    if (sof_address > rx_data_start_address + USB_RECEIVE_LEN) {
        // 缓冲区中没有剩余数据，下次接收数据的起始位置为缓冲区的起始位置
        rx_data_start_address = USB_RX_BUF;
    } else {
        uint16_t remaining_data_len = USB_RECEIVE_LEN - (sof_address - rx_data_start_address);
        // 缓冲区中有剩余数据，下次接收数据的起始位置为缓冲区中剩余数据的起始位置
        rx_data_start_address = USB_RX_BUF + remaining_data_len;
        // 将剩余数据移到缓冲区的起始位置
        memcpy(USB_RX_BUF, sof_address, remaining_data_len);
    }
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

/**
 * @brief 发送IMU数据
 * @param duration 发送周期
 */
uint8_t a,cnt;
static void UsbSendImuData(void)
{
    SEND_DATA_IMU.time_stamp = HAL_GetTick();

    if(a == 0)
    {
        cnt++;
        if(cnt == 255){
            a = 1;
            IMU.EulerAngler_Init.Pitch = IMU.EulerAngler_RAW.Pitch;                            //修改过yaw
            IMU.EulerAngler_Init.Yaw = IMU.EulerAngler_RAW.Yaw;
            IMU.EulerAngler_Init.Roll = IMU.EulerAngler_RAW.Roll;   
        }
    }
    
    IMU.EulerAngler.Pitch = IMU.EulerAngler_RAW.Pitch - IMU.EulerAngler_Init.Pitch;
    IMU.EulerAngler.Yaw = IMU.EulerAngler_RAW.Yaw - IMU.EulerAngler_Init.Yaw;
    IMU.EulerAngler.Roll = IMU.EulerAngler_RAW.Roll - IMU.EulerAngler_Init.Roll;	
    
    float diff = IMU.EulerAngler.Yaw - IMU.EulerAngler.LsatAngle;
    if(diff != IMU.EulerAngler.Yaw)
       IMU.EulerAngler.flag = 1;
    if(IMU.EulerAngler.flag == 1)
    {
        if(diff > 100) 
            IMU.EulerAngler.r--;
        else if(diff < -100) 
            IMU.EulerAngler.r++;      
    }
    
    IMU.EulerAngler.ContinuousYaw = IMU.EulerAngler.r*360+IMU.EulerAngler.Yaw;
    IMU.EulerAngler.LsatAngle = IMU.EulerAngler.Yaw;

	
    SEND_DATA_IMU.data.pitch = IMU.EulerAngler.Pitch*pi/180;
    SEND_DATA_IMU.data.yaw = IMU.EulerAngler.ContinuousYaw*pi/180;
    SEND_DATA_IMU.data.roll = IMU.EulerAngler.Roll*pi/180;

    SEND_DATA_IMU.data.pitch_vel = IMU.AngularVelocity.y;
    SEND_DATA_IMU.data.yaw_vel = IMU.AngularVelocity.z;
    SEND_DATA_IMU.data.roll_vel = IMU.AngularVelocity.x;   
    
//    SEND_DATA_IMU.data.pitch = IMU.EulerAngler_Mech.Pitch*PI/180;
//    SEND_DATA_IMU.data.yaw = IMU.EulerAngler_Mech.Yaw*PI/180;
//    SEND_DATA_IMU.data.roll = IMU.EulerAngler_Mech.Roll*PI/180;
//    SEND_DATA_IMU.data.pitch_vel = IMU.AngularVelocity.y;
//    SEND_DATA_IMU.data.yaw_vel = IMU.AngularVelocity.z;
//    SEND_DATA_IMU.data.roll_vel = IMU.AngularVelocity.x;
    
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
    USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}

/**
 * @brief 发给视觉状态数据
 * @param duration 发送周期
 */

static void UsbSendAutoAimData(void)
{
    SEND_DATA_AutoAim.time_stamp = HAL_GetTick();

    SEND_DATA_AutoAim.data.detect_color = (uint8_t)Judge_Msg.Color;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_AutoAim, sizeof(SendDataAutoAim_s));
    USB_Transmit((uint8_t *)&SEND_DATA_AutoAim, sizeof(SendDataAutoAim_s));
}

