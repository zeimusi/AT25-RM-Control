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
//发给导航 （比赛状态，机器人血量，机器人位置，地面机器人位置，是否检测到各地区的RFID，允许发弹量，陀螺仪信息，自瞄信息）

#include "usb_task.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "Aim.h"

#define USB_OFFLINE_THRESHOLD 1000  // ms USB离线时间阈值
#define USB_CONNECT_CNT 10

// 发送时间间隔，每次发送经过以下时间后发送消息
#define SEND_DURATION_Imu           10   // ms
#define SEND_DURATION_AllRobotHp    10  // ms
#define SEND_DURATION_GameStatus    10  // ms
#define SEND_DURATION_MyLocation    10  // ms
#define SEND_DURATION_ALLLocation   10  // ms
#define SEND_DURATION_IsRFID        10  // ms
#define SEND_DURATION_AutoAimMsg    10  // ms
#define SEND_DURATION_AllowCapasity 10  // ms 

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte

// Variable Declarations
uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

/* 自瞄PC通信 */
SendVision_t SendVision = {.header = 0x5A,.checksum = 0};
ReceiveVisionData_t ReceiveVisionData = {.frame_header.sof =0xA5,.frame_header.id = 3,.checksum = 0};
ReceiveTwist_t ReceiveTwist = {.header = 0xA4,.checksum = 0};

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
static SendDataAllRobotHp_s     SEND_DATA_ALL_ROBOT_HP;    // 发送全场机器人剩余HP
static SendDataGameStatus_s     SEND_DATA_GAME_STATUS;     // 发送比赛运行状态
static SendDataMyRobotPos_t     SEND_DATA_MY_ROBOTPOS;     // 发送本机器人位置
static SendDataAllRobotPos_s    SEND_DATA_ALL_ROBOTPOS;    // 发送所有地面机器人位置
static SendDataRFIDstatus_s     SEND_DATA_RFID_STATUS;     // 发送RFID状态
static SendDataAutoAimMsg_s     SEND_DATA_AUTOAIMMSG;      // 发送自瞄状态
static SendDataAllowCapasity_s  SEND_DATA_ALLOWCAPASITY;   // 发送允许发弹量
                           
// 机器人控制指令数据（导航发来的x,y,z）
RobotCmdData_t ROBOT_CMD_DATA ;
RobotCmdData_t ROBOT_CMD_LASTDATA ;
// 发送数据间隔时间
typedef struct
{
    uint32_t Imu           ;
    uint32_t AllRobotHp    ;
    uint32_t GameStatus    ;
    uint32_t MyLocation    ;
    uint32_t ALLLocation   ;
    uint32_t IsRFID        ;
    uint32_t AutoAimMsg    ;
    uint32_t AllowCapasity ;
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
static void UsbSendAllRobotHpData(void);
static void UsbSendGameStatusData(void);
static void UsbSendMyLocationData(void);
static void UsbSendALLLocationData(void);
static void UsbSendIsRFIDData(void);
static void UsbSendAutoAimMsgData(void);
static void UsbSendAllowCapasityData(void);

/******************************************************************/
/* Task                                                           */
/******************************************************************/
/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void usb_task(void *pvParameters)
{
    UsbInit();
    uint32_t Last_time_stamp;
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
        UsbSendImuData();
        UsbReceiveData();

        if (HAL_GetTick() - RECEIVE_TIME < USB_OFFLINE_THRESHOLD) {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
					
        } else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT) {
            USB_OFFLINE = false;
            memset(&ReceiveVisionData, 0, sizeof(ReceiveVisionData_t));	

        } else {
            CONTINUE_RECEIVE_CNT++;
        }
        
        if(USB_OFFLINE == false)
        {
            ROBOT_CMD_DATA.data.speed_vector.vx = 0;
            ROBOT_CMD_DATA.data.speed_vector.vy = 0;
            ROBOT_CMD_DATA.data.speed_vector.wz = 0;
					
						USB_RX_BUF[0] = 0;
        }
//				if(ABS(ReceiveVisionData.time_stamp - Last_time_stamp) < 1){
//          memset(&ReceiveVisionData, 0, sizeof(ReceiveVisionData_t));	
//				}
        
        ROBOT_CMD_LASTDATA.data.speed_vector.vx = ROBOT_CMD_DATA.data.speed_vector.vx;
        ROBOT_CMD_LASTDATA.data.speed_vector.vy = ROBOT_CMD_DATA.data.speed_vector.vy;
        ROBOT_CMD_LASTDATA.data.speed_vector.wz = ROBOT_CMD_DATA.data.speed_vector.wz;
        
				Last_time_stamp = ReceiveVisionData.time_stamp;
        vTaskDelayUntil(&xLastWakeTime,3);
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
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));

    // 1.初始化IMU数据包
    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    SEND_DATA_IMU.frame_header.id = DATA_SEND_ID_Imu;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));

    // 2.初始化全场机器人剩余HP数据包
    SEND_DATA_ALL_ROBOT_HP.frame_header.sof = SEND_SOF;
    SEND_DATA_ALL_ROBOT_HP.frame_header.len = (uint8_t)(sizeof(SendDataAllRobotHp_s) - 6);
    SEND_DATA_ALL_ROBOT_HP.frame_header.id = DATA_SEND_ID_AllRobotHp;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ALL_ROBOT_HP.frame_header), sizeof(SEND_DATA_ALL_ROBOT_HP.frame_header));
    
    // 3.初始化比赛运行状态数据
    SEND_DATA_GAME_STATUS.frame_header.sof = SEND_SOF;
    SEND_DATA_GAME_STATUS.frame_header.len = (uint8_t)(sizeof(SendDataGameStatus_s) - 6);
    SEND_DATA_GAME_STATUS.frame_header.id = DATA_SEND_ID_GameStatus;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_GAME_STATUS.frame_header),sizeof(SEND_DATA_GAME_STATUS.frame_header));

    // 4.初始化本机器人位置数据
    SEND_DATA_MY_ROBOTPOS.frame_header.sof = SEND_SOF;
    SEND_DATA_MY_ROBOTPOS.frame_header.len = (uint8_t)(sizeof(SendDataMyRobotPos_t) - 6);
    SEND_DATA_MY_ROBOTPOS.frame_header.id = DATA_SEND_ID_MyLocation;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_MY_ROBOTPOS.frame_header),sizeof(SEND_DATA_MY_ROBOTPOS.frame_header));

    // 5.初始化所有地面机器人位置数据
    SEND_DATA_ALL_ROBOTPOS.frame_header.sof = SEND_SOF;
    SEND_DATA_ALL_ROBOTPOS.frame_header.len = (uint8_t)(sizeof(SendDataAllRobotPos_s) - 6);
    SEND_DATA_ALL_ROBOTPOS.frame_header.id = DATA_SEND_ID_AllLocation;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ALL_ROBOTPOS.frame_header),sizeof(SEND_DATA_ALL_ROBOTPOS.frame_header));

    // 6.初始化RFID状态数据
    SEND_DATA_RFID_STATUS.frame_header.sof = SEND_SOF;
    SEND_DATA_RFID_STATUS.frame_header.len = (uint8_t)(sizeof(SendDataRFIDstatus_s) - 6);
    SEND_DATA_RFID_STATUS.frame_header.id = DATA_SEND_ID_IsRFID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_RFID_STATUS.frame_header),sizeof(SEND_DATA_RFID_STATUS.frame_header));
        
    // 7.初始化自瞄状态数据
    SEND_DATA_AUTOAIMMSG.frame_header.sof = SEND_SOF;
    SEND_DATA_AUTOAIMMSG.frame_header.len = (uint8_t)(sizeof(SendDataAutoAimMsg_s) - 6);
    SEND_DATA_AUTOAIMMSG.frame_header.id = DATA_SEND_ID_AutoAimMsg;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_AUTOAIMMSG.frame_header),sizeof(SEND_DATA_AUTOAIMMSG.frame_header));

    // 8.初始化允许发弹量数据
    SEND_DATA_ALLOWCAPASITY.frame_header.sof = SEND_SOF;
    SEND_DATA_ALLOWCAPASITY.frame_header.len = (uint8_t)(sizeof(SendDataAllowCapasity_s) - 6);
    SEND_DATA_ALLOWCAPASITY.frame_header.id = DATA_SEND_ID_AllowCapasity;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ALLOWCAPASITY.frame_header),sizeof(SEND_DATA_ALLOWCAPASITY.frame_header));

}

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
uint16_t Timee;
static void UsbSendData(void)
{
//    CheckDurationAndSend(Imu);

//    CheckDurationAndSend(AllRobotHp);

//    CheckDurationAndSend(GameStatus);

//    CheckDurationAndSend(MyLocation);

//    CheckDurationAndSend(ALLLocation);
//    
//    CheckDurationAndSend(IsRFID);
//    
//    CheckDurationAndSend(AutoAimMsg);
//    
//    CheckDurationAndSend(AllowCapasity);
//    do {                                                                                 
//        if ((HAL_GetTick() - LAST_SEND_TIME.AutoAimMsg) >= SEND_DURATION_AutoAimMsg) { 
//            LAST_SEND_TIME.AutoAimMsg = HAL_GetTick();                                  
//            UsbSendAutoAimMsgData();                                                  
//        }                                                                                
                                                                             
        if ((HAL_GetTick() - LAST_SEND_TIME.Imu) >= SEND_DURATION_Imu) { 
            LAST_SEND_TIME.Imu = HAL_GetTick();                                  
            UsbSendImuData();                                                  

				}
//      UsbSendAutoAimMsgData();
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
uint8_t * sof_address;
static void UsbReceiveData(void)
{ 
	  static uint32_t Tim ,LastTim;
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
                    case ROBOT_CMD_DATA_RECEIVE_ID: {
                        memcpy(&ReceiveVisionData, sof_address, sizeof(ReceiveVisionData_t));
//                        Tim =HAL_GetTick();
//											  LastTim  = Tim;
//											  Predicted_Gap = Tim -LastTim;
											  Feed_Dog(&PC_Dog);
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
static void UsbSendImuData(void)
{
    SEND_DATA_IMU.time_stamp = HAL_GetTick();

    SEND_DATA_IMU.data.pitch = IMU.Angle_Pitch *Pi/180;
    SEND_DATA_IMU.data.yaw = IMU.Angle_Yaw*Pi/180;
    SEND_DATA_IMU.data.roll = IMU.Angle_Roll*Pi/180;
    SEND_DATA_IMU.data.pitch_vel = IMU.Gyro_Pitch;
    SEND_DATA_IMU.data.yaw_vel = IMU.Gyro_Yaw;
    SEND_DATA_IMU.data.roll_vel = IMU.Gyro_Roll;
    SEND_DATA_IMU.self_color = Referee_data_Rx.game_state_robot_color % 10;// 1--蓝 0--红
    SEND_DATA_IMU.aim_opon = Opon_aim;	
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU,sizeof(SendDataImu_s));

    USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));

}

///**
// * @brief 发送全场机器人hp信息数据
// * @param duration 发送周期
// */
//static void UsbSendAllRobotHpData(void)
//{
//    SEND_DATA_ALL_ROBOT_HP.time_stamp = HAL_GetTick();

//    memcpy(&SEND_DATA_ALL_ROBOT_HP.data, &judge_sensor.info->GameRobotHP, sizeof(SEND_DATA_ALL_ROBOT_HP.data));

//    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
//    USB_Transmit((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
//}

///**
// * @brief 发送比赛状态数据
// * @param duration 发送周期
// */
//static void UsbSendGameStatusData(void)
//{
//    SEND_DATA_GAME_STATUS.time_stamp = HAL_GetTick();

//    SEND_DATA_GAME_STATUS.data.game_progress = judge_sensor.info->GameStatus.game_progress;
//    SEND_DATA_GAME_STATUS.data.stage_remain_time = judge_sensor.info->GameStatus.stage_remain_time;

//    append_CRC16_check_sum((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
//    USB_Transmit((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
//}

///**
// * @brief 发送本机器人位置数据
// * @param duration 发送周期
// */
//static void UsbSendMyLocationData(void)
//{
//    SEND_DATA_MY_ROBOTPOS.time_stamp = HAL_GetTick();

//    memcpy(&SEND_DATA_MY_ROBOTPOS.data, &judge_sensor.info->RobotPos, sizeof(SEND_DATA_MY_ROBOTPOS.data));
//    
//    append_CRC16_check_sum((uint8_t *)&SEND_DATA_MY_ROBOTPOS, sizeof(SendDataMyRobotPos_t));
//    USB_Transmit((uint8_t *)&SEND_DATA_MY_ROBOTPOS, sizeof(SendDataMyRobotPos_t));
//}

///**
// * @brief 发送所有地面机器人位置数据
// * @param duration 发送周期
// */
//static void UsbSendALLLocationData(void)
//{
//    SEND_DATA_ALL_ROBOTPOS.time_stamp = HAL_GetTick();

//    memcpy(&SEND_DATA_ALL_ROBOTPOS.data, &judge_sensor.info->RobotPosition, sizeof(SEND_DATA_ALL_ROBOTPOS.data));    
//    
//    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALL_ROBOTPOS, sizeof(SendDataAllRobotPos_s));
//    USB_Transmit((uint8_t *)&SEND_DATA_ALL_ROBOTPOS, sizeof(SendDataAllRobotPos_s));
//}

///**
// * @brief 发送RFID状态数据
// * @param duration 发送周期
// */
//static void UsbSendIsRFIDData(void)
//{
//    SEND_DATA_RFID_STATUS.time_stamp = HAL_GetTick();

//    memcpy(&SEND_DATA_RFID_STATUS.data, &judge_sensor.info->RfidStatus, sizeof(SEND_DATA_RFID_STATUS.data));    
//    
//    append_CRC16_check_sum((uint8_t *)&SEND_DATA_RFID_STATUS, sizeof(SendDataRFIDstatus_s));
//    USB_Transmit((uint8_t *)&SEND_DATA_RFID_STATUS, sizeof(SendDataRFIDstatus_s));
//}

/**
 * @brief 发送自瞄状态数据
 * @param duration 发送周期
 */
static void UsbSendAutoAimMsgData(void)
{
//    SEND_DATA_AUTOAIMMSG.time_stamp = HAL_GetTick();

//    memcpy(&SEND_DATA_AUTOAIMMSG.data.L_AutoAimMsg, &AutoAimMsg_L, sizeof(SEND_DATA_AUTOAIMMSG.data));    
//    memcpy(&SEND_DATA_AUTOAIMMSG.data.R_AutoAimMsg, &AutoAimMsg_R, sizeof(SEND_DATA_AUTOAIMMSG.data));    


    append_CRC16_check_sum((uint8_t *)&SendVision, sizeof(SendVision_t));
    USB_Transmit((uint8_t *)&SendVision, sizeof(SendVision_t));
}

///**
// * @brief 发送允许发弹量数据
// * @param duration 发送周期
// */
//static void UsbSendAllowCapasityData(void)
//{
//    SEND_DATA_ALLOWCAPASITY.time_stamp = HAL_GetTick();

//    SEND_DATA_ALLOWCAPASITY.data.projectile_allowance = judge_sensor.info->ProjectileAllowance.projectile_allowance_17mm;
//    
//    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALLOWCAPASITY, sizeof(SendDataAllowCapasity_s));
//    USB_Transmit((uint8_t *)&SEND_DATA_ALLOWCAPASITY, sizeof(SendDataAllowCapasity_s));
//}
