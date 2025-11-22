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
#define SEND_DURATION_RobotInfo     10  // ms
#define SEND_DURATION_AllRobotHp    10  // ms
#define SEND_DURATION_GameStatus    10  // ms 
#define SEND_DURATION_AutoAim_L     10  // ms 
#define SEND_DURATION_AutoAim_R     10  // ms 
#define SEND_DURATION_GameBegin     10  // ms 
#define SEND_DURATION_ALLLocation   10  // ms
#define SEND_DURATION_IsRFID        10  // ms
#define SEND_DURATION_AllowCapasity 10  // ms 
#define SEND_DURATION_MapCommand    10  // ms 
#define SEND_DURATION_GimbleDiff	10	// ms
#define SEND_DURATION_Color      	10	// ms

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
static SendDataRobotInfo_s      SEND_DATA_ROBOT_INFO;         // 发送机器人各部分类型，在线情况以及比赛时的状态（裁判系统）
static SendDataAllRobotHp_s     SEND_DATA_ALL_ROBOT_HP;    // 发送全场机器人剩余HP
static SendDataGameStatus_s     SEND_DATA_GAME_STATUS;     // 发送比赛运行状态
static SendDataAutoAim_L_s      SendDataAutoAim_L;         // 发送允许发弹量
static SendDataAutoAim_R_s      SendDataAutoAim_R;         // 发送允许发弹量
static SendDataGameBegin_s      SendDataGameBegin;         // 发送允许发弹量
static SendDataAllRobotPos_s    SEND_DATA_ALL_ROBOTPOS;    // 发送所有地面机器人位置
static SendDataRFIDstatus_s     SEND_DATA_RFID_STATUS;     // 发送RFID状态
static SendDataAllowCapasity_s  SEND_DATA_ALLOWCAPASITY;   // 发送允许发弹量
static SendDataMapCommand_s     SEND_DATA_MAPCOMMAND;      // 发送云台手下发坐标
static SendDataGimbleDiff_s		SEND_DATA_GimbleDiff;
static SendDataColor_s	     	SEND_DATA_Color;
// 机器人控制指令数据（视觉导航发来的x,y,z）
ReceiveAutoaimData_t ReceiveAutoaimData;
RobotCmdData_t ROBOT_CMD_DATA ;
RobotCmdData_t ROBOT_CMD_LASTDATA ;
IsUP_t IsUP ;
IsFortress_t IsFortress;
// 发送数据间隔时间
typedef struct
{
    uint32_t Imu           ;
    uint32_t AutoAim       ;
    uint32_t AllRobotHp    ;
    uint32_t GameStatus    ;
    uint32_t RobotInfo     ;
    uint32_t AutoAim_L     ;
    uint32_t AutoAim_R     ;
    uint32_t GameBegin     ;
    uint32_t ALLLocation   ;
    uint32_t IsRFID        ;
    uint32_t AllowCapasity ;
    uint32_t MapCommand    ;
	uint32_t GimbleDiff	   ;
	uint32_t Color         ;
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
static void UsbSendAllRobotHpData(void);
static void UsbSendGameStatusData(void);
static void UsbSendRobotInfoData(void);
static void UsbSendAutoAimData_L(void);
static void UsbSendAutoAimData_R(void);
static void UsbSendGameBeginData(void);
static void UsbSendALLLocationData(void);
static void UsbSendIsRFIDData(void);
static void UsbSendAllowCapasityData(void);
static void UsbSendMapCommandData(void);
static void UsbSendGimbleDiffData(void);
static void UsbSendColorData(void);
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
            ROBOT_CMD_DATA.data.speed_vector.vx = 0;
            ROBOT_CMD_DATA.data.speed_vector.vy = 0;
            ROBOT_CMD_DATA.data.speed_vector.wz = 0;
            memset(&USB_RX_BUF, 0, sizeof(USB_RX_BUF));
        }
        
        ROBOT_CMD_LASTDATA.data.speed_vector.vx = ROBOT_CMD_DATA.data.speed_vector.vx;
        ROBOT_CMD_LASTDATA.data.speed_vector.vy = ROBOT_CMD_DATA.data.speed_vector.vy;
        ROBOT_CMD_LASTDATA.data.speed_vector.wz = ROBOT_CMD_DATA.data.speed_vector.wz;
        
        
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
    memset(&ROBOT_CMD_DATA, 0, sizeof(RobotCmdData_t));
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
    
    // 2.初始化全场机器人剩余HP数据包
    SEND_DATA_ALL_ROBOT_HP.frame_header.sof = SEND_SOF;
    SEND_DATA_ALL_ROBOT_HP.frame_header.len = (uint8_t)(sizeof(SendDataAllRobotHp_s) - 6);
    SEND_DATA_ALL_ROBOT_HP.frame_header.id = ALL_ROBOT_HP_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ALL_ROBOT_HP.frame_header), sizeof(SEND_DATA_ALL_ROBOT_HP.frame_header));
    
    // 3.初始化比赛运行状态数据
    SEND_DATA_GAME_STATUS.frame_header.sof = SEND_SOF;
    SEND_DATA_GAME_STATUS.frame_header.len = (uint8_t)(sizeof(SendDataGameStatus_s) - 6);
    SEND_DATA_GAME_STATUS.frame_header.id = GAME_STATUS_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_GAME_STATUS.frame_header),sizeof(SEND_DATA_GAME_STATUS.frame_header));

    // 3.初始化机器人信息数据包
    // 帧头部分
    SEND_DATA_ROBOT_INFO.frame_header.sof = SEND_SOF;
    SEND_DATA_ROBOT_INFO.frame_header.len = (uint8_t)(sizeof(SendDataRobotInfo_s) - 6);
    SEND_DATA_ROBOT_INFO.frame_header.id = ROBOT_INFO_DATA_SEND_ID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ROBOT_INFO.frame_header), sizeof(SEND_DATA_ROBOT_INFO.frame_header));
        
//    // 4.左
//    SendDataAutoAim_L.frame_header.sof = SEND_SOF;
//    SendDataAutoAim_L.frame_header.len = (uint8_t)(sizeof(SendDataAutoAim_L_s) - 6);
//    SendDataAutoAim_L.frame_header.id = DATA_SEND_ID_AutoAim_L;
//    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
//        (uint8_t *)(&SendDataAutoAim_L.frame_header),sizeof(SendDataAutoAim_L.frame_header));

//    // 5.右
//    SendDataAutoAim_R.frame_header.sof = SEND_SOF;
//    SendDataAutoAim_R.frame_header.len = (uint8_t)(sizeof(SendDataAutoAim_R_s) - 6);
//    SendDataAutoAim_R.frame_header.id = DATA_SEND_ID_AutoAim_R;
//    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
//        (uint8_t *)(&SendDataAutoAim_R.frame_header),sizeof(SendDataAutoAim_R.frame_header));

    // 6.游戏状态
    SendDataGameBegin.frame_header.sof = SEND_SOF;
    SendDataGameBegin.frame_header.len = (uint8_t)(sizeof(SendDataGameBegin_s) - 6);
    SendDataGameBegin.frame_header.id = DATA_SEND_ID_GameBegin;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SendDataGameBegin.frame_header),sizeof(SendDataGameBegin.frame_header));        
        
    // 7.初始化所有地面机器人位置数据
    SEND_DATA_ALL_ROBOTPOS.frame_header.sof = SEND_SOF;
    SEND_DATA_ALL_ROBOTPOS.frame_header.len = (uint8_t)(sizeof(SendDataAllRobotPos_s) - 6);
    SEND_DATA_ALL_ROBOTPOS.frame_header.id = DATA_SEND_ID_AllLocation;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ALL_ROBOTPOS.frame_header),sizeof(SEND_DATA_ALL_ROBOTPOS.frame_header));

    // 8.初始化RFID状态数据
    SEND_DATA_RFID_STATUS.frame_header.sof = SEND_SOF;
    SEND_DATA_RFID_STATUS.frame_header.len = (uint8_t)(sizeof(SendDataRFIDstatus_s) - 6);
    SEND_DATA_RFID_STATUS.frame_header.id = DATA_SEND_ID_IsRFID;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_RFID_STATUS.frame_header),sizeof(SEND_DATA_RFID_STATUS.frame_header));
        
    // 9.初始化允许发弹量数据
    SEND_DATA_ALLOWCAPASITY.frame_header.sof = SEND_SOF;
    SEND_DATA_ALLOWCAPASITY.frame_header.len = (uint8_t)(sizeof(SendDataAllowCapasity_s) - 6);
    SEND_DATA_ALLOWCAPASITY.frame_header.id = DATA_SEND_ID_AllowCapasity;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_ALLOWCAPASITY.frame_header),sizeof(SEND_DATA_ALLOWCAPASITY.frame_header));

    // 9.初始化云台手下发数据
    SEND_DATA_MAPCOMMAND.frame_header.sof = SEND_SOF;
    SEND_DATA_MAPCOMMAND.frame_header.len = (uint8_t)(sizeof(SendDataMapCommand_s) - 6);
    SEND_DATA_MAPCOMMAND.frame_header.id = DATA_SEND_ID_MapCommand;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_MAPCOMMAND.frame_header),sizeof(SEND_DATA_MAPCOMMAND.frame_header));

	SEND_DATA_GimbleDiff.frame_header.sof = SEND_SOF;
	SEND_DATA_GimbleDiff.frame_header.len = (uint8_t)(sizeof(SendDataGimbleDiff_s) - 6);
	SEND_DATA_GimbleDiff.frame_header.id = DATA_SEND_ID_GimbleDiff;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_GimbleDiff.frame_header),sizeof(SEND_DATA_GimbleDiff.frame_header));

	SEND_DATA_Color.frame_header.sof = SEND_SOF;
	SEND_DATA_Color.frame_header.len = (uint8_t)(sizeof(SendDataColor_s) - 6);
	SEND_DATA_Color.frame_header.id = DATA_SEND_ID_Color;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_Color.frame_header),sizeof(SEND_DATA_Color.frame_header));

}

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
uint8_t flag_IsMapCommand = 0,a = 0;
static void UsbSendData(void)
{
    UsbSendAllowCapasityData();//从这里判断发单量是否还足够
    UsbSendGameBeginData();//比赛是否开始的标志位 
    UsbSendAllRobotHpData();//全体机器人血量，从这里获得哨兵血量
//    UsbSendGimbleDiffData();
	UsbSendALLLocationData();

    if(flag_IsMapCommand == 1)
    { 
        UsbSendMapCommandData();//云台手下发的数据，从这里判断是否下堡垒
        flag_IsMapCommand = 0;
    }
    
	if(DeviceStatus.AimAngle_L == Device_Right || DeviceStatus.AimAngle_R == Device_Right)
	{
		UsbSendGimbleDiffData();
	}
	
//    if(DeviceStatus.AimAngle_L == Device_Right)
//        UsbSendAutoAimData_L();
//    if(DeviceStatus.AimAngle_R == Device_Right)
//        UsbSendAutoAimData_R();
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
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
                    case ROBOT_CMD_DATA_RECEIVE_ID: {
                        memcpy(&ROBOT_CMD_DATA, sof_address, sizeof(RobotCmdData_t));
                    } 
                    break;
                        
                    case IsUP_DATA_RECEIVE_ID:{
                        memcpy(&IsUP, sof_address, sizeof(IsUP_t));
                    }
                    break;
					
					case IsFortress_DATA_RECEIVE_ID:{
						memcpy(&IsFortress, sof_address, sizeof(IsFortress_t));
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

    
    SEND_DATA_IMU.data.pitch = IMU.EulerAngler.Pitch*PI/180;
    SEND_DATA_IMU.data.yaw = IMU.EulerAngler.Yaw*PI/180;
    SEND_DATA_IMU.data.roll = IMU.EulerAngler.Roll*PI/180;
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

    if(judge_sensor.info->RobotStatus.robot_id == 7)
        SEND_DATA_AutoAim.data.detect_color = 1;
    else 
        SEND_DATA_AutoAim.data.detect_color = 0;
    
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_AutoAim, sizeof(SendDataAutoAim_s));
    USB_Transmit((uint8_t *)&SEND_DATA_AutoAim, sizeof(SendDataAutoAim_s));
}

/**
 * @brief 发送全场机器人hp信息数据
 * @param duration 发送周期
 */
static void UsbSendAllRobotHpData(void)
{
    SEND_DATA_ALL_ROBOT_HP.time_stamp = HAL_GetTick();

    memcpy(&SEND_DATA_ALL_ROBOT_HP.data, &judge_sensor.info->GameRobotHP, sizeof(SEND_DATA_ALL_ROBOT_HP.data));
//	SEND_DATA_ALL_ROBOT_HP.data.blue_7_robot_hp = 400;   //调试用,记得删
	
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ALL_ROBOT_HP, sizeof(SendDataAllRobotHp_s));
}

/**
 * @brief 发送比赛状态数据
 * @param duration 发送周期
 */
static void UsbSendGameStatusData(void)
{
    SEND_DATA_GAME_STATUS.time_stamp = HAL_GetTick();

    SEND_DATA_GAME_STATUS.data.game_progress = judge_sensor.info->GameStatus.game_progress;
    SEND_DATA_GAME_STATUS.data.stage_remain_time = judge_sensor.info->GameStatus.stage_remain_time;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
    USB_Transmit((uint8_t *)&SEND_DATA_GAME_STATUS, sizeof(SendDataGameStatus_s));
}
/**
 * @brief 发送机器人信息数据
 * @param duration 发送周期
 */
static void UsbSendRobotInfoData(void)
{
    SEND_DATA_ROBOT_INFO.time_stamp = HAL_GetTick();

    SEND_DATA_ROBOT_INFO.data.referee.id = judge_sensor.info->RobotStatus.robot_id;
    SEND_DATA_ROBOT_INFO.data.referee.hp = judge_sensor.info->RobotStatus.current_HP;

    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ROBOT_INFO, sizeof(SendDataRobotInfo_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ROBOT_INFO, sizeof(SendDataRobotInfo_s));
}

/**
 * @brief 发送所有地面机器人位置数据
 * @param duration 发送周期
 */
static void UsbSendALLLocationData(void)
{
    SEND_DATA_ALL_ROBOTPOS.time_stamp = HAL_GetTick();

    memcpy(&SEND_DATA_ALL_ROBOTPOS.data, &judge_sensor.info->RobotPosition, sizeof(SEND_DATA_ALL_ROBOTPOS.data));    
    
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALL_ROBOTPOS, sizeof(SendDataAllRobotPos_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ALL_ROBOTPOS, sizeof(SendDataAllRobotPos_s));
}

/**
 * @brief 发送RFID状态数据
 * @param duration 发送周期
 */
static void UsbSendIsRFIDData(void)
{
    SEND_DATA_RFID_STATUS.time_stamp = HAL_GetTick();

    memcpy(&SEND_DATA_RFID_STATUS.data, &judge_sensor.info->RfidStatus, sizeof(SEND_DATA_RFID_STATUS.data));    
    
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_RFID_STATUS, sizeof(SendDataRFIDstatus_s));
    USB_Transmit((uint8_t *)&SEND_DATA_RFID_STATUS, sizeof(SendDataRFIDstatus_s));
}

/**
 * @brief 发送允许发弹量数据
 * @param duration 发送周期
 */
static void UsbSendAllowCapasityData(void)
{
    SEND_DATA_ALLOWCAPASITY.time_stamp = HAL_GetTick();

    SEND_DATA_ALLOWCAPASITY.data.projectile_allowance = judge_sensor.info->ProjectileAllowance.projectile_allowance_17mm;
//    SEND_DATA_ALLOWCAPASITY.data.projectile_allowance = 300;  //调试用,记得删
	
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_ALLOWCAPASITY, sizeof(SendDataAllowCapasity_s));
    USB_Transmit((uint8_t *)&SEND_DATA_ALLOWCAPASITY, sizeof(SendDataAllowCapasity_s));
}

///**
// * @brief 左头视觉数据
// * @param duration 发送周期
// */
//static void UsbSendAutoAimData_L(void)
//{
//    SendDataAutoAim_L.time_stamp = HAL_GetTick();

//    SendDataAutoAim_L.data.x = AutoAimXY_L.x;    
//    SendDataAutoAim_L.data.y = AutoAimXY_L.y;
//	SendDataAutoAim_L.data.z = AutoAimZ_L.L_z;
//    
//    append_CRC16_check_sum((uint8_t *)&SendDataAutoAim_L, sizeof(SendDataAutoAim_L_s));
//    USB_Transmit((uint8_t *)&SendDataAutoAim_L, sizeof(SendDataAutoAim_L_s));
//}

///**
// * @brief 右头视觉数据
// * @param duration 发送周期
// */
//static void UsbSendAutoAimData_R(void)
//{
//    SendDataAutoAim_R.time_stamp = HAL_GetTick();

//    SendDataAutoAim_R.data.x = AutoAimXY_R.x;    
//    SendDataAutoAim_R.data.y = AutoAimXY_R.y;
//	SendDataAutoAim_R.data.z = AutoAimZ_R.R_z;
//    
//    append_CRC16_check_sum((uint8_t *)&SendDataAutoAim_R, sizeof(SendDataAutoAim_R_s));
//    USB_Transmit((uint8_t *)&SendDataAutoAim_R, sizeof(SendDataAutoAim_R_s));
//}


/**
 * @brief 比赛是否开始
 * @param duration 发送周期
 */
static void UsbSendGameBeginData(void)
{
    SendDataGameBegin.time_stamp = HAL_GetTick();

    if(judge_sensor.info->GameStatus.game_progress == 4)
        SendDataGameBegin.IsGameBegin_Flag = 1;
    else
        SendDataGameBegin.IsGameBegin_Flag = 0; 
    
//	SendDataGameBegin.IsGameBegin_Flag = 1;
	
    append_CRC16_check_sum((uint8_t *)&SendDataGameBegin, sizeof(SendDataGameBegin_s));
    USB_Transmit((uint8_t *)&SendDataGameBegin, sizeof(SendDataGameBegin_s));

}

/**
 * @brief 云台手下发数据
 * @param duration 发送周期
 */
static void UsbSendMapCommandData(void)
{
    SEND_DATA_MAPCOMMAND.time_stamp = HAL_GetTick();
    
    SEND_DATA_MAPCOMMAND.data.target_position_x = judge_sensor.info->map_command.target_position_x;
    SEND_DATA_MAPCOMMAND.data.target_position_y = judge_sensor.info->map_command.target_position_y;
    
    append_CRC16_check_sum((uint8_t *)&SEND_DATA_MAPCOMMAND, sizeof(SendDataMapCommand_s));
    USB_Transmit((uint8_t *)&SEND_DATA_MAPCOMMAND, sizeof(SendDataMapCommand_s));

}

/**
 * @brief 
 * @param duration 发送周期
 */
static void UsbSendGimbleDiffData(void)
{
	SEND_DATA_GimbleDiff.time_stamp = HAL_GetTick();
	
	if(DeviceStatus.AimAngle_L == Device_Right){
		SEND_DATA_GimbleDiff.data.Pitch_Diff_L = GimbleDiff_L.Pitch_Diff;
		SEND_DATA_GimbleDiff.data.Yaw_Diff_L = GimbleDiff_L.Yaw_Diff;
		SEND_DATA_GimbleDiff.data.L_x = AutoAimXY_L.x;
		SEND_DATA_GimbleDiff.data.L_y = AutoAimXY_L.y;
		SEND_DATA_GimbleDiff.data.L_z = AutoAimZ_L.L_z;
	}
	else {
		SEND_DATA_GimbleDiff.data.Pitch_Diff_L = 0;
		SEND_DATA_GimbleDiff.data.Yaw_Diff_L = 0;
		SEND_DATA_GimbleDiff.data.L_x = 0;
		SEND_DATA_GimbleDiff.data.L_y = 0;
		SEND_DATA_GimbleDiff.data.L_z = 0;	
	}
	
	if(DeviceStatus.AimAngle_R == Device_Right){
		SEND_DATA_GimbleDiff.data.Pitch_Diff_R = GimbleDiff_R.Pitch_Diff;
		SEND_DATA_GimbleDiff.data.Yaw_Diff_R = GimbleDiff_R.Yaw_Diff;
		SEND_DATA_GimbleDiff.data.R_x = AutoAimXY_R.x;
		SEND_DATA_GimbleDiff.data.R_y = AutoAimXY_R.y;
		SEND_DATA_GimbleDiff.data.R_z = AutoAimZ_R.R_z;
	}
	else{
		SEND_DATA_GimbleDiff.data.Pitch_Diff_R = 0;
		SEND_DATA_GimbleDiff.data.Yaw_Diff_R = 0;
		SEND_DATA_GimbleDiff.data.R_x = 0;
		SEND_DATA_GimbleDiff.data.R_y = 0;
		SEND_DATA_GimbleDiff.data.R_z = 0;	
	}
	
	append_CRC16_check_sum((uint8_t *)&SEND_DATA_GimbleDiff, sizeof(SendDataGimbleDiff_s));
    USB_Transmit((uint8_t *)&SEND_DATA_GimbleDiff, sizeof(SendDataGimbleDiff_s));	
}


/**
 * @brief 
 * @param duration 发送周期
 */
static void UsbSendColorData(void)
{
	SEND_DATA_Color.time_stamp = HAL_GetTick();

    if(judge_sensor.info->RobotStatus.robot_id == 7)
        SEND_DATA_Color.data.color = 1;
    else 
        SEND_DATA_Color.data.color = 0;
	
	append_CRC16_check_sum((uint8_t *)&SEND_DATA_Color, sizeof(SendDataColor_s));
    USB_Transmit((uint8_t *)&SEND_DATA_Color, sizeof(SendDataColor_s));	
}




