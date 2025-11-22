#include "DMmotor.h"

#include "cmsis_os.h"
#include "string.h"
#include "WatchDog.h"
#include "stdlib.h"

#include "motor.h"
HAL_StatusTypeDef DMMotor_send;
static uint8_t idx = 0;               // 一拖四模式及其类似于大疆电机
/* DM电机的实例,仅用来保存指针,内存的分配通过电机实例初始化时malloc分配内存 */
static DMMotor_Instance *dm_motor_instance[DM_MOTOR_CNT];

//  清除错误帧
uint8_t DM_Motor_CAN_Message_Clear_Error[8] = {0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xff,
                                               0xfb};
// 使能电机, 传统模式有效
uint8_t DM_Motor_CAN_Message_Enter[8] = {0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xff,
                                         0xfc};
// static osThreadId dm_task_handle[DM_MOTOR_CNT]; // MIT模式使用

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x,float x_min, float x_max, uint8_t bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (uint16_t)(x - offset) * ((float)((1 << bits) - 1) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float )x_int * span / ((float )(1 << bits) - 1)) + offset;
}

/**
* DM电机一拖四通用一个协议 :0x3FE, 0x4FE
 * 反馈(rx_id): 0x300+id
 * can1: [0]:0x3FE,[1]:0x4FE
 * can2: [2]:0x3FE,[3]:0x4FE
**/
static CANInstance sender_assignment[4] = {
#if defined(CAN1)
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x3FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x4FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08},
#endif
#if defined(CAN2)
	[2] = {.can_handle = &hcan2, .txconf.StdId = 0x3FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x4FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08},
#endif
};

/**
 * @brief 用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DMMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 *         [0]:0x3FE,[1]:0x4FE,[2]:0x3FE,[3]:0x4FE
**/
static uint8_t sender_enable_flag[4] = {NULL};   // 被填充为1表示此处有电机被注册，为0表示此处没有电机被注册

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping(DMMotor_Instance *motor, CAN_Init_Config_s *_config)
{
    uint8_t motor_id = _config->tx_id - 1; //达妙电机设置ID 
    uint8_t motor_send_num;
    uint8_t motor_grouping;
    
	if (motor_id <= 4) {
		motor_send_num = motor_id;
		motor_grouping = _config->can_handle == &hcan1 ? 0 : 2;  // 0x3FE
	} else {
		motor_send_num = motor_id - 4;
		motor_grouping = _config->can_handle == &hcan1 ? 1 : 3;  // 0x4FE
	}
	_config->rx_id = 0x300 + motor_id + 1;
    sender_enable_flag[motor_grouping] = 1 ;// 设置送标志位,防止发送空帧 可能会重复定义
	
	motor->message_num = motor_send_num;
	motor->sender_group = motor_grouping;
}

#if DMMOTORCTRLMODE == 1
/**
  * @brief 用于改变达妙电机控制模式 MIT可用
**/
static void DMMotorSetMode( DMMotor_Mode_e cmd, DMMotor_Instance *motor)
{
	memset(motor->motor_can_instance->tx_buff, 0xff, 7); // 发送电机指令的时候前面7bytes都是0xff
	motor->motor_can_instance->tx_buff[7] = (uint8_t )cmd; // 最后一位为命令
	CANTransmit(motor->motor_can_instance, 1);
}
/**
 * @brief 修改电机控制编码器0位点
 *
 */
void DMMotorCaliEncoder(DMMotor_Instance *motor)
{
	DMMotorSetMode( DM_CMD_ZERO_POSITION, motor);
	DWT_Delay(0.1);
}
/**
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DMMotorDecode(CANInstance *_instance)
{
	uint8_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
	uint8_t *rxbuff = _instance->rx_buff;
	DMMotor_Instance *motor = (DMMotor_Instance *)_instance->id;
	DMMotor_Measure *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针
	Feed_Dog(motor->watchdog); // 喂狗
	
	measure->last_position = measure->position;
	tmp = (uint16_t )((rxbuff[1] << 8) | rxbuff[2]);
	measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);
	
	tmp = (uint16_t )((rxbuff[3] << 4) | rxbuff[4] >> 4);
	measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);
	
	tmp = (uint16_t )(((rxbuff[4] & 0x0f) << 4) | rxbuff[5]);
	measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);
	
	measure->T_Mos = (float)rxbuff[6];
	measure->T_Rotor = (float)rxbuff[7];
}
#elif DMMOTORCTRLMODE == 2  // 电机一拖四接收ID为 0x301~0x308
static void DMMotorDecode(CANInstance *_instance)
{
    // 数据处理过程
	uint8_t *rxbuff = _instance->rx_buff;
	DMMotor_Instance *motor = (DMMotor_Instance *)_instance->id;
	DMMotor_Measure *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针
	Feed_Dog(motor->watchdog); // 喂狗
	
	measure->MchanicalAngle = (uint16_t )((rxbuff[0] << 8) | rxbuff[1]);
	measure->Speed = (int16_t )((rxbuff[2] << 8) | rxbuff[3]) / 100.0f;  // 真实速度
	measure->TorqueCurrent = (int16_t )((rxbuff[4] << 8) | rxbuff[5]) / 1000.0f;
    measure->Mos_temperature =  (int16_t )(rxbuff[6]);
    measure->ERROR_Handle =  (int16_t )(rxbuff[7]);
	
    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°
    int16_t diff = measure->MchanicalAngle - measure->LastMchanicalAngle;
	if(diff != measure->MchanicalAngle) {
		if (diff > 4096)
			measure->round --;
		else if (diff < -4096)
		  measure->round ++;
    }
	measure->Angle = measure->MchanicalAngle + measure->round * 8192;
	measure->Angle_DEG =( measure->Angle - measure->MotorCenter ) * 0.0356f;
	/* 一阶低通滤波器 */
	measure->SpeedFilter =  (1.0f - DMMOTOR_SPEED_SMOOTH_COEF) * measure->SpeedFilter +
                           DMMOTOR_SPEED_SMOOTH_COEF * (float)(measure->Speed);
    measure->CurrentFilter = (1.0f - DMMOTOR_CURRENT_SMOOTH_COEF) * measure->CurrentFilter +
                            DMMOTOR_CURRENT_SMOOTH_COEF * (float)(measure->TorqueCurrent);	
    
	measure->LastAngle = measure->Angle;
	measure->LastMchanicalAngle = measure->MchanicalAngle ;
}
#endif

void DMMotorChangeFeed(DMMotor_Instance *motor, Feedback_Source_e type)
{
	motor->motor_controller.motor_setting.angle_feedback_source  = type;
	motor->motor_controller.motor_setting.speed_feedback_source  = type;
}

/* 设置期望值 */
void DMMotorSetRef(DMMotor_Instance *motor, float ref)
{
	motor->motor_controller.motor_setting.close_loop_type >= ANGLE_LOOP ? (motor->motor_controller.ref_position = ref )
		: (motor->motor_controller.ref_speed = ref );
}

/* 使能电机 */
void DMMotorEnable(DMMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 停止电机 */
void DMMotorStop(void *id)//不使用使能模式是因为需要收到反馈
{
	DMMotor_Instance *motor = (DMMotor_Instance* )id;
    motor->stop_flag = MOTOR_STOP;
}

/* 修改电机的实际闭环对象 */
void DMMotorOuterLoop(DMMotor_Instance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_controller.motor_setting.close_loop_type	= outer_loop;
}

/* 作为看门狗回调 */
uint8_t DMMotorErroText(void *id)
{
	DMMotor_Instance *motor = (DMMotor_Instance* )id;
	if (motor->measure.ERROR_Handle != 1 || motor->measure.ERROR_Handle != 0) {
          motor->stop_flag = MOTOR_STOP;
	}
	return 1;
}
/**
 * @brief 电机初始化 返回一个电机实例
 *
**/
DMMotor_Instance *DMMotorInit(Motor_Init_config_s *_config)
{
	DMMotor_Instance *instance = (DMMotor_Instance* )RMLIB_MALLOC(sizeof(DMMotor_Instance));
	memset(instance, 0, sizeof(DMMotor_Instance));
	
	instance->other_angle_feedback_ptr = _config->other_angle_feedback_ptr;
	instance->other_speed_feedback_ptr = _config->other_speed_feedback_ptr;
    
    MotorSenderGrouping(instance, &_config->can_init_config);
    _config->can_init_config.can_module_callback = DMMotorDecode;
	_config->can_init_config.id                  = instance;
	instance->motor_can_instance = CANRegister(_config->can_init_config);
	
    instance->motor_controller = *create_controller(&_config->contruller_config);

	_config->dog_init_config.watch_callback = DMMotorStop;
//	_config->dog_init_config.feed_callback  = DMMotorErroText;
	_config->dog_init_config.Max_num        = 5;
	_config->dog_init_config.owner_id       = instance;
	instance->watchdog = WatchDog_Init(_config->dog_init_config); // 注册看门狗
	instance->measure.MotorCenter = _config->contruller_config.MotorCenter;
	
#if DMMOTORCTRLMODE == 1
    DMMotorSetMode(DM_CMD_MOTOR_MODE, instance);
    DWT_Delay(0.1);
    DMMotorCaliEncoder(instance);
    DWT_Delay(0.1);
#endif
    DMMotorEnable(instance);
	dm_motor_instance[idx++] = instance;
    
//	CAN_Send_StdDataFrame(&hcan2, 0x301, DM_Motor_CAN_Message_Clear_Error);
//	CAN_Send_StdDataFrame(&hcan2, 0x301, DM_Motor_CAN_Message_Enter);
	return instance;
}

#if DMMOTORCTRLMODE == 2
     /* 电机计算 */
int16_t can_set[4];                       // 电机控制CAN发送设定值    

void DMMmotor_Update()
{
	// 直接保存一次指针引用从而减小访存的开销,同时提高可读性
	uint8_t group, num;                     // 电机组号和组内编号
	uint16_t can_set;                       // 电机控制CAN发送设定值    
	DMMotor_Instance *motor;
	Controller_s *controller_motor;
	DMMotor_Measure *measure;
	
    // 遍历所有电机实例,并进行串级PID计算
 	for( size_t i = 0; i < idx ; i++ )
	{
		motor = dm_motor_instance[i];
		if (motor->stop_flag == MOTOR_CLOSE) {
			sender_enable_flag[motor->sender_group] = 0; // 停止发送 仅有四个电机同时 CLOSE时才生效比如底盘 
//			sender_assignment[dm_motor_instance[i]->sender_group].tx_buff[2 * dm_motor_instance[i]->message_num]   = 0;
//			sender_assignment[dm_motor_instance[i]->sender_group].tx_buff[2 * dm_motor_instance[i]->message_num+1] = 0;
			continue;
		} else sender_enable_flag[motor->sender_group] = 1;
		
		controller_motor = &dm_motor_instance[i]->motor_controller;
		measure = &dm_motor_instance[i]->measure;
		group = dm_motor_instance[i]->sender_group;
		num   = dm_motor_instance[i]->message_num;
		// 若电机处于停止状态
       if (motor->stop_flag == MOTOR_ENALBED)
	   {
			if (controller_motor->motor_setting.motor_reverse_flag == MOTOR_REVERSE)
					controller_motor->motor_setting.close_loop_type >= ANGLE_LOOP ? (controller_motor->ref_position *= -1 )
						: (controller_motor->ref_speed *= -1 );
			
			if (controller_motor->motor_setting.angle_feedback_source == MOTOR_FEED)
			    controller_motor->fdb_position = measure->Angle_DEG;
			else controller_motor->fdb_position = *motor->other_angle_feedback_ptr;
			
			if (controller_motor->motor_setting.speed_feedback_source == MOTOR_FEED)
				controller_motor->fdb_speed = measure->SpeedFilter;
			else controller_motor->fdb_speed = *motor->other_speed_feedback_ptr;
			controller_motor->fdb_current = measure->CurrentFilter;
			
			controller_clc(controller_motor);
			
			if (controller_motor->motor_setting.feedback_reverse_flag == FEEDBACK_REVERSE)
                 controller_motor->output *= -1;
			// 获得输出
			can_set = (int16_t)controller_motor->output;
			// 分组填入
//			sender_assignment[group].tx_buff[2 * num]   = (uint8_t)(can_set >> 8);
//			sender_assignment[group].tx_buff[2 * num+1] = (uint8_t)(can_set & 0xff);
            
		} else {
//			sender_assignment[group].tx_buff[2 * num]   = 0;
//			sender_assignment[group].tx_buff[2 * num+1] = 0;
		}
	}
	
//	for (size_t j = 0; j < 4; j++)
//	{
//		if (sender_enable_flag[j])
//              CANTransmit(&sender_assignment[j], 0.5);
//	}
}


#endif
