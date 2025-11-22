#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#include "stm32f4xx_hal.h"
#include "can.h"
#include "gpio.h"
#include "stm32f4xx_it.h"
#include "arm_math.h"
#include "tim.h"
#include "usart.h"
#include "dma.h"
#include "stdbool.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "stack_macros.h"

#include "RMLibHead.h"
#include "CANDrive.h"
#include "ramp.h"
#include "Chassis.h"
#include "remote.h"
#include "motor.h"
#include "WatchDog.h"
#include "CRC.h"
#include "RMQueue.h"
#include "PID.h"

#include "Attribute_Typedef.h"
#include "judge.h"
#include "SuperCap.h"

#include "WatchDog_Task.h"
#include "Receive_Task.h"
#include "Send_Task.h"
#include "Usb_Task.h"
#include "Main_Yaw_Task.h"
#include "Chassis_Task.h"
#include "Double_Gimble_Task.h"

#define single_double 1 //单头作战为1，双头作战为0

#define Main_Yaw_Median 1977

#define USART3_LEN 16
extern uint8_t usart3_dma_buf[USART3_LEN];
#define USART6_LEN 50
extern uint8_t usart6_dma_buf[USART6_LEN];
extern RMQueue_Handle judge_queue;

/* 斜坡 */
typedef struct{
    Ramp_Typedef Main_Yaw_Ramp;
    int16_t Main_Yaw_Diff;
}Ramp_;
extern Ramp_ Ramp;

/* 反馈值 */
typedef struct{
	DM4310_TypeDef Main_Yaw_Back;
	
    RM3508_TypeDef Classis1_Back;
    RM3508_TypeDef Classis2_Back;
    RM3508_TypeDef Classis3_Back;
    RM3508_TypeDef Classis4_Back;
}Back_t;
extern Back_t Back;

/* 期望值 */
typedef struct{
    float Main_Yaw_Exp; 
    ChassisSpeed_Ref_t  Chassis_Speed;
    Chassis_Motor_Speed ChassisMotor_Speed;
}Exp_t;
extern Exp_t Exp;

/* PID */
typedef struct{
	PID_Smis Main_Yaw_Place_PID;
	PID Main_Yaw_Speed_PID;
	PID Classis1_PID;
	PID Classis2_PID;
	PID Classis3_PID;
	PID Classis4_PID;
}Pid_t;
extern Pid_t Pid;


void Start_Task(void);

#endif

