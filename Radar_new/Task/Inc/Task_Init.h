#ifndef _TASK_INIT_H_
#define _TASK_INIT_H_

#include "stm32f4xx_hal.h"
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
#include "WatchDog.h"
#include "CRC.h"
#include "RMQueue.h"

#include "Attribute_Typedef.h"
#include "judge.h"

#include "WatchDog_Task.h"
#include "Receive_Task.h"
#include "Send_Task.h"
#include "Usb_Task.h"

#define USART6_LEN 50
extern uint8_t usart6_dma_buf[USART6_LEN];
extern RMQueue_Handle judge_queue;


void Start_Task(void);

#endif

