#ifndef _GIMBAL_TASK_H_
#define _GIMBAL_TASK_H_

#include "Task_Init.h"

void Gimbal_PidInit(void);
void Gimbal_Patrol(void);

extern TaskHandle_t Gimbal_Task_Handle;
void Gimbal_Task(void *pvParameters);

#endif
