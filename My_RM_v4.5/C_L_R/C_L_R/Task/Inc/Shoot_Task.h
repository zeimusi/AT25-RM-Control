#ifndef _SHOOT_TASK_H_
#define _SHOOT_TASK_H_

#include "Task_Init.h"

extern TaskHandle_t Shoot_Task_Handle;
void Shoot_Task(void *pvParameters);

extern TaskHandle_t Shoot_Detect_Task_Handle;
void Shoot_Detect_Task(void *pvParameters);

#endif


