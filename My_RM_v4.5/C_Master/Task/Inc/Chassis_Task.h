#ifndef _CHASSIS_TASK_H_
#define _CHASSIS_TASK_H_

#include "Task_Init.h"

void Power_Limit(void);

extern TaskHandle_t Chassis_Task_Handle;
void Chassis_Task(void *pvParameters);

#endif

