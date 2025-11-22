#ifndef _MAIN_YAW_TASK_H_
#define _MAIN_YAW_TASK_H_

#include "Task_Init.h"
    
void Get_Main_Yaw_Exp(void);

void Main_Yaw_PidInit(void);

extern TaskHandle_t Main_Yaw_Task_Handle;
void Main_Yaw_Task(void *pvParameters);

#endif

