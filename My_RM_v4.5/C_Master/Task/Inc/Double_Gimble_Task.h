#ifndef __DOUBLE_GIMBLE_TASK_H__
#define __DOUBLE_GIMBLE_TASK_H__

#include "Task_Init.h"
extern float main_yaw_calculation_exp;
extern uint8_t Calculation_flag;//每次收到两头信息计算一次

void Double_Guns_Combat_Angle_Calculate(float *Big_Angle_esp, float *LSmall_Angle_esp, float *RSmall_Angle_esp);

extern TaskHandle_t Double_Gimble_Task_handle;
void Double_Gimble_Task(void *pvParameters);

#endif

