#ifndef __INIT_TASK_H
#define __INIT_TASK_H

#include "Variate.h"

TaskHandle_t INS_Task_handle;
TaskHandle_t Aim_Task_handle;
TaskHandle_t Shoot_Task_handle;
TaskHandle_t Gimbal_Task_handle;
TaskHandle_t MainCtrl_Task_handle;
TaskHandle_t Serialport_Task_handle;
void MainCtrl_Task();
void INS_Task();
void Aim_Task();
void Shoot_Task();
void Gimbal_Task();
void Serialport_Task();
#endif