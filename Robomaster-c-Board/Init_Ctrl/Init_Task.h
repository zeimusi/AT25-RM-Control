#ifndef __INIT_TASK_H
#define __INIT_TASK_H

#include "Variate.h"

TaskHandle_t Ins_Task_handle;
TaskHandle_t Aim_Task_handle;
TaskHandle_t Shoot_Task_handle;
TaskHandle_t Gimbal_Task_handle;
TaskHandle_t Chassis_Task_handle;
TaskHandle_t MainCtrl_Task_handle;
TaskHandle_t Serialport_Task_handle;
TaskHandle_t Music_Task_handle;
TaskHandle_t usb_task_handle;
void MainCtrl_Task();
void INS_Task();
void Music_Task();
void Aim_Task();
void Shoot_Task();
void Gimbal_Task();
void Chassis_Task();
void Serialport_Task();
void usb_task();
#endif