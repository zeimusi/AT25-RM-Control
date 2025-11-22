#ifndef __Send_Task_H__
#define __Send_Task_H__

#include "Task_Init.h"


extern TaskHandle_t Send_Task_handle;
void Send_Task(void *pvParameters);

extern TaskHandle_t send_handle;
void send(void *pvParameters);
void Judge_Position_Send(void);
void Judge_Radar_Send(void);

#endif

