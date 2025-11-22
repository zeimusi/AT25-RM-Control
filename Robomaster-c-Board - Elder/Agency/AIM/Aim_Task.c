#include "Aim_Task.h"
void Aim_SendDown();
void Aim_Task(){
     static portTickType currentTime;		 
	   for(;;){
			currentTime = xTaskGetTickCount();
      if(SystemState == SYSTEM_RUNNING) {
				  /* 整车观测 */
	        Aim_Calc();
				  /* 自瞄补偿解算 */
				  Aim_Offset(IMU.Angle_Roll * Pi / 180.0,IMU.Angle_Pitch * Pi / 180.0,IMU.Angle_Yaw * Pi /180.0);
				  /* 自瞄火控 */
				  Aim_Shoot();
				  /* 向底盘发送自瞄数据（画UI） */
				  Aim_SendDown();
			}
			vTaskDelayUntil(&currentTime, 1);		 
	   }
}
    