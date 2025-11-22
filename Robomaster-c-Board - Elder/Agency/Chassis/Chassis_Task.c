#include "Chassis_Task.h"

void Chassis_Task(){
     static portTickType currentTime;
		  /* 底盘初始化 */
			ChassisInit();
	   for(;;){
			currentTime = xTaskGetTickCount();

		  
			/* 决定底盘控制模式 */
		  ChassisCtrl_Decide();
			
			/* 更新底盘期望值 */
			ChassisRef_Update();

			/* 底盘补偿 */
      Chassis_Offset();
			 
			/* 发送控制值给下板 */
			ChassisDown_Send();
			vTaskDelayUntil(&currentTime, 1);		 
	   }
}