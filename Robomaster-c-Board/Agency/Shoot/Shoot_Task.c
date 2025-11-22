#include "Shoot_Task.h"
void Shoot_SendDown();
/*  TODO:增益补偿
    TODO:裁判系统反馈信息：热量，弹速
    TODO:
*/
/* 射击控制主任务 */
void Shoot_Task(void *pvParameters)
{
    static portTickType currentTime;
		 
    for(;;)
	{
     currentTime = xTaskGetTickCount(); 
	   /* 决定控制方式 */
     ShootCtrl_Decide();
		
		/* 检测发射机构 */
	   Detect_Shoot();
		
		 /* 枪口热量限制 */
     ShootHeat_Limit();
		
     /* 设置目标量 */
     ShootRef_Set();
		
		
     /* 计算控制量 */
     Shoot_Console();
     
     /* 发送控制量 */
     Shoot_Send();
     
     Shoot_SendDown();
     vTaskDelayUntil (&currentTime,1);
    }
}
