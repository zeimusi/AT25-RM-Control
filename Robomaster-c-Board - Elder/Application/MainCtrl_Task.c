#include "MainCtrl_Task.h"

void MainCtrl_Task(){
	static portTickType currentTime;
	for (;;)
	{
		currentTime = xTaskGetTickCount(); // 获取当前系统时间
		if(DeviceState.Remote_State != Device_Online){
//			osThreadSuspend(Task_Chassis_down_handle); // 将任务挂起
			osThreadSuspend(Gimbal_Task_handle);
			osThreadSuspend(Aim_Task_handle);
			osThreadSuspend(Shoot_Task_handle);
			RemoteClear();				   // 遥控数据恢复至默认状态
      for(int i = 0;i<4;i++)Key_ch[i] = 0;
			SystemState = SYSTEM_STARTING; // 系统恢复至重启状态
			GimbalInitFlag = 0;
		} else {
			/* 恢复任务 */
//			osThreadResume(Task_Chassis_down_handle);
			osThreadResume(Shoot_Task_handle);
			osThreadResume(Gimbal_Task_handle);
			osThreadResume(Aim_Task_handle);
	      
		}
//		HAL_IWDG_Refresh(&hiwdg);//硬件看门狗
		WatchDog_Polling(); ///软件看门狗轮询		
		vTaskDelayUntil(&currentTime, 15);
	}	   
}
///* 遥控器模式 */
//void RC_Ctrl()
//{
//    /* 遥控器调试机器人 */
//    switch (RC_CtrlData.rc.s1){
//        case 1: {
//            ChassisAction = CHASSIS_CKECK;
//            AimAction = AIM_STOP;
//            if(ShootAction != SHOOT_STUCKING &&
//               AimAction != AIM_AUTO) 
//                ShootAction = SHOOT_NORMAL;
//            break;
//        }
//        case 3: {
//            ChassisAction = CHASSIS_NORMAL;
//            AimAction =AIM_STOP;
//            if(ShootAction != SHOOT_STUCKING)
//                ShootAction = SHOOT_READY;
//            LidAction = LID_OFF;
//            break;
//        }
//        case 2: {
//            ChassisAction = CHASSIS_FOLLOW;
//            AimAction = AIM_STOP;
//            ShootAction = SHOOT_STOP;
//            LidAction = LID_OFF;
//            break;
//        }
//    }
//    
//    /*
//    ChassisAction：                     ShootAction：                             AimAction：
//    CHASSIS_NORMAL //底盘锁             SHOOT_STOP     //停止发射                 AIM_STOP  //关闭自瞄
//    CHASSIS_SPIN   //小陀螺模式	        SHOOT_READY    //准备发射（摩擦轮启动）   AIM_AID   //自瞄不自动射击
//    CHASSIS_FOLLOW //底盘跟随模式       SHOOT_NORMAL   //单发                     AIM_AUTO  //自瞄+自动射击
//    CHASSIS_RADAR  //雷达导航           SHOOT_RUNNING  //速射(单发超过0.3s变连发)
//    CHASSIS_CKECK  //检录正反转         SHOOT_STUCKING //卡弹退弹中
//    */
//}

/* 3个遥控器数据处理函数 */
void RemoteControlProcess(Remote *rc)
{
   RemoteMode=REMOTE_INPUT;
	
	Key_ch[0] =(float )(rc->ch0 -1024)/660;
	Key_ch[1] =(float )(rc->ch1 -1024)/660;
	Key_ch[2] =(float )(rc->ch2 -1024)/660;
	Key_ch[3] =(float )(rc->ch3 -1024)/660;
	
	deadline_limit(Key_ch[0],0.1f);
	deadline_limit(Key_ch[1],0.1f);
	deadline_limit(Key_ch[2],0.1f);
  deadline_limit(Key_ch[3],0.1f);
}

void STOPControlProcess()
{
	RemoteMode=STOP;
}

void MouseKeyControlProcess(Mouse *mouse, Key_t key, Key_t Lastkey) 
{
	RemoteMode=KEY_MOUSE_INPUT;
	
	limit (mouse ->x,100,-100);
	limit (mouse ->y,100,-100);
	limit (mouse ->z,100,-100);
	
	Mouse_ch[0]=(float)(mouse ->x)/100;
	Mouse_ch[1]=(float)(mouse ->y)/100;
	Mouse_ch[2]=(float)(mouse ->z)/100;
	
	deadline_limit(Mouse_ch[0],0.01f);
	deadline_limit(Mouse_ch[1],0.01f);
	deadline_limit(Mouse_ch[2],0.01f);
}
