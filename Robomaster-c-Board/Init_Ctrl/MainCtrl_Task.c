#include "MainCtrl_Task.h"

void MainCtrl_Task(){
	static portTickType currentTime;
	for (;;)
	{
    static uint8_t cnt=0;
		currentTime = xTaskGetTickCount(); // 获取当前系统时间
		if(DeviceState.Remote_State != Device_Online){
			osThreadSuspend(Chassis_Task_handle); // 将任务挂起
			osThreadSuspend(Gimbal_Task_handle);
//			osThreadSuspend(Aim_Task_handle);
			osThreadSuspend(Shoot_Task_handle);
			RemoteClear();				   // 遥控数据恢复至默认状态
      for(int i = 0;i<4;i++)Key_ch[i] = 0;
			SystemState = SYSTEM_STARTING; // 系统恢复至重启状态
			GimbalInitFlag = 0;
//      __set_FAULTMASK(1);//禁止所有的可屏蔽中断
//      NVIC_SystemReset();//软件复位

//			if(RC_CtrlData.key.G){
//      __set_FAULTMASK(1);//禁止所有的可屏蔽中断
//      NVIC_SystemReset();//软件复位
//			}
		} else {
			/* 恢复任务 */
			osThreadResume(Chassis_Task_handle);
			osThreadResume(Shoot_Task_handle);
			osThreadResume(Gimbal_Task_handle);
//			osThreadResume(Aim_Task_handle);
	      
		}
		if(RC_CtrlData.key.Z){
		Gimbal_action.Key = 1;//Z键--Key=1刷新UI Ctrl&&Shift--Key=2解除功率限制
		
		}
		else if(RC_CtrlData.key.Ctrl){
		Gimbal_action.Key = 2;//Z键--Key=1刷新UI Ctrl&&Shift--Key=2解除功率限制	
		}else{
		Gimbal_action.Key = 0;//Z键--Key=1刷新UI Ctrl&&Shift--Key=2解除功率限制		
		}			
	    switch(cnt++){
        case 0:CAN_Send_StdDataFrame(&hcan2, 0x120, (uint8_t *)&Gimbal_action);break;
        case 1:CAN_Send_StdDataFrame(&hcan2, 0x130, (uint8_t *)&Gimbal_data);cnt = 0;break;
    }
//		HAL_IWDG_Refresh(&hiwdg);//硬件看门狗
		WatchDog_Polling(); ///软件看门狗轮询		

		vTaskDelayUntil(&currentTime, 15);
	}	   
}
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
	
	limit (mouse ->x,200,-200);
	limit (mouse ->y,200,-200);
	limit (mouse ->z,200,-200);
	
	Mouse_ch[0]=(float)(mouse ->x)/200;
	Mouse_ch[1]=(float)(mouse ->y)/200;
	Mouse_ch[2]=(float)(mouse ->z)/200;
	
	deadline_limit(Mouse_ch[0],0.01f);
	deadline_limit(Mouse_ch[1],0.01f);
	deadline_limit(Mouse_ch[2],0.01f);
}
