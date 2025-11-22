#include "Gimbal_Task.h"

void Gimbal_SendDown();


void Gimbal_Task(){
     static portTickType currentTime;
     PID_init(&Gimbal_Place_pid_Yaw[INIT],25000,0,0,5,0,100,0,0,0,0,Integral_Limit);	
	   PID_init(&Gimbal_Speed_pid_Yaw[INIT],25000,0,0,10,0,0,0,0,0,0,Integral_Limit);
	   PID_init(&Gimbal_Speed_pid_Pitch[INIT],25000,0,0,2,0,0,0,0,0,0,Integral_Limit);	
	   PID_init(&Gimbal_Place_pid_Pitch[INIT],50000,100,0,0.2,0,0,0,0,0,0,Integral_Limit);

	   for(;;){
        currentTime = xTaskGetTickCount();			 
			   if(SystemState != SYSTEM_RUNNING){
					 if(GimbalInitFlag == 0) GimbalInit();
            MedianInit();
#if !GIMBAL_RUN
            SystemState = SYSTEM_RUNNING;
#endif				
				 }else{					  
					  GimbalCtrl_Decide();
					  
					  GimbalRef_Update();
					 
					  GimbalReal_Update();
					 
					  Gimbal_Pid();
					 
					  Gimbal_Send();

					  Gimbal_SendDown();
					 
/* pidµ÷ÊÔ´®¿ÚÖúÊÖ */
//		 		Serialport[0] = IMU.Angle_Yawcontinuous;
//				Serialport[1] = Gimbal.Ref[Gyro].Yaw ;

//		 		Serialport[0] = IMU.Angle_Pitch ;
//		    Serialport[1] = Gimbal.Ref[Mech].Pitch;
				 }
        vTaskDelayUntil(&currentTime, 1);		 
		 }
}

