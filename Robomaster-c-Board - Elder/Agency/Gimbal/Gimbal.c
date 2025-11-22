#include "Gimbal.h"
#include "Time.h"
#include "usb_task.h"
#include "QuaternionEKF.h"
void usb_task();
void INS_Task();
void InitQuaternion(float *init_q4);
//PID_Regulator_t PITCH_SPD_PID;
//PID_Regulator_t PITCH_POS_PID;

eGimbal Gimbal;
/* 云台 */
FeedForward_Typedef FFGimbal[GIMBAL_SUM] = {{.K1 = 20000.0f,.OutMax = 16000.0f},
																						{.K1 = 20000.0f,.OutMax = 16000.0f}
																						};
PID_TypeDef Gimbal_Speed_pid_Yaw[GIMBAL_MODE];
PID_TypeDef Gimbal_Place_pid_Yaw[GIMBAL_MODE];
PID_TypeDef Gimbal_Speed_pid_Pitch[GIMBAL_MODE];
PID_TypeDef Gimbal_Place_pid_Pitch[GIMBAL_MODE];																						
										 

eGimbalPidMode GimbalPidMode;
eGimbalCtrl GimbalCtrl;
/* CAN发送电机数据缓存区 */
int16_t Can2Send_Gimbal[4]={0};
int16_t Can2Send_DM4310[4]={0};

int16_t Can2Send[4]={0};
int16_t Can2SendDM4310[4] = {0};


uint8_t RecodeGimbal = 0; //记录角度

void INS_Init();
/* 云台初始化 */
void GimbalInit(){
		 Gimbal.Mode = Gyro;
	   GimbalInitFlag = 1;
     RecodeGimbal = 0;
		 Time.GimbalInit = 0;
     GimbalPidMode   = INIT;

	   //PID初始化
     PID_init(&Gimbal_Place_pid_Yaw[MECH],25000,0,0,0.25,0,0,0,0,0,0,Integral_Limit);	
	   PID_init(&Gimbal_Speed_pid_Yaw[MECH],25000,0,0,200,0,100,0,0,0,0,Integral_Limit);
     PID_init(&Gimbal_Place_pid_Yaw[GYRO],25000,0.2,0,0.3,0,0,0,0,0,0,Integral_Limit);	
	   PID_init(&Gimbal_Speed_pid_Yaw[GYRO],25000,5000,0,-15000,-20,-4000,0,0,0,0,Integral_Limit);	
     PID_init(&Gimbal_Place_pid_Yaw[AIM],25000,0.2,0,0.35,0,0,0,0,0,0,Integral_Limit);	
	   PID_init(&Gimbal_Speed_pid_Yaw[AIM],25000,5000,0,-10000,-500,-3000,0,0,0,0,Integral_Limit);	
//     PID_init(&Gimbal_Place_pid_Yaw[AIM],25000,1000,0,70,0.2,1800,0,0,0,0,Integral_Limit);	
//	   PID_init(&Gimbal_Speed_pid_Yaw[AIM],25000,0,0,-60,0,0,0,0,0,0,Integral_Limit);	

	   PID_init(&Gimbal_Speed_pid_Pitch[MECH],25000,5000,0,7000,2,4000,0,0,0,0,Integral_Limit&&ChangingIntegralRate);	
	   PID_init(&Gimbal_Place_pid_Pitch[MECH],50000,1000,0,0.8,0,0,0,0,0,0,Integral_Limit&&ChangingIntegralRate);
	   PID_init(&Gimbal_Speed_pid_Pitch[GYRO],25000,0,0,6000,0,4000,0,0,0,0,Integral_Limit&&ChangingIntegralRate);	
	   PID_init(&Gimbal_Place_pid_Pitch[GYRO],50000,1000,0,0.8,0,0,0,0,0,0,Integral_Limit&&ChangingIntegralRate);
//	   PID_init(&Gimbal_Speed_pid_Pitch[GYRO],25000,0,0,80,0,300,0,0,0,0,Integral_Limit&&ChangingIntegralRate);	
//	   PID_init(&Gimbal_Place_pid_Pitch[GYRO],50000,1000,0,60,0.05,0,0,0,0,0,Integral_Limit&&ChangingIntegralRate);
	
//	   PID_init(&Gimbal_Speed_pid_Pitch[GYRO],25000,0,0,16,0,120,0,0,0,0,Integral_Limit);	
//	   PID_init(&Gimbal_Place_pid_Pitch[GYRO],25000,1000,0,120,0,120,0,0,0,0,Integral_Limit);
     PID_init(&Gimbal_Speed_pid_Pitch[AIM],25000,0,0,0,0,0.0,0,0,0,0,Integral_Limit);	
	   PID_init(&Gimbal_Place_pid_Pitch[AIM],25000,0,0,0,0,0,0,0,0,0,Integral_Limit);	

}


float AimLimit1 = 30,AimLimit2 = -30;

/* 决定云台控制模式 */
void GimbalCtrl_Decide(){

		if(DeviceState.Remote_State == Device_Online){
     RemoteMode == REMOTE_INPUT ? Gimbal_RC_Ctrl():
     RemoteMode == KEY_MOUSE_INPUT ? Gimbal_Key_Ctrl() :
		 Gimbal_Stop();
		}else Gimbal_Close();
}
float PitchOffset;
/* 更新期望值 */
void GimbalRef_Update(){

		if(RecodeGimbal == 0){

		Gimbal.Ref[Gyro].Yaw = IMU.Angle_Yawcontinuous;
		Gimbal.Ref[Gyro].Pitch = IMU.Angle_Pitch;
		Gimbal.Ref[Mech].Yaw = Gimbal.Angle[Mech].ContinuousYaw;
		Gimbal.Ref[Mech].Pitch = Gimbal.Angle[Mech].Pitch;
    PitchOffset = 0;
	 if(Gimbal.Mode == gAim){
		Gimbal.Ref[Mech].Pitch = Gimbal.Angle[Mech].Pitch;
	 }else {
		Gimbal.Ref[Gyro].Pitch = IMU.Angle_Pitch;	 
	 }
		RecodeGimbal ++; 
		}
switch(GimbalCtrl){		
		/* 普通模式 （无跟随） */
	case gNormal:
				Opon_aim = 0;
		if(Gimbal.Mode == Gyro){
		if(RemoteMode == REMOTE_INPUT){
			Gimbal.increase[YAW]   = Key_ch[2] * 0.3f;
			Gimbal.increase[PITCH] = Key_ch[3] * 0.1f;
		}else if(RemoteMode == KEY_MOUSE_INPUT){
			Gimbal.increase[YAW]    = Mouse_ch[0] * 0.25;
			Gimbal.increase[PITCH]  = Mouse_ch[1] * 0.2;
		  }
		}
		else if(Gimbal.Mode == Mech){
		if(RemoteMode == REMOTE_INPUT){
			Gimbal.increase[YAW]   = Key_ch[2] * 1.5;
			Gimbal.increase[PITCH] = Key_ch[3] * 0.02f;		
		}else if(RemoteMode == KEY_MOUSE_INPUT){
			Gimbal.increase[YAW]    = Mouse_ch[0] * 8.0;
			Gimbal.increase[PITCH]  = Mouse_ch[1] * 0.1;	
			}
		}
		if(RC_CtrlData.key.E){
    Gimbal.increase[YAW]    *= 0.2;
    Gimbal.increase[PITCH]  *= 0.2;
		}
		Gimbal.Ref[Mech].Yaw += Gimbal.increase[YAW];
		Gimbal.Ref[Mech].Pitch -= Gimbal.increase[PITCH];
		Gimbal.Ref[Gyro].Yaw   -= Gimbal.increase[YAW];

		limit(Gimbal.Ref[Mech].Pitch,P_ADD_limit,P_LOSE_limit);		 
	break;
		
		/* 自瞄模式 */
	case gAim:
				Opon_aim = 1;
	  if(Gimbal.Mode == Gyro  && ReceiveVisionData.data.tracking == 1){
		  PitchOffset -= Mouse_ch[1] * 0.01;
			Gimbal.Ref[Gyro].Yaw   = Aim_Data.Ref_Yaw;
			Gimbal.Ref[Gyro].Pitch = Aim_Data.Ref_Pitch + PitchOffset;   		
//		limit(Gimbal.Ref[Gyro].Pitch,12,-28);	
		Gimbal.increase[YAW]   = 0;
	  Gimbal.increase[PITCH] = 0;    				
		}
		else{
		if(RemoteMode == REMOTE_INPUT){
			Gimbal.increase[YAW]   = Key_ch[2] * 0.1f;
			Gimbal.increase[PITCH] = Key_ch[3] * 0.1f;
		}else if(RemoteMode == KEY_MOUSE_INPUT){
			Gimbal.increase[YAW]    = Mouse_ch[0] * 0.1;
			Gimbal.increase[PITCH]  = Mouse_ch[1] * 0.02;
		  }
		Gimbal.Ref[Gyro].Yaw   -= Gimbal.increase[YAW];
		Gimbal.Ref[Gyro].Pitch -= Gimbal.increase[PITCH];
		Gimbal.Ref[Mech].Pitch -= Gimbal.increase[PITCH];
			
//		limit(Gimbal.Ref[Mech].Pitch,P_ADD_limit,P_LOSE_limit);		 
			
		}
		Gimbal.LastCtrl = gAim;
	break;
		
		/* 跟随模式 （机械模式）*/
	case gFllow:
		Gimbal.increase[YAW]   = 0;
	  Gimbal.increase[PITCH] = Key_ch[3] * 0.1f;
		limit(Gimbal.Ref[Mech].Pitch,P_ADD_limit,P_LOSE_limit);		 
   	
	break;
	
   /* 测试模式 */	
	case gTest:
		Gimbal.increase[YAW]   = 0;
	  Gimbal.increase[PITCH] = 0;
		Gimbal.Ref[Gyro].Yaw  += 0.1;
	  if(Gimbal.Ref[Gyro].Yaw >= 51)
			Gimbal.Ref[Gyro].Yaw = 0;
	break;
		
	default :
	break;
  }

}
/* 更新反馈值 */
void GimbalReal_Update(){
		   Gimbal.Angle[Gyro].Pitch            = IMU.Angle_Pitch;
			 Gimbal.Angle[Gyro].ContinuousYaw		 = IMU.Angle_Yawcontinuous;
			 Gimbal.Speed[Gyro].Pitch            = IMU.Gyro_Pitch;
			 Gimbal.Speed[Gyro].Yaw              = IMU.Gyro_Yaw;
	
			 Gimbal.Angle[Mech].ContinuousYaw    = Gimbal_Motor[YAW].Angle;
			 Gimbal.Angle[Mech].Yaw              = Gimbal_Motor[YAW].MchanicalAngle;
			 Gimbal.Speed[Mech].Yaw              = Gimbal_Motor[YAW].Speed;
			 Gimbal.Angle[Mech].Pitch            = Gimbal_Motor[PITCH].Angle_DEG;
			 Gimbal.Speed[Mech].Pitch            = IMU.Gyro_Pitch;
}

/* 键鼠模式 */
void Gimbal_Key_Ctrl(){
    /*
    F 转头180
    鼠标右键 自瞄 此时鼠标左键 火控
    ctrl 移动速度变慢
    */
    static char Key_Q_flag = 0,Key_F_flag = 0;
    static float Speed_K = 0.5;
    static char mouse_r_flag = 0;
	
    if(GimbalCtrl != gAim){
        /* Yaw轴快速调头180°*/
        if (RC_CtrlData.key.F == 1 && Key_F_flag == 0){
			  Gimbal.Ref[Gyro].Yaw    += 180;
            Key_F_flag = 1;
        }
        if (RC_CtrlData.key.F == 0)
            Key_F_flag = 0;
        
//        // 手动打弹时用机械角度 
//			if(NormalModeFlag != 0 && GimbalCtrl != gAim && GyroscopeModeFlag != 1){
//			   if(Gimbal.Mode != Mech) RecodeGimbal = 0;
//					GimbalCtrl = gNormal;
//					Gimbal.Mode = Mech;
//					GimbalPidMode = MECH;
//			}else {
					GimbalCtrl = gNormal;
					if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
					Gimbal.Mode = Gyro;
					GimbalPidMode = GYRO;
//      }
		}

  			/* 鼠标右键 启动自瞄 */
        if(RC_CtrlData.mouse.press_r == 1 && mouse_r_flag == 0 ){
					    if(GimbalCtrl != gAim || Gimbal.Mode != Gyro) RecodeGimbal = 0;
							Gimbal.Mode = Gyro;
							GimbalPidMode = AIM;
							if(GimbalCtrl != gAim)GimbalCtrl = gAim;
					    else GimbalCtrl = gNormal;
					    mouse_r_flag = 1;
        }
        if (RC_CtrlData.mouse.press_r == 0)
            mouse_r_flag = 0;		


}
/* 遥控模式 */
void Gimbal_RC_Ctrl(){
    /* 遥控器调试机器人 */
    switch (RC_CtrlData.rc.s1){
        case 1:
					/* 自瞄模式 */
//							GimbalCtrl = gAim;
//					    if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
//							GimbalPidMode = AIM;
//							GimbalCtrl = gAim;
  
							/* 普通模式 */
							GimbalCtrl = gNormal;
					    if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
							Gimbal.Mode = Gyro;
							GimbalPidMode = GYRO;




//					    if(GimbalCtrl != gNormal || Gimbal.Mode != Gyro) RecodeGimbal = 0;
//							GimbalCtrl = gNormal;
//							Gimbal.Mode = Gyro;
//							GimbalPidMode = GYRO;
							break;
        case 3:
//					    if(GimbalCtrl != gAim || Gimbal.Mode != Gyro) RecodeGimbal = 0;
//							Gimbal.Mode = Gyro;
//							GimbalPidMode = AIM;
//							GimbalCtrl = gAim;

							
							GimbalCtrl = gNormal;
					    if(Gimbal.Mode != Gyro) RecodeGimbal = 0;
							Gimbal.Mode = Gyro;
							GimbalPidMode = GYRO;


//				      if(Gimbal.Mode != Mech) RecodeGimbal = 0;
//						  GimbalCtrl = gNormal;
//							Gimbal.Mode = Mech;
//							GimbalPidMode = MECH;
							break;
        case 2:
//				      if(Gimbal.Mode != Mech) RecodeGimbal = 0;
//							GimbalCtrl = gNormal;
//							Gimbal.Mode = Mech;
//							GimbalPidMode = MECH;


					    if(GimbalCtrl != gNormal || Gimbal.Mode != Gyro) RecodeGimbal = 0;
							GimbalCtrl = gNormal;
							Gimbal.Mode = Gyro;
							GimbalPidMode = GYRO;
            break;
    }	
}

/* 急停 */
void Gimbal_Stop(){
		 Gimbal.Ref[Gyro].Yaw   = Gimbal.Angle[Gyro].ContinuousYaw;
		 Gimbal.Ref[Gyro].Pitch = Gimbal.Angle[Gyro].Pitch;
		 Gimbal.Ref[Mech].Yaw   = Gimbal.Angle[Mech].ContinuousYaw;
		 Gimbal.Ref[Mech].Pitch = Gimbal.Angle[Mech].Pitch;
		 Can2Send_Gimbal[YAW]   = 0;	
		 Can2Send_Gimbal[PITCH] = 0;
		 Can2Send_DM4310[1]     = 0;

     MotorSend(&hcan2, 0x1FF, Can2Send_Gimbal);
}
/* 云台关闭 */
void Gimbal_Close(){
		 Gimbal.Ref[Gyro].Yaw   = Gimbal.Angle[Gyro].ContinuousYaw;
		 Gimbal.Ref[Gyro].Pitch = Gimbal.Angle[Gyro].Pitch;
		 Gimbal.Ref[Mech].Yaw   = Gimbal.Angle[Mech].ContinuousYaw;
		 Gimbal.Ref[Mech].Pitch = Gimbal.Angle[Mech].Pitch;
		 Can2Send_Gimbal[YAW]   = 0;	
		 Can2Send_Gimbal[PITCH] = 0;
		 Can2Send_DM4310[1]     = 0;
//	   /* pid清零 */
//		PID_init(&Gimbal_Place_pid_Yaw[MECH],25000,0,0,0,0,0,0,0,0,0,Integral_Limit);	
//		PID_init(&Gimbal_Speed_pid_Yaw[MECH],25000,0,0,0,0,0,0,0,0,0,Integral_Limit);
//		PID_init(&Gimbal_Place_pid_Yaw[GYRO],25000,5000,0,0,0,0,0,0,0,0,Integral_Limit);	
//		PID_init(&Gimbal_Speed_pid_Yaw[GYRO],25000,0,0,0,0,0,0,0,0,0,Integral_Limit);	
//		PID_init(&Gimbal_Place_pid_Yaw[AIM],25000,1000,0,0,0,0,0,0,0,0,Integral_Limit);	
//		PID_init(&Gimbal_Speed_pid_Yaw[AIM],25000,0,0,0,0,0,0,0,0,0,Integral_Limit);	

//		PID_init(&Gimbal_Speed_pid_Pitch[MECH],25000,0,0,0,0,0,0,0,0,0,Integral_Limit&&ChangingIntegralRate);	
//		PID_init(&Gimbal_Place_pid_Pitch[MECH],50000,1000,0,0,0,0,0,0,0,0,Integral_Limit&&ChangingIntegralRate);
//		PID_init(&Gimbal_Speed_pid_Pitch[GYRO],25000,0,0,0,0,0,0,0,0,0,Integral_Limit&&ChangingIntegralRate);	
//		PID_init(&Gimbal_Place_pid_Pitch[GYRO],50000,1000,0,0,0,0,0,0,0,0,Integral_Limit&&ChangingIntegralRate);
//		PID_init(&Gimbal_Speed_pid_Pitch[AIM],25000,0,0,0,0,0.0,0,0,0,0,Integral_Limit);	
//		PID_init(&Gimbal_Place_pid_Pitch[AIM],25000,0,0,0,0,0,0,0,0,0,Integral_Limit);	
        MotorSend(&hcan2, 0x1FF, Can2Send_Gimbal);
}

void Detect_Gimbal(){
		 /* 陀螺仪检测 */
		 static uint16_t RefYaw,Yaw,RefPitch,Pitch;
	   RefYaw    = Gimbal.Ref[Gyro].Yaw;
	   Yaw       = Gimbal.Angle[Gyro].ContinuousYaw;
	   RefPitch  = Gimbal.Ref[Gyro].Pitch;
	   Pitch     = Gimbal.Angle[Gyro].Pitch;
	   if(ABS(RefYaw - Yaw) > STD_Angle * 0.45f 
		 || ABS(RefPitch - Pitch) > STD_Angle * 0.45f){
		   Gimbal_Stop();
		 }
}
float aaa = 0;
/* PID计算输出值 */
static float Ramp_PitchRefG ;	
void Gimbal_Pid(){
	   float FeedForwardYawSpeed;//小陀螺时的yaw轴抗扰前馈
		 FeedForwardYawSpeed = Referee_data_Rx.ChassisSpeed /100;

			 if(Gimbal.Mode == Gyro){
	   static float Ramp_YawRefG;	
//     Ramp_YawRefG = RAMP_float(Gimbal.Ref[Gyro].Yaw,Ramp_YawRefG,0.08); 

		 /* YAW轴 */
//		 if(Aim_Data.Flag == 1)
//		 PID_Calc(&Gimbal_Place_pid_Yaw[GimbalPidMode],Gimbal.Angle[Gyro].ContinuousYaw,Ramp_YawRefG);
//		 else 
		 PID_Calc(&Gimbal_Place_pid_Yaw[GimbalPidMode],Gimbal.Angle[Gyro].ContinuousYaw,Gimbal.Ref[Gyro].Yaw );
		 if(GimbalCtrl == gAim )Gimbal_Place_pid_Yaw[AIM].Output = Gimbal_Place_pid_Yaw[AIM].Output + FeedForwardYawSpeed * 0.165;
		 PID_Calc(&Gimbal_Speed_pid_Yaw[GimbalPidMode],Gimbal.Speed[Gyro].Yaw,Gimbal_Place_pid_Yaw[GimbalPidMode].Output );

		 }
	   else if(Gimbal.Mode == Mech){

     /* Yaw轴 */
		 PID_Calc(&Gimbal_Place_pid_Yaw[GimbalPidMode],Gimbal.Angle[Mech].ContinuousYaw,Gimbal.Ref[Mech].Yaw);
		 PID_Calc(&Gimbal_Speed_pid_Yaw[GimbalPidMode],Gimbal.Speed[Mech].Yaw,Gimbal_Place_pid_Yaw[GimbalPidMode].Output);

}
		 /* PITCH轴 */
		 if(GimbalCtrl == gAim && ReceiveVisionData.data.tracking == 1)
		{
		 PID_Calc(&Gimbal_Place_pid_Pitch[GYRO],Gimbal.Angle[Gyro].Pitch,Gimbal.Ref[Gyro].Pitch);
		 PID_Calc(&Gimbal_Speed_pid_Pitch[GYRO],Gimbal.Speed[Gyro].Pitch,Gimbal_Place_pid_Pitch[GYRO].Output); 
		}else {
		
//     Ramp_PitchRefG = RAMP_float(Gimbal.Ref[Mech].Pitch,Ramp_PitchRefG,0.03);
//		 limit(Ramp_PitchRefG,P_ADD_limit,P_LOSE_limit);		 
//		 PID_Calc(&Gimbal_Place_pid_Pitch[GYRO],Gimbal.Angle[Gyro].Pitch,aaa);
//		 PID_Calc(&Gimbal_Speed_pid_Pitch[GYRO],Gimbal.Speed[Gyro].Pitch,Gimbal_Place_pid_Pitch[GYRO].Output); 
		
		 PID_Calc(&Gimbal_Place_pid_Pitch[MECH],Gimbal.Angle[Mech].Pitch,Gimbal.Ref[Mech].Pitch);
		 PID_Calc(&Gimbal_Speed_pid_Pitch[MECH],Gimbal.Speed[Mech].Pitch,Gimbal_Place_pid_Pitch[MECH].Output); 
		}
		 limit(Gimbal_Speed_pid_Pitch[GYRO].Output,GM6020_LIMIT,-GM6020_LIMIT);
		 limit(Gimbal_Speed_pid_Pitch[MECH].Output,GM6020_LIMIT,-GM6020_LIMIT);
     limit(Gimbal_Speed_pid_Yaw[GimbalPidMode].Output, GM6020_LIMIT, -GM6020_LIMIT);
}
int Pitch_Offset = -5500;
/* 发送控制量 */
void Gimbal_Send(){
	
	   if(RC_CtrlData.rc.s2 == 2){
			 Can2Send_Gimbal[YAW] = 0;
			 Can2Send_Gimbal[PITCH] = 0;	 
		 }
		 if(Gimbal.Mode == Gyro)Can2Send_Gimbal[YAW]  = (int16_t)(Gimbal_Speed_pid_Yaw[GimbalPidMode].Output);
       else if (Gimbal.Mode == Mech)Can2Send_Gimbal[YAW]  = (int16_t)(Gimbal_Speed_pid_Yaw[GimbalPidMode].Output);
		 if(GimbalCtrl == gAim && ReceiveVisionData.data.tracking == 1)
			 Can2Send_Gimbal[PITCH] = (int16_t)(Gimbal_Speed_pid_Pitch[GYRO].Output);
     else Can2Send_Gimbal[PITCH] =(int16_t)(Gimbal_Speed_pid_Pitch[MECH].Output) /*+ Pitch_Offset * cos((IMU.Angle_Pitch ) / 180 * Pi)*/;
//Can2Send_Gimbal[PITCH] =(int16_t)(PITCH_SPD_PID.output);
#if GIMBAL_RUN
		 if(RemoteMode != STOP)
     MotorSend(&hcan2, 0x1FF, Can2Send_Gimbal);
#endif
		 
}
/***  归中  ***/
void MedianInit(){
     static float Expect_PitchInit = 0;
     static float Expect_YawInit = 0;
     uint16_t Expect_YawRamp   = Gimbal_Motor[YAW].MchanicalAngle;
     uint16_t Expect_PitchRamp = Gimbal_Motor[PITCH].MchanicalAngle;
    
     /* 获得归中位置(前后就近归中) */
     if (Time.GimbalInit < 100){
//#if   Yaw_Mid_Left < Yaw_Mid_Right
//    if ( (Gimbal_Motor[YAW].MchanicalAngle >= Yaw_Mid_Left) && (Gimbal_Motor[YAW].MchanicalAngle <= Yaw_Mid_Right) )
//#elif Yaw_Mid_Left > Yaw_Mid_Right
//    if ( ( Gimbal_Motor[YAW].MchanicalAngle >= Yaw_Mid_Left) || (Gimbal_Motor[YAW].MchanicalAngle <= Yaw_Mid_Right) )
//#endif
#if   Yaw_Mid_Right < Yaw_Mid_Left
        if ( (Gimbal_Motor[YAW].MchanicalAngle <= Yaw_Mid_Left) && (Gimbal_Motor[YAW].MchanicalAngle >= Yaw_Mid_Right) )
#elif Yaw_Mid_Right > Yaw_Mid_Left
        if ( (Gimbal_Motor[YAW].MchanicalAngle <= Yaw_Mid_Left) || (Gimbal_Motor[YAW].MchanicalAngle >= Yaw_Mid_Right) )
#endif

             MidMode = FRONT;
         else
             MidMode = BACK;
     } else {
         if (MidMode == FRONT)
             Expect_YawInit = QuickCentering( Gimbal_Motor[YAW].MchanicalAngle, Yaw_Mid_Front );
         else
             Expect_YawInit = QuickCentering( Gimbal_Motor[YAW].MchanicalAngle, Yaw_Mid_Back );
         /* Yaw轴归中 */       
         Expect_YawRamp = RAMP_float(Expect_YawInit,Expect_YawRamp,200); 
				 PID_Calc(&Gimbal_Place_pid_Yaw[INIT],Gimbal_Motor[YAW].MchanicalAngle,Expect_YawRamp);
		     PID_Calc(&Gimbal_Speed_pid_Yaw[INIT],Gimbal_Motor[YAW].Speed,Gimbal_Place_pid_Yaw[INIT].Output);
         limit( Gimbal_Speed_pid_Yaw[INIT].Output,GM6020_LIMIT , -GM6020_LIMIT );

         /* Pitch轴归中 */
         Expect_PitchInit = QuickCentering(Gimbal_Motor[PITCH].MchanicalAngle,Pitch_Mid);
         Expect_PitchRamp = RAMP_float(Expect_PitchInit,Expect_PitchRamp,50); 
				 PID_Calc(&Gimbal_Place_pid_Pitch[INIT],Gimbal_Motor[PITCH].MchanicalAngle,Expect_PitchRamp);
		     PID_Calc(&Gimbal_Speed_pid_Pitch[INIT],Gimbal_Motor[PITCH].Speed,Gimbal_Place_pid_Pitch[INIT].Output);
         limit( Gimbal_Speed_pid_Pitch[INIT].Output,GM6020_LIMIT , -GM6020_LIMIT );
				 

				 Can2Send[YAW] = (int16_t)Gimbal_Speed_pid_Yaw[INIT].Output;
         Can2Send[PITCH] = (int16_t)Gimbal_Speed_pid_Pitch[INIT].Output;
			 

 #if GIMBAL_RUN
         MotorSend(&hcan2, 0x1FF, Can2Send);
 #endif
     }		 
				if(Time.GimbalInit >= 1000){
					 /* 记录角度 */
//           Gimbal_Motor[PITCH].r = 0;
					 Time.GimbalInit = 0;
					 GimbalInitFlag  = 0;
					 Gimbal.Ref[Mech].Pitch           = Gimbal_Motor[PITCH].Angle_DEG;	
					 Gimbal.Ref[Mech].Yaw             = Expect_YawInit;
					 Gimbal.Angle[Mech].ContinuousYaw = Gimbal_Motor[YAW].Angle;
					 Gimbal.Angle[Mech].Pitch         = Gimbal_Motor[PITCH].Angle_DEG;
           Ramp_PitchRefG                   = Gimbal_Motor[PITCH].Angle_DEG;
					 Gimbal.YawInit                   = Expect_YawInit;
					 Gimbal.MidMode                   = MidMode;

					 Gimbal.Angle[Gyro].ContinuousYaw = IMU.Angle_Yawcontinuous;
					 Gimbal.Angle[Gyro].Pitch         = IMU.Angle_Pitch;
					 Gimbal.Ref[Gyro].Pitch           = IMU.Angle_Pitch;
					 Gimbal.Ref[Gyro].Yaw             = IMU.Angle_Yawcontinuous;		   
					 Gimbal.increase[PITCH] = 0;
					 Gimbal.increase[YAW]   = 0;


					 SystemState = SYSTEM_RUNNING;
				}
}
/* 向底盘发送云台数据 */
void Gimbal_SendDown(){
     if(DeviceState.Gimbal_State[PITCH] == Device_Online) Gimbal_action.Gimbal_status.Pitch = Gimbal_online;
	   else Gimbal_action.Gimbal_status.Pitch = Gimbal_offline;
     if(DeviceState.Gimbal_State[YAW] == Device_Online)	Gimbal_action.Gimbal_status.Yaw = Gimbal_online;
	   else Gimbal_action.Gimbal_status.Yaw = Gimbal_offline; 
       
}
/* CAN2接收回调函数 */ 
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN2)
  {
    uint16_t CAN2_ID = CAN_Receive_DataFrame(&hcan2, CAN2_buff);
    switch (CAN2_ID)
    {
        case 0x101: memcpy(&Referee_data_Rx, CAN2_buff, sizeof(Referee_data_Rx));
                    Feed_Dog(&Down_Dog);
//										Feed_Dog(&Referee_Dog);
        break;

        case 0x205: GM6020_Receive( &Gimbal_Motor[PITCH], CAN2_buff); 
                    Feed_Dog(&Gimbal_Dog[PITCH]);
                    break;
        
        case 0x206: GM6020_Receive( &Gimbal_Motor[YAW], CAN2_buff); 

                    Feed_Dog(&Gimbal_Dog[YAW]);
			  break;


        default:    break;
    }
  }
}
