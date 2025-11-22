#include "Chassis.h"
#include "arm_math.h"
#include "usb_task.h"
#include "rng.h"
PID_TypeDef Chassis_Place_pid_Rotate;
PID_TypeDef Chassis_Speed_pid_Rotate;
FeedForward_Typedef Chassis_FF = {.K1 = 1000.00, .OutMax = RM3508_LIMIT};        //前馈
uint16_t Mid_Left,Mid_Right,Mid_Back,Mid_Front;

uint8_t RecodeAngle = 0; //记录陀螺仪角度
uint16_t Angle_rotate_ref;
static	Gimbal_board_send_t send_data;
struct{
enum{
ChassisStop = 0,
ChassisFllow = 1,
ChassisNormal = 2,
ChassisGyroscope = 3,
ChassisRadar = 4,
ChassisCheck = 5//检录正反转
}Action;
int16_t MidAngle;
}CHASSIS;
/* 上板发给下板 (底盘速度) */
Communication_Speed_t Communication_Speed_Tx;
/* 底盘初始化 */
void ChassisInit(){
	   Mid_Front = Yaw_Mid_Front;
		 PID_init(&Chassis_Place_pid_Rotate,8000,5,0,-2.5,0,0,0,0,0,0,Integral_Limit);
		 PID_init(&Chassis_Speed_pid_Rotate,8000,5,0,3,0,0,0,0,0,0,Integral_Limit);
     CHASSIS.Action = ChassisNormal;
}
/* 决定底盘控制模式 */
void ChassisCtrl_Decide(){
		if(DeviceState.Remote_State == Device_Online){
     RemoteMode == REMOTE_INPUT ? Chassis_RC_Ctrl():
     RemoteMode == KEY_MOUSE_INPUT ? Chassis_Key_Ctrl() :
		 Chassis_Stop();
		}else Chassis_Close();

}

void Chassis_RC_Ctrl(){
	
		 Communication_Speed_Tx.Close_flag = 0;			
    /* 遥控器调试机器人 */
    switch (RC_CtrlData.rc.s1){
        case 1:
				 CHASSIS.Action = ChassisNormal;
            break;
        case 3:
			   CHASSIS.Action = ChassisGyroscope;
            break;
        case 2:
				 CHASSIS.Action = ChassisFllow;
        break;
    }
}

static char Key_F_flag = 0;
/* 键鼠模式（底盘） */
void Chassis_Key_Ctrl()
{
    /*
    WSAD 前后左右
    R 小陀螺
    F 转头
    */
    static char Key_R_flag = 0,Key_Ctrl_flag = 0;
	  static uint16_t tim;
	  tim ++;
    /* WS前后 */
    if (RC_CtrlData.key.W)
        Key_ch[1] = 1;
    else if (RC_CtrlData.key.S)
        Key_ch[1] = -1;
    else
        Key_ch[1] = 0;
    /* AD左右 */
    if (RC_CtrlData.key.A)
        Key_ch[0] = -1;
    else if (RC_CtrlData.key.D)
        Key_ch[0] = 1;
    else
        Key_ch[0] = 0;
		 Communication_Speed_Tx.Close_flag = 0;			
	  if(NormalModeFlag != 0 && GyroscopeModeFlag != 1){
    CHASSIS.Action = ChassisNormal;
		}else if(CHASSIS.Action != ChassisGyroscope){
    CHASSIS.Action = ChassisNormal;		
		}
    /* R键切换小陀螺 */
    if (RC_CtrlData.key.R == 1 && Key_R_flag == 0){
        if (CHASSIS.Action == ChassisNormal){
            CHASSIS.Action = ChassisGyroscope;
				    GyroscopeModeFlag = 1;
				}
        else{
   					CHASSIS.Action = ChassisNormal;
				    GyroscopeModeFlag = 0;
				}
        Key_R_flag = 1;
    }
    if (RC_CtrlData.key.R == 0)
        Key_R_flag = 0;
    /* 快速转向180 */

    if (RC_CtrlData.key.F == 1 && Key_F_flag == 0){
        Key_F_flag = 1;
			  tim = 0;
    } else if(Key_F_flag == 1 && tim > 500){
        if (RC_CtrlData.key.F == 0 && Key_F_flag == 1){
            Key_F_flag = 0;
        }
    }
		if(RC_CtrlData.key.Ctrl) CHASSIS.Action = ChassisFllow;
//    /* Ctrl键切换地盘跟随 */
//    if (RC_CtrlData.key.Ctrl == 1 && Key_Ctrl_flag == 0){
//        if (CHASSIS.Action == ChassisNormal){
//            CHASSIS.Action = ChassisFllow;
//				}
//        else{
//   					CHASSIS.Action = ChassisNormal;
//				}
//        Key_Ctrl_flag = 1;
//    }
//    if (RC_CtrlData.key.Ctrl == 0)
//        Key_Ctrl_flag = 0;

}
uint16_t R_C,L_C;
/* 更新底盘期望 */
void ChassisRef_Update(){
     static uint16_t Follow_Speed_MAX = 2000;		 
     static uint16_t Speed,tim;		 
	   static uint16_t Ramp_rotate_ref;	

	   
		 switch(CHASSIS.Action){
			 case ChassisFllow:
				 
			/* 底盘跟随（直接判断Yaw轴机械角度判断最近归中位置） */
       Chassis_FF.Now_DeltIn = Key_ch[2] + Mouse_ch[0] * 0.8;


//#if   Yaw_Mid_Left < Yaw_Mid_Right
//    if ( (Gimbal_Motor[YAW].MchanicalAngle >= Yaw_Mid_Left) && (Gimbal_Motor[YAW].MchanicalAngle <= Yaw_Mid_Right) ){
//			#elif Yaw_Mid_Left > Yaw_Mid_Right
//    if ( ( Gimbal_Motor[YAW].MchanicalAngle >= Yaw_Mid_Left) || (Gimbal_Motor[YAW].MchanicalAngle <= Yaw_Mid_Right) ){
//#endif
#if   Yaw_Mid_Right < Yaw_Mid_Left
        if ( (Gimbal_Motor[YAW].MchanicalAngle <= Yaw_Mid_Left) && (Gimbal_Motor[YAW].MchanicalAngle >= Yaw_Mid_Right) ){
#elif Yaw_Mid_Right > Yaw_Mid_Left
        if ( (Gimbal_Motor[YAW].MchanicalAngle <= Yaw_Mid_Left) || (Gimbal_Motor[YAW].MchanicalAngle >= Yaw_Mid_Right) ){
#endif

        CHASSIS.MidAngle = Yaw_Mid_Front; 
        MidMode = FRONT;
    } else {
        CHASSIS.MidAngle = Yaw_Mid_Back;
        MidMode = BACK;
    }
        PID_Calc(&Chassis_Place_pid_Rotate,Gimbal_Motor[YAW].MchanicalAngle,QuickCentering(Gimbal_Motor[YAW].MchanicalAngle, CHASSIS.MidAngle));
			  PID_Calc(&Chassis_Speed_pid_Rotate,Gimbal_Motor[YAW].Speed,Chassis_Place_pid_Rotate.Output);
//        Chassis_Speed_pid_Rotate.Output -= Chassis_Place_pid_Rotate.Output;//前馈
				Communication_Speed_Tx.Chassis_Speed.rotate_ref = Chassis_Speed_pid_Rotate.Output + FeedForward_Calc(&Chassis_FF);
				limit(Communication_Speed_Tx.Chassis_Speed.rotate_ref, Follow_Speed_MAX, -Follow_Speed_MAX);
	      if(Key_F_flag) Communication_Speed_Tx.Chassis_Speed.rotate_ref = 0;
		    break;
				case ChassisNormal:
					
	      Ramp_rotate_ref   = 0;
	      Communication_Speed_Tx.Chassis_Speed.rotate_ref   = RAMP_float(Ramp_rotate_ref,Communication_Speed_Tx.Chassis_Speed.rotate_ref,15) ;
					
				break;	
				
				case ChassisGyroscope:
		    if(Key_ch[0] == 0 && Key_ch[1] == 0 &&RC_CtrlData.key.Shift == 0){
        Ramp_rotate_ref = 4000;
	      Communication_Speed_Tx.Chassis_Speed.rotate_ref   = RAMP_float(Ramp_rotate_ref,Communication_Speed_Tx.Chassis_Speed.rotate_ref,15) ;	
		    }			
				break;	
				
				case ChassisStop:
				Communication_Speed_Tx.Chassis_Speed.rotate_ref       = 0;
				Communication_Speed_Tx.Chassis_Speed.forward_back_ref = 0;		
				Communication_Speed_Tx.Chassis_Speed.left_right_ref   = 0;
				Communication_Speed_Tx.Close_flag = 1;				
				break;
				case ChassisRadar:
				Communication_Speed_Tx.Chassis_Speed.rotate_ref       = 0;
					
				break;
				case ChassisCheck:
					
				break;
				
		 }
}
/* 底盘补偿计算 */
void Chassis_Offset(){
    static float Level_Gain, chassis_offset;
    static int16_t forward_back_ref = 0, left_right_ref = 0,rotate_ref = 0;
    static int16_t Speed_Gain;
	   static float Ramp_forward_back_ref,Ramp_left_right_ref,Ramp_rotate_ref;	

    chassis_offset = (Gimbal_Motor[YAW].MchanicalAngle - Yaw_Mid_Front) / 1303.64f;   // 底盘补偿角
	  Gimbal_data.Offset_Angle = chassis_offset * 1000;
		
		R_C = Yaw_Mid_Right;
	  L_C = Yaw_Mid_Left;
    //Shift加速
    if(RC_CtrlData.key.Shift)
		{
			Speed_Gain = 1400; 
		}
    else
			Speed_Gain = 800; 

	if(CHASSIS.Action != ChassisRadar && CHASSIS.Action != ChassisCheck){
//		 if(MidMode == FRONT){
//		 forward_back_ref = Key_ch[1] * Speed_Gain;
//		 left_right_ref   = -Key_ch[0] * Speed_Gain * 0.7;
//		 }else if(MidMode == BACK){
//		 forward_back_ref = Key_ch[1] * Speed_Gain;
//		 left_right_ref   = -Key_ch[0] * Speed_Gain * 0.7;
//		 }	
#if RobotID == 0
		 if(MidMode == FRONT){
		 forward_back_ref = -Key_ch[1] * Speed_Gain;
		 left_right_ref   = Key_ch[0] * Speed_Gain * 0.7;
		 }else if(MidMode == BACK){
		 forward_back_ref = -Key_ch[1] * Speed_Gain;
		 left_right_ref   = Key_ch[0] * Speed_Gain * 0.7;
		 }	
#elif RobotID == 1
		 if(MidMode == FRONT){
		 forward_back_ref = Key_ch[1] * Speed_Gain;
		 left_right_ref   = -Key_ch[0] * Speed_Gain * 0.7;
		 }else if(MidMode == BACK){
		 forward_back_ref = Key_ch[1] * Speed_Gain;
		 left_right_ref   = -Key_ch[0] * Speed_Gain * 0.7;
		 }	

#endif
	}else{
			forward_back_ref =  -100 * ROBOT_CMD_DATA.data.speed_vector.vx;
		  left_right_ref   = -100 * ROBOT_CMD_DATA.data.speed_vector.vy;
	}
     
//			Ramp_forward_back_ref = RAMP_float(forward_back_ref,Ramp_forward_back_ref,Speed_Gain * 0.5); 
//			Ramp_left_right_ref = RAMP_float(left_right_ref,Ramp_left_right_ref,Speed_Gain * 0.5); 
	

    if(CHASSIS.Action == ChassisCheck){
			left_right_ref = 0;
			Communication_Speed_Tx.Chassis_Speed.rotate_ref   = Key_ch[0] * 3500;
		}
		if(CHASSIS.Action ==ChassisGyroscope &&(Key_ch[0] || Key_ch[1] || RC_CtrlData.key.Shift == 1)){
			rotate_ref = 4000 * 0.6;
			Ramp_rotate_ref = RAMP_float(rotate_ref,Ramp_rotate_ref,Speed_Gain/750.0); 
     Communication_Speed_Tx.Chassis_Speed.rotate_ref = RAMP_float(rotate_ref,Communication_Speed_Tx.Chassis_Speed.rotate_ref,Speed_Gain/750.0); 
		}
			
    /* 底盘补偿(小陀螺模式也能正常移动)Y轴在线才能解算补偿 */
    if(DeviceState.Gimbal_State[YAW] == Device_Online){
        Communication_Speed_Tx.Chassis_Speed.forward_back_ref = forward_back_ref * arm_sin_f32( -chassis_offset)//底盘解算(小陀螺模式也能正常移动)
                                                                + left_right_ref * arm_cos_f32( -chassis_offset);
        Communication_Speed_Tx.Chassis_Speed.left_right_ref   = forward_back_ref * arm_cos_f32( chassis_offset) 
                                                                + left_right_ref * arm_sin_f32( chassis_offset);
//        Communication_Speed_Tx.Chassis_Speed.forward_back_ref = Ramp_forward_back_ref * arm_sin_f32( -chassis_offset)//底盘解算(小陀螺模式也能正常移动)
//                                                                + Ramp_left_right_ref * arm_cos_f32( -chassis_offset);
//        Communication_Speed_Tx.Chassis_Speed.left_right_ref   = Ramp_forward_back_ref * arm_cos_f32( chassis_offset) 
//                                                                + Ramp_left_right_ref * arm_sin_f32( chassis_offset);
    } else {
        Communication_Speed_Tx.Chassis_Speed.forward_back_ref =  forward_back_ref;
        Communication_Speed_Tx.Chassis_Speed.left_right_ref   =  left_right_ref;
    }
}
void Chassis_Stop(){
CHASSIS.Action = ChassisStop;
}
void Chassis_Close()
{
    Communication_Speed_Tx.Chassis_Speed.rotate_ref       = 0;
    Communication_Speed_Tx.Chassis_Speed.forward_back_ref = 0;
    Communication_Speed_Tx.Chassis_Speed.left_right_ref   = 0;
    Communication_Speed_Tx.Close_flag = 1;
//#if CHASSIS_RUN
//    CAN_Send_StdDataFrame(&hcan2, 0x110, (uint8_t *)&Communication_Speed_Tx);
//#endif
    send_data.vx         = Communication_Speed_Tx.Chassis_Speed.forward_back_ref;
	  send_data.vy         = Communication_Speed_Tx.Chassis_Speed.left_right_ref;
	  send_data.rotate     = Communication_Speed_Tx.Chassis_Speed.rotate_ref;
	  send_data.Close_flag = Communication_Speed_Tx.Close_flag;
	  send_data.Shift_flag = 0;
#if CHASSIS_RUN
    CAN_Send_StdDataFrame(&hcan2, 0x110, (uint8_t *)&send_data);
#endif
}
void ChassisDown_Send(){

    send_data.vx         = Communication_Speed_Tx.Chassis_Speed.forward_back_ref;
	  send_data.vy         = Communication_Speed_Tx.Chassis_Speed.left_right_ref;
	  send_data.rotate     = Communication_Speed_Tx.Chassis_Speed.rotate_ref;
	  send_data.Close_flag = Communication_Speed_Tx.Close_flag;
	  send_data.Shift_flag = 0;
   
	  if(CHASSIS.Action == ChassisStop)Gimbal_action.move_status = stop;
	  else if(CHASSIS.Action == ChassisNormal)Gimbal_action.move_status = normal;
	  else if(CHASSIS.Action == ChassisGyroscope)Gimbal_action.move_status = rotate;
	  else if(CHASSIS.Action == ChassisFllow)Gimbal_action.move_status = fllow;

#if CHASSIS_RUN

    CAN_Send_StdDataFrame(&hcan2, 0x110, (uint8_t *)&send_data);	

#endif

}
void ChassisMotorSpeed_clean(ChassisSpeed_Ref_t *ref) {
    ref->forward_back_ref = 0;
    ref->left_right_ref = 0;
    ref->rotate_ref = 0;
}

__weak void ChassisMotorSpeed_get(Chassis_Motor_Speed *motor, ChassisSpeed_Ref_t *ref) {
    motor->speed_3 = -ref->forward_back_ref -
                     ref->left_right_ref + ref->rotate_ref;

    motor->speed_2 = ref->forward_back_ref -
                     ref->left_right_ref + ref->rotate_ref;

    motor->speed_1 = ref->forward_back_ref +
                     ref->left_right_ref + ref->rotate_ref;

    motor->speed_4 = -ref->forward_back_ref +
                     ref->left_right_ref + ref->rotate_ref;
}
