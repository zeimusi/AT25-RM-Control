#include "Shoot_Task.h"

#define shooter_barrel_heat_limit 400

/* 遥控器射击状态枚举 */
typedef enum{
    Shoot_Stop,
    Shoot_Prepare,
    Shoot_Firing,
    Shoot_Block,
}Shoot_Status_;
Shoot_Status_ Shoot_Status = Shoot_Stop;

/* 射弹期望值 */
#if L_R
#define FRICTION_FIRE_EXP 6200
#define PLUCK_FIRE_EXP 6000
#else
#define FRICTION_FIRE_EXP 6200
#define PLUCK_FIRE_EXP 6000
#endif
void Get_Shoot_Exp(void);
void Get_Shoot_Mode(void);

TaskHandle_t Shoot_Task_Handle;
void Shoot_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        Get_Shoot_Mode();
		Get_Shoot_Exp();
		
		PID_Control(Back.L_Friction_Back.Speed, Exp.L_Friction_Exp, &Pid.L_Friction_PID);
        limit(Pid.L_Friction_PID.pid_out, RM3508_LIMIT, -RM3508_LIMIT);
        Can_Msg_Tx.shoot_msg_tx[0] = Pid.L_Friction_PID.pid_out;
    
		PID_Control(Back.R_Friction_Back.Speed, Exp.R_Friction_Exp, &Pid.R_Friction_PID);
        limit(Pid.R_Friction_PID.pid_out, RM3508_LIMIT, -RM3508_LIMIT);
        Can_Msg_Tx.shoot_msg_tx[1] = Pid.R_Friction_PID.pid_out;
			
		PID_Control(Back.Pluck_Back.Speed, Exp.Pluck_Exp, &Pid.Pluck_PID);
		limit(Pid.Pluck_PID.pid_out, M2006_LIMIT, -M2006_LIMIT);
		Can_Msg_Tx.shoot_msg_tx[2] = Pid.Pluck_PID.pid_out;
		
		MotorSend (&hcan1, 0x200, Can_Msg_Tx.shoot_msg_tx);		
			
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

TaskHandle_t Shoot_Detect_Task_Handle;
void Shoot_Detect_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        static uint16_t Block_Num = 0,Reverse_Num = 0;

        if(Shoot_Status == Shoot_Firing && abs(Back.Pluck_Back.Speed) < 300)
        {
            Block_Num++;
            Reverse_Num = 0;
            if(Block_Num == 100)
            {
                Shoot_Status = Shoot_Block;
                Block_Num = 0;
            }
        }else if(Shoot_Status == Shoot_Block)
        {
            Reverse_Num ++;
            if(Reverse_Num == 100)
            {
                Shoot_Status = Shoot_Firing;
                Reverse_Num = 0;
			}
        }		
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

void Get_Shoot_Exp()
{
    if(Shoot_Status == Shoot_Stop)
    {
		/* 摩擦轮不转，拨弹盘不转 */
		Exp.L_Friction_Exp = 0;
		Exp.R_Friction_Exp = 0;
		
		Exp.Pluck_Exp = 0;
    }
    else if(Shoot_Status == Shoot_Prepare)
    {
		/* 摩擦轮转，拨弹盘不转 */
		Exp.L_Friction_Exp =  FRICTION_FIRE_EXP;
		Exp.R_Friction_Exp = -FRICTION_FIRE_EXP;
        
		Exp.Pluck_Exp = 0;
    }
    else if(Shoot_Status == Shoot_Firing)
    {
		/* 摩擦轮转，拨弹盘正转 */
		Exp.L_Friction_Exp =  FRICTION_FIRE_EXP;
		Exp.R_Friction_Exp = -FRICTION_FIRE_EXP;
        
		Exp.Pluck_Exp = -PLUCK_FIRE_EXP;
    }
    else if(Shoot_Status == Shoot_Block)
    {
		/* 摩擦轮转，左拨弹盘反转 */
		Exp.L_Friction_Exp =  FRICTION_FIRE_EXP;
		Exp.R_Friction_Exp = -FRICTION_FIRE_EXP;
        
		Exp.Pluck_Exp =  PLUCK_FIRE_EXP;
    }
}

void Get_Shoot_Mode(void)
{
    if(Control_Mode == Remote_Mode)
    {
        /* s1 == 3时，如果在射弹准备状态下，遥控器控制云台并且射弹(卡弹状态下，不在进入发射状态，直到时间到后卡弹状态解除)*/        
        if(Shoot_Status == Shoot_Prepare && RC_CtrlData.rc.s1 == 3 && Shoot_Status != Shoot_Block)
            Shoot_Status = Shoot_Firing;
        /* s1 == 2时，遥控器控制射弹准备 */
        else if(RC_CtrlData.rc.s1 == 2)
            Shoot_Status = Shoot_Prepare;        
        /* s1 == 1时，如果在射弹状态下，停止射弹 */
        else if(RC_CtrlData.rc.s1 == 1)
        {
            if(Shoot_Status == Shoot_Prepare || Shoot_Status == Shoot_Firing)
                Shoot_Status = Shoot_Stop;
        }
    }else if(Control_Mode == AutoAim_Mode){
        /* 枪管热量限制那里可以加个阈值 */
        /* 能否开启自瞄(有距离判断 在0.7m - 7m自瞄)) */
#if L_R
        if(_fabsf(IMU.EulerAngler.ContinuousYaw - Aim_Ref.Yaw) <= Aim_Rx.Y_thre && _fabsf(IMU.EulerAngler.Pitch - Aim_Ref.Pitch)<= Aim_Rx.P_thre &&
           Aim_Rx.Predicted_Center_Pose.HorizontalDistance >= 0.7f && Aim_Rx.Predicted_Center_Pose.HorizontalDistance <= 7.5f
           && Judge_Msg.HeatLimit_Heat.shooter_17mm_1_barrel_heat<(shooter_barrel_heat_limit - 30))
        {               
            if(Shoot_Status != Shoot_Block)
                Shoot_Status = Shoot_Firing;
        }else
            Shoot_Status = Shoot_Prepare;
#else
        if(_fabsf(IMU.EulerAngler.ContinuousYaw - Aim_Ref.Yaw) <= Aim_Rx.Y_thre && _fabsf(IMU.EulerAngler.Pitch - Aim_Ref.Pitch)<= Aim_Rx.P_thre &&
           Aim_Rx.Predicted_Center_Pose.HorizontalDistance >= 0.7f && Aim_Rx.Predicted_Center_Pose.HorizontalDistance <= 7.5f
           && Judge_Msg.HeatLimit_Heat.shooter_17mm_2_barrel_heat<(shooter_barrel_heat_limit - 100))
        {               
            if(Shoot_Status != Shoot_Block)
                Shoot_Status = Shoot_Firing;
        }else
            Shoot_Status = Shoot_Prepare;
#endif
    }else{
        Shoot_Status = Shoot_Stop;
    }       
}


