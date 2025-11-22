#include "Double_Gimbal_Task.h"

void Double_Guns_Combat(void)
{
    static uint16_t cnt = 0;
    
    if(Control_Mode == Remote_Mode)
        DouAutoAim_Status = NoAim;
    
    /* 二次瞄准模式下视觉不发数据代表pc掉线，开启巡逻模式   如果不是二次瞄准，哪怕视觉不发信息也是自瞄模式 */
    if(Aim_Rx.aim_runing == 0)
    {
        if((DouAutoAim_Status == Aim_1 || DouAutoAim_Status == Aim_2 || DouAutoAim_Status == NoAim) && Control_Mode != Patrol_Mode)
        {
            Control_Mode = Patrol_Mode;

            /* 自瞄模式与遥控巡逻模式来回切换时，陀螺仪反馈值，电机角度反馈值与期望值有偏差，要在这里纠正一下 */
            Exp.Gimbal_Pitch_Exp = Back.Gimbal_Pitch_Back.Angle;
            Exp.Gimbal_Yaw_Exp = Back.Gimbal_Yaw_Back.Angle;
        
            DouAutoAim_Status = NoAim;        
            
            DoubleGimble_Msg.MainYaw_Arrive_flag = Unknown;
        }
    }else if(Aim_Rx.aim_runing == 1 && DouAutoAim_Status == NoAim && Control_Mode != AutoAim_Mode)
    {
        /* 导航模式下，开始第一次瞄准，底盘跟随和小陀螺模式下，直接进入二次瞄准，即大云台不动 */
        if(RC_CtrlData.rc.s1 == 2)
            DouAutoAim_Status = Aim_1;
        else
            DouAutoAim_Status = Aim_2;

        /* 自瞄模式与遥控巡逻模式来回切换时，陀螺仪反馈值，电机角度反馈值与期望值有偏差，要在这里纠正一下 */
        Exp.Gimbal_Pitch_Exp_imu = IMU.EulerAngler.Pitch;
        Exp.Gimbal_Yaw_Exp_imu = IMU.EulerAngler.ContinuousYaw;        

        Control_Mode = AutoAim_Mode;    
    }
    
    
    if(DoubleGimble_Msg.MainYaw_Arrive_flag == NO && DouAutoAim_Status == Aim_1)
    {
        Control_Mode = AutoAim_Mode;
        DouAutoAim_Status = Aim_Gap;
        Exp.Gimbal_Yaw_Exp = Back.Gimbal_Yaw_Back.Angle;
        Exp.Gimbal_Pitch_Exp_imu = IMU.EulerAngler.Pitch;    
    }else if(DoubleGimble_Msg.MainYaw_Arrive_flag == OK && DouAutoAim_Status == Aim_Gap){
        cnt++;
        if((DeviceStatus.PC_State == Device_Error && cnt >= 2000)||(DeviceStatus.PC_State == Device_Right))//4s
        {
            DouAutoAim_Status = Aim_2;
            cnt = 0;
        }
    }
}


