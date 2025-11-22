#include "Main_Yaw_Task.h"
/**
 * @note    4310电机控位置时输入电压要小于20（好像比这个大也可以，还没测出来），当pidout大于20时，不仅无法到达目标位置，还会反向走一点，之后就不动了，pidout到1000都这样
 * @note    用力掰，会失能
 */

void Get_Main_Yaw_Exp(void);

TaskHandle_t Main_Yaw_Task_Handle;
void Main_Yaw_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;)
    {
        
        Get_Main_Yaw_Exp();
        
        if((Control_Mode == Classis_Follow_Mode && Slope(&Ramp.Main_Yaw_Ramp) == 1)){
            PID_Control_Smis(Back.Main_Yaw_Back.Angle_DEG, Exp.Main_Yaw_Exp, &Pid.Main_Yaw_Place_PID, IMU.AngularVelocity.z);
            PID_Control(IMU.AngularVelocity.z, Pid.Main_Yaw_Place_PID.pid_out, &Pid.Main_Yaw_Speed_PID);
        }else if(Control_Mode == Rotate_Mode || Control_Mode == Radar_Mode || Control_Mode == Remote_Mode){
            PID_Control_Smis(IMU.EulerAngler.ContinuousYaw, Exp.Main_Yaw_Exp, &Pid.Main_Yaw_Place_PID, IMU.AngularVelocity.z);
            PID_Control(IMU.AngularVelocity.z, Pid.Main_Yaw_Place_PID.pid_out, &Pid.Main_Yaw_Speed_PID);
        }
        
        limit(Pid.Main_Yaw_Speed_PID.pid_out, DM4310_LIMIT, -DM4310_LIMIT);
        Can_Msg_Tx.yaw_msg_tx[0] = Pid.Main_Yaw_Speed_PID.pid_out;
        
        /* 将pid.out发送 */
        if(Control_Mode != Stop_Mode)
        {
            MotorSend (&hcan2, 0x3FE, Can_Msg_Tx.yaw_msg_tx);  
        }
        
        vTaskDelayUntil(&xLastWakeTime,1);
    }
}

void Go_To_Middle(void);
void Get_Main_Yaw_Exp()
{   
    switch(Control_Mode)
    {
        case Remote_Mode:
            if(RC_CtrlData.rc.s1 == 1)
                Exp.Main_Yaw_Exp += 0.02f * (RC_CtrlData.rc.ch3 - 1024)*360/8191;
            break;
        case Classis_Follow_Mode:
            /* 先归中 */
            if(Slope(&Ramp.Main_Yaw_Ramp) != 1)
            {
                Go_To_Middle();
            }else{
                Exp.Main_Yaw_Exp = (Main_Yaw_Median + Back.Main_Yaw_Back.r*8192)*360/8191;
            }   
            break;
        case Rotate_Mode:
            Exp.Main_Yaw_Exp -= 0.02f * (RC_CtrlData.rc.ch2 - 1024)*360/8191;
            break;
        case Radar_Mode:
            Exp.Main_Yaw_Exp = fRamp_Stepf(main_yaw_calculation_exp, Exp.Main_Yaw_Exp, 5);
            break;
        default:break;
    }
}

void Main_Yaw_PidInit()
{
    if(Control_Mode != Stop_Mode)
        Pid_Init(&Pid.Main_Yaw_Place_PID, &Pid.Main_Yaw_Speed_PID, 5,0.1,-60,20,1,0,0,0);//30,0,0,20,1,0,0,1);
    else
        Pid_Init(&Pid.Main_Yaw_Place_PID, &Pid.Main_Yaw_Speed_PID, 0,0,0,0,0,0,0,0);
}

