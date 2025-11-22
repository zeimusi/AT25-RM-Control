#include "Gimbal_Task.h"

void Get_Gimbal_Exp(void);

TaskHandle_t Gimbal_Task_Handle;
void Gimbal_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
        Get_Gimbal_Exp();
        
        if(Control_Mode == Remote_Mode || Control_Mode == Patrol_Mode)  
        {
            PID_Control_Smis(Back.Gimbal_Pitch_Back.Angle, Exp.Gimbal_Pitch_Exp, &Pid.Gimbal_Pitch_Place_PID, Back.Gimbal_Pitch_Back.Speed);
            PID_Control(Back.Gimbal_Pitch_Back.Speed, Pid.Gimbal_Pitch_Place_PID.pid_out, &Pid.Gimbal_Pitch_Speed_PID);
            limit(Pid.Gimbal_Pitch_Speed_PID.pid_out, GM6020_LIMIT, -GM6020_LIMIT);
            
            PID_Control_Smis(Back.Gimbal_Yaw_Back.Angle, Exp.Gimbal_Yaw_Exp, &Pid.Gimbal_Yaw_Place_PID, Back.Gimbal_Yaw_Back.Speed);
            PID_Control(Back.Gimbal_Yaw_Back.Speed, Pid.Gimbal_Yaw_Place_PID.pid_out, &Pid.Gimbal_Yaw_Speed_PID);
            limit(Pid.Gimbal_Yaw_Speed_PID.pid_out, GM6020_LIMIT, -GM6020_LIMIT);

#if L_R
            Can_Msg_Tx.gimble_msg_tx[0] = Pid.Gimbal_Pitch_Speed_PID.pid_out;
            Can_Msg_Tx.gimble_msg_tx[1] = Pid.Gimbal_Yaw_Speed_PID.pid_out;
#else
            Can_Msg_Tx.gimble_msg_tx[2] = Pid.Gimbal_Pitch_Speed_PID.pid_out;
            Can_Msg_Tx.gimble_msg_tx[3] = Pid.Gimbal_Yaw_Speed_PID.pid_out;
#endif
        }else if(Control_Mode == AutoAim_Mode)
        {
#if single_double
            PID_Control_Smis(IMU.EulerAngler.ContinuousYaw, Exp.Gimbal_Yaw_Exp_imu, &Pid.Gimbal_Yaw_Place_PID_imu, IMU.AngularVelocity.z);
            PID_Control(IMU.AngularVelocity.z, Pid.Gimbal_Yaw_Place_PID_imu.pid_out, &Pid.Gimbal_Yaw_Speed_PID_imu);            
            limit(Pid.Gimbal_Yaw_Speed_PID_imu.pid_out, GM6020_LIMIT, -GM6020_LIMIT);
#else 
            if(DouAutoAim_Status == Aim_Gap)
            {
                PID_Control_Smis(Back.Gimbal_Yaw_Back.Angle*360/8191, Exp.Gimbal_Yaw_Exp*360/8191, &Pid.Gimbal_Yaw_Place_PID_imu, IMU.AngularVelocity.z);
                PID_Control(IMU.AngularVelocity.z, Pid.Gimbal_Yaw_Place_PID_imu.pid_out, &Pid.Gimbal_Yaw_Speed_PID_imu);
            }else{
                PID_Control_Smis(IMU.EulerAngler.ContinuousYaw, Exp.Gimbal_Yaw_Exp_imu, &Pid.Gimbal_Yaw_Place_PID_imu, IMU.AngularVelocity.z);
                PID_Control(IMU.AngularVelocity.z, Pid.Gimbal_Yaw_Place_PID_imu.pid_out, &Pid.Gimbal_Yaw_Speed_PID_imu);            
            }
            limit(Pid.Gimbal_Yaw_Speed_PID_imu.pid_out, GM6020_LIMIT, -GM6020_LIMIT);
#endif             
            PID_Control_Smis(IMU.EulerAngler.Pitch, Exp.Gimbal_Pitch_Exp_imu, &Pid.Gimbal_Pitch_Place_PID_imu, IMU.AngularVelocity.x);
            PID_Control(IMU.AngularVelocity.x, Pid.Gimbal_Pitch_Place_PID_imu.pid_out, &Pid.Gimbal_Pitch_Speed_PID_imu);
            limit(Pid.Gimbal_Pitch_Speed_PID_imu.pid_out, GM6020_LIMIT, -GM6020_LIMIT);
            
#if L_R
            Can_Msg_Tx.gimble_msg_tx[0] = Pid.Gimbal_Pitch_Speed_PID_imu.pid_out;
            Can_Msg_Tx.gimble_msg_tx[1] = Pid.Gimbal_Yaw_Speed_PID_imu.pid_out;
#else
            Can_Msg_Tx.gimble_msg_tx[2] = Pid.Gimbal_Pitch_Speed_PID_imu.pid_out;
            Can_Msg_Tx.gimble_msg_tx[3] = Pid.Gimbal_Yaw_Speed_PID_imu.pid_out;
#endif
        }else if(Control_Mode == Stop_Mode)
            memset(&Can_Msg_Tx.gimble_msg_tx, 0, sizeof(Can_Msg_Tx.gimble_msg_tx));
            
        MotorSend(&hcan1, 0x1FF, Can_Msg_Tx.gimble_msg_tx);
        
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

void Get_Gimbal_Exp()
{
    switch(Control_Mode)
    {
        case Remote_Mode:
            if(RC_CtrlData.rc.s1 != 1)
            {
#if L_R                
                /* YAW轴位置处理 */
                Exp.Gimbal_Pitch_Exp += -(RC_CtrlData.rc.ch3 - 1024)*0.01;	 
                limit(Exp.Gimbal_Pitch_Exp,Pitch_Limit_Max, Pitch_Limit_Min);
                /* Pitch轴位置处理 */
                Exp.Gimbal_Yaw_Exp += -(RC_CtrlData.rc.ch2 - 1024)*0.01;
                limit(Exp.Gimbal_Yaw_Exp, Yaw_Limit_Max,Yaw_Limit_Min);
#else
                /* YAW轴位置处理 */
                Exp.Gimbal_Pitch_Exp += (RC_CtrlData.rc.ch1 - 1024)*0.01;	 
                limit(Exp.Gimbal_Pitch_Exp,Pitch_Limit_Max, Pitch_Limit_Min);
                /* Pitch轴位置处理 */
                Exp.Gimbal_Yaw_Exp += -(RC_CtrlData.rc.ch0 - 1024)*0.01;
                limit(Exp.Gimbal_Yaw_Exp, Yaw_Limit_Max,Yaw_Limit_Min);
#endif                
            }
            break;
        case Patrol_Mode:
            Gimbal_Patrol();
            break;
        case AutoAim_Mode:
#if single_double
            Exp.Gimbal_Pitch_Exp_imu = Aim_Ref.Pitch;
            Exp.Gimbal_Yaw_Exp_imu = Aim_Ref.Yaw;
#else
            switch(DouAutoAim_Status)
            {
                case Aim_1:
                    Exp.Gimbal_Pitch_Exp_imu = Aim_Ref.Pitch;
                    Exp.Gimbal_Yaw_Exp_imu = Aim_Ref.Yaw;
                    break;
                case Aim_Gap:

#if L_R                    
                    Exp.Gimbal_Pitch_Exp_imu = Ramp_Step(DoubleGimble_Msg.Angle_Pitch, Exp.Gimbal_Pitch_Exp_imu, 8.0f);
                    Exp.Gimbal_Yaw_Exp = Ramp_Step(Yaw_Median-(90-DoubleGimble_Msg.Angle_Yaw)*8191/360, Exp.Gimbal_Yaw_Exp, 10.0f);
#else
                    Exp.Gimbal_Pitch_Exp_imu = Ramp_Step(DoubleGimble_Msg.Angle_Pitch, Exp.Gimbal_Pitch_Exp_imu, 8.0f);
                    Exp.Gimbal_Yaw_Exp = Ramp_Step(Yaw_Median+(90-DoubleGimble_Msg.Angle_Yaw)*8191/360, Exp.Gimbal_Yaw_Exp, 10.0f);
#endif //L_R
                    break;
                case Aim_2:
                    Exp.Gimbal_Pitch_Exp_imu = Aim_Ref.Pitch;
                    Exp.Gimbal_Yaw_Exp_imu = Aim_Ref.Yaw;
                    break;
                default:break;
            }  
#endif //single_double
 
#if L_R             
            limit(Exp.Gimbal_Pitch_Exp_imu,(Pitch_Limit_Max-Back.Gimbal_Pitch_Back.Angle)*360/8191+IMU.EulerAngler.Pitch, (Pitch_Limit_Min-Back.Gimbal_Pitch_Back.Angle)*360/8191+IMU.EulerAngler.Pitch);
            limit(Exp.Gimbal_Yaw_Exp_imu, (Yaw_Limit_Max-Back.Gimbal_Yaw_Back.Angle)*360/8191+IMU.EulerAngler.ContinuousYaw,(Yaw_Limit_Min-Back.Gimbal_Yaw_Back.Angle)*360/8191+IMU.EulerAngler.ContinuousYaw);
#else
            limit(Exp.Gimbal_Pitch_Exp_imu,IMU.EulerAngler.Pitch - (Pitch_Limit_Min-Back.Gimbal_Pitch_Back.Angle)*360/8191, IMU.EulerAngler.Pitch-(Pitch_Limit_Max-Back.Gimbal_Pitch_Back.Angle)*360/8191);
            limit(Exp.Gimbal_Yaw_Exp_imu, (Yaw_Limit_Max-Back.Gimbal_Yaw_Back.Angle)*360/8191+IMU.EulerAngler.ContinuousYaw,(Yaw_Limit_Min-Back.Gimbal_Yaw_Back.Angle)*360/8191+IMU.EulerAngler.ContinuousYaw);
#endif            
            break;
        default:break;
    }
}

void Gimbal_Patrol()
{
    static uint8_t turn_flag_P = 0, turn_flag_Y = 0;
    static float Exp_P, Exp_Y;
    static uint16_t cnt_P = 0,cnt_Y = 0;

    turn_flag_P ? (Exp_P = Pitch_Limit_Max) : (Exp_P = Pitch_Limit_Min);
    Exp.Gimbal_Pitch_Exp = Ramp_Step(Exp_P, Exp.Gimbal_Pitch_Exp, 8);
    if(Exp.Gimbal_Pitch_Exp == Exp_P)
    {
        cnt_P++;
        if(cnt_P >= 20)
        {
            turn_flag_P ? (turn_flag_P = 0) : (turn_flag_P = 1);
            cnt_P = 0;
        }
    }
        
    turn_flag_Y ? (Exp_Y = Yaw_Limit_Max) : (Exp_Y = Yaw_Limit_Min);
    Exp.Gimbal_Yaw_Exp = Ramp_Step(Exp_Y, Exp.Gimbal_Yaw_Exp, 2);
    if(Exp.Gimbal_Yaw_Exp == Exp_Y)
    {
        cnt_Y++;
        if(cnt_Y >= 20)
        {
            turn_flag_Y ? (turn_flag_Y = 0) : (turn_flag_Y = 1);
            cnt_Y = 0;
        }
    }
} 
/*warning 需要将左头pid调弱,右头pid调硬*/
void Gimbal_PidInit()
{
    if(Control_Mode == Stop_Mode)  
    {
        Pid_Init(&Pid.Gimbal_Yaw_Place_PID, &Pid.Gimbal_Yaw_Speed_PID, 0,0,0,0,0,0,0,0);
        Pid_Init(&Pid.Gimbal_Pitch_Place_PID, &Pid.Gimbal_Pitch_Speed_PID, 0,0,0,0,0,0,0,0);
    
        Pid_Init(&Pid.Gimbal_Yaw_Place_PID_imu, &Pid.Gimbal_Yaw_Speed_PID_imu, 0,0,0,0,0,0,0,0);
        Pid_Init(&Pid.Gimbal_Pitch_Place_PID_imu, &Pid.Gimbal_Pitch_Speed_PID_imu, 0,0,0,0,0,0,0,0);
    }else{
#if L_R
        Pid_Init(&Pid.Gimbal_Yaw_Place_PID, &Pid.Gimbal_Yaw_Speed_PID, 20,0.1,-50,5000,5,0,0,5000);//10,0.05,-15,5000,5,0,0,5000);
        Pid_Init(&Pid.Gimbal_Pitch_Place_PID, &Pid.Gimbal_Pitch_Speed_PID,15,0.05,-20,5000,5,0,0,5000 );//20,0.2,-50,5000,5,0,0,5000);

        Pid_Init(&Pid.Gimbal_Yaw_Place_PID_imu, &Pid.Gimbal_Yaw_Speed_PID_imu, 40,0.1,-50,500,100,0,0,5000);//110,0.15,-150,5000,5,0,0,5000);
        Pid_Init(&Pid.Gimbal_Pitch_Place_PID_imu, &Pid.Gimbal_Pitch_Speed_PID_imu, 20,0.15,-30,500,100,0,0,5000);//200,0.3,-1000,5000,5,0,0,5000);    
#else
        Pid_Init(&Pid.Gimbal_Yaw_Place_PID, &Pid.Gimbal_Yaw_Speed_PID, 20,0.1,-30,5000,5,0,0,5000);
        Pid_Init(&Pid.Gimbal_Pitch_Place_PID, &Pid.Gimbal_Pitch_Speed_PID, 30,0.1,-15,5000,5,0,0,5000);

        Pid_Init(&Pid.Gimbal_Yaw_Place_PID_imu, &Pid.Gimbal_Yaw_Speed_PID_imu, 40,0.1,-60,500,100,0,0,5000);//150,0.3,-250,5000,5,0,0,5000);
        Pid_Init(&Pid.Gimbal_Pitch_Place_PID_imu, &Pid.Gimbal_Pitch_Speed_PID_imu, 20, 0.05, -20,500,-100,0,0,5000);//200,0.1,-200,5000,-5,0,0,5000);
#endif
    }
}
