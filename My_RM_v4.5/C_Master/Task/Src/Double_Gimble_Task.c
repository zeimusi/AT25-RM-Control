#include "Double_Gimble_Task.h"

typedef enum{
    OK = 2,
    NO = 1,
}mainyaw_flag_t;
mainyaw_flag_t mainyaw_flag;

uint8_t Calculation_flag = 0;//每次收到两头信息计算一次
float main_yaw_calculation_exp;

TaskHandle_t Double_Gimble_Task_handle;
void Double_Gimble_Task(void *pvParameters)
{    
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {
#if single_double == 0        
        
            if((DeviceStatus.AimAngle_L == Device_Error && DeviceStatus.AimAngle_R == Device_Error)||Control_Mode != Radar_Mode)
            {
                AutoAimMsg_L.flag = 0;
                AutoAimMsg_R.flag = 0;
                Calculation_flag = 0;
                mainyaw_flag = NO;
                Can_Msg_Tx.mainyaw_flag_msg_tx[0] = mainyaw_flag;
            }else{
                if(AutoAimMsg_L.flag == 1)
                {   
                    AutoAimMsg_L.Yaw   = AutoAimMsg_rx_L.AutoAimMsg_rx.Yaw*1.0f/10;
                    AutoAimMsg_L.Pitch = AutoAimMsg_rx_L.AutoAimMsg_rx.Pitch*1.0f/10;
                    AutoAimMsg_L.HorizontalDistance = AutoAimMsg_rx_L.AutoAimMsg_rx.HorizontalDistance*1.0f/10;
                    AutoAimMsg_L.DouAutoAim_Status = AutoAimMsg_rx_L.AutoAimMsg_rx.DouAutoAim_Status;
                    AutoAimMsg_L.IsMainYawMove = AutoAimMsg_rx_L.AutoAimMsg_rx.IsMainYawMove;
                    
                    switch(AutoAimMsg_L.DouAutoAim_Status)
                    {
                        case Aim_1:
                            if(Calculation_flag == 0)
                            {
                                if(AutoAimMsg_L.IsMainYawMove == 1)
                                {
                                    Double_Guns_Combat_Angle_Calculate(&main_yaw_calculation_exp, &Can_Msg_Tx.lgimble_msg_tx.GimbleAngle.Angle_Yaw, &Can_Msg_Tx.rgimble_msg_tx.GimbleAngle.Angle_Yaw);
                                    Can_Msg_Tx.lgimble_msg_tx.GimbleAngle.Angle_Pitch = AutoAimMsg_L.Pitch;
                                    Can_Msg_Tx.rgimble_msg_tx.GimbleAngle.Angle_Pitch = AutoAimMsg_L.Pitch;
                                }else{
                                    Can_Msg_Tx.lgimble_msg_tx.GimbleAngle.Angle_Yaw = AutoAimMsg_L.Yaw;
                                    Can_Msg_Tx.rgimble_msg_tx.GimbleAngle.Angle_Yaw = AutoAimMsg_L.Yaw;

                                    Can_Msg_Tx.lgimble_msg_tx.GimbleAngle.Angle_Pitch = AutoAimMsg_L.Pitch;
                                    Can_Msg_Tx.rgimble_msg_tx.GimbleAngle.Angle_Pitch = AutoAimMsg_L.Pitch;
                                }
                                    
                                mainyaw_flag = NO;
                                Can_Msg_Tx.mainyaw_flag_msg_tx[0] = mainyaw_flag;
                                
                                Calculation_flag = 1;
                            }
                            break;
                        case Aim_Gap:
                            Calculation_flag = 0;
                            if(_fabsf(IMU.EulerAngler.ContinuousYaw - Exp.Main_Yaw_Exp)<2)
                            {
                                static uint8_t cnt = 0;
                                if(cnt >= 50)
                                {
                                    mainyaw_flag = OK;//两头可以开启自瞄    
                                    Can_Msg_Tx.mainyaw_flag_msg_tx[0] = mainyaw_flag;
                                    cnt = 0;
                                }else
                                    cnt++;
                            }
                            break;
                        case Aim_2:
                            //当左头为主头，左头丢失目标，右头未丢失目标时，换右头做主头
                            if(DeviceStatus.AimAngle_L == Device_Error && DeviceStatus.AimAngle_R == Device_Right)
                            {
                                AutoAimMsg_L.flag = 0;
                                AutoAimMsg_R.flag = 1;  
                            }
                        
                            /* 判断主Yaw是否要动的方法：1.上板判断小头距离归中位置是否偏移多于规定值（上板现在给的200），多于就证明主Yaw要移动 
                                                       2.下板判断收到上板发来的AutoAimMsg_L.Yaw（这个是就是上头距离归中位置偏移的度数），大于规定值（200 * 360 / 8191）就证明主Yaw要移动
                               这里方法一有问题，在二次自瞄里，当敌方移动太快，主Yaw跟不上小头移动速度，就会出现，主Yaw转到计算出来的位置时，小头已经又转到，偏移量大于规定值的位置了，之后AutoAimMsg_L.IsMainYawMove和Calculation_flag一直等于1,主Yaw就不会在转动了，小头自然就会很快丢失目标
                               在方法二同理也有问题。这里将规定值增大到500
                            */
                            if(_fabsf(AutoAimMsg_L.Yaw)>(500*360/8191) && Calculation_flag == 0)
                            {
                                Double_Guns_Combat_Angle_Calculate(&main_yaw_calculation_exp, &Can_Msg_Tx.lgimble_msg_tx.GimbleAngle.Angle_Yaw, &Can_Msg_Tx.rgimble_msg_tx.GimbleAngle.Angle_Yaw);
                                Calculation_flag = 1;
                            }else if(_fabsf(AutoAimMsg_L.Yaw)<=(500*360/8191)){
                                Calculation_flag = 0;
                            }
                            break;
                        default:
                            break;
                    }
                }else if(AutoAimMsg_R.flag == 1){                
                    
                    AutoAimMsg_R.Yaw   = AutoAimMsg_rx_R.AutoAimMsg_rx.Yaw*1.0f/10;
                    AutoAimMsg_R.Pitch = AutoAimMsg_rx_R.AutoAimMsg_rx.Pitch*1.0f/10;
                    AutoAimMsg_R.HorizontalDistance = AutoAimMsg_rx_R.AutoAimMsg_rx.HorizontalDistance*1.0f/10;
                    AutoAimMsg_R.DouAutoAim_Status = AutoAimMsg_rx_R.AutoAimMsg_rx.DouAutoAim_Status;
                    AutoAimMsg_R.IsMainYawMove = AutoAimMsg_rx_R.AutoAimMsg_rx.IsMainYawMove;
                    
                    switch(AutoAimMsg_R.DouAutoAim_Status)
                    {
                        case Aim_1:
                            if(Calculation_flag == 0)
                            {
                                if(AutoAimMsg_R.IsMainYawMove == 1)
                                {
                                    Double_Guns_Combat_Angle_Calculate(&main_yaw_calculation_exp, &Can_Msg_Tx.lgimble_msg_tx.GimbleAngle.Angle_Yaw, &Can_Msg_Tx.rgimble_msg_tx.GimbleAngle.Angle_Yaw);
                                    Can_Msg_Tx.lgimble_msg_tx.GimbleAngle.Angle_Pitch = AutoAimMsg_R.Pitch;
                                    Can_Msg_Tx.rgimble_msg_tx.GimbleAngle.Angle_Pitch = AutoAimMsg_R.Pitch;
                                }else{
                                    main_yaw_calculation_exp = IMU.EulerAngler.ContinuousYaw;
                                    
                                    Can_Msg_Tx.lgimble_msg_tx.GimbleAngle.Angle_Yaw = AutoAimMsg_R.Yaw;
                                    Can_Msg_Tx.rgimble_msg_tx.GimbleAngle.Angle_Yaw = AutoAimMsg_R.Yaw;

                                    Can_Msg_Tx.lgimble_msg_tx.GimbleAngle.Angle_Pitch = AutoAimMsg_R.Pitch;
                                    Can_Msg_Tx.rgimble_msg_tx.GimbleAngle.Angle_Pitch = AutoAimMsg_R.Pitch;
                                }
                                    
                                mainyaw_flag = NO;
                                Can_Msg_Tx.mainyaw_flag_msg_tx[0] = mainyaw_flag;
                                
                                Calculation_flag = 1;
                            }
                            break;
                        case Aim_Gap:
                            Calculation_flag = 0;
                            if(_fabsf(IMU.EulerAngler.ContinuousYaw - Exp.Main_Yaw_Exp)<2)
                            {
                                static uint8_t cnt = 0;
                                if(cnt >= 50)
                                {
                                    mainyaw_flag = OK;
                                    Can_Msg_Tx.mainyaw_flag_msg_tx[0] = mainyaw_flag;
                                    cnt = 0;
                                }else
                                    cnt++;
                            }
                            break;
                        case Aim_2:
                            //当左头为主头，左头丢失目标，右头未丢失目标时，换右头做主头
                            if(DeviceStatus.AimAngle_R == Device_Error && DeviceStatus.AimAngle_L == Device_Right)
                            {
                                AutoAimMsg_R.flag = 0;
                                AutoAimMsg_L.flag = 1;  
                            }
                        
                            if(_fabsf(AutoAimMsg_R.Yaw)>(500*360/8191) && Calculation_flag == 0)
                            {
                                Double_Guns_Combat_Angle_Calculate(&main_yaw_calculation_exp, &Can_Msg_Tx.lgimble_msg_tx.GimbleAngle.Angle_Yaw, &Can_Msg_Tx.rgimble_msg_tx.GimbleAngle.Angle_Yaw);
                                Calculation_flag = 1;
                            }else if(_fabsf(AutoAimMsg_R.Yaw)<=(500*360/8191)){
                                Calculation_flag = 0;
                            }
                            break;
                        default:
                            break;
                    }
                }    
            }
#endif
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}

/**
 * @brief 两头协作作战时，计算主yaw应该怎么转，小yaw应该怎么转
 * @param Big_Angle_esp    算出来的大云台应该转到的角度，是陀螺仪角度，可直接用作期望值
 * @param LSmall_Angle_esp 算出来的左头Yaw应该转过的角度，是目标位置与朝向正前方位置的夹角
 * @param RSmall_Angle_esp 算出来的右头Yaw应该转过的角度，是目标位置与朝向正前方位置的夹角
 */
void Double_Guns_Combat_Angle_Calculate(float *Big_Angle_esp, float *LSmall_Angle_esp, float *RSmall_Angle_esp)
{
    static float Small_Gimbal_Angle,//小云台面向敌方装甲板的Yaw轴值
                 Big_Strike_Distance,   //大云台中心到敌方装甲版的距离（算）
                 Small_Strike_Distance, //小云台中心到敌方装甲版的距离
                 Big_Gimbal_Angle_Calculate,   //大云台计算后的Yaw轴值（算）
                 LSmall_Gimbal_Angle_Calculate, //小云台计算后的Yaw轴值（算）
                 RSmall_Gimbal_Angle_Calculate; //小云台计算后的Yaw轴值（算）
                 
    const float Gimbal_Distance = 0.15753f;//小云台中心到大云台中心的间距m
    
    if(AutoAimMsg_L.flag == 1)
    {
        Small_Gimbal_Angle = AutoAimMsg_L.Yaw;
        Small_Strike_Distance = AutoAimMsg_L.HorizontalDistance;
    }
    else if(AutoAimMsg_R.flag == 1)
    {
        Small_Gimbal_Angle = AutoAimMsg_R.Yaw;
        Small_Strike_Distance = AutoAimMsg_R.HorizontalDistance;
    }
    
    Big_Strike_Distance = sqrt(Gimbal_Distance*Gimbal_Distance + Small_Strike_Distance*Small_Strike_Distance - 2*Gimbal_Distance*Small_Strike_Distance*_fabsf(cosf((90+Small_Gimbal_Angle)*PI/180)));
    
    if(_fabsf(Small_Gimbal_Angle)<=90)
    {
        Big_Gimbal_Angle_Calculate = IMU.EulerAngler.ContinuousYaw + (Small_Gimbal_Angle + acosf((Big_Strike_Distance*Big_Strike_Distance + Small_Strike_Distance*Small_Strike_Distance - Gimbal_Distance*Gimbal_Distance)/(2*Big_Strike_Distance*Small_Strike_Distance)));   
    }else{
        Big_Gimbal_Angle_Calculate = IMU.EulerAngler.ContinuousYaw + (Small_Gimbal_Angle - acosf((Big_Strike_Distance*Big_Strike_Distance + Small_Strike_Distance*Small_Strike_Distance - Gimbal_Distance*Gimbal_Distance)/(2*Big_Strike_Distance*Small_Strike_Distance)));   
    }
    
    LSmall_Gimbal_Angle_Calculate = atan(Big_Strike_Distance/Gimbal_Distance)*180/PI;
    RSmall_Gimbal_Angle_Calculate = atan(Big_Strike_Distance/Gimbal_Distance)*180/PI;   
    
    *Big_Angle_esp = Big_Gimbal_Angle_Calculate+IMU.EulerAngler.r*360;
    *LSmall_Angle_esp = LSmall_Gimbal_Angle_Calculate;
    *RSmall_Angle_esp = RSmall_Gimbal_Angle_Calculate;
}
 
