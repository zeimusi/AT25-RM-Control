#include "Chassis_Task.h"

void Get_ClassisMotor_Exp(void);

uint16_t Speed_Radio_Radar = 1500, Speed_Radio_Rotate = 2000;
uint16_t Speed_Radio_Radar_min = 1500;
uint16_t Speed_Radio_Radar_max = 2000;
uint16_t Speed_Radio_Rotate_min = 500;
uint16_t Speed_Radio_Rotate_max = 3000;

TaskHandle_t Chassis_Task_Handle;
void Chassis_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for(;;)
    {     
        Get_ClassisMotor_Exp();
        ChassisMotorSpeed_get(&Exp.ChassisMotor_Speed, &Exp.Chassis_Speed);
                      
        PID_Control(Back.Classis1_Back.Speed, Exp.ChassisMotor_Speed.speed_1, &Pid.Classis1_PID);		
        PID_Control(Back.Classis2_Back.Speed, Exp.ChassisMotor_Speed.speed_2, &Pid.Classis2_PID);
        PID_Control(Back.Classis3_Back.Speed, Exp.ChassisMotor_Speed.speed_3, &Pid.Classis3_PID);
        PID_Control(Back.Classis4_Back.Speed, Exp.ChassisMotor_Speed.speed_4, &Pid.Classis4_PID);
        
        limit(Pid.Classis1_PID.pid_out, RM3508_LIMIT, -RM3508_LIMIT);
        limit(Pid.Classis2_PID.pid_out, RM3508_LIMIT, -RM3508_LIMIT);
        limit(Pid.Classis3_PID.pid_out, RM3508_LIMIT, -RM3508_LIMIT);
        limit(Pid.Classis4_PID.pid_out, RM3508_LIMIT, -RM3508_LIMIT);

        Can_Msg_Tx.chassis_msg_tx[2]=(int16_t)Pid.Classis1_PID.pid_out;
        Can_Msg_Tx.chassis_msg_tx[1]=(int16_t)Pid.Classis2_PID.pid_out;
        Can_Msg_Tx.chassis_msg_tx[0]=(int16_t)Pid.Classis3_PID.pid_out;
        Can_Msg_Tx.chassis_msg_tx[3]=(int16_t)Pid.Classis4_PID.pid_out;

        if(Control_Mode != Stop_Mode)
            MotorSend(&hcan1, 0x200, Can_Msg_Tx.chassis_msg_tx);    
        
        vTaskDelayUntil(&xLastWakeTime,2);
    }
}


float chassisoffset;
void Get_ClassisMotor_Exp()
{
    if((Control_Mode == Remote_Mode && RC_CtrlData.rc.s1 == 1) || Control_Mode == Classis_Follow_Mode)
    {
		Exp.Chassis_Speed.forward_back_ref = (RC_CtrlData.rc.ch1-1024)*3;
		Exp.Chassis_Speed.left_right_ref   = (RC_CtrlData.rc.ch0-1024)*3;
		Exp.Chassis_Speed.rotate_ref       = Ramp_Step((RC_CtrlData.rc.ch2-1024)*3, Exp.Chassis_Speed.rotate_ref, 10);        
    }else if(Control_Mode == Rotate_Mode)
    {
        chassisoffset = (Main_Yaw_Median - Back.Main_Yaw_Back.Angle)*2*3.1415f/8191;
        Exp.Chassis_Speed.forward_back_ref = (RC_CtrlData.rc.ch1-1024) * 3 * arm_cos_f32(chassisoffset) - (RC_CtrlData.rc.ch0-1024) * 3 * arm_sin_f32(chassisoffset);
        Exp.Chassis_Speed.left_right_ref = (RC_CtrlData.rc.ch1-1024) * 3 * arm_sin_f32(chassisoffset) + (RC_CtrlData.rc.ch0-1024) * 3 * arm_cos_f32(chassisoffset);
        Exp.Chassis_Speed.rotate_ref = Ramp_Step(3000, Exp.Chassis_Speed.rotate_ref, 10);
    }else if(Control_Mode == Radar_Mode)
    {
        chassisoffset = (Main_Yaw_Median - Back.Main_Yaw_Back.Angle)*2*3.1415f/8191;
        
        Power_Limit();
        
        Exp.Chassis_Speed.forward_back_ref = Speed_Radio_Radar*ROBOT_CMD_DATA.data.speed_vector.vx * arm_cos_f32(chassisoffset) + Speed_Radio_Radar*ROBOT_CMD_DATA.data.speed_vector.vy * arm_sin_f32(chassisoffset);
        Exp.Chassis_Speed.left_right_ref = Speed_Radio_Radar*ROBOT_CMD_DATA.data.speed_vector.vx * arm_sin_f32(chassisoffset) - Speed_Radio_Radar*ROBOT_CMD_DATA.data.speed_vector.vy * arm_cos_f32(chassisoffset);
        Exp.Chassis_Speed.rotate_ref = Speed_Radio_Rotate;
    }else if(Control_Mode == Remote_Mode && RC_CtrlData.rc.s1 != 1)
    {
        /* 小陀螺返回遥控模式时，由于s1 = 1且遥控模式才会进入遥控底盘模式，所以底盘不会停下来，所以在这里归零一下 */
        Exp.Chassis_Speed.rotate_ref   = Ramp_Step(0, Exp.Chassis_Speed.rotate_ref, 10);
    }
}

/* 功率限制 */
void Power_Limit()
{
    if(IsUP.data.IsUP_Flag == 1)
        Speed_Radio_Radar_max = 3000;
    else
        Speed_Radio_Radar_max = 2000;
    
    if(Speed_Radio_Radar < Speed_Radio_Radar_min)
        Speed_Radio_Radar = Ramp_Step(Speed_Radio_Radar_min, Speed_Radio_Radar, 10);
    else
    {
        if(SuperCap_Msg.out_p < 100)
        {
            Speed_Radio_Radar += 15;
        }else{
            Speed_Radio_Radar -= 5;
        }
        limit(Speed_Radio_Radar, Speed_Radio_Radar_max, Speed_Radio_Radar_min); 
    }                
    
    if(Speed_Radio_Rotate< Speed_Radio_Rotate_min)
        Speed_Radio_Rotate = Ramp_Step(Speed_Radio_Rotate_min, Speed_Radio_Rotate, 10);
    else
    {
        if(SuperCap_Msg.out_p < 100)
        {
            Speed_Radio_Rotate += 5;
        }else{
            Speed_Radio_Rotate -= 15;
        }
        limit(Speed_Radio_Rotate, Speed_Radio_Rotate_max, Speed_Radio_Rotate_min); 
    }                        
}
