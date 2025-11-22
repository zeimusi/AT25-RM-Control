/*!
* @file     Function.c
* @brief    全局调用功能函数
*/
#include "Function.h"

/* PID获取函数 */
PID_Smis PID_Smis_get(PID_Smis PID_Smis){
    return PID_Smis;
}
PID PID_get(PID PID){
    return PID;
}
/* 设备状态检测函数 */
osStatus_t RM3508_Motor_Temp(RM3508_TypeDef *dst)
{
	if (dst->temp >80)
		return osError;
	else
		return osOK;
}

osStatus_t GM6020_Motor_Temp(GM6020_TypeDef *dst)
{
	if (dst->temp >80)
		return osError;
	else
		return osOK;
}

osStatus_t REMOTE_IfDataError( void )
{
if ( (RC_CtrlData.rc.s1 != 1 && RC_CtrlData.rc.s1 != 3 && RC_CtrlData.rc.s1 != 2)
|| (RC_CtrlData.rc.s2 != 1 && RC_CtrlData.rc.s2 != 3 && RC_CtrlData.rc.s2 != 2)
|| (RC_CtrlData.rc.ch0 > 1684 || RC_CtrlData.rc.ch0 < 364)
|| (RC_CtrlData.rc.ch1 > 1684 || RC_CtrlData.rc.ch1 < 364)
|| (RC_CtrlData.rc.ch2 > 1684 || RC_CtrlData.rc.ch2 < 364)
|| (RC_CtrlData.rc.ch3 > 1684 || RC_CtrlData.rc.ch3 < 364) )
    return osError;
else
    return osOK;
}

osStatus_t IMU_IfDataError( void )
{

    if(fabs(IMU.Angle_Pitch)>180||fabs (IMU.Angle_Roll)>180||fabs (IMU.Angle_Yaw )>180
        ||(IMU.Angle_Pitch ==0&&IMU.Angle_Roll==0&&IMU.Angle_Yaw))
        return osError;
    else
        return osOK;
}

/* 斜坡函数（float） */
float RAMP_float( float final, float now, float ramp )
{
    float	buffer =final - now;
    if (buffer > 0){
        if (buffer > ramp)  
                now += ramp;  
        else
                now += buffer;
    } else {
        if (buffer < -ramp)
                now += -ramp;
        else
                now += buffer;
    }
    return now;
}
