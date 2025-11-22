/*!
 * @file     Function.h
 * @brief    声明全局调用功能函数头文件
 */

#ifndef __FUNCTION_H
#define __FUNCTION_H

#include "Variate.h"
#include "PID.h"
#include "motor.h"
/**@brief 求绝对值 */
#define ABS(x) ((x) > 0 ? (x) : (-(x)))

/**@brief 去除遥控器摇杆死区 */
#define deadline_limit(value, dealine)                     \
	{                                                      \
		if ((value) <= (dealine) && (value) >= -(dealine)) \
			value = 0;                                     \
	}

/* PID获取函数 */
PID_Smis PID_Smis_get(PID_Smis PID_Smis);
PID PID_get(PID PID);

/**@brief 斜坡函数 */
float RAMP_float(float final, float now, float ramp);

/**@brief 3508过温检测函数 */
osStatus_t RM3508_Motor_Temp(RM3508_TypeDef *dst);

/**@brief 6020过温检测函数 */
osStatus_t GM6020_Motor_Temp(GM6020_TypeDef *dst);

/**@brief 遥控器数据乱码检测 */
osStatus_t REMOTE_IfDataError(void);

/**@brief 陀螺仪数据乱码检测 */
osStatus_t IMU_IfDataError(void);
/* 3个遥控器数据处理函数 */
void RemoteControlProcess(Remote *rc);
void STOPControlProcess();
void MouseKeyControlProcess(Mouse *mouse, Key_t key, Key_t Lastkey) ;
#endif
