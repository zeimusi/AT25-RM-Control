/**
 * @file    PID.h
 * @author  yao
 * @date    1-May-2020
 * @brief   PID模块头文件
 */

#ifndef _PID_H_
#define _PID_H_
#ifdef __cplusplus
extern "C" {
#endif
    
#include <math.h>
#include "stdint.h"
typedef enum pid_Improvement_e
{
    NONE = 0X00,                        //0000 0000
    Integral_Limit = 0x01,              //0000 0001
    Derivative_On_Measurement = 0x02,   //0000 0010
    Trapezoid_Intergral = 0x04,         //0000 0100
    Proportional_On_Measurement = 0x08, //0000 1000
    OutputFilter = 0x10,                //0001 0000
    ChangingIntegralRate = 0x20,        //0010 0000
    DerivativeFilter = 0x40,            //0100 0000
    ErrorHandle = 0x80,                 //1000 0000
} PID_Improvement_e;

typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

typedef struct _PID_TypeDef
{
    float Target;
    float LastNoneZeroTarget;
    float Kp;
    float Ki;
    float Kd;

    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    float ControlPeriod;
    float MaxErr;
    float ScalarA; //For Changing Integral
    float ScalarB; //ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_Filtering_Coefficient;
    float Derivative_Filtering_Coefficient;

    uint32_t thistime;
    uint32_t lasttime;
    uint8_t dtime;

    uint8_t Improve;

    PID_ErrorHandler_t ERRORHandler;

    void (*PID_param_init)(
        struct _PID_TypeDef *pid,
        uint16_t maxOut,
        float integralLimit,
        float deadband,
        float Kp,
        float ki,
        float kd,
        float A,
        float B,
        float output_filtering_coefficient,
        float derivative_filtering_coefficient,
        uint8_t improve);

    void (*PID_reset)(
        struct _PID_TypeDef *pid,
        float Kp,
        float ki,
        float kd);
} PID_TypeDef;

static void f_Trapezoid_Intergral(PID_TypeDef *pid);
static void f_Integral_Limit(PID_TypeDef *pid);
static void f_Derivative_On_Measurement(PID_TypeDef *pid);
static void f_Changing_Integral_Rate(PID_TypeDef *pid);
static void f_Output_Filter(PID_TypeDef *pid);
static void f_Derivative_Filter(PID_TypeDef *pid);
static void f_Output_Limit(PID_TypeDef *pid);
static void f_Proportion_Limit(PID_TypeDef *pid);
static void f_PID_ErrorHandle(PID_TypeDef *pid);

void PID_init(
    PID_TypeDef *pid,
    uint16_t max_out,
    float intergral_limit,
    float deadband,

    float kp,
    float ki,
    float kd,

    float A,
    float B,

    float output_filtering_coefficient,
    float derivative_filtering_coefficient,

    uint8_t improve);
float PID_Calc(PID_TypeDef *pid, float measure, float target);

/**
 * @brief 标准位置式PID参数
 */
typedef struct {
    float Kp;           //!<@brief 比例系数
    float Ki;           //!<@brief 积分系数
    float Kd;           //!<@brief 微分系数
    float limit;        //!<@brief 积分限幅
    float error_thre;   //!<@brief 误差阈值，用于抗积分饱和
    float error_now;    //!<@brief 当前误差
    float error_last;   //!<@brief 上一次误差
    float error_inter;  //!<@brief 误差积分
    float DeadBand;     //!<@brief 误差死区
    float pid_out;      //!<@brief PID输出
} PID;

/**
 * @brief 带史密斯预估器的位置式PID参数
 */
typedef struct {
    float Kp;           //!<@brief 比例系数
    float Ki;           //!<@brief 积分系数
    float Kd;           //!<@brief 微分系数
    float limit;        //!<@brief 积分限幅
    float error_thre;   //!<@brief 积分分离，用于抗积分饱和
    float error_now;    //!<@brief 当前误差
    float error_inter;  //!<@brief 误差积分
    float DeadBand;     //!<@brief 误差死区
    float pid_out;      //!<@brief PID输出
} PID_Smis;

/**
 * @brief 增量式PID参数
 */
typedef struct {
    float Kp;           //!<@brief 比例系数
    float Ki;           //!<@brief 积分系数
    float Kd;           //!<@brief 微分系数
    float error_now;    //!<@brief 当前误差
    float error_next;   //!<@brief 上一次误差
    float error_last;   //!<@brief 上上次误差
    float increament;   //!<@brief PID增量
} PID_ADD;

/**
 * @brief 前馈控制
 */
typedef struct{
	  float K1;
	  float K2;
	  float Last_DeltIn;
	  float Now_DeltIn;
	  float Out;
	  float OutMax;
}FeedForward_Typedef;

/**
 * @brief 标准位置式PID
 * @param[in] current 实际值
 * @param[in] expected 期望值
 * @param[in] parameter PID参数
 */
void PID_Control(float current, float expected, PID *data);

/**
 * @brief 带史密斯预估器的位置式PID
 * @param[in] current 实际值
 * @param[in] expected 期望值
 * @param[in] parameter PID参数
 * @param[in] speed 实际速度
 */
void PID_Control_Smis(float current, float expected, PID_Smis *data, float speed);

/**
 * @brief 增量式PID
 * @param[in] current 实际值
 * @param[in] expect 期望值
 * @param[in] parameter PID参数
 * @return PID增量
 */
float PID_Increment(float current, float expect, PID_ADD *parameter);
/* 前馈 */
float FeedForward_Calc(FeedForward_Typedef *FF);



#ifdef __cplusplus
}
#endif

#endif
