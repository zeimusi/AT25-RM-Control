#ifndef __SHOOT_H
#define __SHOOT_H
#include "Variate.h"
///* 发射机构变量 */
//typedef struct{
//	int16_t Ref[FRIC_SUM];
//	int16_t Speed[FRIC_SUM];
//	struct{
//	int16_t Speed,Angle;
//	} RefPluck;
//	struct{
//	int16_t Speed,Angle;
//	}Pluck;
//	enum {
//	  ShootStop      = 0,
//		ShootOne       = 1,
//		ShootThree     = 2,
//		ShootRunning   = 3,
//		ShootStucking  = 4,
//		ShootReady     = 5
//  }Mode;
//	uint8_t SingleFlag;
//}eShoot;
//extern eShoot Shoot;

//extern void ShootCtrl_Decide();
//extern void ShootRef_Update();
//extern void ShootBack_Update();
//extern void Shoot_PID();
//extern void Shoot_Send();
//extern void Struck_Detect();
//extern void ShootInit();

//extern void Shoot_Rc_Ctrl();
//extern void Shoot_Key_Ctrl();
//extern void Shoot_Stop();
 /* PID结构体定义(在此修改PID值) */

extern void Shoot_Stop();
extern void Shoot_Rc_Ctrl();    //!< @brief 发射机构遥控器模式
extern void Shoot_Key_Ctrl();   //!< @brief 发射机构键鼠模式
extern void Shoot_Drive();     //!< @brief 发射机构电机驱动
extern void Shoot_Close();
/* 检测发射机构 */
extern void Detect_Shoot();

/* 发布发射机构数据 */
extern void ShootPublish();
/* 更新状态量 */
extern void ShootData_Update();
/* 决定控制方式 */
extern void ShootCtrl_Decide();
/* 处理异常 */
extern void ShootHandleEception();
/* 设置目标量 */
extern void ShootRef_Set();
/* 枪口热量限制 */
extern void ShootHeat_Limit();
/* 计算控制量 */
extern void Shoot_Console();
/* 发送控制量 */
extern void Shoot_Send();

#endif
