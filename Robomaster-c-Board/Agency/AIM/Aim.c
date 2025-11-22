#include "Aim.h"
#include "string.h"
#include "Data_Exchange.h"
#include "IMU.h"
#include "usb_task.h"
#include "Gimbal.h"
#include "arm_math.h"

/* 视觉 */
extern USBD_HandleTypeDef hUsbDeviceFS;
extern IMU_t IMU;
float DistanceHorizontal(Pose_t pose);
float Predicted_Gap = 0; 
struct Armor
{
    float x;
    float y;
    float z;
    double yaw;
} Armor;

uint8_t Opon_aim ;

Aim_Rx_info Aim_Rx_infopack;
Aim_Rx_t Aim_Rx = { .Fixed_Center_time = 30, .Fixed_Armor_time = 0, .K = 0.09203f, .Rx_State = TIMESTAMP ,.K_thre = 2.0f/3.0f};
Aim_Tx_t Aim_Tx;
float DistanceToOrigin(Pose_t pose);
double ballisticSolver(float horizontal, float vertical, double bullet_speed, double k);
float Get_bullet_fly_time( float horizontal, float bullet_speed, float angle_pitch);
float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed, float k);
void transformPointGunLinkToGun(float pointInGunLink[3],float xOffest, float yOffest, float zOffest, float pointInGun[3]) ;
void transformPointGimbalToGunLink(float pointInGimbal[3],
    float roll, float pitch, float yaw,
    float pointOutGunLink[3]);
void rotationMatrixFromRPY(float roll, float pitch, float yaw, float R[3][3]);
float Bullet_Offset(float horizontal, float vertical, float bullet_speed);
/* 单方向空气阻力弹道模型 利用飞行速度求Pitch轴竖直高度 */
float monoAirResistance_Model(float horizontal, float bullet_speed, float angle_pitch, float k);

/**
 * @brief  利用梯度下降法，求解使得子弹击中目标的pitch(弧度)。
 * @param  horizontal    枪口到目标的水平距离(米)
 * @param  vertical      枪口和目标之间的高度差(米)
 * @param  bulletSpeed   子弹发射速度(米/秒)
 * @return 近似求解得到的pitch(弧度)
 */
float computePitchGradientDescent(float horizontal, float vertical, float bulletSpeed);

/**
 * @brief  利用二分查找法，求解使得子弹击中目标的pitch角度(弧度)。
 * @param  horizontal    枪口到目标的水平距离(米)
 * @param  vertical      枪口和目标之间的高度差(米)
 * @param  bulletSpeed   子弹发射速度(米/秒)
 * @return 近似求解得到的pitch(弧度)
 */
float computePitchBisection(float horizontal, float vertical, float bulletSpeed);

/* 自瞄计算 */
        float yaw ;
        float yaw_v;
        float r;
        float x_v;
        float y_v ;
        float z_v ;
        float r1;
        float r2;
        float dz;
        float temp_pitch;
        float armor_x,armor_y,armor_z;
        static  uint8_t idx = 0;  //索引
float derta;
void Aim_Calc(){
	
	  
    /* 自瞄系统正在测试中 */
    static int16_t Shoot_time_Gap = 0, Shoot_time_cnt = 0;      //射击间隔(击打能量机关需要间隔且单发)
    static float Bullet_fly_time = 0 , Bullet_Speed = 0;//动态预测时间
    static float Tar_horizontal, Tar_vertical, Tar_angle_pitch;//弹道模型
    static float distance_min = 0;//获得最近装甲板
    int use_1 = 1;
    float s_bias = 0 ;         //枪口前推的距离
    float z_bias = 0.20;         //yaw轴电机到枪口水平面的垂直距离
    uint8_t time_stamp,last_time_stamp;

      /* 自瞄模式 */
    if (DeviceState.PC_State == Device_Online){
//        Aim_Rx.Rx_State == UPDATING ? Predicted_Gap = 0 : Predicted_Gap ++;//量测更新,在视觉空窗期电控去预测
//        Aim_Rx.Predicted_Armor_time  =  (Aim_Rx.Fixed_Armor_time  + Aim_Rx_infopack.delay + Bullet_fly_time + Predicted_Gap) / 1000.0f ;
#if DEBUG_OFFSET
        Aim_Rx.Predicted_Center_time = 0;//预测时间ms
        Aim_Rx.Predicted_Armor_time  =  0 ;
#endif   
        yaw = ReceiveVisionData.data.yaw;
        yaw_v = ReceiveVisionData.data.v_yaw;
        x_v = ReceiveVisionData.data.vx;
        y_v = ReceiveVisionData.data.vy;
        z_v = ReceiveVisionData.data.vz;
        Aim_Rx.Predicted_Center_Pose.x = ReceiveVisionData.data.x;
        Aim_Rx.Predicted_Center_Pose.y = ReceiveVisionData.data.y;
        Aim_Rx.Predicted_Center_Pose.z = ReceiveVisionData.data.z;
        r1 = ReceiveVisionData.data.r1;
        r2 = ReceiveVisionData.data.r2;
        dz = ReceiveVisionData.data.dz;
				/* 空气阻力系数 */
				st.k = Aim_Rx.K; 

        Aim_Rx.Predicted_Center_Pose.HorizontalDistance = DistanceHorizontal(Aim_Rx.Predicted_Center_Pose);
        Aim_Data.HorizontalDistance = DistanceToOrigin(Aim_Rx.Predicted_Center_Pose);
				Aim_Data.id = ReceiveVisionData.data.id;
        Tar_horizontal        = Aim_Rx.Predicted_Center_Pose.HorizontalDistance;
        Tar_vertical          = Aim_Rx.Predicted_Center_Pose.z;
//				Bullet_fly_time = Aim_Rx.Predicted_Center_Pose.HorizontalDistance/23.0 * 1000.0;
        Bullet_Speed = 20;
				/* 获弹丸飞行时间 */
        Bullet_fly_time = DistanceHorizontal(Aim_Rx.Predicted_Center_Pose) / Bullet_Speed * 1000;
				
//				Shoot_time_Gap = 417.6 / pluck_speed * 1000;
        //预测时间ms
        Aim_Rx.Predicted_Center_time = (Aim_Rx.Fixed_Center_time + Aim_Rx_infopack.delay + Bullet_fly_time + Predicted_Gap + Shoot_time_Gap) / 1000.0f;
       
        /* Center_Pose（车中心）更新(预测) */
        Aim_Rx.Predicted_Center_Pose.x = ReceiveVisionData.data.x + Aim_Rx.Predicted_Center_time * x_v;
        Aim_Rx.Predicted_Center_Pose.y = ReceiveVisionData.data.y + Aim_Rx.Predicted_Center_time * y_v;
        Aim_Rx.Predicted_Center_Pose.z = ReceiveVisionData.data.z + Aim_Rx.Predicted_Center_time * z_v;
        Aim_Rx.Predicted_Center_Yaw = yaw + Aim_Rx.Predicted_Center_time * yaw_v ;
				
//				 //前哨站
//				if(ReceiveVisionData.data.armors_num == ARMOR_NUM_OUTPOST){
//							for (int i = 0; i<3; i++) {
//								float tmp_yaw = Aim_Rx.Predicted_Center_Yaw + i * 2.0 * PI/3.0;  // 2/3PI
//								float r =  (r1 + r2)/2;   //理论上r1=r2 这里取个平均值
//								Aim_Rx.Predicted_Armor_Pose[i].x = Aim_Rx.Predicted_Center_Pose.x - r*sin(tmp_yaw);
//								Aim_Rx.Predicted_Armor_Pose[i].y = Aim_Rx.Predicted_Center_Pose.y + r*cos(tmp_yaw);
//								Aim_Rx.Predicted_Armor_Pose[i].z = Aim_Rx.Predicted_Center_Pose.z;
//								Aim_Rx.Predicted_Armor_Yaw[i] = tmp_yaw;
//				   	}
//        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用

//				}
//				else{
					/* 整车观测 */
							for (int i = 0; i<4; i++) {
								float tmp_yaw;
								if(fabsf(ReceiveVisionData.data.v_yaw)<0.1){
									tmp_yaw = yaw;
								} else {
									tmp_yaw = Aim_Rx.Predicted_Center_Yaw + i *  Pi / 2.0; 								
								}
								
                float r = use_1 ? r1 : r2;
								Aim_Rx.Predicted_Armor_Pose[i].x = Aim_Rx.Predicted_Center_Pose.x - r*sin(tmp_yaw);
								Aim_Rx.Predicted_Armor_Pose[i].y = Aim_Rx.Predicted_Center_Pose.y + r*cos(tmp_yaw);		

//								if(fabsf(yaw_v) < 0.10){
//								Aim_Rx.Predicted_Armor_Pose[i].x = Aim_Rx.Predicted_Center_Pose.x -r;
//								Aim_Rx.Predicted_Armor_Pose[i].y = Aim_Rx.Predicted_Center_Pose.y + r;
//									
//								}
								Aim_Rx.Predicted_Armor_Pose[i].z = use_1 ? Aim_Rx.Predicted_Center_Pose.z : Aim_Rx.Predicted_Center_Pose.z + dz;
								Aim_Rx.Predicted_Armor_Yaw[i] = tmp_yaw;
								use_1 = !use_1;
				   	}
	            //2种常见决策方案：
            //1.计算枪管到目标装甲板yaw最小的那个装甲板
            //2.计算距离最近的装甲板


//            //计算枪管到目标装甲板yaw最小的那个装甲板
//        float yaw_diff_min = fabsf(Aim_Data.Ref_Yaw - Aim_Rx.Predicted_Armor_Yaw[0]);
//        for (int i = 1; i<4; i++) {
//            float temp_yaw_diff = fabsf(Aim_Data.Ref_Yaw - Aim_Rx.Predicted_Armor_Yaw[i]);
//            if (temp_yaw_diff < yaw_diff_min)
//            {
//                yaw_diff_min = temp_yaw_diff;
//                idx = i;
//            }
//			
//				}

							//计算距离最近的装甲板
						float dis_diff_min = sqrt(Aim_Rx.Predicted_Armor_Pose[0].x * Aim_Rx.Predicted_Armor_Pose[0].x + Aim_Rx.Predicted_Armor_Pose[0].y * Aim_Rx.Predicted_Armor_Pose[0].y  + Aim_Rx.Predicted_Armor_Pose[0].z * Aim_Rx.Predicted_Armor_Pose[0].z);
						int idx = 0;
						for (int i = 1; i<4; i++)
						{
							float temp_dis_diff = sqrt(Aim_Rx.Predicted_Armor_Pose[i].x * Aim_Rx.Predicted_Armor_Pose[0].x + Aim_Rx.Predicted_Armor_Pose[i].y * Aim_Rx.Predicted_Armor_Pose[0].y +Aim_Rx.Predicted_Armor_Pose[i].z * Aim_Rx.Predicted_Armor_Pose[0].z);
							if (temp_dis_diff < dis_diff_min)
							{
								dis_diff_min = temp_dis_diff;
								idx = i;
							}
						}
//					armor_x = Aim_Rx.Predicted_Armor_Pose[idx].x;
//					armor_y = Aim_Rx.Predicted_Armor_Pose[idx].y;											
//					if(yaw_v > 0.5){
//					armor_x = Aim_Rx.Predicted_Armor_Pose[idx].x + x_v * Aim_Rx.Predicted_Armor_time;
//					armor_y = Aim_Rx.Predicted_Armor_Pose[idx].y + y_v * Aim_Rx.Predicted_Armor_time;					
//					}

					armor_x = Aim_Rx.Predicted_Armor_Pose[idx].x /*+ x_v * Aim_Rx.Predicted_Armor_time*/;
					armor_y = Aim_Rx.Predicted_Armor_Pose[idx].y /*+ y_v * Aim_Rx.Predicted_Armor_time*/;
					armor_z = Aim_Rx.Predicted_Armor_Pose[idx].z /*+ z_v * Aim_Rx.Predicted_Armor_time*/;
//				/* p轴角度补偿 */
//       temp_pitch = pitchTrajectoryCompensation(sqrt(armor_x * armor_x + armor_y * armor_y) + s_bias,
//            armor_z + z_bias, Bullet_Speed);
////    if(temp_pitch)
////        Aim_Data.Ref_Pitch = - (float)temp_pitch * 180 / PI ;
//        Aim_Data.Ref_Pitch = - (float)atan2(armor_z +z_bias , sqrt(armor_x * armor_x + armor_y * armor_y)) * 180 / PI ;
//    if(armor_x || armor_y)
//        Aim_Data.Ref_Yaw = (float)(atan2(armor_y, armor_x)) * 180 / PI + IMU.r * 360.0f ;//弹道补偿

//			}
		}else Predicted_Gap = 0;

}
/* 自瞄补偿 */
    //在gun_link 坐标系下的点
    float pointGunLink[3] = { 0.0f, 0.0f, 0.0f };
    //在gun坐标系下的点
    float pointGun[3] = { 0.0f, 0.0f, 0.0f };
    // 陀螺仪到枪口的偏移
    float xOffest = 0.0f;
    float yOffest = 0.0f;
    float zOffest = 0.01f;
float AimAddYaw;
float LimitAimAddYaw = 4.0;
void Aim_Offset(float roll,float pitch,float yaw){
	uint8_t Bullet_Speed = 20;
		//在 gimbal 坐标系下的装甲板的三维坐标点:
    float pointGimbal[3];
    pointGimbal[0] = armor_x;
	  pointGimbal[1] = armor_y;
	  pointGimbal[2] = armor_z;
    // 陀螺仪发给视觉的rpy
//    float roll = 0.0f;
//    float pitch = 0.0f;
//    float yaw = 0.5236f;


    
		
    transformPointGimbalToGunLink(pointGimbal, roll, pitch, yaw, pointGunLink);
    transformPointGunLinkToGun(pointGunLink, xOffest, yOffest, zOffest, pointGun);
	  float bullet_offset;
//	  bullet_offset = -Bullet_Offset(sqrt(pointGun[0] * pointGun[0] + pointGun[1] * pointGun[1] + pointGun[2] * pointGun[2]),0,Bullet_Speed) * 180 / Pi;
	   float juli;
	  juli = sqrt(pointGun[0] * pointGun[0] + pointGun[1] * pointGun[1] + pointGun[2] * pointGun[2]);
	  bullet_offset = -computePitchBisection(sqrt(pointGun[0] * pointGun[0] + pointGun[1] * pointGun[1] + pointGun[2] * pointGun[2]),0,Bullet_Speed)* 180 / Pi ;
	  derta = bullet_offset;
		
// 		Aim_Data.Ref_Pitch = pitchTrajectoryCompensation(pointGun[0],pointGun[2],Bullet_Speed) * 180 /Pi + IMU.Angle_Pitch;
      if(ABS(atan(pointGun[2]/pointGun[0]) *180 /Pi) < 15){
			Aim_Data.Ref_Pitch = -atan(pointGun[2]/pointGun[0]) *180 /Pi 
	                         + bullet_offset
	                         + IMU.Angle_Pitch;			
			}
	    AimAddYaw = atan(pointGun[1]/ pointGun[0]) * 180 /Pi ;//计算需要转的角度
//	    limit(AimAddYaw,LimitAimAddYaw,-LimitAimAddYaw);
			if(ABS(AimAddYaw) < 12){
		  Aim_Data.Ref_Yaw   =  AimAddYaw  + IMU.Angle_Yawcontinuous;			
			}

}
//打弹阈值
float thersold = 2.0f;
/* 火控 */
void Aim_Shoot(){
	Aim_Data.AimShoot = AimStop;

	int tim = 0;
	 /* 动态打弹阈值 */
	if(ReceiveVisionData.data.id == ARMOR_HERO)
			thersold = atan2(AM12_ARMOR_X/2.0,sqrt(pointGun[0] * pointGun[0] + pointGun[1] * pointGun[1])) * 180 / Pi ;
	else 	
	thersold = atan2(AM02_ARMOR_X/2.0,sqrt(pointGun[0] * pointGun[0] + pointGun[1] * pointGun[1])) * 180 / Pi ;

	if(GimbalCtrl == gAim ){
		 Aim_Data.AimShoot = AimReady;
		 if(
			 ABS(AimAddYaw) <= thersold * 10 &&
			  ReceiveVisionData.data.tracking
		 )//当yaw轴误差小于打弹阈值并且追踪到目标时打弹
		 {
			  Aim_Data.AimShoot = AimFire;
		 }
	}
}


float computePitchGradientDescent(float horizontal, float vertical, float bulletSpeed) {
    // 重力加速度
    const float g = 9.8;
    // 初始猜测角度(弧度)
    float pitch = 0.035;
    // 学习率(步长大小), 这个调的时候自行调整，最好别给太高了，不然会局部收敛不住
    float learningRate = 0.0008;
    // 当前pitch下，水平方向的命中距离
    float computedHorizontal = 0.0;

    const int maxIter = 60;  // 可以调整迭代次数保证收敛精度,一定要配合最小误差多次测试，避免无用计算。
    for (int i = 0; i < maxIter; i++) {
        // 当前角度下，子弹的初始水平速度和垂直速度
        float vx ;
			  vx = bulletSpeed * cos(pitch);
        float vy ;
			  vy = bulletSpeed * sin(pitch);

        // 根据竖直运动方程和目标高度差，计算实际飞行时间
        // y(t) = vy * t - 0.5*g*t^2
        // 解方程 vy*t - (g/2)*t^2 = -vertical (这里假设vertical>0时目标在上方)
        // 或者用更通用的： 0 = -vertical + vy*t - 0.5*g*t^2
        // 这里用 vy + sqrt(...) 的解来保证正飞行时间
        float flightTime ;
			  flightTime = (vy + sqrt(vy * vy + 2 * g * vertical)) / g;

        // 水平距离
        computedHorizontal = vx * flightTime;

        // 计算与目标距离相差多少
        float error;
			  error = computedHorizontal - horizontal;

        // 如果误差足够小，就认为收敛
        if (fabs(error) < 0.0001) {
            break;
        }

        // 根据误差的正负方向，调整 pitch
        if (error > 0) {
            // 命中距离比目标距离远，需要减小仰角
            pitch -= learningRate;
        }
        else {
            // 命中距离比目标距离近，需要增大仰角
            pitch += learningRate;
        }

        if (pitch < 0.0) {
            pitch = 0.0;
        }
        else if (pitch > PI / 2) {
            pitch = PI / 2;
            break;
        }
    }
    return pitch;
}

float computePitchBisection(float horizontal, float vertical, float bulletSpeed) {
    // 重力加速度
    const float g = 9.8;
    float low = 0.0;

    // 在大部分实际场景下，0~45度足够
    float high = PI / 4.0;

    // 收敛精度角度差小于某个范围就停止
    while (high - low > 1e-7) {
        float mid ;
			  mid = (low + high) / 2.0;

        float vx;
			  vx = bulletSpeed * cos(mid);
        float vy;
    		vy = bulletSpeed * sin(mid);

        float flightTime;
			  flightTime = (vy + sqrt(vy * vy + 2 * g * vertical)) / g;
        float computedHorizontal;
		    computedHorizontal	= vx * flightTime;

        // 计算水平误差
        float error ;
			  error = computedHorizontal - horizontal;

        // 如果误差足够小就退出
        if (fabs(error) < 0.00001) {
            return mid;
        }

        // 二分查找的区间缩放
        if (error > 0) {
            // 表示打得太远，需要减小角度
            high = mid;
        }
        else {
            // 表示打得太近，需要增大角度
            low = mid;
        }
    }

    // 最终 low 或 high 都可作为结果，这里取平均值。
    return (low + high) / 2.0;
}

double simulateHorizontalWithDrag(
    double pitch,          // 初始仰角
    double bulletSpeed,    // 初速度
    double vertical,       // 目标相对枪口的高度差
    double horizontal,
    double dragCoeff       // 阻力系数
) {
    // 重力加速度
    const double g = 9.8;
    // 设定数值积分步长
    const double dt = 0.001; // 时间步长(秒)，可根据需求调大/调小
    // 初始速度分量
    double vx = bulletSpeed * cos(pitch);
    double vy = bulletSpeed * sin(pitch);

    // 初始位置(以枪口为 (0, 0), y正方向向上)
    double x = 0.0;
    double y = 0.0;

    // 使用简单的 Euler 法循环积分
    while (true) {
        // 当前速度大小
        double v = sqrt(vx * vx + vy * vy);

        // 计算加速度(假设弹丸质量 m=1)
        double ax = -dragCoeff * vx * v;         // 阻力的 x 分量
        double ay = -g - dragCoeff * vy * v;      // 阻力的 y 分量(含重力)

        // 更新速度
        vx += ax * dt;
        vy += ay * dt;

        // 更新位置
        x += vx * dt;
        y += vy * dt;

        if (x >= fabs(horizontal) && vy < 0) {
            break;
        }
        if ((vertical <= 0.0 && y <= vertical) ||
            (vertical > 0.0 && y >= vertical && vy < 0)) {
            break;
        }

        if (v < 0.001) {
            break;
        }
        if (x > 10000.0 || fabs(y) > 10000.0) {
            break;
        }
    }

    return x;
}

//二分查找（带空气阻力）
double computePitchBisectionDrag(
    double horizontal,
    double vertical,
    double bulletSpeed,
    double dragCoeff
) {
    double low = 0.0;
    double high = PI / 2.0;   // 允许到 90 度

    // 防止水平距离过小或为 0
    if (fabs(horizontal) < 1e-8) {
        return 0.0;
    }

    // 二分搜索
    while (high - low > 1e-6) {
        double mid = (low + high) / 2.0;
        double rangeMid = simulateHorizontalWithDrag(mid, bulletSpeed, vertical, horizontal, dragCoeff);

        double error = rangeMid - horizontal;

        if (fabs(error) < 0.001) {
            return mid;
        }

        if (error > 0) {
            // 打得太远 -> 减小角度
            high = mid;
        }
        else {
            // 打得太近 -> 增大角度
            low = mid;
        }
    }
    return (low + high) / 2.0;
}

//梯度下降(带空气阻力)
double computePitchGradientDescentDrag(
    double horizontal,
    double vertical,
    double bulletSpeed,
    double dragCoeff
) {
    double pitch = 0.035;       // 初始猜测
    double learningRate = 0.0008;   // 学习率，可调整
    const int maxIter = 60;   // 最大迭代次数，可调整

    for (int i = 0; i < maxIter; i++) {
        double rangeCur = simulateHorizontalWithDrag(pitch, bulletSpeed, vertical, horizontal, dragCoeff);
        double error = rangeCur - horizontal;

        // 若误差很小，可以视为收敛
        if (fabs(error) < 0.001) {
            break;
        }
        // 依据误差正负调整 pitch
        if (error > 0) {
            // 打得太远 -> 减小角度
            pitch -= learningRate;
        }
        else {
            // 打得太近 -> 增大角度
            pitch += learningRate;
        }
        // 防止 pitch 越界
        if (pitch < 0.0) {
            pitch = 0.0;
        }
        else if (pitch > PI / 2.0) {
            pitch = PI / 2.0;
            break;
        }
    }
    return pitch;
}

/* 获得子弹飞行时间 */
float Get_bullet_fly_time( float horizontal, float bullet_speed, float angle_pitch) {
    float t;
    t = (float)((exp(Aim_Rx.K * horizontal) - 1) / (Aim_Rx.K * bullet_speed * cosf(angle_pitch)));//水平方向求得飞行时间 t
    if(t < 0){
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
        //重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }
    return t;
}

/**
 * 将三维点 pointInGimbal 从 gimbal 坐标系
 * 转换到 gun_link 坐标系。
 *
 * param:
 *  - pointInGimbal: 输入装甲板三维点(在 gimbal 坐标系下)
 *  - roll, pitch, yaw: gimbal_link -> gimbal 的欧拉角
 *                      (即正向时, P_gimbal = R * P_gimbal_link)
 *  - pointOutGimbalLink: 输出装甲板三维点(在 gun_link 坐标系下)
 */
float R[3][3];
void transformPointGimbalToGunLink(float pointInGimbal[3],
    float roll, float pitch, float yaw,
    float pointOutGunLink[3])
{
    // 先获取正向旋转矩阵 R (gimbal_link -> gimbal)
    float x,y,z;
    rotationMatrixFromRPY(roll, pitch, yaw, R);

    // 因为要反向(gimbal -> gun_link)，
    //    对纯旋转来说，逆矩阵 R_inv = R^T

    // 用 R^T * pointInGimbal 得到 pointInGimbalLink
    // R^T = [ R[0][0], R[1][0], R[2][0]
    //         R[0][1], R[1][1], R[2][1]
    //         R[0][2], R[1][2], R[2][2] ]
    x = R[0][0] * pointInGimbal[0] + R[1][0] * pointInGimbal[1] + R[2][0] * pointInGimbal[2];
    y = R[0][1] * pointInGimbal[0] + R[1][1] * pointInGimbal[1] + R[2][1] * pointInGimbal[2];
    z = R[0][2] * pointInGimbal[0] + R[1][2] * pointInGimbal[1] + R[2][2] * pointInGimbal[2];

    pointOutGunLink[0] = x;
    pointOutGunLink[1] = y;
    pointOutGunLink[2] = z;
}

float Bullet_Offset(float horizontal, float vertical, float bullet_speed) {
    float g = 9.8f;          
    float pitch;
    float learning_rate = 0.001f;   
    float compute_horizontal = 0.0f;
    float v_x ;
    float v_y ;
    float t;
    float error;
    pitch	= vertical / horizontal;           

    for (int i = 0; i < 50; i++) {
			  v_x = bullet_speed * cos(pitch);
			  v_y= bullet_speed * sin(pitch);

			  t = (v_y + sqrt(v_y * v_y + 2 * g * vertical)) / g;

        compute_horizontal = v_x * t;

        error	= compute_horizontal - horizontal;

        if (fabs(error) < 0.05) {
            break;
        }

        if (error > 0)
            pitch -= learning_rate;
        else
            pitch += learning_rate;
    }
    return pitch;  
}
/**
 * 根据欧拉角(roll, pitch, yaw)生成旋转矩阵R (3x3)。
 * 旋转顺序为 Z-Y-X(航向-俯仰-滚转)。
 */
float sr,cr,sp,cp,sy,cy;
void rotationMatrixFromRPY(float roll, float pitch, float yaw, float R[3][3])
{
	
    sr = arm_sin_f32(roll);
    cr = arm_cos_f32(roll);
    sp = arm_sin_f32(pitch);
    cp = arm_cos_f32(pitch);
    sy = arm_sin_f32(yaw);
    cy = arm_cos_f32(yaw);

    // Rz(yaw) * Ry(pitch) * Rx(roll)
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}
void transformPointGunLinkToGun(float pointInGunLink[3],float xOffest, float yOffest, float zOffest, float pointInGun[3]) {
	pointInGun[0] = pointInGunLink[0] + xOffest;
	pointInGun[1] = pointInGunLink[1] + yOffest;
	pointInGun[2] = pointInGunLink[2] + zOffest;
}
		
/* 坐标点到原点水平距离 */
float DistanceHorizontal(Pose_t pose){
    return sqrt(  pose.x * pose.x + pose.y * pose.y);
}

/* 坐标点到原点的距离 */
float DistanceToOrigin(Pose_t pose){
    return sqrt(  pose.x * pose.x + pose.y * pose.y + pose.z * pose.z);
}

void Aim_SendDown(){
	   if(DeviceState.PC_State == Device_Online)
		 Gimbal_action.vision_status = vision_online;
		 else Gimbal_action.vision_status = vision_offline;
		 Gimbal_data.vision_distance = Aim_Data.HorizontalDistance * 100;
	   Gimbal_action.vision_number   = Aim_Data.id;

}