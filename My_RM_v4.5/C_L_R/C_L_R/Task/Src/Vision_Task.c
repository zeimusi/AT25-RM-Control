#include "Vision_Task.h"
#include "usb_device.h"

#define DEBUG_OFFSET 0  //调试偏移量时使用，减去所有预测部分，也就是定打定那种
#define GRAVITY 9.78f
uint8_t Robot_Color = 0;  // 回头加上裁判系统信息

extern SendDataAutoAim_s SEND_DATA_AutoAim;         // 发给视觉的数据

void transformPointGunLinkToGun(float pointInGunLink[3],float xOffest, float yOffest, float zOffest, float pointInGun[3]) ;
void transformPointGimbalToGunLink(float pointInGimbal[3],
    float roll, float pitch, float yaw,
    float pointOutGunLink[3]);
void rotationMatrixFromRPY(float roll, float pitch, float yaw, float R[3][3]);  
float Bullet_Offset(float horizontal, float vertical, float bullet_speed);
float computePitchBisection(float horizontal, float vertical, float bulletSpeed);
	void Aim_Offset(float roll,float pitch,float yaw);
/* 自瞄PC通信 */
#if L_R
Aim_Rx_t Aim_Rx = { .Fixed_Center_time = 100,.Fixed_Armor_time = 0, .K = 0.01903f, .Rx_State = TIMESTAMP ,.K_thre = 0.3f};
#else
Aim_Rx_t Aim_Rx = { .Fixed_Center_time = 100,.Fixed_Armor_time = 0, .K = 0.01903f, .Rx_State = TIMESTAMP ,.K_thre = 0.5f};
#endif
// k = 0.01903; // 25°C, 1atm, 小弹丸 空气阻力系数
// k = 0.000556; // 25°C, 1atm, 大弹丸
// k = 0.000530; // 25°C, 1atm, 发光大弹丸

/* 自瞄判断出来的云台期望值 */
Aim_t Aim_Ref = {.Yaw = 0, .Pitch = 0, .HorizontalDistance = 0};
/**
 * 电控视觉通信流程:（时间戳模式）电控索要时间戳，视觉发送时间戳，电控收到后计算时间差并改变模式（获得通讯延迟）
 *                 （颜色模式）电控发送敌我颜色，视觉收到后开始发送自瞄信息
 *                 （信息更新模式，信息正在更新模式）收到自瞄信息，立刻变成正在更新模式（UPDATING）（再回调里），处理了一次数据后立刻变到更新模式（UPDATE）(在主程序里)，开始记录预测时间（Predicted_Gap），在收到下一包自瞄信息时，在变回正在更新模式（UPDATING）（再回调里），预测时间归零
 *                                                 自瞄空窗期，电控预测，就是记录时间，根据之前的自瞄信息（目标位置，速度），和记录的时间（目标位置+速度*时间），预测没有自瞄信息的这一段时间，目标会移动到的位置，算完就变回信息更新模式
 */

/**
 * 目前代码还需要调试的地方：1.静态时间（Fixed_Center_time，Fixed_Armor_time）还需要调试，现在为零，这个应该是固定延时，包括拨弹盘转动到子弹被摩擦轮发送出去的时间
 *                          2.计算子弹飞行时间，应该是要用完美弹道模型什么的，模拟出弹道，根据弹道算出飞行时间，但现在代码里这一部分属于调试代码里的，飞行时间直接给的100
 *                           （也可能因为短距离发射不需要太精确的弹道，那也应该用距离*弹速算一下，不能直接给100）
 *                            弹速这里直接给的27，以后要换成裁判系统反馈    
 *                          3.根据高瑞说的，坐标好像完全反了，得到的坐标数据都要加-
 */
uint8_t b = 0;
extern uint8_t ignore_id;
extern uint16_t c;
TaskHandle_t Vision_Task_Handle;
void Vision_Task(void *pvParameters)
{
    portTickType xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
	
    for(;;)
    {
        Aim_Control();
        b++;
		if(b == 2)
		{
			AutoAimXYZ_Msg_Send();
			b = 0;
		}
		
		if(Is_Fortress_Msg.Is_Fortress_Flag == 1)
		{
			if(c == 0)
			{
				ignore_id = ReceiveAutoaimData.data.id;
			}
			c++;
		}
		if(c >= 1000)
		{
			Is_Fortress_Msg.Is_Fortress_Flag = 0;
			c = 0;
		}
		
        vTaskDelayUntil(&xLastWakeTime,1);
    }
}

/*陀螺仪坐标系到转轴坐标系转换*/
void Coordinate_Transform(ReceiveAutoaimData_t *AutoaimData,Pose_t *pose)
{
    pose->Y = AutoaimData->data.y;// - 0.01255f;
    pose->X = AutoaimData->data.x;// - 0.01704f;
    pose->Z = AutoaimData->data.z;// + 0.125875f;   
}

#if L_R
float offset_p_c = 1.5, offset_y_c = 0; //center
float offset_p_a = 3.0f, offset_y_a = 0.0f;
#else
float offset_p_c = 0, offset_y_c = 1.7; //center
float offset_p_a = 0.0f, offset_y_a = 0.0f;
#endif

uint8_t text_R = 0;
void Aim_Control()
{
    /* 自瞄系统正在测试中 */
    static uint16_t Predicted_Gap = 0, Bullet_fly_time = 0, Bullet_Speed = 23;//动态预测时间
    static float Tar_horizontal, Tar_vertical, Tar_angle_pitch;//弹道模型
    static float distance_min = 0;//获得最近装甲板
    static uint8_t idx = 0; //索引

    /* 自瞄模式 */
    if(DeviceStatus.PC_State == Device_Right){
        Aim_Rx.Rx_State == UPDATING ? Predicted_Gap = 0 : Predicted_Gap ++;//量测更新,在视觉空窗期电控去预测

        Aim_Rx.Predicted_Center_time = 0;//(Aim_Rx.Fixed_Center_time + Bullet_fly_time + Predicted_Gap) / 1000.0f ;//预测时间ms
        Aim_Rx.Predicted_Armor_time = 0;//(Aim_Rx.Fixed_Armor_time + Bullet_fly_time + Predicted_Gap) / 1000.0f ;           暂时改0

        Coordinate_Transform(&ReceiveAutoaimData, &Aim_Rx.Predicted_Center_Pose);

        /* Center_Pose更新(预测) */
        Aim_Rx.Predicted_Center_Pose.X = Aim_Rx.Predicted_Center_Pose.X + ReceiveAutoaimData.data.vx * Aim_Rx.Predicted_Center_time;
        Aim_Rx.Predicted_Center_Pose.Y = Aim_Rx.Predicted_Center_Pose.Y + ReceiveAutoaimData.data.vy * Aim_Rx.Predicted_Center_time;
        Aim_Rx.Predicted_Center_Pose.Z = Aim_Rx.Predicted_Center_Pose.Z + ReceiveAutoaimData.data.vz * Aim_Rx.Predicted_Center_time;
        Aim_Rx.Predicted_Center_Pose.theta = ReceiveAutoaimData.data.yaw + ReceiveAutoaimData.data.v_yaw * Aim_Rx.Predicted_Center_time;
        Aim_Rx.Predicted_Center_Pose.HorizontalDistance = DistanceHorizontal(Aim_Rx.Predicted_Center_Pose);
        
        /* 获得目标Amror_Pose并得到Tar_Armor */
        distance_min = Aim_Rx.Predicted_Center_Pose.HorizontalDistance;
        for (uint8_t i = 0; i < 4; i++) {
            float r = ReceiveAutoaimData.data.r1 * ((i+1) % 2) + ReceiveAutoaimData.data.r2 * (i % 2); //r1是当前装甲板
            float dz = ReceiveAutoaimData.data.dz * (i % 2) ; // 另一个装甲板的高度
            float yaw;
            
            if(fabs(ReceiveAutoaimData.data.v_yaw) > 0.15f)
                yaw = Aim_Rx.Predicted_Center_Pose.theta;
            else
                yaw = Aim_Rx.Predicted_Center_Pose.theta + 0*pi/2;            
            
            if(text_R == 0)
            {
                Aim_Rx.Predicted_Armor_Pose[i].X = Aim_Rx.Predicted_Center_Pose.X - r * sin(yaw);
                Aim_Rx.Predicted_Armor_Pose[i].Y = Aim_Rx.Predicted_Center_Pose.Y + r * cos(yaw);           
            }else{
                Aim_Rx.Predicted_Armor_Pose[i].X = Aim_Rx.Predicted_Center_Pose.X - 0.22f * sin(yaw);
                Aim_Rx.Predicted_Armor_Pose[i].Y = Aim_Rx.Predicted_Center_Pose.Y + 0.22f * cos(yaw);           
            }
            
            Aim_Rx.Predicted_Armor_Pose[i].Z = Aim_Rx.Predicted_Center_Pose.Z + dz;
            Aim_Rx.Predicted_Armor_Pose[i].theta = yaw;
            Aim_Rx.Predicted_Armor_Pose[i].HorizontalDistance = DistanceHorizontal(Aim_Rx.Predicted_Armor_Pose[i]);

            if(Aim_Rx.Predicted_Armor_Pose[i].HorizontalDistance < distance_min){
                distance_min = Aim_Rx.Predicted_Armor_Pose[i].HorizontalDistance;
                idx = i;
            }
        }
        
        /* 调试用，用于视觉创建可视化 */
        SEND_DATA_AutoAim.data.aim_x = Aim_Rx.Predicted_Armor_Pose[idx].X;
        SEND_DATA_AutoAim.data.aim_y = Aim_Rx.Predicted_Armor_Pose[idx].Y;
        SEND_DATA_AutoAim.data.aim_z = Aim_Rx.Predicted_Armor_Pose[idx].Z;        

        /* 弹道模型更新：估计弹道，计算该弹道下的飞行时间 */
        if(Aim_Rx.Rx_State == UPDATING){
            Tar_horizontal = Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance;
            Tar_vertical = Aim_Rx.Predicted_Armor_Pose[idx].Z;
            Bullet_Speed = Judge_Msg.ShootAbout.initial_speed;
//            Aim_Rx.PitchOffset = ballisticSolver(Tar_horizontal, Tar_vertical, Bullet_Speed, Aim_Rx.K); //完美空气阻力
            Aim_Rx.PitchOffset = Get_Pitch_Angle_Compensation(Tar_horizontal, Tar_vertical, Bullet_Speed);
            
         //   Tar_angle_pitch = atan2(Tar_vertical, Tar_horizontal);
           // Bullet_fly_time = Get_bullet_fly_time(Tar_horizontal, Bullet_Speed, Tar_angle_pitch + Aim_Rx.PitchOffset);      // 暂时关
            
            Aim_Rx.Rx_State = UPDATE;
        }

        /* 坐标变换成角度 */
        Aim_Rx.Predicted_Armor_Yaw = atan2( Aim_Rx.Predicted_Armor_Pose[idx].Y, Aim_Rx.Predicted_Armor_Pose[idx].X) * 180 / pi + IMU.EulerAngler.r;
        Aim_Rx.Predicted_Armor_Pitch = -atan2(Aim_Rx.Predicted_Armor_Pose[idx].Z, Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance) * 180 / pi;
        Aim_Rx.Predicted_Center_Yaw = atan2( Aim_Rx.Predicted_Center_Pose.Y, Aim_Rx.Predicted_Center_Pose.X ) * 180 / pi+ IMU.EulerAngler.r;
        Aim_Rx.Predicted_Center_Pitch = -atan2( Aim_Rx.Predicted_Center_Pose.Z, Aim_Rx.Predicted_Center_Pose.HorizontalDistance) * 180 / pi;
//        Aim_Rx.Predicted_Center_Yaw = atan2( ReceiveAutoaimData.data.y, ReceiveAutoaimData.data.x ) * 180 / pi;
//        Aim_Rx.Predicted_Center_Pitch = -atan2( ReceiveAutoaimData.data.z, sqrt( ReceiveAutoaimData.data.x * ReceiveAutoaimData.data.x + ReceiveAutoaimData.data.y * ReceiveAutoaimData.data.y)) * 180 / pi;
//              
//        if(ReceiveAutoaimData.data.v_yaw < 3.0f)
//        {
//            Aim_Ref.Yaw = Aim_Rx.Predicted_Armor_Yaw;// + offset_y_a;
//            Aim_Ref.Pitch = Aim_Rx.Predicted_Armor_Pitch;// - offset_p_a - Aim_Rx.PitchOffset;
//            Aim_Ref.HorizontalDistance = Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance;
//        }else{
//            Aim_Ref.Yaw = Aim_Rx.Predicted_Center_Yaw;// + offset_y_c;
//            Aim_Ref.Pitch = Aim_Rx.Predicted_Center_Pitch;// - Aim_Rx.PitchOffset - offset_p_c;
//            Aim_Ref.HorizontalDistance = Aim_Rx.Predicted_Center_Pose.HorizontalDistance;       
//        }
        /* 自瞄补偿解算 */
		Aim_Offset((double)IMU.EulerAngler.Roll * pi / 180.0,(double)IMU.EulerAngler.Pitch * pi / 180.0,(double)IMU.EulerAngler.ContinuousYaw * pi /180.0);
		
        /* 打弹动态阈值，由于此时水平距离是最近距离，可以当做装甲板是正面朝向我们，将装甲板半径与车到装甲板距离做反正切，得到到装甲板边缘的距离，再乘个系数，也就是到中心周围的角度，当云台角度与期望值差值小于这个数，就相当于对准了，就开始射击 */
		/* warning  右头需要加大阈值*/
			if( ReceiveAutoaimData.data.id != 1){//根据装甲板半径与距离解算
            Aim_Rx.Y_thre = atan2(AM02_ARMOR_X / 2.0f ,Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance)*180/pi * Aim_Rx.K_thre;
            Aim_Rx.P_thre = atan2(AM02_ARMOR_Y / 2.0f ,Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance)*180/pi * Aim_Rx.K_thre;
        } else {
            Aim_Rx.Y_thre = atan2(AM12_ARMOR_X / 2.0f ,Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance)*180/pi * Aim_Rx.K_thre;
            Aim_Rx.P_thre = atan2(AM12_ARMOR_Y / 2.0f ,Aim_Rx.Predicted_Armor_Pose[idx].HorizontalDistance)*180/pi * Aim_Rx.K_thre;
        }

        Aim_Rx.aim_runing = 1;
          
    }else{
        Aim_Rx.aim_runing = 0;
        Predicted_Gap = 0;
    }
}

/* 自瞄补偿 */
    //在gun_link 坐标系下的点
    float pointGunLink[3] = { 0.0f, 0.0f, 0.0f };
    //在gun坐标系下的点
    float pointGun[3] = { 0.0f, 0.0f, 0.0f };
    // 陀螺仪到枪口的偏移
    float xOffest = 0.0f;
    float yOffest = 0.0f;
    float zOffest = 0.09f;
float AimAddYaw;
float LimitAimAddYaw = 4.0;
void Aim_Offset(float roll,float pitch,float yaw){
	uint8_t Bullet_Speed = 23;
		//在 gimbal 坐标系下的装甲板的三维坐标点:
    float pointGimbal[3];
    pointGimbal[0] = SEND_DATA_AutoAim.data.aim_x;
	pointGimbal[1] = SEND_DATA_AutoAim.data.aim_y;
	pointGimbal[2] = SEND_DATA_AutoAim.data.aim_z;


    
		 
    transformPointGimbalToGunLink(pointGimbal, roll, pitch, yaw, pointGunLink);
    transformPointGunLinkToGun(pointGunLink, xOffest, yOffest, zOffest, pointGun);
	  float bullet_offset;
	  bullet_offset = -computePitchBisection(sqrt(pointGun[0] * pointGun[0] + pointGun[1] * pointGun[1] + pointGun[2] * pointGun[2]),0,Bullet_Speed)* 180 / pi ;
		Aim_Ref.Pitch = -atan(pointGun[2]/pointGun[0]) *180 /pi 
	                         + bullet_offset
	                         + IMU.EulerAngler.Pitch;
	    AimAddYaw = atan(pointGun[1]/ pointGun[0]) * 180 /pi ;//计算需要转的角度
	    limit(AimAddYaw,LimitAimAddYaw,-LimitAimAddYaw);
		Aim_Ref.Yaw   =  AimAddYaw  + IMU.EulerAngler.ContinuousYaw;
		

}/**
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
void transformPointGunLinkToGun(float pointInGunLink[3],float xOffest, float yOffest, float zOffest, float pointInGun[3]) {
	pointInGun[0] = pointInGunLink[0] + xOffest;
	pointInGun[1] = pointInGunLink[1] + yOffest;
	pointInGun[2] = pointInGunLink[2] + zOffest;
}
/**
 * 根据欧拉角(roll, pitch, yaw)生成旋转矩阵R (3x3)。
 * 旋转顺序为 Z-Y-X(航向-俯仰-滚转)。
 */
float sr,cr,sp,cp,sy,cy;
void rotationMatrixFromRPY(float roll, float pitch, float yaw, float R[3][3])
{
	
    sr = sin(roll);
    cr = cos(roll);
    sp = sin(pitch);
    cp = cos(pitch);
    sy = sin(yaw);
    cy = cos(yaw);

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
float computePitchBisection(float horizontal, float vertical, float bulletSpeed) {
    // 重力加速度
    const float g = 9.8;
    float low = 0.0;

    // 在大部分实际场景下，0~45度足够
    float high = pi / 4.0;

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
/* 坐标点到原点水平距离 */
float DistanceHorizontal(Pose_t pose){
    return sqrt( pose.X * pose.X + pose.Y * pose.Y);
}

float t;
float t_ideal;
/* 获得子弹飞行时间 */
float Get_bullet_fly_time( float horizontal, float bullet_speed, float angle_pitch){
//    float t;
    t = (float)((exp(Aim_Rx.K * horizontal) - 1) / (Aim_Rx.K * bullet_speed * cos(angle_pitch)));//水平方向求得飞行时间t
    t_ideal = horizontal / (bullet_speed * cos(angle_pitch)); 
    if(t < 0){
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
        //重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }
    return t;
}
/* 求得Pitch轴角度补偿(单方向空气阻力，适用于远距离，小角度) */
float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed){
    float temp_vertical, actual_vertical, error_vertical;
    float pitch, pitch_new;

    pitch = atan2(vertical, horizontal);
    temp_vertical = vertical;
    //迭代重力法
    for (uint8_t i = 0; i < 20; i++){
        pitch_new = atan2(temp_vertical, horizontal);
        actual_vertical = monoAirResistance_Model(horizontal, bullet_speed, pitch_new);
        error_vertical = 0.3f * (vertical - actual_vertical);
        temp_vertical = temp_vertical + error_vertical;
        if (fabsf(error_vertical) < 0.001f)
            break;
    }

    return (pitch_new - pitch) * 180.0f / pi;
}
/* 单方向空气阻力弹道模型 */
float monoAirResistance_Model(float horizontal, float bullet_speed, float angle_pitch){
    float actual_vertical, t;

    t = Get_bullet_fly_time(horizontal, bullet_speed, angle_pitch);
    actual_vertical = bullet_speed * sin(angle_pitch) * t - GRAVITY * t * t / 2; //得出子弹会打到的竖直高度

    return actual_vertical;
}

///* 坐标点到原点的距离 */
//float DistanceToOrigin(Pose_t pose){
//    return sqrt( pose.X * pose.X + pose.Y * pose.Y + pose.Z * pose.Z);
//}

///* 完美弹道模型 */
//double ballisticSolver(float horizontal, float vertical, double bullet_speed, double k){
//    // FIXME: NaN Failed 是否因为 vertical 有时候为负
//    // TODO: 世界坐标系下进行迭代
//    double pitch = atan2(vertical, horizontal);
//    double temp_vertical = vertical;
//    double pitch_new = pitch;

//    // 迭代求解
//    for (int i = 0; i < 20; ++i) {
//        double x = 0.0;
//        double y = 0.0;
//        double p = tan(pitch_new);
//        double v = bullet_speed;
//        double u = v / sqrt(1 + p * p);
//        double delta_x = horizontal / 10;

//        // 使用四阶龙格-库塔法求解微分方程，步长决定精度
//        for (int j = 0; j < 20; ++j) {
//            double k1_u = -k * u * sqrt(1 + (p*p) );
//            double k1_p = -GRAVITY / (u*u);
//            double k1_u_sum = u + k1_u * (delta_x / 2);
//            double k1_p_sum = p + k1_p * (delta_x / 2);

//            double k2_u = -k * k1_u_sum * sqrt(1 + (k1_p_sum*k1_p_sum) );
//            double k2_p = -GRAVITY / (k1_u_sum*k1_u_sum);
//            double k2_u_sum = u + k2_u * (delta_x / 2);
//            double k2_p_sum = p + k2_p * (delta_x / 2);

//            double k3_u = -k * k2_u_sum * sqrt(1 + (k2_p_sum*k2_p_sum) );
//            double k3_p = -GRAVITY / (k2_u_sum*k2_u_sum);
//            double k3_u_sum = u + k3_u * (delta_x / 2);
//            double k3_p_sum = p + k3_p * (delta_x / 2);

//            double k4_u = -k * k3_u_sum * sqrt(1 + (k3_p_sum*k3_p_sum));
//            double k4_p = -GRAVITY / (k3_u_sum*k3_u_sum);

//            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
//            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

//            x += delta_x;
//            y += p * delta_x;
//        }

//        double error = vertical - y;

//        // 如果误差满足停止条件，则跳出迭代
//        if (fabs(error) <= 0.00001) {
//            break;
//        } else {
//            temp_vertical += error;
//            pitch_new = atan2(temp_vertical, horizontal);
//        }
//    }

//    // 返回仰角修正值
//    return (pitch_new - pitch) * 180 / PI;
//}

//float st_k = 0.01903f;
//float t = 0.5f; // 飞行时间

//float monoDirectionalAirResistanceModel(float s, float v, float angle)
//{
//    float z;
//    //t为给定v与angle时的飞行时间
//    t = (float)((exp(st_k * s) - 1) / (st_k * v * cos(angle)));
//    if(t < 0)
//    {
//        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
//        //重置t，防止下次调用会出现nan
//        t = 0;
//        return 0;
//    }
//    //z为给定v与angle时的高度
//    z = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
//    return z;
//}

//float pitchTrajectoryCompensation(float s, float z, float v)
//{
//    float z_temp, z_actual, dz;
//    float angle_pitch;
//    int i = 0;
//    z_temp = z;
//    // iteration
//    for (i = 0; i < 20; i++)
//    {
//        angle_pitch = atan2(z_temp, s); // rad
//        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
//        if(z_actual == 0)
//        {
//            angle_pitch = 0;
//            break;
//        }
//        dz = 0.3*(z - z_actual);
//        z_temp = z_temp + dz;
//        if (fabsf(dz) < 0.00001)
//        {
//            break;
//        }
//    }
//    return angle_pitch;
//}


///*旋转矩阵*/
//void applyRotation(const RotationMatrix_t *rotationMatrix, ReceiveAutoaimData_t *info) {
//    // Define matrices A and B
//    arm_matrix_instance_f32 matA, matB, matResult;
//    float32_t A_data[3][3], B_data[3][1], result_data[3][1];

//    // Copy rotation matrix data to matrix A
//    for (int i = 0; i < 3; i++) {
//        for (int j = 0; j < 3; j++) {
//            A_data[i][j] = rotationMatrix->r[i][j];
//        }
//    }

//    // Copy pose data to matrix B
//    B_data[0][0] = info->data.x;
//    B_data[1][0] = info->data.y;
//    B_data[2][0] = info->data.z;

//    // Initialize matrix structures
//    arm_mat_init_f32(&matA, 3, 3, (float32_t *)A_data);
//    arm_mat_init_f32(&matB, 3, 1, (float32_t *)B_data);
//    arm_mat_init_f32(&matResult, 3, 1, (float32_t *)result_data);

//    // Perform matrix multiplication: Result = A * B
//    arm_mat_mult_f32(&matA, &matB, &matResult);

//    // Copy result back to pose
//    info->data.x = result_data[0][0];
//    info->data.y = result_data[1][0];
//    info->data.z = result_data[2][0];
//}



