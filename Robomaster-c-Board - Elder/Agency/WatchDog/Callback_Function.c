#include "Callback_Function.h"
#include "WatchDog.h"

/* 喂狗回调函数 */
void FeedDog_CallBack(WatchDogp handle)
{
  /*
  ID
  1    WatchDog_Init(&Remote_Dog,30);
  2    WatchDog_Init(&IMU_Dog,15);
  3    WatchDog_Init(&Gimbal_Dog[YAW],10);
  4    WatchDog_Init(&Gimbal_Dog[PITCH],10);
  5    WatchDog_Init(&Shoot_Dog[Left],10);
  6    WatchDog_Init(&Shoot_Dog[Right],10);
  7    WatchDog_Init(&Pluck_Dog,10);
  8    WatchDog_Init(&Down_Dog,15);
  9    WatchDog_Init(&PC_Dog,50);
	10   WatchDog_Init(&Referee_Dog, 50);

  */
  switch (handle->ID)
  {

      case 1:
            if (REMOTE_IfDataError() == osError){
//			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,15000);
              DeviceState.Remote_State = Device_Error;
            } else {
              DeviceState.Remote_State = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
            }
        break;

      case 2:
            if (IMU_IfDataError() == osError){
//			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,15000);
              DeviceState.IMU_State = Device_Error;
            } else {
              DeviceState.IMU_State = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0);
            }
        break;

      case 3:
            if (GM6020_Motor_Temp(&Gimbal_Motor[YAW]) == osError){
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,15000);
              DeviceState.Gimbal_State[YAW] = Device_Error;
            } else {
              DeviceState.Gimbal_State[YAW] = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,0);
            }
        break;

      case 4:
            if (GM6020_Motor_Temp(&Gimbal_Motor[PITCH]) == osError || DM4310_Motor_Temp(&DM4310_Pitch) == osError) {
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,15000);
              DeviceState.Gimbal_State[PITCH] = Device_Error;
            } else {
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,0);
              DeviceState.Gimbal_State[PITCH] = Device_Online;
            }
        break;

      case 5:
           if (RM3508_Motor_Temp(&Shoot_Motor[LEFT]) == osError){
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,15000);
              DeviceState.Shoot_State[LEFT] = Device_Error;
            } else {
              DeviceState.Shoot_State[LEFT] = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,0);
            }
        break;

      case 6:
            if (RM3508_Motor_Temp(&Shoot_Motor[RIGHT]) == osError){
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,15000);
              DeviceState.Shoot_State[RIGHT] = Device_Error;
            } else {
              DeviceState.Shoot_State[RIGHT] = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,0);
            }
        break;

      case 7:
          DeviceState.Pluck_State = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,0);
        break;

      case 8:
            DeviceState.Down_State = Device_Online;
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,0);
        break;

      case 9:
            DeviceState.PC_State = Device_Online;
        break;
			case 10:
				    DeviceState.Referee_State = Device_Online;
				break;
  }
}

/* 饿死回调函数 */
void WatchDog_CallBack(WatchDogp handle)
{
  /*
  ID
  1    WatchDog_Init(&Remote_Dog,30)
  2    WatchDog_Init(&IMU_Dog,15);
  3    WatchDog_Init(&Gimbal_Dog[YAW],10);
  4    WatchDog_Init(&Gimbal_Dog[PITCH],10);
  5    WatchDog_Init(&Shoot_Dog[Left],10);
  6    WatchDog_Init(&Shoot_Dog[Right],10);
  7    WatchDog_Init(&Pluck_Dog,10);
  8    WatchDog_Init(&Down_Dog,15);
  9    WatchDog_Init(&PC_Dog,50);
  */
  switch (handle->ID)
  {

  case 1:
        DeviceState.Remote_State = Device_Offline;
//			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,15000);
    break;

  case 2:
//			  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,15000);
        DeviceState.IMU_State = Device_Offline;
    break;

  case 3:
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,15000);
        DeviceState.Gimbal_State[YAW] = Device_Offline;
    break;

  case 4:
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1,15000);
        DeviceState.Gimbal_State[PITCH] = Device_Offline;
    break;

  case 5:
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,15000);
        DeviceState.Shoot_State[LEFT] = Device_Offline;
    break;

  case 6:
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,15000);
        DeviceState.Shoot_State[RIGHT] = Device_Offline;
    break;

  case 7:
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2,15000);
        DeviceState.Pluck_State = Device_Offline;
    break;

  case 8:
//			  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3,15000);
        DeviceState.Down_State = Device_Offline;
    break;

  case 9:
//
        DeviceState.PC_State = Device_Offline;
//        Aim_Rx.Rx_State = TIMESTAMP;
//        AimAction = AIM_STOP;
    break;
	case 10:
        DeviceState.Referee_State = Device_Offline;
	  break;
  }
}
