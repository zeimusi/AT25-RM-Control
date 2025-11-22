#include "Time.h"
#include "Variate.h"
eTime Time;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }

	if(htim == &htim3)  //判断中断是否来自于定时器3     1000hz
   {
		 if(GimbalInitFlag == 1) Time.GimbalInit++;
		 if(StuckFlag == 1) Time.ShootStuck++;
		 Time.Single++;

   }
}