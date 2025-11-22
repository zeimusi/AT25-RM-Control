#include "Task_Music.h"

#include "bsp_buzzer.h"
#include "cmsis_os.h"
#include "data_exchange.h"
#include "music.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t music_high_water;
#endif

#define STEP_INIT 1
#define STEP_NORMAL 2

#define is_play_cali()                                                                 \
    (is_play == CALI_BEGIN || is_play == CALI_MIDDLE_TIME || is_play == CALI_GIMBAL || \
     is_play == CALI_IMU || is_play == CALI_CHASSIS)
	 
#define cali_buzzer_begin() buzzer_on(50, 10000)   // 蜂鸣器的设置频率和强度
#define cali_buzzer_middle() buzzer_on(20, 10000)  // 蜂鸣器的设置频率和强度
#define cali_buzzer_gimbal() buzzer_on(30, 19999)  // 当云台在校准,蜂鸣器的设置频率和强度
#define cali_buzzer_imu() buzzer_on(60, 19999)  // 当imu在校准,蜂鸣器的设置频率和强度
#define cali_buzzer_chassis() buzzer_on(100, 19999)  // 当底盘在校准,蜂鸣器的设置频率和强度
#define cali_buzzer_off() buzzer_off()               // buzzer off，关闭蜂鸣器



extern uint32_t play_id;       // Index of the note to be played
void Music_Task(void const * pvParameters)
{
	static int16_t tim = 0;
    // 空闲一段时间
    vTaskDelay(MUSIC_TASK_INIT_TIME);
    // 初始化音乐
    MusicStartInit();
     static portTickType currentTime;
	   for(;;){
			currentTime = xTaskGetTickCount();		
      tim++;			 
      MusicStartPlay();
			if(tim >= 900){
      buzzer_off();
			osThreadSuspend(Music_Task_handle);			 
			}
			vTaskDelayUntil(&currentTime, 1);		 

			}
}

