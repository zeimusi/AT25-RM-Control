/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for INSTask */
osThreadId_t INSTaskHandle;
const osThreadAttr_t INSTask_attributes = {
  .name = "INSTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Init_Task */
osThreadId_t Init_TaskHandle;
const osThreadAttr_t Init_Task_attributes = {
  .name = "Init_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void INS_Init();
void INS_Task();
void Init_Task();
/* USER CODE END FunctionPrototypes */

void StartINSTask(void *argument);
void StartInit_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of INSTask */
  INSTaskHandle = osThreadNew(StartINSTask, NULL, &INSTask_attributes);

  /* creation of Init_Task */
  Init_TaskHandle = osThreadNew(StartInit_Task, NULL, &Init_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartINSTask */
/**
  * @brief  Function implementing the INSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartINSTask */
void StartINSTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartINSTask */
INS_Init();
  /* Infinite loop */
  for(;;)
  {
	  INS_Task();
    osDelay(1);
  }
  /* USER CODE END StartINSTask */
}

/* USER CODE BEGIN Header_StartInit_Task */
/**
* @brief Function implementing the Init_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartInit_Task */
void StartInit_Task(void *argument)
{
  /* USER CODE BEGIN StartInit_Task */
	  MX_USB_DEVICE_Init();
     Init_Task();
  /* Infinite loop */
  /* USER CODE END StartInit_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

