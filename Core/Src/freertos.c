/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
/* Definitions for Task_SafetyWatc */
osThreadId_t Task_SafetyWatcHandle;
const osThreadAttr_t Task_SafetyWatc_attributes = {
  .name = "Task_SafetyWatc",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime5,
};
/* Definitions for Task_Detector */
osThreadId_t Task_DetectorHandle;
const osThreadAttr_t Task_Detector_attributes = {
  .name = "Task_Detector",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_Gripper */
osThreadId_t Task_GripperHandle;
const osThreadAttr_t Task_Gripper_attributes = {
  .name = "Task_Gripper",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for Task_MicroROS */
osThreadId_t Task_MicroROSHandle;
const osThreadAttr_t Task_MicroROS_attributes = {
  .name = "Task_MicroROS",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskDetector(void *argument);
void StartTaskGripper(void *argument);
void StartTaskMicroROS(void *argument);

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
  /* creation of Task_SafetyWatc */
  Task_SafetyWatcHandle = osThreadNew(StartDefaultTask, NULL, &Task_SafetyWatc_attributes);

  /* creation of Task_Detector */
  Task_DetectorHandle = osThreadNew(StartTaskDetector, NULL, &Task_Detector_attributes);

  /* creation of Task_Gripper */
  Task_GripperHandle = osThreadNew(StartTaskGripper, NULL, &Task_Gripper_attributes);

  /* creation of Task_MicroROS */
  Task_MicroROSHandle = osThreadNew(StartTaskMicroROS, NULL, &Task_MicroROS_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the Task_SafetyWatc thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskDetector */
/**
* @brief Function implementing the Task_Detector thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskDetector */
void StartTaskDetector(void *argument)
{
  /* USER CODE BEGIN StartTaskDetector */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskDetector */
}

/* USER CODE BEGIN Header_StartTaskGripper */
/**
* @brief Function implementing the Task_Gripper thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskGripper */
void StartTaskGripper(void *argument)
{
  /* USER CODE BEGIN StartTaskGripper */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskGripper */
}

/* USER CODE BEGIN Header_StartTaskMicroROS */
/**
* @brief Function implementing the Task_MicroROS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMicroROS */
void StartTaskMicroROS(void *argument)
{
  /* USER CODE BEGIN StartTaskMicroROS */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTaskMicroROS */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

