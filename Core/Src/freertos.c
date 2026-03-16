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
/* Definitions for OBSERVE_Task_St */
osThreadId_t OBSERVE_Task_StHandle;
const osThreadAttr_t OBSERVE_Task_St_attributes = {
  .name = "OBSERVE_Task_St",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ARM_Task_Name */
osThreadId_t ARM_Task_NameHandle;
const osThreadAttr_t ARM_Task_Name_attributes = {
  .name = "ARM_Task_Name",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal7,
};
/* Definitions for INS_Task_Start */
osThreadId_t INS_Task_StartHandle;
const osThreadAttr_t INS_Task_Start_attributes = {
  .name = "INS_Task_Start",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal3,
};
/* Definitions for PS2_Task_Start */
osThreadId_t PS2_Task_StartHandle;
const osThreadAttr_t PS2_Task_Start_attributes = {
  .name = "PS2_Task_Start",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal5,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void OBSERVE_Task(void *argument);
void ARM_Task(void *argument);
void INS_Task(void *argument);
void PS2_Task(void *argument);

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
  /* creation of OBSERVE_Task_St */
  OBSERVE_Task_StHandle = osThreadNew(OBSERVE_Task, NULL, &OBSERVE_Task_St_attributes);

  /* creation of ARM_Task_Name */
  ARM_Task_NameHandle = osThreadNew(ARM_Task, NULL, &ARM_Task_Name_attributes);

  /* creation of INS_Task_Start */
  INS_Task_StartHandle = osThreadNew(INS_Task, NULL, &INS_Task_Start_attributes);

  /* creation of PS2_Task_Start */
  PS2_Task_StartHandle = osThreadNew(PS2_Task, NULL, &PS2_Task_Start_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_OBSERVE_Task */
/**
  * @brief  Function implementing the OBSERVE_Task_St thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_OBSERVE_Task */
__weak void OBSERVE_Task(void *argument)
{
  /* USER CODE BEGIN OBSERVE_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OBSERVE_Task */
}

/* USER CODE BEGIN Header_ARM_Task */
/**
* @brief Function implementing the ARM_Task_Name thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ARM_Task */
__weak void ARM_Task(void *argument)
{
  /* USER CODE BEGIN ARM_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ARM_Task */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the INS_Task_Start thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
__weak void INS_Task(void *argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_PS2_Task */
/**
* @brief Function implementing the PS2_Task_Start thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PS2_Task */
__weak void PS2_Task(void *argument)
{
  /* USER CODE BEGIN PS2_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END PS2_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

