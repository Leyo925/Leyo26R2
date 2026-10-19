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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
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
/* Definitions for Chassis */
osThreadId_t ChassisHandle;
uint32_t ChassisBuffer[ 256 ];
osStaticThreadDef_t ChassisControlBlock;
const osThreadAttr_t Chassis_attributes = {
  .name = "Chassis",
  .cb_mem = &ChassisControlBlock,
  .cb_size = sizeof(ChassisControlBlock),
  .stack_mem = &ChassisBuffer[0],
  .stack_size = sizeof(ChassisBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Diver */
osThreadId_t DiverHandle;
uint32_t DiverBuffer[ 128 ];
osStaticThreadDef_t DiverControlBlock;
const osThreadAttr_t Diver_attributes = {
  .name = "Diver",
  .cb_mem = &DiverControlBlock,
  .cb_size = sizeof(DiverControlBlock),
  .stack_mem = &DiverBuffer[0],
  .stack_size = sizeof(DiverBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void My_Chassis(void *argument);
void Moto_Diver(void *argument);

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
  /* creation of Chassis */
  ChassisHandle = osThreadNew(My_Chassis, NULL, &Chassis_attributes);

  /* creation of Diver */
  DiverHandle = osThreadNew(Moto_Diver, NULL, &Diver_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_My_Chassis */
/**
  * @brief  Function implementing the Chassis thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_My_Chassis */
__weak void My_Chassis(void *argument)
{
  /* USER CODE BEGIN My_Chassis */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END My_Chassis */
}

/* USER CODE BEGIN Header_Moto_Diver */
/**
* @brief Function implementing the Diver thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Moto_Diver */
__weak void Moto_Diver(void *argument)
{
  /* USER CODE BEGIN Moto_Diver */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Moto_Diver */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

