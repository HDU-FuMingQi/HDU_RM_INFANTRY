/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
osThreadId ShowRunningTaskHandle;
osThreadId ShowOLEDTaskHandle;
osThreadId GimbalTaskHandle;
osThreadId ChassisTaskHandle;
osThreadId ShootTaskHandle;
osThreadId OutControl_TaskHandle;
osThreadId IMUTaskHandle;
osThreadId hero_shootHandle;
osTimerId Get6020SpeedTimerHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void ShowRunningFun(void const * argument);
void ShowOLEDFun(void const * argument);
void GimbalFun(void const * argument);
void ChassisFun(void const * argument);
void ShootFun(void const * argument);
void OutControl_Fun(void const * argument);
void IMUFun(void const * argument);
void hero_shoot_fun(void const * argument);
void Get6020SpeedCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];
  
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )  
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}                   
/* USER CODE END GET_TIMER_TASK_MEMORY */

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

  /* Create the timer(s) */
  /* definition and creation of Get6020SpeedTimer */
  osTimerDef(Get6020SpeedTimer, Get6020SpeedCallback);
  Get6020SpeedTimerHandle = osTimerCreate(osTimer(Get6020SpeedTimer), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	osTimerStart(Get6020SpeedTimerHandle,10);//10毫秒钟启动一次定时
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
/////////////////////////创建开始任务任务函数///////////////////////////	
  /* USER CODE END RTOS_QUEUES */

	////////////////////创建 失联保护 任务，实时任务，/////////////////////////
  /* Create the thread(s) */
  /* definition and creation of ShowRunningTask */
  osThreadDef(ShowRunningTask, ShowRunningFun, osPriorityRealtime, 0, 128);
  ShowRunningTaskHandle = osThreadCreate(osThread(ShowRunningTask), NULL);

  ////////////////////创建 OLED 任务，实时任务，/////////////////////////
  /* definition and creation of ShowOLEDTask */
  osThreadDef(ShowOLEDTask, ShowOLEDFun, osPriorityIdle, 0, 128);
  ShowOLEDTaskHandle = osThreadCreate(osThread(ShowOLEDTask), NULL);

  /////////////////////创建 云台 任务，实时任务，/////////////////////////
  /* definition and creation of GimbalTask */
  osThreadDef(GimbalTask, GimbalFun, osPriorityHigh, 0, 128);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

   //////////////////////创建 底盘 任务，实时任务，/////////////////////////
  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, ChassisFun, osPriorityHigh, 0, 128);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

	//////////////////////创建 射击 任务，实时任务，/////////////////////////
  /* definition and creation of ShootTask */
  osThreadDef(ShootTask, ShootFun, osPriorityHigh, 0, 128);
  ShootTaskHandle = osThreadCreate(osThread(ShootTask), NULL);

  ////////////////////创建 失控保护 任务，实时任务，/////////////////////////
  /* definition and creation of OutControl_Task */
  osThreadDef(OutControl_Task, OutControl_Fun, osPriorityAboveNormal, 0, 128);
  OutControl_TaskHandle = osThreadCreate(osThread(OutControl_Task), NULL);

  ////////////////////创建 加载陀螺仪 任务，实时任务，/////////////////////////
  /* definition and creation of IMUTask */
  osThreadDef(IMUTask, IMUFun, osPriorityRealtime, 0, 128);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of hero_shoot */
  osThreadDef(hero_shoot, hero_shoot_fun, osPriorityHigh, 0, 128);
  hero_shootHandle = osThreadCreate(osThread(hero_shoot), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_ShowRunningFun */
/**
  * @brief  Function implementing the ShowRunningTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_ShowRunningFun */
__weak void ShowRunningFun(void const * argument)
{
  /* USER CODE BEGIN ShowRunningFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ShowRunningFun */
}

/* USER CODE BEGIN Header_ShowOLEDFun */
/**
* @brief Function implementing the ShowOLEDTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ShowOLEDFun */
__weak void ShowOLEDFun(void const * argument)
{
  /* USER CODE BEGIN ShowOLEDFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ShowOLEDFun */
}

/* USER CODE BEGIN Header_GimbalFun */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GimbalFun */
__weak void GimbalFun(void const * argument)
{
  /* USER CODE BEGIN GimbalFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END GimbalFun */
}

/* USER CODE BEGIN Header_ChassisFun */
/**
* @brief Function implementing the ChassisTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ChassisFun */
__weak void ChassisFun(void const * argument)
{
  /* USER CODE BEGIN ChassisFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ChassisFun */
}

/* USER CODE BEGIN Header_ShootFun */
/**
* @brief Function implementing the ShootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ShootFun */
__weak void ShootFun(void const * argument)
{
  /* USER CODE BEGIN ShootFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ShootFun */
}

/* USER CODE BEGIN Header_OutControl_Fun */
/**
* @brief Function implementing the OutControl_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OutControl_Fun */
__weak void OutControl_Fun(void const * argument)
{
  /* USER CODE BEGIN OutControl_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OutControl_Fun */
}

/* USER CODE BEGIN Header_IMUFun */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMUFun */
__weak void IMUFun(void const * argument)
{
  /* USER CODE BEGIN IMUFun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMUFun */
}

/* USER CODE BEGIN Header_hero_shoot_fun */
/**
* @brief Function implementing the hero_shoot_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_hero_shoot_fun */
__weak void hero_shoot_fun(void const * argument)
{
  /* USER CODE BEGIN hero_shoot_fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END hero_shoot_fun */
}

/* Get6020SpeedCallback function */
__weak void Get6020SpeedCallback(void const * argument)
{
  /* USER CODE BEGIN Get6020SpeedCallback */
  
  /* USER CODE END Get6020SpeedCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
