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
#include "motion_planner.h"
#include "fish_controller.h"
#include "p8864_sensor.h"
#include "p8864_plane_solver.h"
#include "network_app.h"
#include "log.h"
#include "atk_ms53l1m.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern double channel_value[10];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
p8864_data_t pdata_buffer;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MainProcessTask */
osThreadId_t MainProcessTaskHandle;
const osThreadAttr_t MainProcessTask_attributes = {
  .name = "MainProcessTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for LidarProcessTas */
osThreadId_t LidarProcessTasHandle;
const osThreadAttr_t LidarProcessTas_attributes = {
  .name = "LidarProcessTas",
  .stack_size = 10000 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for LidarInquiryTas */
osThreadId_t LidarInquiryTasHandle;
const osThreadAttr_t LidarInquiryTas_attributes = {
  .name = "LidarInquiryTas",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for MainProcessTimer */
osTimerId_t MainProcessTimerHandle;
const osTimerAttr_t MainProcessTimer_attributes = {
  .name = "MainProcessTimer"
};
/* Definitions for MainProcessSem */
osSemaphoreId_t MainProcessSemHandle;
const osSemaphoreAttr_t MainProcessSem_attributes = {
  .name = "MainProcessSem"
};
/* Definitions for LidarDataReady */
osEventFlagsId_t LidarDataReadyHandle;
const osEventFlagsAttr_t LidarDataReady_attributes = {
  .name = "LidarDataReady"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void LidarInquiryTaskEntry(void *argument);
void Callback01(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);
void vApplicationDaemonTaskStartupHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
   Error_Handler();
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
    Error_Handler();
}
/* USER CODE END 5 */

/* USER CODE BEGIN DAEMON_TASK_STARTUP_HOOK */
void vApplicationDaemonTaskStartupHook(void)
{
}
/* USER CODE END DAEMON_TASK_STARTUP_HOOK */

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

  /* Create the semaphores(s) */
  /* creation of MainProcessSem */
  MainProcessSemHandle = osSemaphoreNew(1, 0, &MainProcessSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of MainProcessTimer */
  MainProcessTimerHandle = osTimerNew(Callback01, osTimerPeriodic, NULL, &MainProcessTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of MainProcessTask */
  MainProcessTaskHandle = osThreadNew(StartTask02, NULL, &MainProcessTask_attributes);

  /* creation of LidarProcessTas */
  LidarProcessTasHandle = osThreadNew(StartTask03, NULL, &LidarProcessTas_attributes);

  /* creation of LidarInquiryTas */
  LidarInquiryTasHandle = osThreadNew(LidarInquiryTaskEntry, NULL, &LidarInquiryTas_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of LidarDataReady */
  LidarDataReadyHandle = osEventFlagsNew(&LidarDataReady_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
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
    osDelay(10);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the MainProcessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
      uint32_t task_start_tick = osKernelGetTickCount();
      Fish_CPGUpdate();
      Fish_CPGToMotor();
      Motion_Update(&remora);
      osDelayUntil(task_start_tick+20);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the LidarProcessTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  uint16_t distance1, distance2;
  float neck_pitch = 0.0f, included_angle_raw;
  float disc_deg, disc_prev = 0.0f, disc_vel = 0.0f, disc_vel_prev = 0.0f, disc_acc = 0.0f;
  float height_cm, height_prev = 0.0f, height_vel = 0.0f, height_vel_prev = 0.0f, height_acc = 0.0f;
  uint8_t first_sample = 1;
  /* Infinite loop */
  for(;;)
  {
      uint32_t task_start_tick = osKernelGetTickCount();
      osEventFlagsWait(LidarDataReadyHandle, 1, osFlagsWaitAll, 0);
      osEventFlagsClear(LidarDataReadyHandle ,1);
//      memcpy(&pdata_buffer, &pdata, sizeof(p8864_data_t));
//      p8864_read_data();
//      p8864_plane_slove(&pdata_buffer, &pitch, &roll, &distance);
      //如果这里出bug了，报FE错误，MSL1多从机记得关掉错误返回
      if (atk_ms53l1m_modbus_get_data(2, &distance1) == ATK_MS53L1M_EOK &&
          atk_ms53l1m_modbus_get_data(3, &distance2) == ATK_MS53L1M_EOK)
      {
          included_angle_raw = atanf(fabsf((float)distance1+LIDAR_OFFSET-(float)distance2)/
                  LIDAR_RELATE_OFFSET);
          if (distance1+LIDAR_OFFSET<distance2)
              included_angle_raw = -included_angle_raw;

          disc_deg = included_angle_raw*180.0f/(float)PI;
          height_cm = (float)distance1*NN_DIST_SCALE;

          if (first_sample){
              disc_vel = 0.0f; disc_acc = 0.0f;
              height_vel = 0.0f; height_acc = 0.0f;
              first_sample = 0;
          } else {
              disc_vel = (disc_deg - disc_prev)/NN_SAMPLE_DT;
              disc_acc = (disc_vel - disc_vel_prev)/NN_SAMPLE_DT;
              height_vel = (height_cm - height_prev)/NN_SAMPLE_DT;
              height_acc = (height_vel - height_vel_prev)/NN_SAMPLE_DT;
          }
          disc_prev = disc_deg;       disc_vel_prev = disc_vel;
          height_prev = height_cm;    height_vel_prev = height_vel;

          neck_pitch = RemoraNN_6t1_run(disc_deg, (float)distance1,
                                        disc_vel, disc_acc, height_vel, height_acc);

          LOGLN_INFO("ms53l1", "dist1:%d, dist2:%d, angraw:%f, h:%f",
                     distance1, distance2, disc_deg, height_cm);
      }

      if (channel_value[4] == 1){
          remora.bumper_neck_pitch.dynamic=5;//改变运动是脖子的速度
          remora.bumper_neck_pitch.mode = BUMPER_MODE_POS;
          remora.bumper_neck_pitch.set = neck_pitch*NN_NECK_TO_BUMPER;
      }
      LOGLN_INFO("ai", "neck:%f", neck_pitch);
      //shared p/r/d
      osDelayUntil(task_start_tick+200);

  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_LidarInquiryTaskEntry */
/**
* @brief Function implementing the LidarInquiryTas thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LidarInquiryTaskEntry */
void LidarInquiryTaskEntry(void *argument)
{
  /* USER CODE BEGIN LidarInquiryTaskEntry */
  /* Infinite loop */
  for(;;)
  {
      osDelay(1000);
  }
  /* USER CODE END LidarInquiryTaskEntry */
}

/* Callback01 function */
void Callback01(void *argument)
{
  /* USER CODE BEGIN Callback01 */

  /* USER CODE END Callback01 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

