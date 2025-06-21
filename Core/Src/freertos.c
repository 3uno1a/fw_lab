/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for buttonTask */
osThreadId_t buttonTaskHandle;
const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledQueue */
osMessageQueueId_t ledQueueHandle;
const osMessageQueueAttr_t ledQueue_attributes = {
  .name = "ledQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartButtonTask(void *argument);
void StartLedTask(void *argument);

extern void MX_USB_HOST_Init(void);
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

  /* Create the queue(s) */
  /* creation of ledQueue */
  ledQueueHandle = osMessageQueueNew (4, sizeof(uint16_t), &ledQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of buttonTask */
  buttonTaskHandle = osThreadNew(StartButtonTask, NULL, &buttonTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(StartLedTask, NULL, &ledTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  /* init code for USB_HOST */
  MX_USB_HOST_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartButtonTask */
/**
* @brief Function implementing the buttonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void *argument)
{
  /* USER CODE BEGIN StartButtonTask */
  uint8_t btnState = 0;
  uint8_t prevBtnState = 0;
  uint32_t pressedTime = 0;

  /* Infinite loop */
  for(;;)
  {
    uint8_t stableCount = 0;

    for (uint8_t i = 0; i < 5; i++)
    {
      if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)
      {
        stableCount ++;
        osDelay(10);
      }
    }

    btnState = (stableCount >= 4) ? GPIO_PIN_SET : GPIO_PIN_RESET;        // btn debouncing

    if (btnState == GPIO_PIN_SET && prevBtnState == GPIO_PIN_RESET)
    {
      pressedTime = HAL_GetTick();
    }

    if (btnState == GPIO_PIN_SET && (HAL_GetTick() - pressedTime >= 2000))
    {
      LedCommand cmd = LED_BLINK_START;
      osMessageQueuePut(ledQueueHandle, &cmd, 0, 0);
    }

    if (btnState == GPIO_PIN_RESET && prevBtnState == GPIO_PIN_SET)
    {
      if (HAL_GetTick() - pressedTime < 2000)
      {
        LedCommand cmd = LED_TOGGLE;       // Green
        osMessageQueuePut(ledQueueHandle, &cmd, 0, 0);
      }
      else
      {
        LedCommand cmd = LED_BLINK_STOP;
        osMessageQueuePut(ledQueueHandle, &cmd, 0, 0);
      }
    }
    prevBtnState = btnState;
    osDelay(10);
  }
  /* USER CODE END StartButtonTask */
}

/* USER CODE BEGIN Header_StartLedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedTask */
void StartLedTask(void *argument)
{
  /* USER CODE BEGIN StartLedTask */
  LedCommand cmd;
  uint8_t blinking = 0;

  /* Infinite loop */
  for(;;)
  {
    if (osMessageQueueGet(ledQueueHandle, &cmd, NULL, blinking ? 50 : osWaitForever) == osOK)
    {
      switch(cmd)
      {
        case LED_ON:
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);    // orange
          blinking = 0;
          break;

        case LED_OFF:
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
          blinking = 0;
          break;

        case LED_TOGGLE:
          HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);      // green
          blinking = 0;
          break;

        case LED_BLINK_START:
           blinking = 1;
           break;

        case LED_BLINK_STOP:
          blinking = 0;
          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);    // b;ue
          break;
      }
    }
    if (blinking)
    {
      HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
      osDelay(50);
    }
  }
  /* USER CODE END StartLedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

