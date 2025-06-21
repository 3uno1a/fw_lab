/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CS_LOW() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)  // Start SPI communication
#define CS_HIGH() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)   // End SPI communication

#define CS43L22_I2C_ADDR 0x94
#define AUDIO_BUF_SIZE 256
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char buffer[100];
uint8_t idx = 0;
uint8_t uart_rx;

uint16_t audioBuffer[AUDIO_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
void LIS3DSH_Init(void);
void LIS3DSH_WhoAmI(void);
void LIS3DSH_ReadAxes(uint16_t* x, uint16_t* y, uint16_t* z);
uint8_t LIS3DSH_ReadRegister(uint8_t reg);

void CS43L22_Init(I2C_HandleTypeDef *hi2c);
void Generate_SineWave(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart2, &uart_rx, 1);


  CS_HIGH();

  LIS3DSH_WhoAmI();
  LIS3DSH_Init();

  CS43L22_Init(&hi2c1);
  Generate_SineWave();
  HAL_I2S_Transmit_DMA(&hi2s3, audioBuffer, AUDIO_BUF_SIZE);

  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    uint16_t x,y,z;
    LIS3DSH_ReadAxes(&x, &y, &z);

    printf("X: %d, Y: %d, Z: %d\r\n", x, y, z);
    HAL_Delay(500);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart -> Instance == USART2)
  {
    if(uart_rx != '\r' && uart_rx != '\n')
    {
      if(idx < sizeof(buffer) - 1)
      {
        buffer[idx++] = uart_rx;
      }
    }
    else
    {
      buffer[idx] = '\0';
      printf("received msg:%s\r\n", buffer);

      if (strcmp(buffer, "on") == 0)
      {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
      }
      else if (strcmp(buffer, "off") == 0)
      {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
      }
      idx = 0;
    }
    HAL_UART_Receive_IT(&huart2, &uart_rx, 1);
  }
}

void LIS3DSH_Init(void)
{
  uint8_t tx[2];

  tx[0] = 0x20;     // CTRL_REG4 - set Output Data Rate & xyz enable
  tx[1] = 0x67;     // 0110 0111 - ODR: 100Hz, XYZ on

  CS_LOW();
  HAL_SPI_Transmit(&hspi1, tx, 2, HAL_MAX_DELAY);
  CS_HIGH();
}


uint8_t LIS3DSH_ReadRegister(uint8_t reg)
{
  uint8_t tx[2], rx[2];

  tx[0] = reg | 0x80;     // read mode MSB 1
  tx[1] = 0x00;           // data recv dummy byte

  CS_LOW();
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, HAL_MAX_DELAY);
  CS_HIGH();

  // printf("TX: 0x%02X, RX0: 0x%02X, RX1: 0x%02X\r\n", tx[0], rx[0], rx[1]);

  return rx[1];
}


void LIS3DSH_WhoAmI(void)       // check sensor(LIS3DSH) connection
{
  uint8_t who_am_i = LIS3DSH_ReadRegister(0x0F);      // 0x0F = who am i reg
  printf("WHO_AM_I: 0x%02X\r\n", who_am_i);           // 0x3F -> LIS3DSH

  if (who_am_i == 0x3F)
  {
      printf("LIS3DSH detected successfully!\r\n");
  }
  else
  {
      printf("LIS3DSH not detected / SPI communication failed.\r\n");
  }
}

void LIS3DSH_ReadAxes(uint16_t* x, uint16_t* y, uint16_t* z)
{
//  uint8_t tx[7] = {0};
//  uint8_t rx[7] = {0};
//
//  tx[0] = 0x28 | 0x80 | 0x40;  // Read, auto-increment
//
//  CS_LOW();
//  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 7, HAL_MAX_DELAY);
//  CS_HIGH();
//
//  *x = (uint16_t)((rx[2] << 8) | rx[1]);
//  *y = (uint16_t)((rx[4] << 8) | rx[3]);
//  *z = (uint16_t)((rx[6] << 8) | rx[5]);

  uint8_t xl = LIS3DSH_ReadRegister(0x28);
  uint8_t xh = LIS3DSH_ReadRegister(0x29);

  uint8_t yl = LIS3DSH_ReadRegister(0x2A);
  uint8_t yh = LIS3DSH_ReadRegister(0x2B);

  uint8_t zl = LIS3DSH_ReadRegister(0x2C);
  uint8_t zh = LIS3DSH_ReadRegister(0x2D);

  *x = (uint16_t)((xh << 8) | xl);
  *y = (uint16_t)((yh << 8) | yl);
  *z = (uint16_t)((zh << 8) | zl);
}

void CS43L22_Init(I2C_HandleTypeDef *hi2c)
{
  uint8_t data[2];

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);      // Reset pin
  HAL_Delay(10);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);    // Mute pin

  // Power Control 1: disable power down
  data[0] = 0x02;   // Register address
  data[1] = 0x01;   // Power ON
  HAL_I2C_Mem_Write(hi2c, CS43L22_I2C_ADDR, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);

  // Clocking Control
  data[0] = 0x05;
  data[1] = 0x81;
  HAL_I2C_Mem_Write(hi2c, CS43L22_I2C_ADDR, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);

  // Interface Control 1
  data[0] = 0x06;
  data[1] = 0x07;  // I2S, 16-bit data
  HAL_I2C_Mem_Write(hi2c, CS43L22_I2C_ADDR, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);

  // Passthrough A & B
  data[0] = 0x0A;
  data[1] = 0x00;
  HAL_I2C_Mem_Write(hi2c, CS43L22_I2C_ADDR, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);

  // Power Ctl2: speaker ON
  data[0] = 0x04;
  data[1] = 0xAF;  // Headphone + Speaker enable
  HAL_I2C_Mem_Write(hi2c, CS43L22_I2C_ADDR, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);

  // Master Volume
  data[0] = 0x20;
  data[1] = 0x00;  // 0dB
  HAL_I2C_Mem_Write(hi2c, CS43L22_I2C_ADDR, data[0], I2C_MEMADD_SIZE_8BIT, &data[1], 1, HAL_MAX_DELAY);

  printf("DAC Init Done!\n");
}

void Generate_SineWave(void)
{
  for (int i = 0; i < AUDIO_BUF_SIZE; i++)
  {
    audioBuffer[i] = (int16_t)(10000 * sin(2 * 3.141592 * i / AUDIO_BUF_SIZE));
  }
}


void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s)
{
    if (hi2s->Instance == SPI3)
    {
        HAL_I2S_Transmit_DMA(hi2s, audioBuffer, AUDIO_BUF_SIZE);
    }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
