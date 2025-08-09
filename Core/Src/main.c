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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_i2c.h"
#include "mpu6050.h"
#include "loop_profiler.h"
#include "gps_time.h"
#include "nmea_min.h"
#include <stdint.h>
#include <stdio.h>
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
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
MPU6050_Device g_mpu;
GpsTime gps_time;
static uint8_t uart2_rx;
static char nmea_buf[128];
static volatile uint16_t nmea_len = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void uart_broadcast(const char *text)
{
  if (huart1.Instance != NULL) {
    (void)HAL_UART_Transmit(&huart1, (uint8_t*)text, strlen(text), 200);
  }
  if (huart2.Instance != NULL) {
    (void)HAL_UART_Transmit(&huart2, (uint8_t*)text, strlen(text), 200);
  }
}

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
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Using HAL_GetTick() for time since boot; no counter needed */
  char msg[192];
  char boot[160];
  snprintf(boot, sizeof(boot),
           "\r\nBOOT sys=%lu Hz hclk=%lu Hz pclk1=%lu Hz pclk2=%lu Hz\r\n",
           (unsigned long)HAL_RCC_GetSysClockFreq(),
           (unsigned long)HAL_RCC_GetHCLKFreq(),
           (unsigned long)HAL_RCC_GetPCLK1Freq(),
           (unsigned long)HAL_RCC_GetPCLK2Freq());
  uart_broadcast(boot);

  /* Initialize loop profiler for 100 Hz (10000 us) */
  LoopProfiler profiler;
  loop_profiler_init(&profiler, 10000u);
  gps_time_init(&gps_time);
  /* Start UART2 RX interrupt for NMEA (GPGGA) */
  HAL_UART_Receive_IT(&huart2, &uart2_rx, 1);
  for (uint8_t a = 0x03; a <= 0x77; a++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, a<<1, 1, 10) == HAL_OK) {
      sprintf(msg, "Found I2C addr 0x%02X\r\n", a);
      uart_broadcast(msg);
    }
  }
  /* If I2C1 is enabled/configured by CubeMX, try to detect MPU */
#ifdef HAL_I2C_MODULE_ENABLED
  if (mpu6050_detect(&g_mpu, &hi2c1) == HAL_OK)
  {
    (void)mpu6050_wake(&g_mpu);
    /* Allow clock to stabilize */
    HAL_Delay(100);
    /* Configure DLPF=3 (~44 Hz accel/gyro BW) and 100 Hz sample rate */
    (void)mpu6050_configure(&g_mpu,
                            MPU6050_ACCEL_FS_2G,
                            MPU6050_GYRO_FS_250DPS,
                            3, /* dlpf_cfg */
                            9  /* smplrt_div */);
    /* Discard first sample after wake/config */
    int16_t dx, dy, dz, dt, dgx, dgy, dgz;
    (void)mpu6050_read_raw(&g_mpu, &dx, &dy, &dz, &dt, &dgx, &dgy, &dgz);
    HAL_Delay(10);
    sprintf(msg, "MPU detected at 0x%02X\r\n", (unsigned)(g_mpu.address >> 1));
    uart_broadcast(msg);
  }
  else
  {
    sprintf(msg, "MPU not detected\r\n");
    uart_broadcast(msg);
  }
#else
  sprintf(msg, "I2C not enabled. Configure I2C1 in CubeMX.\r\n");
  uart_broadcast(msg);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    float ax, ay, az, t, gx, gy, gz;
    uint32_t now = HAL_GetTick();
    unsigned long sec = now / 1000UL;
    unsigned long ms  = now % 1000UL;
#ifdef HAL_I2C_MODULE_ENABLED
    loop_profiler_begin(&profiler);
    int64_t gps_sec; uint32_t gps_us; bool gps_ok = gps_time_now(&gps_time, &gps_sec, &gps_us);
    if (mpu6050_read_f(&g_mpu, &ax, &ay, &az, &t, &gx, &gy, &gz) == HAL_OK)
    {
      GpsTimeState st = gps_time_get_state(&gps_time);
      const char *st_str = gps_time_state_str(st);
      if (gps_ok) {
        snprintf(msg, sizeof(msg), "[CPU %lu.%03lu s | GPS %lld.%06u | %s] AX:%.3f g AY:%.3f g AZ:%.3f g TEMP:%.2f C GX:%.2f dps GY:%.2f dps GZ:%.2f dps\r\n",
                 sec, ms, (long long)gps_sec, (unsigned)gps_us, st_str,
                 ax, ay, az, t, gx, gy, gz);
      } else {
        snprintf(msg, sizeof(msg), "[CPU %lu.%03lu s | GPS ----.------ | %s] AX:%.3f g AY:%.3f g AZ:%.3f g TEMP:%.2f C GX:%.2f dps GY:%.2f dps GZ:%.2f dps\r\n",
                 sec, ms, st_str, ax, ay, az, t, gx, gy, gz);
      }
    }
    else
    {
      snprintf(msg, sizeof(msg), "[CPU %lu.%03lu s] I2C read error\r\n", sec, ms);
      uint32_t err = HAL_I2C_GetError(&hi2c1);
      snprintf(msg, sizeof(msg), "[CPU %lu.%03lu s] I2C error=0x%08lX\r\n", sec, ms, (unsigned long)err);
      uart_broadcast(msg);
    }
    loop_profiler_end(&profiler);
    
#else
    sprintf(msg, "[%lu.%03lu s] I2C not enabled\r\n", sec, ms);
#endif
    uart_broadcast(msg);

    /* Every 100 samples, print loop timing stats */
    uint32_t min_us, avg_us, max_us, last_us, slack_us; float load_pct;
    uint32_t cnt = loop_profiler_get_stats_us(&profiler, &min_us, &avg_us, &max_us, &last_us, &slack_us, &load_pct);
    if ((cnt % 100u) == 0u && cnt != 0u) {
      snprintf(msg, sizeof(msg), "loop_us min/avg/max/last=%u/%u/%u/%u, slack=%u, load=%.1f%%\r\n",
               min_us, avg_us, max_us, last_us, slack_us, load_pct);
      uart_broadcast(msg);
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : PPS_Pin */
  GPIO_InitStruct.Pin = pps_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(pps_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_4) { /* PA4 PPS */
    gps_time_on_pps_isr(&gps_time);
  }
}
/* Add UART2 RX complete callback to parse NMEA and arm UTC for next PPS */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2) {
    uint8_t byte = uart2_rx;
    if (nmea_len < (uint16_t)(sizeof(nmea_buf) - 1)) {
      nmea_buf[nmea_len++] = (char)byte;
    } else {
      nmea_len = 0; /* overflow, reset */
    }
    if (byte == '\n') {
      if (nmea_len > 0) nmea_buf[nmea_len] = '\0';
      if (nmea_len > 0 && nmea_buf[nmea_len-1] == '\r') nmea_buf[nmea_len-1] = '\0';
      int hh, mm, ss, us;
      if (nmea_len > 10 && nmea_buf[0] == '$' &&
          nmea_parse_gga_time(nmea_buf, &hh, &mm, &ss, &us)) {
        int64_t sec_of_day = (int64_t)hh * 3600 + (int64_t)mm * 60 + (int64_t)ss;
        gps_time_set_utc_on_next_pps(&gps_time, sec_of_day);
      }
      nmea_len = 0;
    }
    HAL_UART_Receive_IT(&huart2, &uart2_rx, 1);
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
