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
#include "sai.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "shell.h"
#include "drv_uart.h"
#include "leds.h"
#include "sgtl5000.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AUDIO_BLOCK_SIZE 24
#define AUDIO_BUFFER_SIZE (AUDIO_BLOCK_SIZE * 4)

// Defines for triangular wave generation
#define SAMPLE_RATE_HZ 48000
#define TRIANGLE_TEST_FREQ_HZ 440
#define TRIANGLE_MAX_AMPLITUDE 15000
#define TRIANGLE_STEP ((2 * TRIANGLE_MAX_AMPLITUDE) / (SAMPLE_RATE_HZ / TRIANGLE_TEST_FREQ_HZ))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
SemaphoreHandle_t uartRxSemaphore;
uint8_t rxCharBuffer;
h_sgtl5000_t sgtl5000_handle;

uint16_t audio_tx_buffer[AUDIO_BUFFER_SIZE];
uint16_t audio_rx_buffer[AUDIO_BUFFER_SIZE];

// Global variables for triangle wave generation
int16_t triangle_current_value = 0;
int8_t triangle_direction = 1; // 1 for rising, -1 for falling
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);

	return ch;
}


void ShellTask(void * unused)
{
	h_shell_t h_shell;
	drv_shell_t drv_shell;

	drv_shell.transmit = drv_uart_transmit;
	drv_shell.receive = drv_uart_receive;

	h_shell.drv = drv_shell;

	shell_init(&h_shell);

	// Init LED Driver and add command
	LED_Driver_Init(&led_driver);
	shell_add(&h_shell, 'l', shell_control_led, "Control LEDs: l <port> <pin> <state>");
	shell_add(&h_shell, 'k', shell_chenillard, "Chenillard effect");
	shell_add(&h_shell, 'b', shell_blink_all, "Blink all LEDs");

	shell_run(&h_shell);
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

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI3_Init();
  MX_I2C2_Init();
  MX_SAI2_Init();
  /* USER CODE BEGIN 2 */
  __HAL_SAI_ENABLE(&hsai_BlockA2);

  sgtl5000_handle.hi2c = &hi2c2;
  sgtl5000_handle.i2c_address = SGTL5000_I2C_ADDR_WRITE;
  sgtl5000_init(&sgtl5000_handle);

  // Start SAI DMA transfers
  HAL_SAI_Transmit_DMA(&hsai_BlockA2, (uint8_t*)audio_tx_buffer, AUDIO_BUFFER_SIZE);
  HAL_SAI_Receive_DMA(&hsai_BlockB2, (uint8_t*)audio_rx_buffer, AUDIO_BUFFER_SIZE);

  uartRxSemaphore = xSemaphoreCreateBinary();
  HAL_UART_Receive_IT(&huart2, &rxCharBuffer, 1);

  if (xTaskCreate(ShellTask, "Shell", 1024, NULL, 1, NULL) != pdPASS)
  {
	  printf("Task Creation Failed\r\n");
	  Error_Handler();
  }
  /* USER CODE END 2 */

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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SAI2;
  PeriphClkInit.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 13;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV17;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(uartRxSemaphore, &xHigherPriorityTaskWoken);

    HAL_UART_Receive_IT(&huart2, &rxCharBuffer, 1);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}

// SAI Transmit Half-Complete Callback
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI2_Block_A)
  {
    for (int i = 0; i < AUDIO_BLOCK_SIZE; i++) {
        triangle_current_value += triangle_direction * TRIANGLE_STEP;

        if (triangle_current_value >= TRIANGLE_MAX_AMPLITUDE) {
            triangle_current_value = TRIANGLE_MAX_AMPLITUDE; // Cap at max
            triangle_direction = -1; // Change direction to falling
        } else if (triangle_current_value <= -TRIANGLE_MAX_AMPLITUDE) {
            triangle_current_value = -TRIANGLE_MAX_AMPLITUDE; // Cap at min
            triangle_direction = 1; // Change direction to rising
        }
        uint16_t output_sample = (uint16_t)triangle_current_value;

        audio_tx_buffer[(2 * i)] = output_sample;     // Left Channel
        audio_tx_buffer[(2 * i) + 1] = output_sample; // Right Channel
    }
  }
}

// SAI Transmit Complete Callback
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI2_Block_A)
  {
    for (int i = 0; i < AUDIO_BLOCK_SIZE; i++) {
        triangle_current_value += triangle_direction * TRIANGLE_STEP;

        if (triangle_current_value >= TRIANGLE_MAX_AMPLITUDE) {
            triangle_current_value = TRIANGLE_MAX_AMPLITUDE;
            triangle_direction = -1;
        } else if (triangle_current_value <= -TRIANGLE_MAX_AMPLITUDE) {
            triangle_current_value = -TRIANGLE_MAX_AMPLITUDE;
            triangle_direction = 1;
        }

        uint16_t output_sample = (uint16_t)triangle_current_value;

        audio_tx_buffer[AUDIO_BLOCK_SIZE * 2 + (2 * i)] = output_sample;
        audio_tx_buffer[AUDIO_BLOCK_SIZE * 2 + (2 * i) + 1] = output_sample;
    }
  }
}

// SAI Receive Half-Complete Callback
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI2_Block_B)
  {
    // Process the first half of the RX buffer
    // For now, this is a placeholder. Bypass logic will go here.
  }
}

// SAI Receive Complete Callback
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
  if (hsai->Instance == SAI2_Block_B)
  {
    // Process the second half of the RX buffer
    // For now, this is a placeholder. Bypass logic will go here.
  }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
#ifdef USE_FULL_ASSERT
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
