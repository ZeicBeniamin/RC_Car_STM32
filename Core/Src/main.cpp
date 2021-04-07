/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "UartHelper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
  STM_INITIALIZING_TIMER,
  STM_SENDING_UPDATE,
  STM_WAITING_MESSAGE,
  STM_SENDING_COMMAND,
  STM_CAR_BLOCKED,
  STM_WAITING_MESSAGE_BLOCK
} STM32_StateTypeDef;

/**
  * @brief STM32 Connection State definition
  * @note  There are only two possible states: connection established or not established.
  */
typedef enum
{
  CONN_ESTABLISHED,
  CONN_NOT_ESTABLISHED
} STM32_ConnStateTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Lets you register UART callback functions with custom name
// If macro is not changed in stm32f4xx_hal_conf_template.h, the change made here
// has no effect. The redefinition from main.cpp was only added as a reminder
// for the programmer that the macro is redefined elsewhere.
// If the original macro is defined to a different value, the line below (which
// redefines it) will generate a warning in the compile phase.
//#define USE_HAL_UART_REGISTER_CALLBACKS 1U;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t main_rx_buff[10];
uint8_t tx_buffer[8];
uint8_t led_state;
// TODO : REMove after tests
uint8_t var[8];
uint64_t variable;

uint8_t tx_size = 8;
uint8_t hstate;
// Holds the global state of the STM controller
STM32_StateTypeDef gState;

STM32_ConnStateTypeDef connState;

// Holds the state of the UART
HAL_UART_StateTypeDef gstate;
HAL_UART_StateTypeDef rxstate;

// Holds the time
uint32_t init_time;
uint32_t time_diff;
// Holds timeout value;
uint32_t timeout;

// Pointer to the rx buffer from the UartHelper class
uint8_t *rx_b;

UartHelper uart_helper;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void init();
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // Deposit the received data in the uart object
  uart_helper.receive(main_rx_buff);
  // Normally, after a message is received, UART should be put in reception
  // mode again:
//  HAL_UART_Receive_IT(&huart2, main_rx_buff, 8);
//  led_state = !led_state;
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (led_state == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET));
   HAL_UART_Transmit_IT(&huart2, uart_helper.read(), 8);

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  // Do nothing
}

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
  init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // Send the handler object together with the read buffer from main.
  // This is required so that the buffer can be accessed both from UartHelper.cpp
  // and main.cpp.
  // main.cpp contains the callback that redirects the read data to UartHelper
  // through this buffer. UartHelper uses this buffer as an argument for the
  // HAL_UART_Receive_IT function, in order to be used as a storage buffer for
  // the messages sent through uart.
  uart_helper.setHandler(&huart2, main_rx_buff);
  // No need to call HAL_UART_Receive_IT here, since it's called in the
  // setHandler method, from class UartHelper.

  /* USER CODE END 2 */
  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  while (1)
  {
//    var[7] = '0' + (variable%10);
//    var[6] = '0' + (variable/10%10);
//    var[5] = '0' + (variable/100%10);
//    var[4] = '0' + (variable/1000%10);
//    var[3] = '0' + (variable/10000%10);
//    var[2] = '0' + (variable/100000%10);
//    var[1] = '0' + (variable/1000000%10);
//    var[0] = '0' + (variable/10000000%10);
//
//    variable++;

//    strcpy((char*)var, (const char*)"11122233");
//    HAL_UART_Transmit_IT(&huart2, var, 8);
//    HAL_Delay(10);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief Initialize the state of the components of STM32
  * @param None
  * @retval None
  */
void init(void) {
  led_state = 0;
  // TODO: Initialize motors and servos to desired positions
  connState = CONN_NOT_ESTABLISHED;
  gState = STM_WAITING_MESSAGE;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
