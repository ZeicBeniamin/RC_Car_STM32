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
  HAL_UART_Receive_IT(&huart2, main_rx_buff, 8);
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
  uart_helper.setHandler(&huart2);
  HAL_UART_Receive_IT(&huart2, main_rx_buff, 8);
  /* USER CODE END 2 */
  /* Infinite loop */

  /* USER CODE BEGIN WHILE */

  while (1)
  {

    // Check the received buffer from uart_helper

    // Send the received message back, through uart_helper


/*
    rxstate = huart2.RxState;

    // Main state machine

    // Connection not established
    if (connState == CONN_NOT_ESTABLISHED) {
      // Waiting connection message /
      if (gState == STM_WAITING_MESSAGE) {
        gstate = huart2.gState;
        // Make the uart listen for messages from the RPI, if it's not busy
        // with other tasks
        // Check that there is no ongoing data transmission
        if (huart2.gState != HAL_UART_STATE_BUSY_TX) {
          // There is no problem with making overlapping receive requests,
          // since UART automatically checks for ongoing data reception.
          HAL_UART_Receive_IT(&huart2, rx_buffer, 8);
        }
      }
      // Received connection message, sending a response
      if (gState == STM_SENDING_UPDATE) {
        // Send the response marking the fact that the connection was established
        strcpy(tx_buffer, (uint8_t *) "CON_EST\n");
        // Force the transmission of the update. Otherwise, the reception part
        // would block the transmission part.
        HAL_UART_Transmit_IT(&huart2, tx_buffer, 8);
        // Pass to the next logical state for both the connection and the STM.
        connState = CONN_ESTABLISHED;
        gState = STM_INITIALIZING_TIMER;
    }
    }
    // Connection established //
    else { // if(connState == CONN_ESTABLISHED)
      // Initialising timer //
      if (gState == STM_INITIALIZING_TIMER) {
        init_time = HAL_GetTick();
        gState = STM_SENDING_UPDATE;
      }
      // Sending update over to RPI //
      else if (gState == STM_SENDING_UPDATE) {
        // Timeout in milliseconds
        timeout = 5000;
        // Wait for previous uart TX's or RX's to complete
        // There can be no data reception at this point; data reception should
        // have just finished.
        HAL_UART_Transmit_IT(&huart2, tx_buffer, 8);
        gState = STM_WAITING_MESSAGE;


        timeout = 5000;
      }
      // Waiting for message from RPI //
      else if (gState == STM_WAITING_MESSAGE) {
        // TODO: Handle RPI disconnection
        // If the timeout has not passed, start listening for a message from the RPI
        time_diff = HAL_GetTick() - init_time;
        if (time_diff < timeout) {
          // Listen for incoming data
          if (huart2.gState != HAL_UART_STATE_BUSY_TX &&
              huart2.RxState != HAL_UART_STATE_BUSY_RX) {
            HAL_UART_Receive_IT(&huart2, rx_buffer, 8);
          }
        }
        // Block the car //
        else {
          gState = STM_CAR_BLOCKED;
        }
      }
      else if (gState == STM_SENDING_COMMAND) {
        // Bypass this state, as we do not have any motor to controll
        // TODO: Send real commands to actuators; remove bypass of this state
        gState = STM_INITIALIZING_TIMER;
      }
      else if (gState == STM_CAR_BLOCKED) {
        // Light up the on-board led
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        // TODO: See if this state is required
        gState = STM_WAITING_MESSAGE_BLOCK;
      }
      else if (gState == STM_WAITING_MESSAGE_BLOCK) {
        // Wait for a message from the RPI
        // Check that there is no ongoing data transmission
        if (huart2.gState != HAL_UART_STATE_BUSY_TX &&
            huart2.RxState != HAL_UART_STATE_BUSY_RX) {
          HAL_UART_Receive_IT(&huart2, rx_buffer, 8);
        }
      }
    }

    hstate = (huart2.gState | huart2.RxState);
*/
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
  * @brief This function is executed upon UART rx completion
  * @param huart: pointer to the handler of the UART interface
  * @retval None
  */
/* ------------------------------------ COMM OUT
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  // Perform actions corresponding to the current state of the connection
  switch(connState) {
  // Connection not established //
  case CONN_NOT_ESTABLISHED:
      // Message received while establishing connection.
      // Decode the message
      if (rx_buffer[0] == 'E' &&
          rx_buffer[1] == 'S' &&
          rx_buffer[2] == 'T' &&
          rx_buffer[3] == '_' &&
          rx_buffer[4] == 'C' &&
          rx_buffer[5] == 'O' &&
          rx_buffer[6] == 'N') {
        // The RPI wants to establish a connection, so we change the gState
        // such that an update will be sent to the RPI, to confirm the connection
        gState = STM_SENDING_UPDATE;
      }
      else {
        gState = STM_WAITING_MESSAGE;
      }

      //HAL_UART_Transmit_IT(&huart2, (uint8_t *) "CONN_EST", 8);
      break;

  // Connection already established //
  case CONN_ESTABLISHED:
    if (gState == STM_WAITING_MESSAGE) {
      // TODO: Remove test code
      // Test: Reroute the input message to the output:
      // Change the state first, such that the state machine will not start
      // another receiving process.
      gState = STM_SENDING_COMMAND;
      tx_buffer[0] = rx_buffer[0];
      tx_buffer[1] = rx_buffer[1];
      tx_buffer[2] = rx_buffer[2];
      tx_buffer[3] = rx_buffer[3];
      tx_buffer[4] = rx_buffer[4];
      tx_buffer[5] = rx_buffer[5];
      tx_buffer[6] = rx_buffer[6];
      tx_buffer[7] = '\n';
    }
    else if (gState == STM_WAITING_MESSAGE_BLOCK) {
      // TODO: Remove test code
      // Test: Reroute the input message to the output:
      // Change the state first, such that the state machine will not start
      // another receiving process.
      gState = STM_SENDING_COMMAND;
      tx_buffer[0] = rx_buffer[0];
      tx_buffer[1] = rx_buffer[1];
      tx_buffer[2] = rx_buffer[2];
      tx_buffer[3] = rx_buffer[3];
      tx_buffer[4] = rx_buffer[4];
      tx_buffer[5] = rx_buffer[5];
      tx_buffer[6] = rx_buffer[6];
      tx_buffer[7] = '\n';


      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    } else {
      // TODO: Remove test code
      // Test: Reroute the input message to the output:
      strcpy(tx_buffer, "except-\n");

      gState = STM_SENDING_COMMAND;
    }
    break;
  default:
    break;
  }

  // -- bypassed --
  // Toggle the state of the on-board led, to signal a completed reading
//  led_state = !led_state;
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, led_state);

}
------------------------------------ COMM OUT */
/**
  * @brief This function is executed upon UART tx completion
  * @param huart: pointer to the handler of the UART interface
  * @retval None
  */
/* ------------------------------------ COMM OUT
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // Perform actions corresponding to the current state of the connection
    switch(connState) {
    case CONN_ESTABLISHED:
      // TODO: Set behaviour for the established connection
      break;

    case CONN_NOT_ESTABLISHED:
      // TODO: Check the behaviour for the non-established connection
      // Change the state of the connection. The connection has been established.
      connState = CONN_ESTABLISHED;
      // Change the state of the STM to initialize the counter.
      gState = STM_INITIALIZING_TIMER;
      break;

    default:
      break;
    }
}
-------------------------------------- COMM OUT*/
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

/**
    * @brief Wait for the connection message from the RPI
    * @param None
    * @retval None
    */
void wait_conn_message(void) {
  // TODO: Check if the connection message was received. If so, pass to the next state, and send the connection update (response)
}

/**
    * @brief Called after the connection message was received, sends a connection response
    * @param None
    * @retval None
    */
void send_conn_update(void) {
  // TODO: Fire the UART Tx once, with a connection confirmation message, then pass to the next state and rise the
  // TODO: connection ESTABLISHED flag
}

/**
    * @brief Initialises the timeout counter, before sending an update and waiting for confirmation from RPI.
    * @param None
    * @retval None
    */
void init_timer(void) {
  // TODO: Initialise timer and pass to the next state, which is SEND_UPDATE
}

/**
    * @brief Sends an update with the relevant sensor readings and actuator states
    * @param None
    * @retval None
    */
void send_update(void) {
  // TODO: Create an update message and send it once, using UART Tx. Pass to the next state, which is WAIT_MESSAGE
  // The message should contain data regarding motor speed and servo angle,
  // as well as ultrasonic readings and IMU angle.
}

/**
    * @brief Checks the RPI message timeout. If the message timed out, it blocks the motors.
    * @param None
    * @retval None
    */
void wait_message(uint16_t timeout) {
  // TODO: check the reception of the message. If it was not received, block the motors.
  // Processing of the received message will be done in UART's callback.
}

/**
    * @brief Called when RPI message times out. Blocks all movement of the car
    * @param None
    * @retval None
    */
void block() {
  // TODO: Reset the motor and servo variables and stop both motors.
}

/**
    * @brief Waits for the RPI message after the car was blocked
    * @param None
    * @retval None
    */
void wait_message_blocked(void) {
  // TODO: Check if the RPI message was received. If not, restart a process of sending updates and listening for responses.
}

/**
  * @brief This function is executed when UART errors are encountered
  * @param huart: pointer to the handler of the UART interface
  * @retval None
  */
/* -------------------------------------- COMM OUT
 * void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  // If an error occurs during information transmission, reset the transmission
  // by setting UART to one of the reading or writing states
  HAL_UART_Transmit_IT(&huart2, rx_buffer, 4);
}
-------------------------------------- COMM OUT */
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
