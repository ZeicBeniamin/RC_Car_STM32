/*
 * uart.c
 *
 *  Created on: May 5, 2021
 *      Author: Beniamin Zeic
 */

#include <uart.h>

UART_HandleTypeDef *local_huart;
uint8_t rx_buff[10];
uint8_t tx_buff[10];

void uartInitRx(UART_HandleTypeDef *huart)
{
  local_huart = huart;
  HAL_UART_Receive_IT(local_huart, rx_buff, 8);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uartDecode(rx_buff);
  HAL_UART_Transmit_IT(local_huart, rx_buff, 8);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(local_huart, rx_buff, 8);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  HAL_UART_Receive_IT(local_huart, rx_buff, 8);
}

/**
  * @brief  Decodes the actuators and commands in the received message.
  * @note   The message received through UART contains actuator commands.
  *         The message is firstly reordered (circular permutations may occur
  *         during UART transmission) and then the values are taken and sent
  *         to the proper actuators, through a specific control function
  *         Due to the chosen transmission method, only two actuator commands
  *         may be sent with one message.
  *         The received command takes values in the interval (0, 99)
  * @param  string - pointer to uint8_t array containing the message
  * @retval None
  */

void uartDecode(uint8_t *string)
{
  uint8_t heading_pos;
  uint8_t i; // dummy variable

  uint8_t act1Command;
  uint8_t act2Command;
  i = 0;
  /* Reorders the message, in case it suffered a circular permutation*/
  while (i < 8)
  {
    if ((char)(string[i]) == '<')
    {
      /* Heading byte found -> store heading byte position and exit.*/
      heading_pos = i;
      /* Exit loop */
      i = 8;
    }
    ++i;
  }
  /* Reorder the string into temp_read_buffer */
  /* Copying the string is expensive, so I decided to extract data from it by
   * hand, since the structure of the message is already known
   */
  //  for (i = 0; i < 8; ++i) {
  //      temp_read_buffer[i] = string[(i + heading_pos) % 8];
  //  }
  /* Extract the number (between 0 and 99) that should be passed as a command to
   * the actuators
   */
  act1Command = (string[(heading_pos + 3) % 8] - 48) +
                (string[(heading_pos + 2) % 8] - 48) * 10;
  act2Command = (string[(heading_pos + 6) % 8] - 48) +
                (string[(heading_pos + 5) % 8] - 48) * 10;
  /* Send the command to the right actuators */
  switch (string[(heading_pos + 1)])
  {
  case 'M':
    motorControl(act1Command);
    break;
  case 'S':
    servoControl(act1Command);
    break;
  }
  switch (string[(heading_pos + 4)])
  {
  case 'M':
    motorControl(act2Command);
    break;
  case 'S':
    servoControl(act2Command);
    break;
  }

  return;
}
