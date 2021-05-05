/*
 * uart.h
 *
 *  Created on: May 5, 2021
 *      Author: Beniamin Zeic
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include <motor.h>
#include <servo.h>
#include <utils.h>
#include "stm32f4xx_hal.h"


void uartInitRx(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void uartDecode(uint8_t *);

#endif /* INC_UART_H_ */
