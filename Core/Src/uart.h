/*
 * uart.h
 *
 *  Created on: May 5, 2021
 *      Author: Beniamin Zeic
 */

#ifndef SRC_UART_H_
#define SRC_UART_H_

#include "stm32f4xx_hal.h"

#include "enums.h"
#include "motor.h"
#include "servo.h"

void uartInitRx(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void uartDecode(uint8_t *);

#endif /* SRC_UART_H_ */
