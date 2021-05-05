/*
 * motor.h
 *
 *  Created on: 5 May 2021
 *      Author: Beniamin Zeic
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include "utils.h"
#include "stm32f4xx_hal.h"

void motorSetTimHandler(TIM_HandleTypeDef *);
void motorControl(uint8_t);


#endif /* INC_MOTOR_H_ */
