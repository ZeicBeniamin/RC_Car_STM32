/*
 * servo.h
 *
 *  Created on: 5 May 2021
 *      Author: Beniamin Zeic
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdint.h>
#include "utils.h"
#include "stm32f4xx_hal.h"

void servoSetTimHandler(TIM_HandleTypeDef *);
void servoControl(uint8_t);


#endif /* INC_SERVO_H_ */
