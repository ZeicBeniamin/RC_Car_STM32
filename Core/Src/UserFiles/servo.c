/*
 * servo.c
 *
 *  Created on: 5 May 2021
 *      Author: Beniamin Zeic
 */

#define SERVO_LOW_PWM 20
#define SERVO_HIGH_PWM 110

#include <servo.h>

int servoPwmDty = 0;
TIM_HandleTypeDef *servoHtim;

void servoSetTimHandler(TIM_HandleTypeDef *h) {
  servoHtim = h;
}


void servoControl(uint8_t command) {
  servoPwmDty = map(command, 0, 99, SERVO_LOW_PWM, SERVO_HIGH_PWM);
  __HAL_TIM_SET_COMPARE(servoHtim, TIM_CHANNEL_1, servoPwmDty);

}

