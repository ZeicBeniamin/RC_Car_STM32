/*
 * motor.c
 *
 *  Created on: 5 May 2021
 *      Author: Beniamin Zeic
 */

#define MOTOR_LOW_PWM 0
#define MOTOR_HIGH_PWM 80

#include <motor.h>

int pwm_duty = 0;
TIM_HandleTypeDef *servoHtim;

void motorSetTimHandler(TIM_HandleTypeDef *h) {
  servoHtim = h;
}


void motorControl(uint8_t command) {
  pwm_duty = map(command, 0, 99, MOTOR_LOW_PWM, MOTOR_HIGH_PWM);
  __HAL_TIM_SET_COMPARE(servoHtim, TIM_CHANNEL_1, pwm_duty);

}

