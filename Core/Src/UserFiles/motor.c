/*
 * motor.c
 *
 *  Created on: 5 May 2021
 *      Author: Beniamin Zeic
 */

#define MOTOR_LOW_PWM 30
#define MOTOR_HIGH_PWM 60

#include <motor.h>

int pwm_duty = 0;
TIM_HandleTypeDef *motorHtim;

void motorSetTimHandler(TIM_HandleTypeDef *h) {
  motorHtim = h;
}


void motorControl(uint8_t command) {
  if (command != 0) {
	  pwm_duty = map(command, 0, 99, MOTOR_LOW_PWM, MOTOR_HIGH_PWM);
  } else {
	  pwm_duty = 0;
  }
  __HAL_TIM_SET_COMPARE(motorHtim, TIM_CHANNEL_1, pwm_duty);

}

