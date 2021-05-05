/*
 * enums.h
 *
 *  Created on: 5 May 2021
 *      Author: Beniamin Zeic
 */

#ifndef INC_ENUMS_H_
#define INC_ENUMS_H_

typedef enum {
  NONE,
  DC_MOTOR,
  SERVO_MOTOR
} Actuator;

float map(float, float, float, float, float);


#endif /* INC_ENUMS_H_ */
