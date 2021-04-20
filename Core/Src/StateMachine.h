/*
 * StateMachine.h
 *
 *  Created on: 1 Apr 2021
 *      Author: phantomcoder
 */

#ifndef SRC_STATEMACHINE_H_
#define SRC_STATEMACHINE_H_

#include <string.h>
#include "UartHelper.h"
#include <cctype>

typedef enum {
  MSG_NO_MESSAGE,
  MSG_CONNECTION_ACCEPTED = 1,
  MSG_ACTUATORS_COMMAND
} Message;

/**
  * @brief  STM32 <-> RPI Connection State definition
  *         There are only two possible states: connection established or not established.
  */
typedef enum
{
  CONN_ESTABLISHED = 1,
  CONN_NOT_ESTABLISHED
} STM32_ConnStateTypeDef;


/**
  * @brief  Stores the state of the STM32
  */
typedef enum
{
  STM_INITIALIZED = 1,
  STM_RUNNING,
  STM_BLOCKED,

  STM_INITIALIZING_TIMER,
  STM_SENDING_UPDATE,
  STM_WAITING_MESSAGE,
  STM_SENDING_COMMAND,
  STM_CAR_BLOCKED,
  STM_WAITING_MESSAGE_BLOCK
} STM32_StateTypeDef;

class StateMachine {
public:
  StateMachine();
  void main();
  void establish_connection();
  void sendUpdate();
  void blockCar();
  void waitMsg();
  void command_motors();
  void readSensors();
  void setUartHelper(UartHelper *);
  void decode_message(uint8_t *);
  void setTimHandler(TIM_HandleTypeDef *);
  void setConnectionTimeout(uint32_t);

private:

  // TODO: Remove after tests
  uint8_t _data_tx_buffer[10];

  /* Pointers to objects used in communication */
  UartHelper *_p_uart_helper;
  uint8_t *_p_message_rx;
  TIM_HandleTypeDef *_htim;

  /* Store the commands to be given to the motors */
  int _motor_speed_percent;
  int _servo_angle_percent;
  int _motor_pwm;
  int _servo_pwm;

  /* Store the message type for the last received message */
  Message _msg_type;


  /* Store state of the on-board led */
  uint8_t led_state = 0;

  /* Enums used as state variables */
  STM32_ConnStateTypeDef _connection;
  STM32_StateTypeDef _stm_state;

  /* Stores maximum time period (in milliseconds) for which the car should run
   * if no message is received.
   */
  uint32_t _timeout = 1000;


};

#endif /* SRC_STATEMACHINE_H_ */
