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
private:
  UartHelper *_uart_helper;
  uint8_t _data_tx_buffer[10];
  uint8_t _data_rx_buffer[10];
};

#endif /* SRC_STATEMACHINE_H_ */
