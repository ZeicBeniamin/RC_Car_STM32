/*
 * StateMachine.cpp
 *
 *  Created on: 1 Apr 2021
 *      Author: phantomcoder
 */

#include "StateMachine.h"

StateMachine::StateMachine() {
  // TODO Auto-generated constructor stub
}

void StateMachine::setUartHelper(UartHelper *uart_helper){
  _uart_helper = uart_helper;
}

void StateMachine::main() {
  // TODO: Delte test code
  // Transmission to RPI working well
  strcpy((char*) _data_tx_buffer, "3456><12");
  _uart_helper->transmit(_data_tx_buffer);
  strcpy((char*)_data_rx_buffer, (char*)_uart_helper->read());
  HAL_Delay(1000);
}

