/*
 * StateMachine.cpp
 *
 *  Created on: 1 Apr 2021
 *      Author: phantomcoder
 */

#include "StateMachine.h"

StateMachine::StateMachine() {
  _connection = CONN_NOT_ESTABLISHED;
  _stm_state = STM_INITIALIZED;
  _msg_type = MSG_NO_MESSAGE;
}

void StateMachine::setTimHandler(TIM_HandleTypeDef *htim) {
  this -> _htim = htim;
}

void StateMachine::setUartHelper(UartHelper *uart_helper) {
  _p_uart_helper = uart_helper;
}

void StateMachine::setConnectionTimeout(uint32_t timeout) {
  this -> _timeout = timeout;
}

/** @brief  Decodes the message received through UART.
  * @note   Each UART message must belong to a predefined set of messages. This
  *         method decodes the messages and returns an enum value corresponding
  *         to each message type. If necessary, it also stores the numerical
  *         values sent as commands for different actuators (motors and servos,
  *         for the moment).
  *
  * @param  msg The message to decode.
  * @retval None
  */
void StateMachine::decode_message(uint8_t *msg) {
  /*  There are currently only two possible message types:
   *  - connection accepted
   *  - actuators command
   */

  /*  Connection accepted message should be "<CONACC>*/
  if (!strcmp((char*) msg, (char*) "<CONACC>")) {
    _msg_type = MSG_CONNECTION_ACCEPTED;
  }
  /*  Actuators command message has this structure: "<M00S11>"
   *  Where:
   *  - '<' and '>'are heading/stop bytes
   *  - 'M' stands for 'motor'
   *  - '00' represents an integer in range (0,99) that later will be
   *    translated into a motor command signal
   *  - 'S' stands for 'servomotor'
   *  - '11' represents an integer in range (0,99) that will later be
   *    translated into a servomotor command signal
   */
  else if (msg[1] == 'M' && msg[4] == 'S') {
    _msg_type = MSG_ACTUATORS_COMMAND;
    _motor_speed_percent = (msg[2] - 48) * 10 + (msg[3] - 48);
    _servo_angle_percent = (msg[5] - 48) * 10 + (msg[6] - 48);
  }
}

void StateMachine::sendActuatorsCommand(uint8_t motor_comm, uint8_t servo_comm) {
  // TODO: Send commands to motor and servo
}

void StateMachine::stopActuators() {
  // TODO: Stop actuators
}

void StateMachine::main() {
    /*  Hold only a pointer to the message array. We are sure that the UartHelper
   *  instance outlives this class's life cycle, so we can use it without
   *  getting NULL pointer exceptions.
   */
  _p_message_rx = _p_uart_helper->read();
  decode_message(_p_message_rx);

  // Main state machine

  // Toggle led on/off, to signal certain connection stages
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, (led_state ? GPIO_PIN_SET : GPIO_PIN_RESET));

  /** If connection was not established & stm not running:
     *  - send connection request
     *  - check response of RPI
     *    (if response is positive <CONACC>:
     *      - update connection flag (CONN_ESTABLISHED)
     *      - pass to next state (STM_RUNNING)
     *      )
     */

  if (_connection == CONN_NOT_ESTABLISHED) {
    /* Continuously send connection requests and check for the response */
    strcpy((char*) _data_tx_buffer, (char*) "<CONREQ>");
    _p_uart_helper->transmit(_data_tx_buffer);
    /* Check if the response is a connection confirmation */
    if(_msg_type == MSG_CONNECTION_ACCEPTED) {
      _connection = CONN_ESTABLISHED;
      _stm_state = STM_RUNNING;
      led_state = !led_state;
    }
  }


  /**
   *  If connection was established && stm running:
   *  - check timestamp of last message - essentially checks if a new message was received
   *    (if different from that of last validly processed message:
   *      - process message (this includes actuator commands interpreting)
   *      - send commands (call method that manages command sending
   *     else:
   *      - calculate difference between last timestamp and current time - essentially checks if the connection is still active\
   *                    i.e. if a message was received before timeout
   *        (if greater than timeout:
   *          - stop car
   *          )
   *     )
   *  - send sensor measurements
   */

  else if (_connection == CONN_ESTABLISHED && _stm_state == STM_RUNNING) {
    if (! _p_uart_helper -> isLastMessageProcessed()) {
      led_state = !led_state;
      decode_message(_p_uart_helper->read());

      sendActuatorsCommand(_motor_speed_percent, _servo_angle_percent);
      strcpy((char*) _data_tx_buffer, (char*) _p_uart_helper->read());
      _p_uart_helper->transmit(_data_tx_buffer);
    }
    else if (! (_p_uart_helper -> getMessageTimeDiff() < _timeout)) {
      led_state = 1;
      _stm_state = STM_BLOCKED;
    }
  }

  /*
   *
   *  If connection was established && stm blocked:
   *  - check timestamp of last message
   *    (if different from that of last validly processed message:
   *      - process message
   *      - change state (STM_RUNNING)
   *      )
   *
   */

  else if (_connection == CONN_ESTABLISHED && _stm_state == STM_BLOCKED) {
    // Check if a new message was received
    if (! _p_uart_helper -> isLastMessageProcessed()) {
      // TODO: Remove delay
      HAL_Delay(1000);
      strcpy((char*) _data_tx_buffer, (char*) "<ReFRSH>");
      _p_uart_helper->transmit(_data_tx_buffer);
      led_state = !led_state;
      decode_message(_p_uart_helper->read());
      sendActuatorsCommand(_motor_speed_percent, _servo_angle_percent);
      _stm_state = STM_RUNNING;
    } else {
      stopActuators();
      strcpy((char*) _data_tx_buffer, (char*) "<BLCKST>");
      _p_uart_helper->transmit(_data_tx_buffer);
    }
  }

}
