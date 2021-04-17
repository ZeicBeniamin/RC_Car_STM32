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

void StateMachine::setUartHelper(UartHelper *uart_helper){
  _p_uart_helper = uart_helper;
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
  else if (msg[1] == 'M' && msg[4] == 'S' &&
      isdigit(msg[2]) && isdigit(msg[3]) &&
      isdigit(msg[4]) && isdigit(msg[5])) {
    _msg_type = MSG_ACTUATORS_COMMAND;
    _motor_speed_percent = msg[2] * 10 + msg[3];
    _servo_angle_percent = msg[5] * 10 + msg[6];
  }
}

void StateMachine::main() {
  // TODO: Delte test code
  // Transmission to RPI working
//  strcpy((char*) _data_tx_buffer, "3456><12");
//  _uart_helper->transmit(_data_tx_buffer);
//  strcpy((char*)_data_rx_buffer, (char*)_uart_helper->read());
//  HAL_Delay(1000);

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

  /* Connection not established */
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
   *  - check timestamp of last message
   *    (if different from that of last validly processed message:
   *      - process message (this includes conversions and motor commands)
   *     else:
   *      - calculate difference between last timestamp and current time
   *        (if greater than timeout:
   *          - stop car
   *          )
   *     )
   *  - send sensor measurements
   */

  else if (_connection == CONN_ESTABLISHED && _stm_state == STM_RUNNING) {
    if(_p_uart_helper->isConnectionActive()) {
      led_state = !led_state;
      HAL_Delay(100);
    } else {
      led_state = 1;
      HAL_Delay(500);
      _stm_state = STM_CAR_BLOCKED;
    }
  }

  /** If connection was not established:
     *  - send connection request
     *  - check response of RPI
     *    (if response is positive:
     *      - update connection flag (CONN_ESTABLISHED)
     *      - pass to next state (STM_RUNNING)
     *      )
     */
  else if (_connection == CONN_ESTABLISHED && _stm_state == STM_BLOCKED) {

  }

//
//
//  /*
//   *
//   *  If connection was established && stm blocked:
//   *  - check timestamp of last message
//   *    (if different from that of last validly processed message:
//   *      - process message
//   *      - change state (STM_RUNNING)
//   *      )
//   *
//   */
//
//
//
//
//    /* Waiting connection message */
//    if (gState == STM_WAITING_MESSAGE) {
//      gstate = huart2.gState;
//      // Make the uart listen for messages from the RPI, if it's not busy
//      // with other tasks
//      // Check that there is no ongoing data transmission
//      if (huart2.gState != HAL_UART_STATE_BUSY_TX) {
//        // There is no problem with making overlapping receive requests,
//        // since UART automatically checks for ongoing data reception.
//        HAL_UART_Receive_IT(&huart2, rx_buffer, 8);
//      }
//    }
//    /* Received connection message, sending a response */
//    if (gState == STM_SENDING_UPDATE) {
//      // Send the response marking the fact that the connection was established
//      strcpy(tx_buffer, (uint8_t *) "CON_EST\n");
//      // Force the transmission of the update. Otherwise, the reception part
//      // would block the transmission part.
//      HAL_UART_Transmit_IT(&huart2, tx_buffer, 8);
//      // Pass to the next logical state for both the connection and the STM.
//      connState = CONN_ESTABLISHED;
//      gState = STM_INITIALIZING_TIMER;
//  }
//
//  /* Connection established */
//  else { // if(connState == CONN_ESTABLISHED)
//    /* Initialising timer */
//    if (gState == STM_INITIALIZING_TIMER) {
//      init_time = HAL_GetTick();
//      gState = STM_SENDING_UPDATE;
//    }
//    /* Sending update over to RPI */
//    else if (gState == STM_SENDING_UPDATE) {
//      // Timeout in milliseconds
//      timeout = 5000;
//      // Wait for previous uart TX's or RX's to complete
//      // There can be no data reception at this point; data reception should
//      // have just finished.
//      HAL_UART_Transmit_IT(&huart2, tx_buffer, 8);
//      gState = STM_WAITING_MESSAGE;
//
//
//      timeout = 5000;
//    }
//    /* Waiting for message from RPI */
//    else if (gState == STM_WAITING_MESSAGE) {
//      // TODO: Handle RPI disconnection
//      // If the timeout has not passed, start listening for a message from the RPI
//      time_diff = HAL_GetTick() - init_time;
//      if (time_diff < timeout) {
//        // Listen for incoming data
//        if (huart2.gState != HAL_UART_STATE_BUSY_TX &&
//            huart2.RxState != HAL_UART_STATE_BUSY_RX) {
//          HAL_UART_Receive_IT(&huart2, rx_buffer, 8);
//        }
//      }
//      /* Block the car */
//      else {
//        gState = STM_CAR_BLOCKED;
//      }
//    }
//    else if (gState == STM_SENDING_COMMAND) {
//      // Bypass this state, as we do not have any motor to controll
//      // TODO: Send real commands to actuators; remove bypass of this state
//      gState = STM_INITIALIZING_TIMER;
//    }
//    else if (gState == STM_CAR_BLOCKED) {
//      // Light up the on-board led
//      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
//      // TODO: See if this state is required
//      gState = STM_WAITING_MESSAGE_BLOCK;
//    }
//    else if (gState == STM_WAITING_MESSAGE_BLOCK) {
//      // Wait for a message from the RPI
//      // Check that there is no ongoing data transmission
//      if (huart2.gState != HAL_UART_STATE_BUSY_TX &&
//          huart2.RxState != HAL_UART_STATE_BUSY_RX) {
//        HAL_UART_Receive_IT(&huart2, rx_buffer, 8);
//      }
//    }
//  }



}
