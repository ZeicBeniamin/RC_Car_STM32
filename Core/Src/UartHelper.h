/*
 * UartHelper.h
 *
 *  Created on: Mar 15, 2021
 *      Author: Beniamin Zeic
 */

#ifndef SRC_UARTHELPER_H_
#define SRC_UARTHELPER_H_

#include <string.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

typedef enum {
  BUFFER_REQUEST_SWAP,
  BUFFER_READY
} UART_Rx_Buffer_Flag;

typedef enum {
  RX_BUFFER_BUSY_READING,
  RX_BUFFER_READY
} UART_Rx_Buffer_State;

class UartHelper {
public:
  UartHelper();
  virtual ~UartHelper();
  void receive(uint8_t *rx);
  void transmit(uint8_t *tx);
  uint8_t* read();
  void write();
  void setHandlerBeginReception(UART_HandleTypeDef*);
//  void ReceiveCb(UART_HandleTypeDef*);

private:
  // Write locations for the RXCpltCallback method - will be used alternatively
  uint8_t rx_buffer1[10];
  uint8_t rx_buffer2[10];

  uint8_t tx_buffer1[10];
  uint8_t tx_buffer2[10];
  // Will be used as return value for the receive() method
  uint8_t temp_read_buffer[10];
  uint8_t temp_write_buffer[10];

  // Pointers to the active buffers
  uint8_t *tx_buffer;
  // Holds the in use rx buffer
  uint8_t *rx_buffer;

  // Holds the buffer swap requests
  UART_Rx_Buffer_Flag swap_flag;
  UART_Rx_Buffer_State rx_state;

  UART_HandleTypeDef* huart;

  // TODO: Remove tests
  uint8_t rx[10];
  uint8_t *rx_b;
};

#endif /* SRC_UARTHELPER_H_ */
