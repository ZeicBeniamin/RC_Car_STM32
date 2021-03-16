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

typedef enum {
  RX_SWAP_BUFFER,
  RX_SWAPPED
} UART_Rx_Buffer_Flag;

typedef enum {
  RX_BUFFER_BUSY_READING,
  RX_BUFFER_READY
} UART_Rx_Buffer_State;

class UartHelper {
public:
  UartHelper();
  virtual ~UartHelper();
  void receive(uint8_t rx[]);
  void transmit();
  uint8_t* read();
  void write();

private:
  // Write locations for the RXCpltCallback method - to be used alternatively
  uint8_t rx_buffer1[10];
  uint8_t rx_buffer2[10];

  uint8_t tx_buffer1[10];
  uint8_t tx_buffer2[10];
  // To be used as return value for the receive() method
  uint8_t read_buffer[10];

  uint8_t write_buffer[10];

  // Pointers to the buffer to be used
  uint8_t *tx_buffer;
  uint8_t *rx_buffer;

  // Hold the buffer that
  UART_Rx_Buffer_Flag swap_flag;
  UART_Rx_Buffer_State rx_state;
};

#endif /* SRC_UARTHELPER_H_ */
