/*
 * UartHelper.cpp
 *
 *  Created on: Mar 15, 2021
 *      Author: Beniamin Zeic
 */

#include "UartHelper.h"

// Constructor - initialize the buffers
UartHelper::UartHelper() {
  // Assign the first buffer as the storage reading buffer
  rx_state = RX_BUFFER_READY;
  rx_buffer = rx_buffer1;
  swap_flag = RX_SWAPPED;
}

// Destructor
UartHelper::~UartHelper() {
  // TODO Auto-generated destructor stub
}


// Receive data from the UART callback
void UartHelper::receive(uint8_t rx[]) {
//  cout << "UartHelper::receive() >> \n";
  // Use a double-buffer system, so that one buffer is always available
  // for receiving UART data and the other one is always available for sending
  // data over to the program (on-request).

  // Only proceed with writing if the buffers have been swapped
  if (swap_flag == RX_SWAPPED) {
    if(rx_buffer == rx_buffer1) {
//      strcpy((char*) rx_buffer2, (const char*) rx);
      strcpy((char*) rx_buffer1, (const char*) rx);
      // Swap buffers only if no read request has been made by the program.
      if (rx_state == RX_BUFFER_READY) {
        rx_buffer = rx_buffer2;
      }
      // Otherwise, raise a flag so that the buffer is changed when reading is
      // completed.
      else {
        swap_flag = RX_SWAP_BUFFER;
      }

    } else if(rx_buffer == rx_buffer2) {
      strcpy((char*) rx_buffer2, (const char*) rx);
      // Swap buffers only if no read request has been made by the program.
      if (rx_state == RX_BUFFER_READY) {
        rx_buffer = rx_buffer1;
      }
      // Otherwise, raise a flag so that the buffer is changed when reading is
      // completed.
      else {
        swap_flag = RX_SWAP_BUFFER;
      }
    }
  }
}

// Reads data stored in this class, previously transferred by UART.
uint8_t* UartHelper::read() {
//  cout << "UartHelper::read() >> \n";
  // Copy data from the available register - the one which is not currently used
  // for writing rx data - to the read_buffer, which will be sent to the program
  // via the method's return parameter.

  // Protect access to the buffer by flagging the reading operation with
  // RX_BUFFER_BUSY_READING.
  rx_state = RX_BUFFER_BUSY_READING;
  if (rx_buffer == rx_buffer1) {
    strcpy((char*) read_buffer, (char*) rx_buffer2);
  } else {
    strcpy((char*) read_buffer, (char*) rx_buffer1);
  }

  // If a buffer swap was attempted during reading, execute that operation now.
  if (swap_flag == RX_SWAP_BUFFER) {
    if (rx_buffer == rx_buffer1) {
      rx_buffer = rx_buffer2;
      swap_flag = RX_SWAPPED;
    } else {
      rx_buffer = rx_buffer1;
      swap_flag = RX_SWAPPED;
    }
  }
  // Flag the termination of the reading operation.
  rx_state = RX_BUFFER_READY;
  return read_buffer;
}
