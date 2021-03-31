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
  swap_flag = BUFFER_READY;
}

// Destructor
UartHelper::~UartHelper() {
  // TODO Auto-generated destructor stub
}


// Receive data from the UART callback
void UartHelper::receive(uint8_t rx[]) {
//  cout << "UartHelper::receive() >> \n";
  // Use a double-buffer system, so that one buffer is always available
  // for receiving UART data and the other one is always available for
  // returning the stored data.

  // Only proceed with writing if the buffers have been swapped
  if (swap_flag == BUFFER_READY) {
    // Write UART data to the active (in-use) buffer
    if(rx_buffer == rx_buffer1) {
      // Copy received data to the active buffer
      strcpy((char*) rx_buffer1, (const char*) rx);
      // If no read operation is currently performed on the inactive buffer,
      // swap the two buffers. Making the currently used buffer inactive, we
      // give the chance for the received data to be read from that buffer without
      // interfering with data reception from UART.
      if (rx_state == RX_BUFFER_READY) {
        rx_buffer = rx_buffer2;
      }
      // Otherwise, raise a flag so that the buffer is changed when reading
      // from the inactive flag is complete
      else {
        swap_flag = BUFFER_REQUEST_SWAP;
      }
    }
    // The same code as above, but for rx_buffer2 as the active buffer
    else if(rx_buffer == rx_buffer2) {
      strcpy((char*) rx_buffer2, (const char*) rx);
      if (rx_state == RX_BUFFER_READY) {
        rx_buffer = rx_buffer1;
      }
      else {
        swap_flag = BUFFER_REQUEST_SWAP;
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
    strcpy((char*) temp_read_buffer, (char*) rx_buffer2);
  } else {
    strcpy((char*) temp_read_buffer, (char*) rx_buffer1);
  }

  // If a buffer swap was attempted during reading, execute that operation now.
  if (swap_flag == BUFFER_REQUEST_SWAP) {
    if (rx_buffer == rx_buffer1) {
      rx_buffer = rx_buffer2;
      swap_flag = BUFFER_READY;
    } else {
      rx_buffer = rx_buffer1;
      swap_flag = BUFFER_READY;
    }
  }
  // Flag the termination of the reading operation.
  rx_state = RX_BUFFER_READY;
  return temp_read_buffer;
}

//void UartHelper::ReceiveCb(UART_HandleTypeDef *huart) {
//  //TODO: Delete tests
//  // Testing purposes only - this code re-routes the received data to UART Tx, to check correctness
//  // Store the received data in the double-buffer system in this class
//  receive(rx);
//  // Read the currently stored data into another buffer
//  rx_b = read();
//  // Send the received data
//  HAL_UART_Transmit_IT(huart, rx_b, 8);
//}

void UartHelper::setHandlerBeginReception(UART_HandleTypeDef* huart) {
  this -> huart = huart;

  // Register the ReceiveCb method from this class as the callback function
  // for the UART RX Complete
  //HAL_UART_RegisterCallback(this->huart, HAL_UART_RX_COMPLETE_CB_ID, static_cast <UartHelper*>(UartHelper::ReceiveCb));

  // TODO: REMOVE TEST
  // Test UART reception called from this class


  HAL_UART_Receive_IT(huart, rx, 8);
}

void UartHelper::transmit(uint8_t *tx) {
  // Activate the transmission line and send the data stored in tx[]
  HAL_UART_Transmit_IT(huart, tx, 8);
};

