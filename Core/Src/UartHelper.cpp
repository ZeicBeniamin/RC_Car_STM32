/*
 * UartHelper.cpp
 *
 *  Created on: Mar 15, 2021
 *      Author: Beniamin Zeic
 */

#include "UartHelper.h"

/**
  * @brief  Initialises state variables of the class
  * @param  None
  * @retval None
  */
UartHelper::UartHelper() {
  // Assign the first buffer as the storage reading buffer
  rx_state = RX_BUFFER_READY;
  rx_buffer = rx_buffer1;
  swap_flag = BUFFER_READY;
}

/**
  * @brief  Receives data from UART and writes it into one buffer.
  * @note   Writes data in the active buffer, that is the buffer which has its
  *         address stored in the rx_buffer pointer.
  *         Data is written only if a swap was not already requested.
  *         Requests a buffer swap after each reception process.
  *
  * @param  rx Pointer to the array received through UART.
  * @retval None
  */
void UartHelper::receive(uint8_t rx[]) {
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

/**
  * @brief  Returns reception data stored in the inactive buffer.
  * @note   A check is initially performed, to see which of the tow buffers is
  *         the inactive one. That buffer is copied to a temporary buffer and
  *         returned to the caller function.
  *         An additional check is performed, to see if buffer swap was
  *         requested. Buffer swap requests are made from UartHelper::receive()
  *         upon data reception completion. It is guaranteed that no other
  *         reception event is allowed to trigger a writing into the active
  *         buffer until the swap request is fulfilled.
  *
  * @param  None
  * @retval None
  */
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

/**
  * @brief Set the UART handler object to use for transmission
  * @param huart: pointer to the handler of the UART interface
  * @retval None
  */
void UartHelper::setHandler(UART_HandleTypeDef* huart) {
  this -> huart = huart;
}

void UartHelper::transmit(uint8_t *tx) {
  // Activate the transmission line and send the data stored in tx[]
  HAL_UART_Transmit_IT(huart, tx, 8);
};


