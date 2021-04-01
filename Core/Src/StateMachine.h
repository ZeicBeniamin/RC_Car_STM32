/*
 * StateMachine.h
 *
 *  Created on: 1 Apr 2021
 *      Author: phantomcoder
 */

#ifndef SRC_STATEMACHINE_H_
#define SRC_STATEMACHINE_H_

class StateMachine {
public:
  StateMachine();

  // TODO: Send the UartHelper object through copy assignment
  // constructor to this class. This class will then manage the object that
  // was initially created in main.cpp. This class wil then
  // deal with the communication process
  void main();
  void establish_connection();
  void send_update();
  void block_car();
  void wait_msg();
  void command_motors();
  void read_sensors();
};

#endif /* SRC_STATEMACHINE_H_ */
