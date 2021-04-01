# RC_Car_STM32
STM32 embedded code for the RC car project

The project consists of an STM32 F401RE microcontroller unit, connected to the following components:
- Pololu G2 18v17 power motor driver
- Towerpro MG950 servomotor
- Adafruit 4754 IMU (based on BNO085)
- HC-SR04 ultrasonic sensor

The microcontroller is also connected to a Rapsberry PI 4B via an usb cable. The UART communication capabilities of the mcu are used for the purpose of communicating with the RPI.

### UART communication
The STM32 and the Rapsberry PI communicate through the UART port over USB. The communication content consists of motor commands which RPI gives to STM32 and sensor measurements that the STM32 sends to RPI.
The `UartHelper` class in conjunction with the UART callbacks defined in `main.cpp` control the transmission and reception process done through the UART interface. The class holds the received messages and can return the last received message on-demand, through a class method. It can also perform on-demand data transmission.
Both the transmission and the reception process are done asynchronously, by using the interrupt reception and transmission functions.
