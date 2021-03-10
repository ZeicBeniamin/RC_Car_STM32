# RC_Car_STM32
STM32 embedded code for the RC car project

The project consists of an STM32 F401RE microcontroller unit, connected to the following components:
- Pololu G2 18v17 power motor driver
- Towerpro MG950 servomotor
- Adafruit 4754 IMU (based on BNO085)
- HC-SR04 ultrasonic sensor

The microcontroller is also connected to a Rapsberry PI 4B via an usb cable. The UART communication capabilities of the mcu are used for the purpose of communicating with the RPI.
