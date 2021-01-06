# Wheely

Yet another, two-wheel balancing robot.

Features:

- In a cardboard box
- RC Controlled
- buzzer / audio feedback

## Hardware

- Arduino Micro
- MPU6050
- L298 Motor driver
- 2x Geared toy DC motors and wheels
- Buzzer
- Turnigy 9X8C PWM Receiver

## Libraries

**L298NX2** Control 2 DC motors.
[Tutorial](https://create.arduino.cc/projecthub/ryanchan/how-to-use-the-l298n-motor-driver-b124c5)

**RC Receiver** lib (PWM and PPM)
[link](https://github.com/barafael/RC-Receiver-Interface/blob/master/RC-Receiver-Interface.ino)

Also:
https://github.com/dmadison/ServoInput

https://www.partsnotincluded.com/how-to-use-an-rc-controller-with-an-arduino/

**Interrupts** needed to efficiently read RC receiver PWD signal.
[Pin Reference and Docs](https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/)

**I2C** interface (for IMU)

[Pin Reference and Docs](https://www.arduino.cc/en/reference/wire)

[MPU6050 docs](https://components101.com/sensors/mpu6050-module)

### Tutorials

- A build very similar h/w [here](https://maker.pro/arduino/projects/build-arduino-self-balancing-robot/). Uses interupt for MPU
- A more interesting from [instructables](https://www.instructables.com/Arduino-Self-Balancing-Robot-1/). No interupt in this case

### Pins

- MotorA: 9 (PWM), 18, 19
- MotorB: 10 (PWM), 20, 21

- I2C (for IMU): 2 (SDA), 3 (SCL)

- RC Receiver: Available with interrupt 0,1 (serial RX, TX) and 7.

- Buzzer: Pin 6 (PWM)

Receiver to Arduino:

- CH1 -> PIN1 (frw/rev, pitch)
- CH2 -> PIN0 (left/right, roll)
- CH6 -> PIN7 (pot for trimming)
