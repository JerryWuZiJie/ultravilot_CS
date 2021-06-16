# overview

The sentry design is **TODO onshape link**, and the one we built is shown in the pic below.

**TODO add pic**

## function

The code make the motor spin in random speed and random direction. When the sensors detect obstacles (two walls on the side), the motor will turn one way to avoid hitting to the walls. The sensors are using analog input which type-c board doesn't provide a user pin out for analog input (or maybe yes?) so we use Arduino to read the analog value and turn it into digit signal and send to the main MCU

The motor will start random spinning once power on

## wiring

Pin 13 and 12 from Arduino is connected to user pin 1 and 7 on the main MCU (type-c board). The ground pin of Arduino is connected to user pin 2. The sensors draw 5v supply from Arduino, and the white wires (analog readings of the sensor) are connect to **TODO**

## code

**TODO add MACRO usage**
