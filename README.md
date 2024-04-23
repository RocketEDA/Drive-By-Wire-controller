# DBW-controller
Drive by wire controller, allows DBW setups in all vehicles

Code is fully commented.

Schematic description:
arduino nano handles all the code and control. 
D2 has an input for switch SW2, signaling to enter setup mode
A6 has a potentiometer, handling idle control.
For servo drive a BTS7960 module from amazon is used.
![image](https://github.com/joesphan/DBW-controller/assets/38387138/a67a781d-94bb-44e6-97f9-704be2bd6566)
PCA9685 is used for better PWM control. PWM0 and PWM is used for the BTS, the rest are open
![image](https://github.com/joesphan/DBW-controller/assets/38387138/8d0d1d98-86ba-4d6f-be9b-3c5f5dc71ace)

J2 and J4 connect to the TB TPS and e-pedal TPS.

