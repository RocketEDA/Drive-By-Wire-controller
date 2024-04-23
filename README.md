# DBW-controller

DISCLAIMER: Please read the LICENSE file. Electronic throttle body control is not for the faint of heart. It can result in injury or death of road users. Test, verify, and test again before using on a road with other people. Understand the software and hardware, how they interact, apply good engineering principles and common sense.
As stated in the LICENSE file, by using this project in any way, you agree to take on full responsibilities to damages caused by this project.

Drive by wire controller, maps throttle to throttle body 1:1 with idle control

For my application I used a Chrysler/mercedes 75mm throttle body with a corvette c6 throttle pedal.
P/N 0280750570
P/N 25835421

SOFTWARE:
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

