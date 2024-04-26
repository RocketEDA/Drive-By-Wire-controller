# DBW-controller

DISCLAIMER: Please read the LICENSE file. Electronic throttle body control is not for the faint of heart. It can result in injury or death of road users. Test, verify, and test again before using on a road with other people. Understand the software and hardware, how they interact, apply good engineering principles and common sense.
As stated in the LICENSE file, by using this project in any way, you agree to take on full responsibilities to damages caused by this project.

Drive by wire controller, maps throttle to throttle body 1:1 with idle control
![PXL_20240426_063848482 MP](https://github.com/RocketEDA/Drive-By-Wire-controller/assets/38387138/7ef811e8-0b7b-4cad-bd3b-003bfd3a8697)

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

Optionally popuplate R11 and R12. The chrylser TB used a HAR3726 magnetic sensor which specifies for a 10k RL
![image](https://github.com/RocketEDA/Drive-By-Wire-controller/assets/38387138/7eb9880c-01ae-4ac9-8a78-45ac8a8d3134)

Assembly notes:
The capacitor and terminal blocks will need to moved to the other side on the BTS7960 to clear. I took this opportunity to upsize the capacitor as well, not sure if there's any appreciatable benefit.
![PXL_20240426_063914383](https://github.com/RocketEDA/Drive-By-Wire-controller/assets/38387138/6cba1674-f8b4-48e1-93e9-267b8330019c)

J2 and J4 connect to the TB TPS and e-pedal TPS.
Both e-pedal TPS's are positive direction, aka voltage increases as TPS approaches "open"
TB TPS has one signal inverted. To change this the calculation can be modified:
<img width="1093" alt="image" src="https://github.com/RocketEDA/Drive-By-Wire-controller/assets/38387138/6f09399e-d704-457d-9371-b631f7389733">

Upon first run, the "setup" switch should be set. Plugin a USB and set baud to 115200 to follow setup instructions. The potentiometer sets idle (minimum) throttle position. The equation is as follows. Insert a DMM probe between GND and IDLE to measure the pot's voltage. The idle % equation is: Idle position % = (IDLE voltage / 5) * (MAX_IDLE_TB_OPEN / 1023) * 100
![image](https://github.com/RocketEDA/Drive-By-Wire-controller/assets/38387138/0bfbf5d1-2be2-4a6b-aa37-aaa149661467)
![image](https://github.com/RocketEDA/Drive-By-Wire-controller/assets/38387138/4cfe2af5-3155-49a8-9cdc-dd3464e9fc7c)

