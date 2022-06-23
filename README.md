# PID Maximum Power Tracking

Implementation of a PID control for tracking the maximum power generation of a solar panel. 

The idea is to use an PID algorithm to improve the efficiency of an solar panel. The system uses 2 LDRs in each extreme of the solar plate, and according to the difference of luminosity betwen them, a servomotor makes the solar plate rotates to the point where the sun has more luminosity.

This codes were the first version of [this article](https://www.brazilianjournals.com/index.php/BRJD/article/view/25152/20054). In this version, the microcontroler is a simple ATMega328P and the supervisory program was built in [Processing](https://processing.org/).

The final version, on the article, the microcontroler is a Teensy 3.0 with 32-bit ARM Cortex-M4 chip and the supervisory program was built in [Elipse Scada Software](https://www.elipse.com.br/en/).

# Arduino

On the arduino side, this was the requirements:

- 2 LDRs
- 1 servomotor

Here is the schematic of all the components.

![arduino_schematic](https://github.com/marcos-moura97/pid-maximum-power-tracking/blob/main/arduino_schematic.PNG)

# Supervisory Program

The supervisory program enable the user to see how efficient the system is.

This version plots in which angle the solar plate is as well as some PID parameters, to track how stable the algorithm is.

![supervisory](https://github.com/marcos-moura97/pid-maximum-power-tracking/blob/main/supervisory.PNG)
