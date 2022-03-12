
# Ultrasonic Ranging for Two Raspberry Pis

This is a part of the proximity detection system of our project. The project is to develop an accurate proximity detection system based on the Raspberry Pi platform, which integrates Bluetooth, ultrasonic, and/or UWB techniques. The initial purpose of this project is to help to blunt the spread of COVID-19 by deploying a contact tracing system. The proximity detection system can also be used for collision avoidance, robot localization system, etc.  The project is divided into multiple phases. 
* In the first phase, we used Bluetooth RSSI to estimate the distance between two devices, which could only provide a coarse estimation. 
* In the second phase, which is the current one, we are developing an accurate proximity detection system using an ultrasonic signal. Specifically, instead of using strength to estimate the distance between two devices, we use the time-of-flight (ToF) of the ultrasonic signal to estimate the distance. 
* In the third phase, we will investigate if the UWB technique can be applied to the Raspberry Pi platform to perform accurate ranging. 
* In the fourth phase, we will integrate all the techniques we developed into the proximity detection system. 

Note that one feature of the ultrasonic signal is that the signal is easily blocked by the obstacles such as walls and doors, compared with the RF signal such as Bluetooth, Wi-Fi, or UWB. It becomes an advantage in terms of the COVID-19 proximity detection, because when two persons are separated by a wall,  the system won't estimate a very close distance, even though the distance between the two persons is close.


## Project Status: [Active Development]

We have developed two versions of the prototype of the ultrasonic ranging system. Both versions use two-way ranging techniques. The difference between the versions is how the system acquires timestamps to calculate the ToF. 

* **Version 1 (Current Branch)** -  A completed version that uses a Raspberry Pi,  a piezoelectric buzzer, and a microelectromechanical systems (MEMS) microphone. The system records timestamps of sending the signal out and receiving the signal by `time.time()` in Python. Compared with Version 2, this version doesn't need an additional Raspberry Pi Pico. 

* **Version 2** - A developing version that uses a Raspberry Pi,  a piezoelectric buzzer, a MEMS microphone, and a Raspberry Pi Pico. The Raspberry Pi Pico controls the buzzer to send the ultrasonic signal. In this version, the microphone is turned on all the time. It not only detects the signal from the other device, but also detects the signal sending from its own device. The timestamps of the sending and receiving events are the indices of ultrasonic signal samples. Compared with Version 1, this version can obtain more accurate timestamps, which makes the ranging more accurate. 

The detailed descriptions can be found in each version, including the hardware assembling, the algorithm design, and performance evaluation. Here are the links to specific versions:
* [Version 1 - Use Raspberry Pi Only](https://github.com/ececli/Ultrasonic_Ranging/tree/RPi-4B-Only)
* [Version 2 - Use Raspberry Pi and Raspberry Pi Pico](https://github.com/ececli/Ultrasonic_Ranging/tree/RPi-4B-and-RPi-Pico)

## Testing Summary: [Complete Functional Tests]

The developed code has been tested on Raspberry Pi 4.

## Getting Started

### 1. Prerequisites

The ultrasonic ranging system requires at least 2 Raspberry Pis. In our project, we used [Raspberry Pi 4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/). In addition, a buzzer and a microphone are needed for each Raspberry Pi. The buzzer we used is a piezo buzzer from TDK Corporation with model no. PS1720P02. Please check this [link](https://www.digikey.com/en/products/detail/tdk-corporation/PS1720P02/935932) for reference. Although this buzzer is not designed for ultrasound, we found that it works fine with a frequency around 20-30 kHz. The microphone we choose is [Adafruit I2S MEMS Microphone Breakout](https://www.adafruit.com/product/3421) and its wiring and setup instruction to the Raspberry Pi can be found [here](https://learn.adafruit.com/adafruit-i2s-mems-microphone-breakout/raspberry-pi-wiring-test). 


To estimate the distance based on two-way ranging, we need to know 4 timestamps. Let's name the two devices by initiator and responder. The initiator first sends a signal to the responder. The responder listens and detects this signal. After the signal is detected, the responder sends a signal back, and the initiator listens and detects this signal. The 4 timestamps are defined as:
* $T_1$: The time when the signal is sent from the initiator. 
* $T_2$: The time when the signal is received by the responder.
* $T_3$: The time when the signal is sent from the responder. 
* $T_4$: The time when the signal is received by the initiator.

The distance between the two devices can be then estimated by
$D = v[(T_4-T_1) - (T_3-T_2)]/2$,
where $v$ is the speed of the sound. 


### 2. Hardware Assembling 

To assemble the microphone to the Raspberry Pi, follow the setup [here](https://learn.adafruit.com/adafruit-i2s-mems-microphone-breakout/raspberry-pi-wiring-test). 

Different from Version 1, in this version, we use a Raspberry Pi Pico to control the buzzer. Thus, the buzzer is connected to 2 pins on the Raspberry Pi Pico. Note that the Raspberry Pi Pico is powered by the Raspberry Pi via the USB port. In addition, we need one wire between the Raspberry Pi and the Raspberry Pi Pico. This wire is used by the Raspberry Pi to send a signal to Pico, telling Pico to send an ultrasound signal. The advantage of such a structure is that the Raspberry Pi is free of generating the square wave. In this way, the microphone can be turned on all the time. 

<!---
![Figure 1: Version 2 Hardware](https://github.com/ececli/Ultrasonic_Ranging/blob/RPi-4B-and-RPi-Pico/images/IMG_20210910_114748.jpg)
--->
<p align="center">
<img src="https://github.com/ececli/Ultrasonic_Ranging/blob/RPi-4B-and-RPi-Pico/images/IMG_20210910_114748.jpg" alt="drawing" width="400"/>
</p>
<p align="center">
Figure 1: Version 2 Hardware
</p>

Please note that it is simple to drive the buzzer. If you want the buzzer to generate a certain frequency sound, you can provide the same frequency square-wave to the buzzer. 



### 3. System Setup

To obtain the microphone data in Python, pyaudio package is needed. The installation instruction can be found [here](http://people.csail.mit.edu/hubert/pyaudio/). 

_The other part of system setup will be finished later._
