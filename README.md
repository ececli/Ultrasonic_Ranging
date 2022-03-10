# Ultrasonic Ranging for Two Raspberry Pis

This is a part of the proximity detection system of our project. The project is to develop an accurate proximity detection system based on the Raspberry Pi platform, which integrates Bluetooth, ultrasonic, and/or UWB techniques. The initial purpose of this project is to help to blunt the spread of COVID-19 by deploying a contact tracing system. The proximity detection system can also be used for collision avoidance, robot localization system, etc.  The project is divided into multiple phases. 
* In the first phase, we used Bluetooth RSSI to estimate the distance between two devices, which could only provide a coarse estimation. 
* In the second phase, which is the current one, we are developing an accurate proximity detection system using an ultrasonic signal. Specifically, instead of using strength to estimate the distance between two devices, we use the time-of-flight (ToF) of the ultrasonic signal to estimate the distance. 
* In the third phase, we will investigate if the UWB technique can be applied to the Raspberry Pi platform to perform accurate ranging. 
* In the fourth phase, we will integrate all the techniques we developed into the proximity detection system. 

Note that one feature of the ultrasonic signal is that the signal is easily blocked by the obstacles such as walls and doors, compared with the RF signal such as Bluetooth, Wi-Fi, or UWB. It becomes an advantage in terms of the COVID-19 proximity detection, because when two persons are separated by a wall,  the system won't estimate a very close distance, even though the distance between the two persons is close.


## Project Status: [Active Development]

We have developed two versions of the prototype of the ultrasonic ranging system. Both versions use two-way ranging techniques. The difference between the versions is how the system acquires timestamps to calculate the ToF. 

* **Version 1** -  A completed version that uses a Raspberry Pi,  a piezoelectric buzzer, and a microelectromechanical systems (MEMS) microphone. The system records timestamps of sending the signal out and receiving the signal by `time.time()` in Python. Compared with Version 2, this version doesn't need an additional Raspberry Pi Pico. 

* **Version 2** - A developing version that uses a Raspberry Pi,  a piezoelectric buzzer, a MEMS microphone, and a Raspberry Pi Pico. The Raspberry Pi Pico controls the buzzer to send the ultrasonic signal. In this version, the microphone is turned on all the time. It not only detects the signal from the other device, but also detects the signal sending from its own device. The timestamps of the sending and receiving events are the indices of ultrasonic signal samples. Compared with Version 1, this version can obtain more accurate timestamps, which makes the ranging more accurate. 

The detailed descriptions can be found in each version, including the hardware assembling, the algorithm design, and performance evaluation. Here are the links to specific versions:
* [Version 1 - Use Raspberry Pi Only](https://github.com/ececli/Ultrasonic_Ranging/tree/RPi-4B-Only)
* [Version 2 - Use Raspberry Pi and Raspberry Pi Pico](https://github.com/ececli/Ultrasonic_Ranging/tree/RPi-4B-and-RPi-Pico)

Please note that the code in the main branch may not be the latest updates. 

