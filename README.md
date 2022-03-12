
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


### 2. Hardware Assembling 

To assemble the microphone to the Raspberry Pi, follow the setup [here](https://learn.adafruit.com/adafruit-i2s-mems-microphone-breakout/raspberry-pi-wiring-test). Then, choose two pins on the Raspberry Pi and connect them to the two pins on the buzzer. The two pins on the Raspberry Pi can be one GPIO and one GND, or two GPIOs. If you choose two GPIOs, then you will use differential voltage, which provides a larger amplitude of the buzzer output. The figure below shows the hardware assembling, where Pin 4 and Pin 12 are used to connect the buzzer. 
<!---
![Figure 1: Version 1 Hardware](https://github.com/ececli/Ultrasonic_Ranging/blob/RPi-4B-Only/Images/IMG_20210908_164705.jpg)
--->
<p align="center">
<img src="/Images/IMG_20210908_164705.jpg" alt="drawing" width="400"/>
</p>
<p align="center">
Figure 1: Version 1 Hardware
</p>

Please note that it is simple to drive the buzzer. If you want the buzzer to generate a certain frequency sound, you can provide the same frequency square-wave to the buzzer. 




### 3. System Setup

To provide precise waveform the buzzer, the pigpio package needs to be installed first. Detailed information about pigpio package can be found [here](https://abyz.me.uk/rpi/pigpio/). Based on the instruction from their [website](https://abyz.me.uk/rpi/pigpio/download.html), type:
```
$ sudo apt-get update
$ sudo apt-get install pigpio python-pigpio python3-pigpio
```
After installing pigpio package, we need to enable pigpiod daemon, by
<!---
```
$ sudo pigpiod
```
or
--->
```
sudo systemctl enable pigpiod
```

To obtain the microphone data in Python, pyaudio package is needed. The installation instruction can be found [here](http://people.csail.mit.edu/hubert/pyaudio/). 

## Two-Way Ranging Algorithm

The two-way ranging algorithm contains two parts. One is receiving and detecting the signal, while the other part is sending an ultrasonic signal. The two parts work in turns. When the system sends the ultrasonic signal out, the microphone is paused receiving data. After the signal is sent out, the microphone is resumed to receive data and detect peaks. 
* **Sending Ultrasonic Signal**
	* We choose to use a simple signal currently. Specifically, a single-frequency signal with a certain duration is used. (Here we use a 25kHz signal with 4 ms). We use the pigpio library to generate a square waveform, which will be fed to the buzzer. This way can guarantee the square wave has exact frequency and duration. Please refer this [link](https://abyz.me.uk/rpi/pigpio/python.html#wave_add_generic) for details. 
* **Receiving Ultrasonic Signal**
	* We use the blocking-mode of the pyaudio to receive ultrasonic signals. The sampling rate is 64 kHz and the chunk size is set as 2048 (samples). Note that 2048 chunk size means that every 32 ms (2048/64000 seconds = 32 ms) the receiver gets 2048 samples. In the blocking-mode, it means that the signal processing part needs to be done within 32 ms so that the system can wait to receive the next 2048 samples. If the signal processing part takes more than 32 ms, then the system will throw *InputOverflow Error*. Therefore, be careful to choose the chunk size. Smaller chunk size will let the system detect the signal faster, while larger chunk size allows longer processing time before receiving the next chunk data. 
* **Signal Processing: Detecting Signal**
	*   To detect the ultrasonic signal, we proposed an improved matched filter method. Specifically, we use both sine and cosine as the reference signal of the matched filter. It can be proved that without noise, this detection method can achieve zero errors with the random phase of the received signal. In late 2021, we realized that the proposed method is equivalent to the Goertzel filter. 


## Contributing

Please read [CONTRIBUTING.md](/CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Authors & Main Contributors

* Chang Li (NIST in Gaithersburg, MD) 
* Lu Shi (NIST in Gaithersburg, MD)
* Sae Woo Nam (NIST in Boulder, CO)
* Nader Moayeri (NIST in Gaithersburg, MD)




<!--See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.-->


## Copyright

See [LICENSE.md](/LICENSE.md).

<!--
## Acknowledgments

*Note: Add this if you want to acknowledge people beyond the main contributors.*

* Hat tip to anyone whose code was used
* Inspiration
* etc
-->
## Contact

Please contact Chang Li (<chang.li@nist.gov>) or Nader Moayeri (<nader.moayeri@nist.gov>) if you have any questions. Thank you.
