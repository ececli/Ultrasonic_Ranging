# Ultrasonic Ranging for Two Raspberry Pi

The purpose of this project is to obtain ranging estimation between two Raspberry Pi. To obtain accurate ranging, we deploy ultrasonic transmitter and receiver on a Raspberry Pi, and estimate the time-of-flight (ToF) to get the distance. The conventional two-way ranging method is applied. 

## Assembling Hardware

In this project, we use Raspberry Pi 4B as micro-controllers, and we add a buzzer (i.e., speaker) and a microphone to a Raspberry Pi. The buzzer we used is a piezo buzzer from TDK Corporation with model no. PS1720P02. Please check this [link](https://www.digikey.com/en/products/detail/tdk-corporation/PS1720P02/935932) for reference. Although this buzzer is not designed for ultrasound, we found that it works fine with freqeuncy around 20-30 kHz. The microphone we choose is [Adafruit I2S MEMS Microphone Breakout](https://www.adafruit.com/product/3421) and its wiring and setup instrcution to the Raspberry Pi can be found [here](https://learn.adafruit.com/adafruit-i2s-mems-microphone-breakout/raspberry-pi-wiring-test). 

In the beginning, we used Pin 12 and GND to connect the buzzer. But now we have changed to use Pin 4 and Pin 12 to connect the buzzer and use diffierential signaling to drive the buzzer. In this way, the buzzer generates sound with more power. 



## Setup

To run the code, pigpio package needs to be installed first. Detailed information about pigpio package can be found [here](https://abyz.me.uk/rpi/pigpio/). Based on the instruction from their [website](https://abyz.me.uk/rpi/pigpio/download.html), type:
```
$ sudo apt-get update
$ sudo apt-get install pigpio python-pigpio python3-pigpio
```
After installing pigpio package, we need to enable pigpiod, by
```
$ sudo pigpiod
```
or
```
sudo systemctl enable pigpiod
```


## Two-Way Ranging Algorithm

## To Do List

1. Test Murata Speaker at 40kHz
2. Use chirp or barker codes
3. Optimizing code (numba)
4. Compare different method (low pass filter vs. non-coherence detector)
5. Battery power up Raspberry Pi
6. Try "top" and "nice"
7. Analyzing results with histogram and cdf
8. Check [z-score algorithm](https://github.com/MatteoBattilana/robust-peak-detection-algorithm/blob/master/main.py) and related [link](https://stackoverflow.com/questions/22583391/peak-signal-detection-in-realtime-timeseries-data/22640362#22640362).
9. Use 2 pins with reversed wave sequence to the buzzer
