# Ultrasonic Ranging for Two Raspberry Pi

The purpose of this project is to obtain ranging estimation between two Raspberry Pi. To obtain accurate ranging, we deploy ultrasonic transmitter and receiver on a Raspberry Pi, and estimate the time-of-flight (ToF) to get the distance. The conventional two-way ranging method is applied. 

## Assembling Hardware

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
