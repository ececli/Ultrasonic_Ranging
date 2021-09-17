# Ultrasonic Ranging for Two Raspberry Pi

This branch stores the code with a different approach. In this approach, we introduced a Raspberry Pi Pico to control the buzzer, instead of using the Raspberry Pi 4B  to control the buzzer directly. 

## Assembling Hardware

In this approach, we use a Raspberry Pi 4B as micro-conputer, a Raspberry Pi Pico as a micro-controller, a buzzer (i.e., speaker) and a microphone. The buzzer we used is a piezo buzzer from TDK Corporation with model no. PS1720P02. Please check this [link](https://www.digikey.com/en/products/detail/tdk-corporation/PS1720P02/935932) for reference. Although this buzzer is not designed for ultrasound, we found that it works fine with freqeuncy around 20-30 kHz. The microphone we choose is [Adafruit I2S MEMS Microphone Breakout](https://www.adafruit.com/product/3421). 

It is simple to drive the buzzer. If you want the buzzer to generate a certain frequency sound, you can provide the same frequency square-wave to the buzzer. 



## Setup

To obtain the microphone data in Python, pyaudio package is needed. The installation instruction can be found [here](http://people.csail.mit.edu/hubert/pyaudio/). 


## Two-Way Ranging Algorithm



## To Do List

1. Measure multiple distances to get the constant to be substacted. 
2. Change different orientations to see if multi-path distortion affects much. May need to change the position of the speaker on the breadboard. 

## Known Issues
1. OverFlow at counter_NumRanging 198 when chunk size is 2048. Note that it works fine when increasing chunk size to 4096. 
2. Large outliers may happen. Need to know why. 
