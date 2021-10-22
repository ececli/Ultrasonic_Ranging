# Ultrasonic Ranging for Two Raspberry Pi

This branch stores the code with a different approach. In this approach, we introduced a Raspberry Pi Pico to control the buzzer, instead of using the Raspberry Pi 4B  to control the buzzer directly. 

## Assembling Hardware

In this approach, we use a Raspberry Pi 4B as micro-conputer, a Raspberry Pi Pico as a micro-controller, a buzzer (i.e., speaker) and a microphone. The buzzer we used is a piezo buzzer from TDK Corporation with model no. PS1720P02. Please check this [link](https://www.digikey.com/en/products/detail/tdk-corporation/PS1720P02/935932) for reference. Although this buzzer is not designed for ultrasound, we found that it works fine with freqeuncy around 20-30 kHz. The microphone we choose is [Adafruit I2S MEMS Microphone Breakout](https://www.adafruit.com/product/3421). 

It is simple to drive the buzzer. If you want the buzzer to generate a certain frequency sound, you can provide the same frequency square-wave to the buzzer. 



## Setup

### 1. Install Raspberry Pi OS Lite

To install Raspberr Pi OS Lite headless, we need to first flash the OS image to the SD card. Here I used balenaEtcher to complete this step. Next, we need to enable SSH and setup Wi-Fi connections before putting the SD card to the Raspberry Pi. To do so, remove the SD card from the computer and inset it agian. 

To enable SSH, create an empty file named "ssh" without any extension. 

To setup Wi-Fi, create a text file. Then add the following information into the text file. Note that the ssid and psk is the name and password of the Wi-Fi. 

```
country=US
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev 
update_config=1
network={
    ssid="YourNetworkSSID"
    psk="Your Network's Passphrase"
}
```

Then, rename this file as "wpa_supplicant.conf". 

Now, we can remove the SD card from the computer and put it in the Raspberry Pi. 

### Install Microphone Driver

Please follow this [link](https://learn.adafruit.com/adafruit-i2s-mems-microphone-breakout/raspberry-pi-wiring-test) to install the i2s microphone driver. **Remember** to reboot the Raspberry Pi after updating the system:

```
sudo apt update
sudo apt upgrade
```
After the system is rebooted, install the pip if the system didn't pre-install it. 
```
sudo apt install python3-pip
```
Then, install the script:
```
cd ~
sudo pip3 install --upgrade adafruit-python-shell
wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/i2smic.py
sudo python3 i2smic.py
```
After the system is rebooted again, check if the microphone driver is successfully installed or not by 
```
arecord -l
```


## Install Necessary Packages

The Raspberry Pi OS Lite didn't pre-install packages like numpy, scipy, etc. We need to install them one by one. To install some basic Python packages, use:
```
sudo pip3 install numpy
sudo pip3 install scipy
```

To obtain the microphone data in Python, pyaudio package is needed. The installation instruction can be found [here](http://people.csail.mit.edu/hubert/pyaudio/). 


## Two-Way Ranging Algorithm



## To Do List

1. Measure multiple distances to get the constant to be substacted. 
2. Change different orientations to see if multi-path distortion affects much. May need to change the position of the speaker on the breadboard. 

## Known Issues
1. OverFlow at counter_NumRanging 198 when chunk size is 2048. Note that it works fine when increasing chunk size to 4096. 
2. Large outliers may happen. Need to know why. 
