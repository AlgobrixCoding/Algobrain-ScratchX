<h1 align="center">Algobrain-ScratchX</h1>

## *Table of contents*
- [Introduction](https://github.com/AlgobrixCoding/Algobrain-ScratchX#introduction)
- [Prerequisites](https://github.com/AlgobrixCoding/Algobrain-ScratchX#prerequisites--dependencies)
- [Installation](https://github.com/AlgobrixCoding/Algobrain-ScratchX#installation)

## *Introduction*
This repo includes the ScartchX extension for working with the [Algobrain Board](https://github.com/AlgobrixCoding/Algobrain-Board), as well as the files and arduino sketch required for the board.

## *Prerequisites \ Dependencies*
- [Algobrix's Algobrain Board](http://www.algobrix.com/)
- [Arduino Algobrain Board Package](https://github.com/AlgobrixCoding/Algobrain-Board)
- [Arduino IDE 1.69+](https://www.arduino.cc)
- [Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) Library Installed (Guide for installation is included below)
- [Micro USB](https://www.amazon.com/s?k=Micro%20USB)

## *Installation*
After following all the prerequisites and dependecies installations...
- **Download \ Clone this repo.**
- Open the **"ScratchX Device Plugin" folder** and install **"ScratchDevicePlugin"** (MSI for Windows, DMG for Mac)
- Open the Arduino folder (Default location: **C:\Program Files (x86)\Arduino**)
- Copy the **"Boards.h"** file to **"...\Arduino\libraries\Firmata"**
- Install the **Adafruit_NeoPixel library** from the Arduino IDE : Tools ---> Manage Libraries... ---> Search for Neopixel
![Neopixel](https://i.imgur.com/F80jUsA.png)
- Connect the Algobrain with the **Micro USB**
- Open **"StandardFirmata.ino"** (Arduino sketch) and upload it to the Algobrain.
- Open the following link **WITH IE11 Browser** and wait for the Algobrain to connect : [ScratchX Site Algobrain Extension](https://scratchx.org/?url=https://algobrixcoding.github.io/Algobrain-ScratchX/Algobrain.js)
