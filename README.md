<h1 align="center">Algobrain-ScratchX</h1>

## *Table of contents*
- [Introduction](https://github.com/AlgobrixCoding/Algobrain-ScratchX#introduction)
- [Prerequisites](https://github.com/AlgobrixCoding/Algobrain-ScratchX#prerequisites--dependencies)
- [Installation](https://github.com/AlgobrixCoding/Algobrain-ScratchX#installation)

## *Introduction*
This repo includes the ScartchX extension for working with the Algobrain, as well as the files and arduino sketch required for the board.

## *Prerequisites \ Dependencies*
- [Purchase Algobrix's Algobrain Board](http://www.algobrix.com/)
- [Arduino IDE 1.69+](https://www.arduino.cc)
- [Arduino Algobrain Board Package](https://github.com/AlgobrixCoding/Algobrain-Board)
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
- Connect the Algobrain with the **Micro USB** to the computer.
- Open **"StandardFirmata.ino"** (Arduino sketch) and upload it to the Algobrain ([Learn how to upload here](https://github.com/AlgobrixCoding/Algobrain-Board)).
- Open the following link **WITH IE11 Browser** and wait for the Algobrain to connect : [ScratchX Site Algobrain Extension](https://scratchx.org/?url=https://algobrixcoding.github.io/Algobrain-ScratchX/Algobrain.js).

## *How do I know if the device is connected?*
1. After installing, once you plug the Algobrain to the computer (via the USB) you can see that the Battery Level LED will blink RED, this indicates that there is an attemp to connect with the ScratchX extension.
2. Once a connection has been made, you will see a green light in the extension screen and the "Play" LED on the Algobrain will also light up.
3. If the above (Step 2) didn't happen, try to remove and reconnect the Algobrain.
