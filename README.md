<h1 align="center">Algobrain-ScratchX</h1>

## *Table of contents*
- [Introduction](https://github.com/AlgobrixCoding/Algobrain-ScratchX#introduction)
- [Prerequisites](https://github.com/AlgobrixCoding/Algobrain-ScratchX#prerequisites--dependencies)
- [Installation](https://github.com/AlgobrixCoding/Algobrain-ScratchX#installation)

## *Introduction*
This repo includes the ScratchX extension for working with the Algobrain, as well as the files and arduino sketch required for the board.

## *Prerequisites \ Dependencies*
- [First, if needed purchase Algobrix's Algobrain Board here](http://www.algobrix.com/)
- [Download the Arduino IDE 1.69 or higher](https://www.arduino.cc)
- [Follow the installation instructions for the Arduino Algobrain Board Package](https://github.com/AlgobrixCoding/Algobrain-Board)
- [Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) Library Installed (Guide for installation is included below)
- [Micro USB](https://www.amazon.com/s?k=Micro%20USB)

## *Installation*
After following all the prerequisites and dependecies installations...
1. **Download \ Clone this repo.**<br>To download it, press the clone or download button. Then, unzip the folder where you want it.
2. Open the **"ScratchX Device Plugin" folder** and install **"ScratchDevicePlugin"** (MSI for Windows, DMG for Mac)
3. Open the Arduino folder (Default location: **C:\Program Files (x86)\Arduino**)
4. Copy the **"Boards.h"** file from this repository, to **"...\Arduino\libraries\Firmata"**
5. Install the **Adafruit_NeoPixel library** from the Arduino IDE : Tools ---> Manage Libraries... ---> Search for Neopixel
![Neopixel](https://i.imgur.com/F80jUsA.png)
6. Connect the Algobrain with the **Micro USB** to the computer.
7. Open **"StandardFirmata.ino"** from this repo (Arduino sketch) and upload it to the Algobrain ([Learn how to upload here](https://github.com/AlgobrixCoding/Algobrain-Board)).
8. Open the following link **WITH IE11 (Not Edge) Browser** and wait for the Algobrain to connect : [ScratchX Site Algobrain Extension](https://scratchx.org/?url=https://algobrixcoding.github.io/Algobrain-ScratchX/Algobrain.js).
9. Follow the **"ScratchX Algobrix Tutorial"** to understand better how to use this extension.

## *How do I know if the device is connected?*
1. After installing, once you plug the Algobrain to the computer (via the USB) you can see that the Battery Level LED will blink RED, this indicates that there is an attemp to connect with the ScratchX extension.
2. Once a connection has been made, you will see a green light in the extension screen and the "Play" LED on the Algobrain will also light up.
3. If the above (Step 2) didn't happen, try to remove and reconnect the Algobrain.
