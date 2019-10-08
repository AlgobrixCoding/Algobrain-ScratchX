/*
 *This program is free software: you can redistribute it and/or modify
 *it under the terms of the GNU General Public License as published by
 *the Free Software Foundation, either version 3 of the License, or
 *(at your option) any later version.
 *
 *This program is distributed in the hope that it will be useful,
 *but WITHOUT ANY WARRANTY; without even the implied warranty of
 *MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *GNU General Public License for more details.
 *
 *You should have received a copy of the GNU General Public License
 *along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

(function (ext) {

    var PIN_MODE = 0xF4,
        REPORT_DIGITAL = 0xD0,
        REPORT_ANALOG = 0xC0,
        DIGITAL_MESSAGE = 0x90,
        START_SYSEX = 0xF0,
        END_SYSEX = 0xF7,
        QUERY_FIRMWARE = 0x79,
        REPORT_VERSION = 0xF9,
        ANALOG_MESSAGE = 0xE0,
        ANALOG_MAPPING_QUERY = 0x69,
        ANALOG_MAPPING_RESPONSE = 0x6A,
        CAPABILITY_QUERY = 0x6B,
        CAPABILITY_RESPONSE = 0x6C,
        NEOPIXEL = 0x72,
        NEOPIXEL_REGISTER = 0x73,
        PULSE_IN = 0x74,
        PULSE_IN_RESPONSE = 0x75;

    var INPUT = 0x00,
        OUTPUT = 0x01,
        ANALOG = 0x02,
        PWM = 0x03,
        SERVO = 0x04,
        SHIFT = 0x05,
        I2C = 0x06,
        ONEWIRE = 0x07,
        STEPPER = 0x08,
        ENCODER = 0x09,
        SERIAL = 0x0A,
        PULLUP = 0x0B,
        IGNORE = 0x7F,
        TOTAL_PIN_MODES = 13;

    var LOW = 0,
        HIGH = 1;

    var MAX_DATA_BYTES = 4096;
    var MAX_PINS = 128;

    var parsingSysex = false,
        waitForData = 0,
        executeMultiByteCommand = 0,
        multiByteChannel = 0,
        sysexBytesRead = 0,
        storedInputData = new Uint8Array(MAX_DATA_BYTES);

    var digitalOutputData = new Uint8Array(16),
        digitalInputData = new Uint8Array(16),
        analogInputData = new Uint16Array(16);

    var analogChannel = new Uint8Array(MAX_PINS);
    var pinModes = [];
    for (var i = 0; i < TOTAL_PIN_MODES; i++) pinModes[i] = [];

    var majorVersion = 0,
        minorVersion = 0;

    var connected = false;
    var notifyConnection = false;
    var device = null;
    var inputData = null;

    // TEMPORARY WORKAROUND
    // Since _deviceRemoved is not used with Serial devices
    // ping device regularly to check connection
    var pinging = false;
    var pingCount = 0;
    var pinger = null;

    var hwList = new HWList();

    // Algobrain Related :
    // Pinout
    var Motor_A_PWM_Pin = 9,
        Motor_A_Dir_Pin = 10,
        Motor_B_PWM_Pin = 3,
        Motor_B_Dir_Pin = 2,
        Motor_C_PWM_Pin = 5,
        Motor_C_Dir_Pin = 6,
        Motor_Sleep_Pin = 13,
        Led_A_Pin = 19,
        Led_B_Pin = 18,
        PlayLedPin = 4;
        Sensor_A_Pin = 17,
        Sensor_B_Pin = 16;
    // Pulse In Function
    var pulseInBuffer = [];
    var pulseInResult;
    var sensorValue;
    var newPulseInResult = false;

    function HWList() {
        this.devices = [];

        this.add = function (dev, pin) {
            var device = this.search(dev);
            if (!device) {
                device = {
                    name: dev,
                    pin: pin,
                    val: 0
                };
                this.devices.push(device);
            } else {
                device.pin = pin;
                device.val = 0;
            }
        };

        this.search = function (dev) {
            for (var i = 0; i < this.devices.length; i++) {
                if (this.devices[i].name === dev)
                    return this.devices[i];
            }
            return null;
        };
    }

    function init() {

        for (var i = 0; i < 16; i++) {
            var output = new Uint8Array([REPORT_DIGITAL | i, 0x01]);
            device.send(output.buffer);
        }
        queryCapabilities();

        // TEMPORARY WORKAROUND
        // Since _deviceRemoved is not used with Serial devices
        // ping device regularly to check connection
        pinger = setInterval(function () {
            if (pinging) {
                if (++pingCount > 6) {
                    clearInterval(pinger);
                    pinger = null;
                    connected = false;
                    if (device) device.close();
                    device = null;
                    return;
                }
            } else {
                if (!device) {
                    clearInterval(pinger);
                    pinger = null;
                    return;
                }
                queryFirmware();
                pinging = true;
            }
        }, 100);
    }

    function hasCapability(pin, mode) {
        if (pinModes[mode].indexOf(pin) > -1)
            return true;
        else
            return false;
    }

    function queryFirmware() {
        var output = new Uint8Array([START_SYSEX, QUERY_FIRMWARE, END_SYSEX]);
        device.send(output.buffer);
    }

    function queryCapabilities() {
        console.log('Querying ' + device.id + ' capabilities');
        var msg = new Uint8Array([
            START_SYSEX, CAPABILITY_QUERY, END_SYSEX
        ]);
        device.send(msg.buffer);
    }

    function queryAnalogMapping() {
        console.log('Querying ' + device.id + ' analog mapping');
        var msg = new Uint8Array([
            START_SYSEX, ANALOG_MAPPING_QUERY, END_SYSEX
        ]);
        device.send(msg.buffer);
    }

    function setDigitalInputs(portNum, portData) {
        digitalInputData[portNum] = portData;
    }

    function setAnalogInput(pin, val) {
        analogInputData[pin] = val;
    }

    function setVersion(major, minor) {
        majorVersion = major;
        minorVersion = minor;
    }

    function processSysexMessage() {
        switch (storedInputData[0]) {
            case CAPABILITY_RESPONSE:
                for (var i = 1, pin = 0; pin < MAX_PINS; pin++) {
                    while (storedInputData[i++] != 0x7F) {
                        pinModes[storedInputData[i - 1]].push(pin);
                        i++; //Skip mode resolution
                    }
                    if (i == sysexBytesRead) break;
                }
                queryAnalogMapping();
                break;
            case ANALOG_MAPPING_RESPONSE:
                for (var pin = 0; pin < analogChannel.length; pin++)
                    analogChannel[pin] = 127;
                for (var i = 1; i < sysexBytesRead; i++)
                    analogChannel[i - 1] = storedInputData[i];
                for (var pin = 0; pin < analogChannel.length; pin++) {
                    if (analogChannel[pin] != 127) {
                        var out = new Uint8Array([
                            REPORT_ANALOG | analogChannel[pin], 0x01
                        ]);
                        device.send(out.buffer);
                    }
                }
                notifyConnection = true;
                setTimeout(function () {
                    notifyConnection = false;
                }, 100);
                break;
            case QUERY_FIRMWARE:
                if (!connected) {
                    clearInterval(poller);
                    poller = null;
                    clearTimeout(watchdog);
                    watchdog = null;
                    connected = true;
                    setTimeout(init, 200);
                    setTimeout(setupPeripherals, 500);
                }
                pinging = false;
                pingCount = 0;
                break;
            case PULSE_IN_RESPONSE:
                pulseInBuffer.length = 0; // Clear the buffer
                for(var i = 1; i < sysexBytesRead; i++)
                    pulseInBuffer.push(storedInputData[i]);
                // Debug :
                // console.log("FROM PULSE_IN_RESPONSE: " + String.fromCharCode.apply(String, pulseInBuffer));
                pulseInResult = parseInt(String.fromCharCode.apply(String, pulseInBuffer));
                newPulseInResult = true;
                break;
        }
    }

    function processInput(inputData) {
        for (var i = 0; i < inputData.length; i++) {
            if (parsingSysex) {
                if (inputData[i] == END_SYSEX) {
                    parsingSysex = false;
                    processSysexMessage();
                } else {
                    storedInputData[sysexBytesRead++] = inputData[i];
                }
            } else if (waitForData > 0 && inputData[i] < 0x80) {
                storedInputData[--waitForData] = inputData[i];
                if (executeMultiByteCommand !== 0 && waitForData === 0) {
                    switch (executeMultiByteCommand) {
                        case DIGITAL_MESSAGE:
                            setDigitalInputs(multiByteChannel, (storedInputData[0] << 7) + storedInputData[1]);
                            break;
                        case ANALOG_MESSAGE:
                            setAnalogInput(multiByteChannel, (storedInputData[0] << 7) + storedInputData[1]);
                            break;
                        case REPORT_VERSION:
                            setVersion(storedInputData[1], storedInputData[0]);
                            break;
                    }
                }
            } else {
                if (inputData[i] < 0xF0) {
                    command = inputData[i] & 0xF0;
                    multiByteChannel = inputData[i] & 0x0F;
                } else {
                    command = inputData[i];
                }
                switch (command) {
                    case DIGITAL_MESSAGE:
                    case ANALOG_MESSAGE:
                    case REPORT_VERSION:
                        waitForData = 2;
                        executeMultiByteCommand = command;
                        break;
                    case START_SYSEX:
                        parsingSysex = true;
                        sysexBytesRead = 0;
                        break;
                }
            }
        }
    }

    function pinMode(pin, mode) {
        var msg = new Uint8Array([PIN_MODE, pin, mode]);
        device.send(msg.buffer);
    }

    function analogRead(pin) {
        if (pin >= 0 && pin < pinModes[ANALOG].length) {
            return Math.round((analogInputData[pin] * 100) / 1023);
        } else {
            var valid = [];
            for (var i = 0; i < pinModes[ANALOG].length; i++)
                valid.push(i);
            console.log('ERROR: valid analog pins are ' + valid.join(', '));
            return;
        }
    }

    function digitalRead(pin) {
        if (!hasCapability(pin, INPUT)) {
            console.log('ERROR: valid input pins are ' + pinModes[INPUT].join(', '));
            return;
        }
        pinMode(pin, INPUT);
        return (digitalInputData[pin >> 3] >> (pin & 0x07)) & 0x01;
    }

    function analogWrite(pin, val) {
        if (!hasCapability(pin, PWM)) {
            console.log('ERROR: valid PWM pins are ' + pinModes[PWM].join(', '));
            return;
        }
        if (val < 0) val = 0;
        else if (val > 255) val = 255;
        pinMode(pin, PWM);
        var msg = new Uint8Array([
            ANALOG_MESSAGE | (pin & 0x0F),
            val & 0x7F,
            val >> 7
        ]);
        device.send(msg.buffer);
    }

    function digitalWrite(pin, val) {
        if (!hasCapability(pin, OUTPUT)) {
            console.log('ERROR: valid output pins are ' + pinModes[OUTPUT].join(', '));
            return;
        }
        var portNum = (pin >> 3) & 0x0F;
        if (val == LOW)
            digitalOutputData[portNum] &= ~(1 << (pin & 0x07));
        else
            digitalOutputData[portNum] |= (1 << (pin & 0x07));
        pinMode(pin, OUTPUT);
        var msg = new Uint8Array([
            DIGITAL_MESSAGE | portNum,
            digitalOutputData[portNum] & 0x7F,
            digitalOutputData[portNum] >> 0x07
        ]);
        device.send(msg.buffer);
    }

    function rotateServo(pin, deg) {
        if (!hasCapability(pin, SERVO)) {
            console.log('ERROR: valid servo pins are ' + pinModes[SERVO].join(', '));
            return;
        }
        pinMode(pin, SERVO);
        var msg = new Uint8Array([
            ANALOG_MESSAGE | (pin & 0x0F),
            deg & 0x7F,
            deg >> 0x07
        ]);
        device.send(msg.buffer);
    }
    
    // Algobrain Functions :
    function setupMotors() {
        pinMode(Motor_A_Dir_Pin, OUTPUT);
        pinMode(Motor_A_PWM_Pin, OUTPUT);
        pinMode(Motor_B_Dir_Pin, OUTPUT);
        pinMode(Motor_B_PWM_Pin, OUTPUT);
        pinMode(Motor_C_Dir_Pin, OUTPUT);
        pinMode(Motor_C_PWM_Pin, OUTPUT);
        pinMode(Motor_Sleep_Pin, OUTPUT);
        digitalWrite(Motor_Sleep_Pin, HIGH);
    }

    function setupLeds() {
        registerNeopixel(Led_A_Pin, 1);
        setNeopixel(0, 0, 0, 0);
        registerNeopixel(Led_B_Pin, 1);
        setNeopixel(0, 0, 0, 0);
    }

    function setupSensors() {
        pinMode(Sensor_A_Pin, INPUT);
        pinMode(Sensor_B_Pin, INPUT);
    }

    function setupPeripherals() {
        // Need some real hardware indication to know when the device has been connected. (Debug and UI)
        pinMode(PlayLedPin, OUTPUT);
        digitalWrite(PlayLedPin, HIGH);
        setupSensors();
        setupMotors();
        setupLeds();
        console.log("Algobrain Version 3.0 - Setup Complete ");
    }
    
    function setMotor(motorId, dir, pwm) {
        setMotorSpeed(motorId, pwm);
        setMotorDir(motorId, dir);
        // Debug :
        // console.log("Moving Motor ", motorId, " In ", dir, " Direction at ", pwm, " PWM");
    }
    
    function setMotorSpeed(motorId, pwm) {
        switch (motorId) {
            case 'A':
                analogWrite(Motor_A_PWM_Pin, pwm);
                break;
            case 'B':
                analogWrite(Motor_B_PWM_Pin, pwm);
                break;
            case 'C':
                analogWrite(Motor_C_PWM_Pin, pwm);
                break;
            default:
                break;
        }
    }

    function setMotorDir(motorId, _dir) {
        var dir = 0; // Default is Clockwise
        if (_dir != menus[lang].motorDirection[0]) {
            dir = 1; // Set as Counter Clockwise
        }
        switch (motorId) {
            case 'A':
                digitalWrite(Motor_A_Dir_Pin, dir);
                break;
            case 'B':
                digitalWrite(Motor_B_Dir_Pin, dir);
                break;
            case 'C':
                digitalWrite(Motor_C_Dir_Pin, dir);
                break;
        }
    }

    function registerNeopixel(pin, count) {
        var msg = new Uint8Array([
            START_SYSEX,
            NEOPIXEL_REGISTER,
            pin,
            count,
            END_SYSEX
        ]);
        device.send(msg.buffer);
    }

    function setNeopixel(index, red, green, blue) {
        var msg = new Uint8Array([
            START_SYSEX,
            NEOPIXEL,
            index,
            red,
            green,
            blue,
            END_SYSEX
        ]);
        device.send(msg.buffer);
    }

    function pulseIn(pin, value, pulseDuration, timeout) {
        /*
        >> The protocol is as follows:
        >> To Request a Pulse In
        >> START_SYSEX(0xF0)
        >> pulseIn / pulseOut(0x74)
        >> pin(0-127)
        >> value(1 or 0, HIGH or LOW)
        >> pulseDuration 0 (LS Byte)
        >> pulseDuration 1
        >> pulseDuration 2
        >> pulseDuration 3 (MS Byte)
        >> pulseTimeout 0
        >> pulseTimeout 1
        >> pulseTimeout 2
        >> pulseTimeout 3
        >> END_SYSEX(0xF7)

        >> pulseDuration : 4 byte long value that specifies that amount of time to do the pulseIn \ pulseOut.
        >> pulseTimeout : 4 byte long value passed to the pulseIn function as timeout.
        */
        var pulseDurationByteArray = longToByteArray(pulseDuration);
        var timeoutByteArray = longToByteArray(timeout);
        var msg = new Uint8Array([
            START_SYSEX,
            PULSE_IN,
            pin,
            value,
            pulseDurationByteArray[0],
            pulseDurationByteArray[1],
            pulseDurationByteArray[2],
            pulseDurationByteArray[3],
            timeoutByteArray[0],
            timeoutByteArray[1],
            timeoutByteArray[2],
            timeoutByteArray[3],
            END_SYSEX
        ]);
        device.send(msg.buffer);
        /*
        >> After the request you get the following response:
        >> START_SYSEX(0xF0)
        >> pulseIn/pulseOut Response(0x75)
        >> resultBytes[]
        >> END_SYSEX(0xF7)

        >> duration : 4 byte long value of the returned pulse duration from pulseIn
        */
    }

    function longToByteArray(long) {
        // we want to represent the input as a 8-bytes array
        var byteArray = [0, 0, 0, 0];

        for ( var index = 0; index < byteArray.length; index ++ ) {
            var byte = long & 0xff;
            byteArray [ index ] = byte;
            long = (long - byte) / 256;
        }
        // For debug.
        // console.log("byteArray: ");
        // console.log(byteArray);
        // console.log(byteArrayToLong(byteArray));
        return byteArray;
    };
    
    function byteArrayToLong(byteArray) {
        var value = 0;
        for ( var i = byteArray.length - 1; i >= 0; i--) {
            value = (value * 256) + byteArray[i] * 1;
        }
        return value;
    };

    // Ends here.

    ext.whenConnected = function () {
        if (notifyConnection) return true;
        return false;
    };

    ext.analogWrite = function (pin, val) {
        analogWrite(pin, val);
    };

    ext.digitalWrite = function (pin, val) {
        if (val == menus[lang]['outputs'][0])
            digitalWrite(pin, HIGH);
        else if (val == menus[lang]['outputs'][1])
            digitalWrite(pin, LOW);
    };

    ext.analogRead = function (pin) {
        return analogRead(pin);
    };

    ext.digitalRead = function (pin) {
        return digitalRead(pin);
    };

    ext.whenAnalogRead = function (pin, op, val) {
        if (pin >= 0 && pin < pinModes[ANALOG].length) {
            if (op == '>')
                return analogRead(pin) > val;
            else if (op == '<')
                return analogRead(pin) < val;
            else if (op == '=')
                return analogRead(pin) == val;
            else
                return false;
        }
    };

    ext.whenDigitalRead = function (pin, val) {
        if (hasCapability(pin, INPUT)) {
            if (val == menus[lang]['outputs'][0])
                return digitalRead(pin);
            else if (val == menus[lang]['outputs'][1])
                return digitalRead(pin) === false;
        }
    };

    ext.connectHW = function (hw, pin) {
        hwList.add(hw, pin);
    };

    ext.setLED = function (led, val) {
        var hw = hwList.search(led);
        if (!hw) return;
        analogWrite(hw.pin, val);
        hw.val = val;
    };

    ext.changeLED = function (led, val) {
        var hw = hwList.search(led);
        if (!hw) return;
        var b = hw.val + val;
        if (b < 0) b = 0;
        else if (b > 100) b = 100;
        analogWrite(hw.pin, b);
        hw.val = b;
    };

    ext.digitalLED = function (led, val) {
        var hw = hwList.search(led);
        if (!hw) return;
        if (val == 'on') {
            digitalWrite(hw.pin, HIGH);
            hw.val = 255;
        } else if (val == 'off') {
            digitalWrite(hw.pin, LOW);
            hw.val = 0;
        }
    };

    ext.readInput = function (name) {
        var hw = hwList.search(name);
        if (!hw) return;
        return analogRead(hw.pin);
    };

    ext.whenButton = function (btn, state) {
        var hw = hwList.search(btn);
        if (!hw) return;
        if (state === 'pressed')
            return digitalRead(hw.pin);
        else if (state === 'released')
            return !digitalRead(hw.pin);
    };

    ext.isButtonPressed = function (btn) {
        var hw = hwList.search(btn);
        if (!hw) return;
        return digitalRead(hw.pin);
    };

    ext.whenInput = function (name, op, val) {
        var hw = hwList.search(name);
        if (!hw) return;
        if (op == '>')
            return analogRead(hw.pin) > val;
        else if (op == '<')
            return analogRead(hw.pin) < val;
        else if (op == '=')
            return analogRead(hw.pin) == val;
        else
            return false;
    };

    ext.mapValues = function (val, aMin, aMax, bMin, bMax) {
        var output = (((bMax - bMin) * (val - aMin)) / (aMax - aMin)) + bMin;
        return Math.round(output);
    };

    ext._getStatus = function () {
        if (!connected)
            return {
                status: 1,
                msg: 'Disconnected'
            };
        else
            return {
                status: 2,
                msg: 'Connected'
            };
    };

    ext._deviceRemoved = function (dev) {
        console.log('Device removed');
        // Not currently implemented with serial devices
        if (device != dev) return;
        if (poller) poller = clearInterval(poller);
        device = null;
    };

    var potentialDevices = [];
    ext._deviceConnected = function (dev) {
        potentialDevices.push(dev);
        if (!device)
            tryNextDevice();
    };

    var poller = null;
    var watchdog = null;

    function tryNextDevice() {
        device = potentialDevices.shift();
        if (!device) return;

        device.open({
            bitRate: 57600, // This is the baudrate Firmate is using, dont change.
            stopBits: 0,
            ctsFlowControl: 0

        });
        console.log('Attempting connection with ' + device.id);
        device.set_receive_handler(function (data) {
            var inputData = new Uint8Array(data);
            processInput(inputData);
        });

        poller = setInterval(function () {
            queryFirmware();
        }, 1000);

        watchdog = setTimeout(function () {
            clearInterval(poller);
            poller = null;
            device.set_receive_handler(null);
            device.close();
            device = null;
            tryNextDevice();
        }, 5000);
    }

    ext._shutdown = function () {
        // TODO: Bring all pins down
        if (device) device.close();
        if (poller) clearInterval(poller);
        device = null;
    };

    // Algobrain ext Functions (ScratchX Blocks) :
    
    // Motors
    ext.setMotor = function (motorId, dir, pwm, seconds, callback) {
        setMotor(motorId, dir, pwm);
        if (seconds < 0) { // Infinity \ Forever
            callback();
        } else {
            setTimeout(function () {
                setMotor(motorId, dir, 0);
                callback();
            }, seconds * 1000);
        }
    };
    
    ext.setMotorHE = function (seconds, pwm, dir, motorId, callback) {
        ext.setMotor(motorId, dir, pwm, seconds, callback);
    };
    
    ext.setMotorForever = function (motorId, dir, pwm, callback) {
        ext.setMotor(motorId, dir, pwm, -1, callback);
    };

    ext.setMotorForeverHE = function (pwm, dir, motorId, callback) {
        ext.setMotorForever(motorId, dir, pwm, callback);
    };

    ext.moveRobot = function(robotDir, numSteps, callback) {
        var moveRobotTime = 0;
        if(robotDir == menus[lang].robotDirection[0]) {
            // Forward
            setMotor('A', menus[lang].robotDirection[1], 170);
            setMotor('B', menus[lang].robotDirection[0], 170);
        } else {
            // Backwards
            setMotor('A', menus[lang].robotDirection[0], 170);
            setMotor('B', menus[lang].robotDirection[1], 170);
        }
        switch(numSteps) {
            case 0:
                moveRobotTime = 0;
                break;
            case 1:
                moveRobotTime = 1400;
                break;
            case 2:
                moveRobotTime = 2750;
                break;
            case 3:
                moveRobotTime = 4100;
                break;
            default:
                moveRobotTime = 1500 * numSteps;
                break;
        }
        setTimeout(function() {
            setMotor('A', menus[lang].robotDirection[0], 0);
            setMotor('B', menus[lang].robotDirection[0], 0);
            callback();
        }, moveRobotTime);
    };
    
    ext.moveRobotHE = function(numSteps, robotDir, callback) {
        ext.moveRobot(robotDir, numSteps, callback);
    };

    ext.rotateRobot = function (robotRotate, degrees, callback) {
        // CW - Right , CCW - Left
        var rotateDir = (robotRotate == menus[lang].robotRotation[0]) ? menus[lang].robotRotation[1] : menus[lang].robotRotation[0];
        
        setMotor('A', rotateDir, 85);
        setMotor('B', rotateDir, 85);
        
        // Calculate the time of the given degrees
        // 90 degrees is 2.2 Milliseconds, we use this to calculate the right time for the given degrees of rotation
        var deltaTime = (degrees % 360.0) * 2200 / 90;
        setTimeout(function() {
            setMotor('A', rotateDir, 0);
            setMotor('B', rotateDir, 0);
            callback();
        }, deltaTime)
    };

    ext.rotateRobotHE = function (degrees, robotRotate, callback) {
        ext.rotateRobot(robotRotate, degrees, callback);
    };

    // LED's
    ext.setNeopixelColor = function (ledSelect, color, seconds, callback) {
        var LedPin = (ledSelect == '1') ? Led_A_Pin : Led_B_Pin;    
        registerNeopixel(LedPin, 1);
        switch(color) {
            case menus[lang].ledColor[0]:
                setNeopixel(0, 255, 0, 0);
                break;
            case menus[lang].ledColor[1]:
                setNeopixel(0, 0, 255, 0);
                break;
            case menus[lang].ledColor[2]:
                setNeopixel(0, 0, 0, 255);
                break;
        }
        if(seconds < 0) {  // Infinity \ Forever
            callback();
        } else {
            setTimeout(function () {
                setNeopixel(0, 0, 0, 0);
                callback();
            }, seconds * 1000);
        }
    };
    
    ext.setNeopixelColorHE = function (seconds, color, ledSelect, callback) {
      ext.setNeopixelColor(ledSelect, color, seconds, callback);
    };
    
    ext.setNeopixelColorForever = function (ledSelect, color, callback) {
        ext.setNeopixelColor(ledSelect, color, -1, callback);
    };
    
    ext.setNeopixelColorForeverHE = function (color, ledSelect, callback) {
        ext.setNeopixelColorForever(ledSelect, color, callback);
    };
    
    ext.setNeopixel = function (ledSelect, red, green, blue, seconds, callback) {
        var LedPin = (ledSelect == '1') ? Led_A_Pin : Led_B_Pin;
        registerNeopixel(LedPin, 1);
        setNeopixel(0, red, green, blue);
        if(seconds < 0) { // Infinity \ Forever
            callback();
        } else {
            setTimeout(function () {
                setNeopixel(0, 0, 0, 0);
                callback();
            }, seconds * 1000);
        }
    };

    ext.setNeopixelHE = function (seconds, blue, green, red, ledSelect, callback) {
        ext.setNeopixel(ledSelect, red, green, blue, seconds, callback);
    };

    ext.setNeopixelForever = function (ledSelect, red, green, blue, callback) {
        ext.setNeopixel(ledSelect, red, green, blue, -1, callback);
    };

    ext.setNeopixelForeverHE = function (blue, green, red, ledSelect, callback) {
        ext.setNeopixelForever(ledSelect, red, green, blue, callback);
    }

    // Wait \ Sensors
    ext.waitSeconds = function(seconds, callback) {
        setTimeout(function() {
            callback();
        }, seconds * 1000);
    };
    
    ext.getSensor = function (sensorId) {
        var mSensorId = (sensorId == menus[lang].sensorSelection[0]) ? Sensor_A_Pin : Sensor_B_Pin;
        var pulseTimeout = 4000; // 4000 us --> 4 ms (0FA0)
        var cycleTime = 2000; // 2000 us --> 2 ms (07D0)
        var isInternalTimeout = false;
        newPulseInResult = false; // Reset the result flag
        
        setTimeout(function() {
            isInternalTimeout = true;
        }, 100); // 100 ms timeout

        pulseIn(mSensorId, HIGH, cycleTime, pulseTimeout); // Send a pulseIn request to the board
        
        return wait();

        // The wait function will wait for the board to return a response
        function wait() {
            if(isInternalTimeout || newPulseInResult) {
                if(isInternalTimeout) // No answer after 100 milliseconds ? 
                    return 0;
                return getSensor(pulseInResult);
            }
            setTimeout(wait, 0);
        }
        
        function getSensor(pwmValueH) {
            var dutyCycle = 0;
            if (pwmValueH != 0)
                dutyCycle = (pwmValueH / cycleTime) * 100.0;
            else if (digitalRead(mSensorId) > 0)
                dutyCycle = 100;
            console.log("dutyCycle Value: " + dutyCycle / 10);
            console.log("getSensor Value: " + Math.round(dutyCycle / 10));
            sensorValue = Math.round(dutyCycle / 10);
            return sensorValue;
        }
    };
    
    ext.waitSensor = function(sensorId, sensorLevel, callback) {
        /*
        Documentation on the Distance Sensor:
        Senses distance between 2 - 100 centimeters
        The 100 cm is divided between 0-10 in the code
        0-2 cm will match sensor output of 0 --> Sense below 2 cm --> False
        2-20 .......................... of 1-2 --> Low / True
        20-50 ......................... of 3-5 --> Med / True
        50-100 ........................ of 5-10 --> High / True
        100+ ...........................of 0 --> Sense above 100 cm --> False
        */
        
        // Possible Levels :
        // 'Low', 'Medium', 'High', 'Low-Medium', 'Medium-High', 'True', 'False'

        var mSensorValue;
        var isLevelDetected = false;
        ext.getSensor(sensorId);
        mSensorValue = sensorValue;
        switch (sensorLevel) {
            case menus[lang].sensorLevels[0]: // Low
                isLevelDetected = (1 <= mSensorValue && mSensorValue <= 2);
                break;
            case menus[lang].sensorLevels[1]: // Medium
                isLevelDetected = (3 <= mSensorValue && mSensorValue <= 5);
                break;
            case menus[lang].sensorLevels[2]: // High
                isLevelDetected = (6 <= mSensorValue && mSensorValue <= 10);
                break;
            case menus[lang].sensorLevels[3]: // Low-Medium
                isLevelDetected = (1 <= mSensorValue && mSensorValue <= 5);
                break;
            case menus[lang].sensorLevels[4]: // Medium-High
                isLevelDetected = (3 <= mSensorValue && mSensorValue <= 10);
                break;
            case menus[lang].sensorLevels[5]: // True
                isLevelDetected = (mSensorValue != 0);
                break;
            case menus[lang].sensorLevels[6]: // False
                isLevelDetected = (mSensorValue == 0);
                break;  
        }
        if(isLevelDetected)
            callback();
        else
            setTimeout(function() {
                ext.waitSensor(sensorId, sensorLevel, callback);
            }, 50);
    };

    // Ends Here.

    // Check for GET param 'lang'
    var paramString = window.location.search.replace(/^\?|\/$/g, '');
    var vars = paramString.split("&");
    var lang = 'en';
    for (var i = 0; i < vars.length; i++) {
        var pair = vars[i].split('=');
        if (pair.length > 1 && pair[0] == 'lang')
            lang = pair[1];
    }

    var blocks = {
        en: [
            // Algobrain Blocks :
            // ['--'], // Motors
            ['w', 'Motor %m.motorSelection %m.motorDirection at %n power for %n seconds', 'setMotor', 'A', 'Clockwise', 255, 1],
            ['w', 'Motor %m.motorSelection %m.motorDirection at %n power forever', 'setMotorForever', 'A', 'Clockwise', 255],
            ['w', 'Robot %m.robotDirection for %n steps', 'moveRobot', 'Forward', 1],
            ['w', 'Rotate %m.robotRotation at %n degrees', 'rotateRobot', 'Left', 90],
            ['--'], // LED's
            ['w', 'LED %m.ledSelect color %m.ledColor for %n seconds', 'setNeopixelColor', '1', 'Red', 1],
            ['w', 'LED %m.ledSelect color %m.ledColor forever', 'setNeopixelColorForever', '1', 'Red'],
            ['w', 'LED %m.ledSelect %n Red, %n Green, and %n Blue for %n seconds', 'setNeopixel', '1', 255, 255, 255, 1],
            ['w', 'LED %m.ledSelect %n Red, %n Green, and %n Blue forever', 'setNeopixelForever', '1', 255, 255, 255],
            ['--'], // Sensors
            ['w', 'Wait %n seconds', 'waitSeconds', 1],
            // ['r', 'Get value from sensor %m.sensorSelection', 'getSensor', '1'],
            ['w', 'Sensor %m.sensorSelection wait %m.sensorLevels', 'waitSensor', '1', 'Low']
            // Ends Here
        ],
        he: [
            // Algobrain Blocks :
            // ['--'], // Motors
            ['w', 'שניות %n במשך %n במהירות שעון כיוון %m.motorDirection %m.motorSelection מנוע', 'setMotorHE', 1, 255, 'עם', 'A'],
            ['w', 'לתמיד %n במהירות שעון כיוון %m.motorDirection %m.motorSelection מנוע', 'setMotorForeverHE', 255, 'עם', 'A'],
            ['w', 'צעדים %n %m.robotDirection רובוט', 'moveRobotHE', 1, 'קדימה'],
            ['w', 'מעלות %n -ב %m.robotRotation סובב', 'rotateRobotHE', 'שמאלה', 90],
            ['--'], // LED's
            ['w', 'שניות %n במשך %m.ledColor בצבע %m.ledSelect מנורה', 'setNeopixelColorHE', 1, 'Red', '1'],
            ['w', 'לתמיד %m.ledColor בצבע %m.ledSelect מנורה', 'setNeopixelColorHE', 'Red', '1'],
            ['w', 'שניות %n במשך כחול %n ירוק %n אדום %n בגוון %m.ledSelect מנורה', 'setNeopixelHE', 1, 255, 255, 255, '1'],
            ['w', 'לתמיד כחול %n ירוק %n אדום %n בגוון %m.ledSelect מנורה', 'setNeopixelForeverHE', 255, 255, 255, '1'],
            ['--'], // Sensors
            ['w', 'שניות %n למשך חכה', 'waitSeconds', 1],
            ['w', 'Sensor %m.sensorSelection wait %m.sensorLevels', 'waitSensor', '1', 'Low'],
            ['w', 'המתן לעוצמה גבוהה מחיישן איי']
            ['w', '%m.sensorSelection מחיישן %m.sensorLevels לערך המתן', 'waitSensor', '1', 'נמוכה']
            
            // Ends Here
        ]
    };

    var menus = {
        en: {
            // Move Motor
            motorSelection: ['A', 'B', 'C'],
            motorDirection: ['Clockwise', 'Counter-Clockwise'],
            // Move Robot
            robotDirection: ['Forward', 'Backward'],
            robotRotation: ['Left', 'Right'],
            // LED's
            ledSelect: ['1', '2'],
            ledColor: ['Red', 'Green', 'Blue'],
            // Get Sensor
            sensorSelection: ['1', '2'],
            sensorLevels: ['Low', 'Medium', 'High', 'Low-Medium', 'Medium-High', 'True', 'False']
        },
        he: {
            // Move Motor
            motorSelection: ['A', 'B', 'C'],
            motorDirection: ['עם', 'נגד'],
            // Move Robot
            robotDirection: ['קדימה', 'אחורה'],
            robotRotation: ['שמאלה', 'ימינה'],
            // LED's
            ledSelect: ['1', '2'],
            ledColor: ['אדום', 'ירוק', 'כחול'],
            // Get Sensor
            sensorSelection: ['1', '2'],
            sensorLevels: ['נמוכה', 'בינונית', 'גבוהה', 'נמוכה-בינונית', 'בינונית-גבוהה', 'אמת', 'שקר']
        }
    };

    var descriptor = {
        blocks: blocks[lang],
        menus: menus[lang],
        url: 'https://github.com/AlgobrixCoding/Algobrain-ScratchX',
        displyName: 'Algobrain ScratchX'
        // https://algobrixcoding.github.io/Algobrain-ScratchX/Algobrain.js - Extension URL (ScratchX)
        // https://scratchx.org/?url=https://algobrixcoding.github.io/Algobrain-ScratchX/Algobrain.js - English (Default)
        // https://scratchx.org/?url=https://algobrixcoding.github.io/Algobrain-ScratchX/Algobrain.js&lang=he - Hebrew
    };

    var serial_info = {
        type: 'serial'
    };
    ScratchExtensions.register('Algobrain', descriptor, ext, serial_info);

})({});