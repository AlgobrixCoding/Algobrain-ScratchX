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
 *along with this program.  If not, see <http://www.gnu.org/licenses/>.
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
        NEOPIXEL_REGISTER = 0x74;

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
                    setTimeout(setupPeripherals, 1000);
                }
                pinging = false;
                pingCount = 0;
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
        register_neopixel(Led_A_Pin, 1);
        register_neopixel(Led_B_Pin, 1);
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
        console.log("Algobrain Version 1.5 - Setup Complete ");
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
                console.log("default switch at setMotorSpeed");
                break;
        }
    }

    function setMotorDir(motorId, _dir) {
        var dir = 0; // Default is Clockwise
        if (_dir != menus[lang][motorDirection][0]) {
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
            default:
                console.log("default switch at setMotorDir");
                break;
        }
    }

    function register_neopixel(pin, count) {
        var msg = new Uint8Array([
            START_SYSEX,
            NEOPIXEL_REGISTER,
            pin,
            count,
            END_SYSEX
        ]);
        device.send(msg.buffer);
    }

    function neopixel(index, red, green, blue) {
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
    ext.setLedNeoPixelColor = function(ledSelect, color) {
        if (ledSelect == '1') {
            register_neopixel(Led_A_Pin, 1);
        } else {
            register_neopixel(Led_B_Pin, 1);
        }
        console.log(menus[lang]);
        console.log(menus[lang][ledColor]);
        console.log(menus[lang][ledColor][0]);
        switch(color) {
            case menus[lang][ledColor][0]:
                neopixel(0, 255, 0, 0);
                break;
            case menus[lang][ledColor][1]:
                neopixel(0, 0, 255, 0);
                break;
            case menus[lang][ledColor][2]:
                neopixel(0, 0, 0, 255);
                break;
        }
    };

    ext.setLedNeoPixel = function (ledSelect, red, green, blue) {
        if (ledSelect == '1') {
            register_neopixel(Led_A_Pin, 1);
        } else {
            register_neopixel(Led_B_Pin, 1);
        }
        neopixel(0, red, green, blue);
    };

    ext.move_robot = function (robot_direction, num_steps) {
        if (num_steps === '1') {
            if (robot_direction === 'Forward') {
                analogWrite(6, 0);
                analogWrite(2, 0);
                analogWrite(3, 127);
                analogWrite(5, 127);
                setTimeout(function () {
                    analogWrite(5, 0);
                    analogWrite(6, 0);
                    analogWrite(2, 0);
                    analogWrite(3, 0);
                }, 1500);
            } else if (robot_direction === 'Backward') {
                analogWrite(5, 0);
                analogWrite(3, 0);
                analogWrite(6, 127);
                analogWrite(2, 127);
                setTimeout(function () {
                    analogWrite(5, 0);
                    analogWrite(6, 0);
                    analogWrite(2, 0);
                    analogWrite(3, 0);
                }, 1500);
            } else if (robot_direction === 'Left') {
                analogWrite(6, 0);
                analogWrite(3, 0);
                analogWrite(5, 127);
                analogWrite(2, 127);
            } else if (robot_direction === 'Right') {
                analogWrite(5, 0);
                analogWrite(2, 0);
                analogWrite(6, 127);
                analogWrite(3, 127);
            } else {
                analogWrite(5, 0);
                analogWrite(6, 0);
                analogWrite(2, 0);
                analogWrite(3, 0);
            }
        } else {
            if (robot_direction === 'Forward') {
                analogWrite(6, 0);
                analogWrite(2, 0);
                analogWrite(3, 127);
                analogWrite(5, 127);
                setTimeout(function () {
                    analogWrite(5, 0);
                    analogWrite(6, 0);
                    analogWrite(2, 0);
                    analogWrite(3, 0);
                }, 3000);
            } else if (robot_direction === 'Backward') {
                analogWrite(5, 0);
                analogWrite(3, 0);
                analogWrite(6, 127);
                analogWrite(2, 127);
                setTimeout(function () {
                    analogWrite(5, 0);
                    analogWrite(6, 0);
                    analogWrite(2, 0);
                    analogWrite(3, 0);
                }, 3000);
            } else if (robot_direction === 'Left') {
                analogWrite(6, 0);
                analogWrite(3, 0);
                analogWrite(5, 127);
                analogWrite(2, 127);
            } else if (robot_direction === 'Right') {
                analogWrite(5, 0);
                analogWrite(2, 0);
                analogWrite(6, 127);
                analogWrite(3, 127);
            } else {
                analogWrite(5, 0);
                analogWrite(6, 0);
                analogWrite(2, 0);
                analogWrite(3, 0);
            }
        }
    };

    ext.setMotor = function (motorId, dir, pwm) {
        setMotorSpeed(motorId, pwm);
        setMotorDir(motorId, dir);
        console.log("Moving Motor ", motorId, " In ", dir, " Direction at ", pwm, " PWM");
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
            [' ', 'Move Motor %m.motorSelect %m.motorDirection at %n power', 'setMotor', 'A', 'Clockwise', 0],
            [' ', 'Move Robot %m.robotDirection for %n steps', 'moveRobot', 'Forward', 1],
            [' ', 'Rotate Robot %m.robotRotate at %n degrees', 'rotateRobot', 'Left', 90],
            [' ', 'Set LED %m.ledSelect to %m.ledColor', 'setLedNeoPixelColor', '1', 'Red'],
            [' ', 'Set LED %m.ledSelect to %n Red, %n Green, and %n Blue', 'setLedNeoPixel', '1', 0, 0, 0]
            // Ends Here
        ],
        he: [
            // Algobrain Blocks :
            [' ', 'הפעל מנוע %m.motorSelect בכיוון %m.motorDirection במהירות של %n', 'setMotor', 'A', 'כיוון השעון', 255],
            [' ', '%n במהירות של %m.motorDirection בכיוון %m.motorSelect הפעל מנוע', 'setMotor', 0, 'כיוון השעון', 'A'],
            [' ', '%m.robotDirection צעדים בכיוון %n הזז רובוט', 'moveRobot', 1, 'קדימה'],
            [' ', 'מעלות %n בזווית של %m.robotRotate סובב רובוט בכיוון', 'rotateRobot', 90, 'שמאלה'],
            [' ', '%m.ledColor בצבע %m.ledSelect הפעל לד', 'אדום', '1', 'setLedNeoPixelColor'],
            [' ', 'כחול %n, ירוק %n, אדום %n בגוונים %m.ledSelect הפעל לד', 0, 0, 0, '1' , 'setLedNeoPixel']
            // Ends Here
        ]
    };

    var menus = {
        en: {
            // Move Motor
            motorSelect: ['A', 'B', 'C'],
            motorDirection: ['Clockwise', 'Counter-Clockwise'],
            // LED's
            ledSelect: ['1', '2'],
            ledColor: ['Red', 'Green', 'Blue'],
            // Move Robot
            robotDirection: ['Forward', 'Backward'],
            robotRotate: ['Left', 'Right']
        },
        he: {
            // Move Motor
            motorSelect: ['A', 'B', 'C'],
            motorDirection: ['כיוון השעון', 'נגד כיוון השעון'],
            // LED's
            ledSelect: ['1', '2'],
            ledColor: ['אדום', 'ירוק', 'כחול'],
            // Move Robot
            robotDirection: ['קדימה', 'אחורה'],
            robotRotate: ['שמאלה', 'ימינה']
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