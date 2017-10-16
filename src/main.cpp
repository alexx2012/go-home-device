#include "Defines.h"
#include "Device.h"
#include "Arduino.h"
#include <ArduinoLog.h>
#include <mcp_can.h>
#include <LiquidCrystal_I2C.h>
#include <Bounce2.h>
#include <TaskScheduler.h>
#include "Wire.h"

#define DEBUG_MODE 1
#define LOG_LEVEL LOG_LEVEL_VERBOSE
#define CAN_BUS_INT_PIN 15
#define AREF_VOLTAGE 3.3

void printMemoryUsageTaskCallback();

void sendCANDataTaskCallback();

void sendDeviceBeaconTaskCallback();

void readCANDataTaskCallback();

void sendTemperatureTaskCallback();

void watchPinStateTaskCallback();

void disablePulseTaskCallback();

void pulseTaskCallback();

void initCanBus();

void initSubDevices();

void startBeaconMode();

void stopBeaconMode();

void float2Bytes(float val, byte *bytes_array);

void float2String(float val, char *strBuff);

unsigned int converBytestToShort(byte bytes[2]);

float getLM35TemperatureValue(unsigned int analogPin);

int freeMemory();

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

MCP_CAN canBus(9);

long unsigned int canMessageId;
unsigned char rxExt;
unsigned char canMessageDlc = 0;
unsigned char canMessageData[8];

Device device;

LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

Scheduler taskRunner;

Task printMemoryUsageTask = Task(1000, TASK_FOREVER, &printMemoryUsageTaskCallback, &taskRunner, true);
Task sendCANDataTask = Task(10000, TASK_FOREVER, &sendCANDataTaskCallback, &taskRunner, true);
Task sendDeviceBeaconTask = Task(5000, TASK_FOREVER, &sendDeviceBeaconTaskCallback, &taskRunner, true);
Task readCANDataTask = Task(1, TASK_FOREVER, &readCANDataTaskCallback, &taskRunner, true);
Task sendTemperatureTask = Task(10000, TASK_FOREVER, &sendTemperatureTaskCallback, &taskRunner, true);
Task watchPinStateTask = Task(1, TASK_FOREVER, &watchPinStateTaskCallback, &taskRunner, true);

int messagesSent = 0;

byte subDevicesCount = 0;

SubDevice subDevices[] = {
        {0x01, SUB_DEVICE_DIGITAL_OUTPUT, 4},
        {0x02, SUB_DEVICE_DIGITAL_INPUT,  5},
        {0x03, SUB_DEVICE_ANALOG_OUTPUT,  6}
};

byte debouncersCount = 0;

SubDeviceDebouncer *debouncers;

void setup() {
    analogReference(INTERNAL);

    Serial.begin(115200);
    Log.begin(LOG_LEVEL, &Serial);

    delay(1000);

    lcd.begin(16, 2);
    lcd.backlight();

    lcd.setCursor(0, 0); //Start at character 4 on line 0
    lcd.print("Hello, I'm slave!");

    initCanBus();
    initSubDevices();

    taskRunner.init();

    taskRunner.addTask(printMemoryUsageTask);
    taskRunner.addTask(readCANDataTask);
    taskRunner.addTask(watchPinStateTask);
    //taskRunner.addTask(sendCANDataTask);
    taskRunner.addTask(sendTemperatureTask);

    device.init(&canBus);

    if (device.isBeaconMode()) {
        startBeaconMode();
    } else {
        //taskRunner.addTask(sendCANDataTask);
    }
}

void loop() {
    taskRunner.execute();
}

void initCanBus() {
    Log.verbose(F("Starting MCP2515 initialization..."));

    if (canBus.begin(MCP_ANY, CAN_125KBPS, MCP_8MHZ) == CAN_OK)
        Log.verbose(F("MCP2515 initialized successfully!"));
    else
        Log.error(F("Error initializing MCP2515..."));

    canBus.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted
}

void initSubDevices() {
    subDevicesCount = sizeof(subDevices) / sizeof(SubDevice);

    for (int i = 0; i < subDevicesCount; i++) {
        switch (subDevices[i].type) {
            case SUB_DEVICE_DIGITAL_INPUT:
                pinMode(subDevices[i].pin, INPUT);
                debouncersCount++;
                break;

            case SUB_DEVICE_DIGITAL_OUTPUT:
            case SUB_DEVICE_ANALOG_OUTPUT:
                pinMode(subDevices[i].pin, OUTPUT);

                break;
        }
    }

    if (debouncersCount > 0) {
        byte dCount = 0;
        debouncers = new SubDeviceDebouncer[debouncersCount];

        for (int i = 0; i < subDevicesCount; i++) {
            if (subDevices[i].type == SUB_DEVICE_DIGITAL_INPUT) {
                pinMode(subDevices[i].pin, INPUT_PULLUP);

                debouncers[dCount] = {subDevices[i].id, Bounce()};
                debouncers[dCount].debouncer.attach(subDevices[i].pin);
                debouncers[dCount].debouncer.interval(5);

                dCount++;
            }
        }
    }
}

void startBeaconMode() {
    Log.notice(F("Starting beacon mode."));

    taskRunner.addTask(sendDeviceBeaconTask);
}

void stopBeaconMode() {
    taskRunner.deleteTask(sendDeviceBeaconTask);
    //taskRunner.addTask(sendCANDataTask);
}

void sendDeviceBeaconTaskCallback() {
    device.sendBeaconMessage();
}

void watchPinStateTaskCallback() {
    for (int i = 0; i < debouncersCount; i++) {
        if (debouncers[i].debouncer.update()) {
            bool pinState = debouncers[i].debouncer.read();

            byte data[] = {device.getDeviceId(), debouncers[i].subDeviceId, pinState};
            byte sndStat = canBus.sendMsgBuf(MESSAGE_DIGITAL_STATE_CHANGE, 0, sizeof(data), data);

            Log.trace(F("State change: DID: %X, SDID: %X, new state: %T"), device.getDeviceId(),
                      debouncers[i].subDeviceId, pinState);
        }
    }
}

void readCANDataTaskCallback() {
    if (!digitalRead(CAN_BUS_INT_PIN)) {
        canBus.readMsgBuf(&canMessageId, &canMessageDlc, canMessageData);

        if (canMessageId != NULL) {
            Log.trace(F("CAN Message ID: %X"), canMessageId);

            if (device.isBeaconMode()) {
                switch (canMessageId) {
                    case MESSAGE_SET_ID_TO_BEACON_DEVICE: {
                        DEVICE_ID newDeviceId = canMessageData[0];

                        Log.trace(F("New beacon device ID: %X"), newDeviceId);

                        device.updateDeviceId(newDeviceId);
                        device.disableBeaconMode();

                        stopBeaconMode();
                    }
                        break;
                }
            }

            if (!device.isBeaconMode()) {
                unsigned short destinationDeviceId = canMessageData[0];

                if (destinationDeviceId == device.getDeviceId()) {
                    switch (canMessageId) {
                        case MESSAGE_CHANGE_DEVICE_ID: {
                            DEVICE_ID newDeviceId = canMessageData[1];

                            Log.trace(F("New device ID: %X"), newDeviceId);

                            device.updateDeviceId(newDeviceId);
                        }
                            break;

                        case MESSAGE_CLEAR_DEVICE_ID: {
                            Log.trace(F("Clear device ID: %X"), device.getDeviceId());

                            device.clearDeviceId();

                            startBeaconMode();
                        }
                            break;

                        case MESSAGE_GET_SUB_DEVICES: {
                            Log.trace(F("Get sub devices for device ID: %X"), device.getDeviceId());

                            for (byte i = 0; i < subDevicesCount; i++) {
                                byte data[] = {device.getDeviceId(), i, subDevicesCount, subDevices[i].id,
                                               subDevices[i].type};
                                byte sndStat = canBus.sendMsgBuf(MESSAGE_SUB_DEVICE_CONFIG, 0, sizeof(data), data);
                            }
                        }
                            break;

                        case MESSAGE_DIGITAL_SET_STATE: {
                            byte subDeviceId = canMessageData[1];
                            byte state = canMessageData[2];

                            for (int i = 0; i < subDevicesCount; i++) {
                                if (subDevices[i].id == subDeviceId) {
                                    digitalWrite(subDevices[i].pin, state);
                                }
                            }
                        }
                            break;

                        case MESSAGE_DIGITAL_PULSE: {
                            byte subDeviceId = canMessageData[1];
                            byte state = canMessageData[2];

                            byte bytes[] = {canMessageData[3], canMessageData[4]};
                            unsigned int length = converBytestToShort(bytes);

                            for (int i = 0; i < subDevicesCount; i++) {
                                if (subDevices[i].id == subDeviceId) {
                                    digitalWrite(subDevices[i].pin, state);

                                    if (subDevices[i].task == NULL) {
                                        subDevices[i].task = new Task(TASK_IMMEDIATE, TASK_ONCE, &pulseTaskCallback,
                                                                      &taskRunner, true, NULL,
                                                                      &disablePulseTaskCallback);
                                        subDevices[i].task->setLtsPointer(&subDevices[i]);
                                    } else {
                                        Log.trace(
                                                F("Pulse already exists subdevice ID: %X, length: %d ms, started: %l"),
                                                subDeviceId, length, millis());
                                    }

                                    subDevices[i].task->enableDelayed(length);

                                    Log.trace(F("Pulse subdevice ID: %X, length: %d ms, started: %l"), subDeviceId,
                                              length, millis());
                                }
                            }
                        }
                            break;

                        case MESSAGE_ANALOG_SET_VALUE: {
                            byte subDeviceId = canMessageData[1];
                            byte state = canMessageData[2];

                            for (int i = 0; i < subDevicesCount; i++) {
                                if (subDevices[i].id == subDeviceId) {
                                    analogWrite(subDevices[i].pin, state);

                                    Log.trace(F("Analog set state, subdevice ID: %X, state: %d"), subDeviceId, state);
                                }
                            }
                        }
                            break;
                    }
                }
            }

            canMessageId = NULL;
        }
    }
}

void pulseTaskCallback() {
    SubDevice &sdev = *((SubDevice *) taskRunner.currentLts());

    //digitalWrite(sdev.pin, !digitalRead(sdev.pin));

    Log.trace(F("pulseTaskCallback: %l"), millis());
}

void disablePulseTaskCallback() {
    SubDevice &sdev = *((SubDevice *) taskRunner.currentLts());

    digitalWrite(sdev.pin, !digitalRead(sdev.pin));

    Log.trace(F("disablePulseTaskCallback: %l"), millis());

    free(sdev.task);
    sdev.task = NULL;
}

void sendCANDataTaskCallback() {
    byte sndStat = device.sendPingMessage();

    if (sndStat == CAN_OK) {
        messagesSent++;

        char *str = (char *) malloc(16 * sizeof(char));
        memset(str, ' ', 16);
        sprintf(str, "CAN sent: %d     ", messagesSent);

        lcd.setCursor(0, 0);
        lcd.print(str);

        free(str);
    }
}

void sendTemperatureTaskCallback() {
    float value = getLM35TemperatureValue(0);
    //sd.value = getDS18B20TemperatureValue(0);

    char str[100];
    float2String(value, str);

    Log.verbose(F("Temperature: %s"), str);

    byte bytes[4];

    float2Bytes(value, &bytes[0]);


    byte data[] = {device.getDeviceId(), 0x00, bytes[0], bytes[1], bytes[2], bytes[3]};
    byte sndStat = canBus.sendMsgBuf(MESSAGE_TEMPERATURE_VALUE, 0, sizeof(data), data);
}

void float2String(float val, char *strBuff) {
    char *tmpSign = (val < 0) ? "-" : "";
    float tmpVal = (val < 0) ? -val : val;

    int tmpInt1 = tmpVal;                  // Get the integer
    float tmpFrac = tmpVal - tmpInt1;      // Get fraction
    int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer

    sprintf(strBuff, "%s%d.%04d", tmpSign, tmpInt1, tmpInt2);
}

void float2Bytes(float val, byte *bytes) {
    // Create union of shared memory space
    union {
        float f;
        byte temp_array[4];
    } u;

    // Overite bytes of union with float variable
    u.f = val;

    // Assign bytes to input array
    memcpy(bytes, u.temp_array, 4);

    int i, j;

    for (i = 0, j = sizeof(float) - 1; i < j; ++i, --j) {
        char temp = bytes[i];
        bytes[i] = bytes[j];
        bytes[j] = temp;
    }
}

unsigned int converBytestToShort(byte bytes[2]) {
    return (unsigned char) bytes[0] << 8 | (unsigned char) bytes[1];
}

float getLM35TemperatureValue(unsigned int analogPin) {
    int reading = 0;

    for (int i = 0; i < 10; i++) { // Average 10 readings for accurate reading
        reading += analogRead(analogPin);
        delay(20);
    }

    float tempC = (AREF_VOLTAGE * reading * 10) / 1023;

    return tempC;
}

void printMemoryUsageTaskCallback() {
    char *memStr = (char *) malloc(16 * sizeof(char));
    sprintf(memStr, "Mem: %d bytes", freeMemory());

    lcd.setCursor(0, 1);
    lcd.print(memStr);

    free(memStr);
}

int freeMemory() {
    int free_memory;

    if ((int) __brkval == 0)
        free_memory = ((int) &free_memory) - ((int) &__bss_end);
    else
        free_memory = ((int) &free_memory) - ((int) __brkval);

    return free_memory;
}
