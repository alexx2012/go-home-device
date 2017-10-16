// Device.h

#ifndef _DEVICE_h
#define _DEVICE_h

#if defined(ARDUINO) && ARDUINO >= 100

#include "arduino.h"

#else
#include "WProgram.h"
#endif

#include "Defines.h"
#include <ArduinoLog.h>
#include <Bounce2.h>
#include <TaskScheduler.h>
#include <mcp_can.h>

#ifndef DEVICE_ID
#define DEVICE_ID unsigned short
#endif

struct SubDevice {
    byte id;
    byte type;
    unsigned int pin;
    Task *task;
};

struct SubDeviceDebouncer {
    byte subDeviceId;
    Bounce debouncer;
};

class Device {
private:
    unsigned short deviceId;
    bool beaconMode;
    MCP_CAN *canBus;

protected:


public:
    Device(void);

    void init(MCP_CAN *can);

    bool hasDeviceId();

    DEVICE_ID getDeviceId();

    void readDeviceId();

    void updateDeviceId(DEVICE_ID);

    void clearDeviceId();

    byte sendBeaconMessage();

    bool isBeaconMode();

    void disableBeaconMode();

    byte sendPingMessage();
};

#endif

