// DeviceConfig.h

#ifndef _DEVICECONFIG_h
#define _DEVICECONFIG_h

#if defined(ARDUINO) && ARDUINO >= 100

#include "arduino.h"

#else
#include "WProgram.h"
#endif

class DeviceConfig {
protected:


public:
    void init();

    bool hasDeviceId();

    unsigned long getDeviceId();

    void setDeviceId(unsigned long);

    void clearDeviceId();
};

//extern DeviceConfig DeviceConfig;

#endif

