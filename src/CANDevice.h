// CANDevice.h

#ifndef _CANDEVICE_h
#define _CANDEVICE_h

#if defined(ARDUINO) && ARDUINO >= 100

#include "arduino.h"

#else
#include "WProgram.h"
#endif

class CANDevice {
private:


protected:


public:
    void init();
};

//extern CANDeviceClass CANDevice;

#endif

