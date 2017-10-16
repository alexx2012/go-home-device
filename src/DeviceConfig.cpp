// 
// 
// 

#include "DeviceConfig.h"
#include "Defines.h"

#include <EEPROM.h>

void DeviceConfig::init() {


}

bool DeviceConfig::hasDeviceId() {
    return this->getDeviceId() != NULL;
}

unsigned long DeviceConfig::getDeviceId() {
    unsigned long id;

    EEPROM.get(DEVICE_ID_ADDRESS, id);

    if (id == 0) {
        return NULL;
    }

    return id;
}

void DeviceConfig::setDeviceId(unsigned long id) {
    EEPROM.put(DEVICE_ID_ADDRESS, id);
}

void DeviceConfig::clearDeviceId() {
    for (int i = 0; i < EEPROM.length(); i++) {
        EEPROM.write(i, 0);
    }
}