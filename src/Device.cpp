#include "Device.h"
#include <EEPROM.h>
#include <mcp_can.h>

Device::Device(void) {}

void Device::init(MCP_CAN *pCanBus) {
    this->canBus = pCanBus;

    this->readDeviceId();
    this->beaconMode = !hasDeviceId();
}

bool Device::isBeaconMode() {
    return !this->hasDeviceId();
}

void Device::disableBeaconMode() {
    this->beaconMode = false;
}

byte Device::sendBeaconMessage() {
    byte data[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    byte sndStat = this->canBus->sendMsgBuf(MESSAGE_BEACON_DEVICE, 0, 8, data);

    if (sndStat == CAN_OK) {
        Log.verbose(F("Beacon message sent."));
    } else {
        Log.error(F("Error during beacon send. Result: %b"), sndStat);
    }

    return sndStat;
}

byte Device::sendPingMessage() {
    byte data[8] = {this->getDeviceId(), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    byte sndStat = canBus->sendMsgBuf(MESSAGE_PING, 0, 8, data);

    if (sndStat == CAN_OK) {
        Log.verbose(F("Ping message sent. Device ID: %x"), this->getDeviceId());
    } else {
        Log.error(F("Error during ping message send. Result: %b"), sndStat);
    }

    return sndStat;
}

bool Device::hasDeviceId() {
    return this->deviceId != NULL;
}

DEVICE_ID Device::getDeviceId() {
    return this->deviceId;
}

void Device::readDeviceId() {
    unsigned short id;

    EEPROM.get(DEVICE_ID_ADDRESS, id);

    this->deviceId = id;
}

void Device::updateDeviceId(DEVICE_ID id) {
    EEPROM.put(DEVICE_ID_ADDRESS, id);
    this->deviceId = id;
}

void Device::clearDeviceId() {
    EEPROM.put(DEVICE_ID_ADDRESS, NULL);
    this->deviceId = NULL;
}