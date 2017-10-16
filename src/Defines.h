#ifndef _DEFINES_h
#define _DEFINES_h

#define _TASK_SLEEP_ON_IDLE_RUN                // Compile with support for entering IDLE SLEEP state for 1 ms if not tasks are scheduled to run
#define _TASK_WDT_IDS                        // Compile with support for Task IDs and Watchdog timer
#define _TASK_LTS_POINTER                    // Compile with support for Local Task Storage pointer

#define MESSAGE_BEACON_DEVICE                0x01 // 1
#define MESSAGE_SET_ID_TO_BEACON_DEVICE        0x02 // 2
#define MESSAGE_PING                        0x03 // 3
#define MESSAGE_CHANGE_DEVICE_ID            0x04 // 4
#define MESSAGE_CLEAR_DEVICE_ID                0x05 // 5
#define MESSAGE_GET_SUB_DEVICES                0x06 // 6
#define MESSAGE_SUB_DEVICE_CONFIG            0x07 // 7

#define MESSAGE_TEMPERATURE_VALUE            0x31 // 49
#define MESSAGE_TEMPERATURE_GET_VALUE        0x32 // 50

#define MESSAGE_DIGITAL_GET_STATE            0x33 // 51
#define MESSAGE_DIGITAL_SET_STATE            0x34 // 52
#define MESSAGE_DIGITAL_STATE_CHANGE        0x35 // 53
#define MESSAGE_DIGITAL_PULSE                0x36 // 54

#define MESSAGE_ANALOG_GET_VALUE            0x64 // 100
#define MESSAGE_ANALOG_SET_VALUE            0x65 // 101
#define MESSAGE_ANALOG_PULSE                0x67 // 103

#define DEVICE_ID_ADDRESS                    0
#define DEVICE_ID_LENGTH                    2

#define SUB_DEVICE_DIGITAL_INPUT            0x01
#define SUB_DEVICE_DIGITAL_OUTPUT            0x02
#define SUB_DEVICE_ANALOG_OUTPUT            0x03
#define SUB_DEVICE_TEMPERATURE_SENSOR        0x04

#endif