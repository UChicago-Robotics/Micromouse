#include "Arduino.h"
#include "ArduinoBLE.h"
#ifndef COMM_H
#define COMM_H

class BluetoothController {
    private:
    BLEService service;
    BLECharacteristic characteristic;
   public:
    BluetoothController();
    void init();
    void publish(String data);
    void poll();
};
#endif