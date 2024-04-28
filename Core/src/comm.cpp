#include "comm.h"
#include <ArduinoBLE.h>

BLEService service("30551738-84fc-4a60-bb8e-221a069b51f5");
BLECharacteristic characteristic("5b86af8f-925f-4029-9c95-458599341f96", BLERead | BLENotify, 100);

void bt_setup() {
    Serial.println("Initializing bluetooth controller in bt_setup()...");
    while (1) {
        if (!BLE.begin()) {
            Serial.println("Can't start BLE.");
        } else break;
    }
    // Set the device name and local name to "UCMicromouse"
    BLE.setDeviceName("UCMicromouse");
    BLE.setLocalName("UCMicromouse");

    // Add the service to the BLE module
    BLE.setAdvertisedService(service);

    // Add the two characteristics to the service
    service.addCharacteristic(characteristic);

    // Add the service to the BLE module
    BLE.addService(service);

    // Set the connection interval for the BLE connection
    // BLE.setConnectionInterval(8, 8);

    // Enable the BLE module to be connectable
    BLE.setConnectable(true);

    // Start advertising the BLE connection
    BLE.advertise();

    Serial.println("Done initializing Bluetooth controller in bt_setup().");
}
void bt_loop(String data) {
    characteristic.writeValue(data.c_str());
}