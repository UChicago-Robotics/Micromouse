#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

const int MOD = 1e6;
BLEService service("30551738-84fc-4a60-bb8e-221a069b51f5");
BLECharacteristic bleData("5b86af8f-925f-4029-9c95-458599341f96", BLERead | BLENotify, 40);

void bt_setup() {
    // Set the device name and local name to "UCMicromouse"
    BLE.setDeviceName("UCMicromouse");
    BLE.setLocalName("UCMicromouse");

    // Add the service to the BLE module
    BLE.setAdvertisedService(service);

    // Add the two characteristics to the service
    service.addCharacteristic(bleData);

    // Add the service to the BLE module
    BLE.addService(service);

    // Set the connection interval for the BLE connection
    BLE.setConnectionInterval(8, 8);

    // Enable the BLE module to be connectable
    BLE.setConnectable(true);

    // Start advertising the BLE connection
    BLE.advertise();

    Serial.println("Done setup");
}

void bt_loop(String data) {
    BLE.poll();
    Serial.print("datastring: ");
    Serial.println(data.c_str());
    bleData.writeValue(data.c_str());
}