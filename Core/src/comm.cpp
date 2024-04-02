#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

const int MOD = 1e6;
BLEService IMUservice("30551738-84fc-4a60-bb8e-221a069b51f5");
BLECharacteristic passData("5b86af8f-925f-4029-9c95-458599341f96", BLERead | BLENotify, 40);

void bt_setup() {


    // Set the device name and local name to "IMU"
    BLE.setDeviceName("UCMicromouse");
    BLE.setLocalName("UCMicromouse");

    // Add the service to the BLE module
    BLE.setAdvertisedService(IMUservice);

    // Add the two characteristics to the service
    IMUservice.addCharacteristic(passData);

    // Add the service to the BLE module
    BLE.addService(IMUservice);

    // Set the connection interval for the BLE connection
    BLE.setConnectionInterval(8, 8);

    // Enable the BLE module to be connectable
    BLE.setConnectable(true);

    // Start advertising the BLE connection
    BLE.advertise();

    Serial.println("Done setup");
}

void bt_loop(String indata) {
    BLE.poll();
    Serial.print("datastring: ");
    Serial.println(indata.c_str());
    passData.writeValue(indata.c_str());
}