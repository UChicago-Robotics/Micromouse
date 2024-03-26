#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

const int MOD = 1e6;
BLEService IMUservice("10000");

BLECharacteristic accelData("5b86af8f-925f-4029-9c95-458599341f96", BLERead | BLENotify, 20);
BLECharacteristic gyroData("373ceb13-35ad-41b5-b924-0604e56d8a4d", BLERead | BLENotify, 20);

void bt_setup() {
    // Initialize the BLE module
    Serial.begin(9600);
    if (!BLE.begin()) {
        // If the BLE module fails to initialize, enter an infinite loop
        while (1) {
        }
    }

    // Initialize the IMU
    if (!IMU.begin()) {
        // If the IMU fails to initialize, enter an infinite loop
        while (1) {
        }
    }

    // Set the device name and local name to "IMU"
    BLE.setDeviceName("UCMicromouse");
    BLE.setLocalName("UCMicromouse");

    // Add the service to the BLE module
    BLE.setAdvertisedService(IMUservice);

    // Add the two characteristics to the service
    IMUservice.addCharacteristic(accelData);
    IMUservice.addCharacteristic(gyroData);

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

void bt_loop() {
    float acX, acY, acZ, gX, gY, gZ;

    // acclerometer data
    IMU.readAcceleration(acX, acY, acZ);

    String accelString = String(acX) + "," + String(acY) + "," + String(acZ);
    
    Serial.print("accelstring: ");
    Serial.print(accelString.c_str());
    accelData.writeValue(accelString.c_str());

    // gyroscope data
    IMU.readGyroscope(gX, gY, gZ);

    String gyroString = String(gX) + "," + String(gY) + "," + String(gZ);

    Serial.print(" gyrostring: ");
    Serial.println(gyroString.c_str());
    gyroData.writeValue(gyroString.c_str());

    delay(1000);
}