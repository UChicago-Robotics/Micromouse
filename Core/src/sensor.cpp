#include "sensor.h"
#include "Arduino.h"
#include "Arduino_LSM9DS1.h"
#include "stdlib.h"

SensorController::SensorController(int L_val) {
    this->L = L_val;
    this->sensorHistory = new int*[4];
    for (int i = 0; i < 4; ++i) {
        this->sensorHistory[i] = new int[L];
        for (int j = 0; j < L; ++j) {
            this->sensorHistory[i][j] = 0;  // Initialize all elements to 0
        }
    }
    this->currentIndex = 0;  // Start from the first slot
}
void SensorController::init() {
    // Initialize the IMU
    if (!IMU.begin()) {
        // If the IMU fails to initialize, enter an infinite loop
        while (1) Serial.println("Can't start IMU.");
    }
}
void SensorController::allOn() {
    digitalWrite(RF_IRo, HIGH);
    digitalWrite(RR_IRo, HIGH);
    digitalWrite(LF_IRo, HIGH);
    digitalWrite(LL_IRo, HIGH);
}
void SensorController::allOff() {
    digitalWrite(RF_IRo, LOW);
    digitalWrite(RR_IRo, LOW);
    digitalWrite(LF_IRo, LOW);
    digitalWrite(LL_IRo, LOW);
}
void SensorController::frontOn() {
    digitalWrite(RF_IRo, HIGH);
    digitalWrite(LF_IRo, HIGH);
}
void SensorController::frontOff() {
    digitalWrite(RF_IRo, LOW);
    digitalWrite(LF_IRo, LOW);
}
void SensorController::sidesOn() {
    digitalWrite(RR_IRo, HIGH);
    digitalWrite(LL_IRo, HIGH);
}
void SensorController::sidesOff() {
    digitalWrite(RR_IRo, LOW);
    digitalWrite(LL_IRo, LOW);
}

int* SensorController::readAll() {
    int* arr = new int[4];
    arr[0] = analogRead(LF_IRi);
    arr[1] = analogRead(LL_IRi);
    arr[2] = analogRead(RR_IRi);
    arr[3] = analogRead(RF_IRi);
    return arr;
}
int* SensorController::readFront() {
    int* arr = new int[2];
    arr[0] = analogRead(LF_IRi);
    arr[1] = analogRead(RF_IRi);
    return arr;
}
int* SensorController::readSides() {
    int* arr = new int[2];
    arr[0] = analogRead(LL_IRi);
    arr[1] = analogRead(RR_IRi);
    return arr;
}
void SensorController::push(int* inIR) {
    for (int i = 0; i < 4; i++) {
        // Shift elements up by 1
        for (int j = L - 1; j > 0; j--) {
            this->sensorHistory[i][j] = this->sensorHistory[i][j - 1];
        }
        // Insert new data into the first slot
        this->sensorHistory[i][0] = inIR[i];
    }
    this->currentIndex++;
}

double* SensorController::expSmooth(double lambda) {
    double* result = new double[4];
    int ci = std::min(this->L, this->currentIndex);
    for (int i = 0; i < 4; i++) {
        double weightedSum = 0.0;
        double denominator = 0.0;
        for (int j = 0; j < ci; j++) {
            double weight = exp(-lambda * j);
            weightedSum += weight * this->sensorHistory[i][j];
            denominator += weight;
        }
        result[i] = weightedSum / denominator;
    }
    return result;
}

void SensorController::readIMU() {
    IMU.readAcceleration(this->ax, this->ay, this->az);
    IMU.readGyroscope(this->gx, this->gy, this->gz);
    Serial.print(this->ax);
    Serial.print(" ");
    Serial.print(this->ay);
    Serial.print(" ");
    Serial.print(this->az);
    Serial.print(" ");
    Serial.print(this->gx);
    Serial.print(" ");
    Serial.print(this->gy);
    Serial.print(" ");
    Serial.println(this->gz);
    String accelString = String(this->ax) + "," + String(this->ay) + "," + String(this->az) + "," + String(this->gx) + "," + String(this->gy) + "," + String(this->gz);
}