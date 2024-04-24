#include "sensor.h"
#include "Arduino.h"
#include "Arduino_BMI270_BMM150.h"
#include "stdlib.h"

SensorController::SensorController(int L_val, double lambda_val) {
    // this->q_est = {1, 0, 0, 0};       // initialize with as unit vector with real component  = 1
    this->L = L_val;
    this->lambda = lambda_val;
    this->RR = 0;
    this->RF = 0;
    this->LF = 0;
    this->LL = 0;
    this->RRs = 0;
    this->RFs = 0;
    this->LFs = 0;
    this->LLs = 0;
    this->roll = 0;
    this->yaw = 0;
    this->pitch = 0;
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
    allOff();
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

void SensorController::readAll() {
    this->LF = analogRead(LF_IRi);
    this->LL = analogRead(LL_IRi);
    this->RR = analogRead(RR_IRi);
    this->RF = analogRead(RF_IRi);
}

int SensorController::getRR() {
    return this->RR;
}

int SensorController::getRF() {
    return this->RF;
}

int SensorController::getLF() {
    return this->LF;
}

int SensorController::getLL() {
    return this->LL;
}

float SensorController::getRRs() {
    return this->LLs;
}

float SensorController::getRFs() {
    return this->LLs;
}

float SensorController::getLFs() {
    return this->LLs;
}

float SensorController::getLLs() {
    return this->LLs;
}

void SensorController::readFront() {
    this->LF = analogRead(LF_IRi);
    this->RF = analogRead(RF_IRi);
}

void SensorController::readSides() {
    this->LL = analogRead(LL_IRi);
    this->RR = analogRead(RR_IRi);
}

void SensorController::read() {
    frontOn();
    delay(10);
    readFront();
    frontOff();
    sidesOn();
    delay(10);
    readSides();
    sidesOff();
}

void SensorController::push() {
    // LF, LL, RR, RF
    for (int i = 0; i < 4; i++) {
        // Shift elements up by 1
        for (int j = L - 1; j > 0; j--) {
            this->sensorHistory[i][j] = this->sensorHistory[i][j - 1];
        }
        // Insert new data into the first slot
        switch(i) {
            case 0:
                this->sensorHistory[i][0] = LF;
                break;
            case 1:
                this->sensorHistory[i][0] = LL;
                break;
            case 2:
                this->sensorHistory[i][0] = RR;
                break;
            case 3:
                this->sensorHistory[i][0] = RF;
                break;
            default:
                break;
        }
    }
    this->currentIndex++;
    // L is how many values it smooths back over, lambda is the discount factor
    int ci = std::min(this->L, this->currentIndex);
    for (int i = 0; i < 4; i++) {
        double weightedSum = 0.0;
        double denominator = 0.0;
        for (int j = 0; j < ci; j++) {
            double weight = exp(-lambda * j);
            weightedSum += weight * this->sensorHistory[i][j];
            denominator += weight;
        }
        // I know this looks disgusting but idc -CB
        switch(i) {
            case 0:
                LFs = weightedSum / denominator;
                break;
            case 1:
                LLs = weightedSum / denominator;
                break;
            case 2:
                RRs = weightedSum / denominator;
                break;
            case 3:
                RFs = weightedSum / denominator;
                break;
            default:
                break;
        }
    }
}

void SensorController::readIMU() {
    IMU.readAcceleration(this->ax, this->ay, this->az);
    IMU.readGyroscope(this->gx, this->gy, this->gz);
}
void SensorController::readFilterIMU() {
    this->readIMU();
    // imu_filter(this->ax,this->ay,this->az,this->gx,this->gy,this->gz);
    // eulerAngles(this->q_est, &this->roll, &this->pitch, &this->yaw);
}

float SensorController::getRoll() {
    return this->roll;
}
float SensorController::getPitch() {
    return this->pitch;
}
float SensorController::getYaw() {
    return this->yaw;
}

String SensorController::dumpString() {
    // LFs,LLs,RRs,RFs,ax,ay,az,gx,gy,gz with 2 decimals of precision
    return String(this->LFs,2) + "," + String(this->LLs,2) + "," + String(this->RRs,2) + "," + String(this->RFs,2) + "," + String(this->ax,2) + "," + String(this->ay,2) + "," + String(this->az,2) + "," + String(this->gx,2) + "," + String(this->gy,2) + "," + String(this->gz,2);
    // return String(this->ax,2) + "," + String(this->ay,2) + "," + String(this->az,2) + "," + String(this->gx,2) + "," + String(this->gy,2) + "," + String(this->gz,2) + "\t\t" + String(this->roll,2) + "," + String(this->pitch,2) + "," + String(this->yaw,2);

}