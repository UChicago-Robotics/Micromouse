#include "sensor.h"
#include "Arduino.h"
#include "Arduino_BMI270_BMM150.h"
#include "stdlib.h"
#include "MahonyAHRS.h"

SensorController::SensorController(int L_val, double lambda_val) {
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
    this->LL_cutoff = 5; // TODO cutoff for missing wall
    this->RR_cutoff = 5;
    this->RF_cutoff = 300;
    this->LF_cutoff = 200;
    this->LL_coeff = .05; // weighting of wall dist vs encoder diff
    this->RR_coeff = .05;
    this->RF_coeff = .1;
    this->LF_coeff = .1;


    this->sensorHistory = new int*[4];
    this->lastReadingTime = 0;
    this->dt = 0.02;
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
    Serial.print("Accelerometer sample rate = ");
    Serial.print(IMU.accelerationSampleRate());
    Serial.println(" Hz");

    Serial.print("Gyroscope sample rate = ");
    Serial.print(IMU.gyroscopeSampleRate());
    Serial.println(" Hz");

    this->lastReadingTime = millis();
    allOff();
    this->readIMU();
    this->Gz_offset = this->gz;
    Serial.print("Gz Offset: ");
    Serial.println(this->Gz_offset);
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
    return this->RRs;
}

float SensorController::getRFs() {
    return this->RFs;
}

float SensorController::getLFs() {
    return this->LFs;
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
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(this->ax, this->ay, this->az);
    }
    
    if (IMU.gyroscopeAvailable()) {
        IMU.readGyroscope(this->gx, this->gy, this->gz);
    }
    
    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(this->mx, this->my, this->mz);
    }

    unsigned long temp = millis();
    this->dt = (float)(temp - this->lastReadingTime)/1000;
    this->lastReadingTime = temp;

    if (this->calibrated) {
        this->ax -= this->ax0;
        this->ay -= this->ay0;
        this->gx -= this->gx0;
        this->gy -= this->gy0;
        this->gz -= this->gz0;
        if (fabs(this->gx) < 0.3) this->gx = 0;
        if (fabs(this->gy) < 0.3) this->gy = 0;
        if (fabs(this->gz) < 0.3) this->gz = 0;
    }
    this->mahony.invSampleFreq = this->dt;
    this->mahony.updateIMU(gx, gy, gz, ax, ay, az);
    this->mahony.computeAngles();
    this->gz -= this->Gz_offset;
}

void SensorController::readFilterIMU() {
    this->readIMU();
}
String SensorController::dumpIMUString() {
    // dt,ax,ay,az,gx,gy,gz with 2 decimals of precision
    return String(this->dt) + "\t a:" + String(this->ax,2) + "," + String(this->ay,2) + "," + String(this->az,2) + "\t g:" + String(this->gx,2) + "," + String(this->gy,2) + "," + String(this->gz,2) + "\t heading: " + String(this->mahony.roll * 57.29578f) + "," + String(this->mahony.pitch * 57.29578f) + "," + String(this->mahony.yaw * 57.29578f + 180.0f);
}
String SensorController::dumpIRString() {
    // LFs,LLs,RRs,RFs, with 2 decimals of precision
    return String(this->dt) + "\t IR:" + String(this->LFs,2) + "," + String(this->LLs,2) + "," + String(this->RRs,2) + "," + String(this->RFs);
}

String SensorController::calibrate() {
    int N = 50;
    for (int i = 0;i< N;++i) this->readIMU();
    delay(500);
    for (int i = 0; i < N; ++i) {
        this->readIMU();
        this->ax0 += this->ax/N;
        this->ay0 += this->ay/N;
        this->az0 += this->az/N;
        this->gx0 += this->gx/N;
        this->gy0 += this->gy/N;
        this->gz0 += this->gz/N;
        this->mx0 += this->mx/N;
        this->my0 += this->my/N;
        this->mz0 += this->mz/N;
        delay(30);
    }
    this->calibrated = true;
    return String(this->ax0,2) + "," + String(this->ay0,2) + "," + String(this->az0,2) + "," + String(this->gx0,2) + "," + String(this->gy0,2) + "," + String(this->gz0,2) + "," + String(this->mx0,2) + "," + String(this->my0,2) + "," + String(this->mz0,2);
}

void SensorController::resetWallBase() {
    // calibrates the "center values" of the left and right sensors based on current walls
    this->read();
    this->push();
    this->read();
    this->push();
    this->read();
    this->push();
    this->LL_base = this->LLs;
    this->RR_base = this->RRs;
}
void SensorController::setBaseL(float l) {
    this->LL_base = l;
}
void SensorController::setBaseR(float r) {
    this->RR_base = r;
}
float SensorController::getBaseL() {
    return this->LL_base;
}
float SensorController::getBaseR() {
    return this->RR_base;
}

float SensorController::getLLCut() {
    return this->LL_cutoff;
}
float SensorController::getRRCut() {
    return this->RR_cutoff;
}
float SensorController::getLFCut() {
    return this->LF_cutoff;
}
float SensorController::getRFCut() {
    return this->RF_cutoff;
}

float SensorController::getLLCoeff() {
    return this->LL_coeff;
}
float SensorController::getRRCoeff() {
    return this->RR_coeff;
}
float SensorController::getLFCoeff() {
    return this->LF_coeff;
}
float SensorController::getRFCoeff() {
    return this->RF_coeff;
}

float SensorController::getAz() {
    return this->az;
}

float SensorController::getGz(){
    return this->gz;
}
float SensorController::getYaw() {
    return this->mahony.yaw;
}