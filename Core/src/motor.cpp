#include "motor.h"

#include "Arduino.h"
#include "Encoder.h"
#include "const.h"
#include "math.h"
#include "pid.h"

MotorController::MotorController() {}

void MotorController::init() {
    this->lastRun = millis();
}

void MotorController::setSpeed(double l, double r) {
    if (l > 0) {
        analogWrite(LB_MOTOR, 0);
        analogWrite(LF_MOTOR, l);
    } else {
        analogWrite(LB_MOTOR, -l);
        analogWrite(LF_MOTOR, 0);
    }
    if (r > 0) {
        analogWrite(RB_MOTOR, 0);
        analogWrite(RF_MOTOR, r);
    } else {
        analogWrite(RB_MOTOR, -r);
        analogWrite(RF_MOTOR, 0);
    }
}

void MotorController::read() {
    this->encLTicks = this->encL.read();
    this->encRTicks = this->encR.read();
}

double MotorController::getEncL() {  // in cm
    return ((double)(this->encLTicks - this->encLStart)) / TICKS_PER_REV * WHEEL_CIRC;
    ;
}

double MotorController::getEncR() {  // in cm
    return ((double)(this->encRTicks - this->encRStart)) / TICKS_PER_REV * WHEEL_CIRC;
}

void MotorController::resetEncs() {
    this->read();
    this->encLStart = this->encLTicks;
    this->encRStart = this->encRTicks;
}

void MotorController::setTarget(double targetL, double targetR) {
    this->targL = targetL;
    this->targR = targetR;
}

bool MotorController::isInMotion() {
    return this->inMotion;
}

void MotorController::driveStraight(double dist, int bSpeed) {
    // add task to drive with dist and bSpeed
    // left-biased driving straight
    Serial.println("Driving Straight");
    this->resetEncs();
    this->setTarget(dist, dist);
    this->inMotion = true;
    this->baseSpeed = bSpeed;
    this->wheelPID.resetI();
    this->lastRun = millis();
}

void MotorController::control() {
    this->read();
    double l = this->getEncL();
    double r = this->getEncR();
    if ((l < this->targL) && this->isInMotion()) {
        long int ct = millis();
        int dt = ct - this->lastRun + 1;
        double diff = l - r;
        double op = this->wheelPID.feedback(diff, dt);
        this->setSpeed(this->baseSpeed, this->baseSpeed);
        this->lastRun = ct;

        if (this->counter % 100 == 0) {
            Serial.print(ct);
            Serial.print("\t");
            Serial.print(dt);
            Serial.print("\t");
            Serial.print(l);
            Serial.print("\t");
            Serial.print(r);
            Serial.print('\t');
            Serial.print(this->baseSpeed);
            Serial.print("\t");
            Serial.println(this->baseSpeed + op);
        }

        ++this->counter;
    } else {
        this->inMotion = false;
        this->setSpeed(0, 0);
    }
}