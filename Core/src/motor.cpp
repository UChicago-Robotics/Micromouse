#include "motor.h"
#include "Arduino.h"
#include "Encoder.h"
#include "const.h"
#include "math.h"
#include "pid.h"
#include <algorithm>

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
}

double MotorController::getEncR() {  // in cm
    return ((double)(this->encRTicks - this->encRStart)) / TICKS_PER_REV * WHEEL_CIRC;
}

void MotorController::resetEncs() {
    this->read();
    this->encLStart = this->encLTicks;
    this->encRStart = this->encRTicks;
}

bool MotorController::isInMotion() {
    return this->inMotion;
}

bool MotorController::isInTurn() {
    return this->isTurn;
}

void MotorController::setInMotion(bool a) {
    this->inMotion = a;
}

void MotorController::setInTurn(bool a) {
    this->isTurn = a;
}
void MotorController::setBaseSpeed(double bs) {
    this->baseSpeed = bs;
}
double MotorController::getBaseSpeed() {
    return this->baseSpeed;
}

void MotorController::setLastRun(long int lr) {
    this->lastRun = lr;
}
long int MotorController::getLastRun() {
    return this->lastRun;
}

void MotorController::driveStraight(double dist, int bSpeed) {
    // add task to drive with dist and bSpeed
    // left-biased driving straight
    this->inMotion = true;
    this->isTurn = false;
    this->resetEncs();
    this->targL = dist;
    this->baseSpeed = bSpeed;
    this->wheelPID.resetI();
    this->lastRun = millis();
}

void MotorController::turnToYaw(int target) {
    this->inMotion = true;
    this->isTurn = true;
    this->targYaw = target;
    this->lastRun = millis();
}

float MotorController::getTargetYaw() {
    return this->targYaw;
}

double MotorController::getTargetL() {
    return this->targL;
}

void MotorController::setTargetYaw(float yawIn) {
    this->targYaw = yawIn;
}

double MotorController::wheelPIDfeedback(double diff, double difft) {
    return this->wheelPID.feedback(diff, difft);
}
double MotorController::turnPIDfeedback(double diff, double difft) {
    return this->turnPID.feedback(diff,difft);
}

double MotorController::mdriveSpeed() {
    return this->driveMinSpeed;
}

double MotorController::mturnSpeed() {
    return this->turnMinSpeed;
}

float MotorController::getTurnTime() {
    return this->turnTime;
}