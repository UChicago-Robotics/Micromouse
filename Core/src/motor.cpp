#include "Arduino.h"
#include "Encoder.h"
#include "math.h"
#include "motor.h"
#include "const.h"

MotorController::MotorController(){}
void MotorController::setSpeed(double l, double r) {
    if (l > 0) {
        analogWrite(LB_MOTOR,0);
        analogWrite(LF_MOTOR,l);
    } else {
        analogWrite(LB_MOTOR,-l);
        analogWrite(LF_MOTOR,0);
    }
    if (r > 0) {
        analogWrite(RB_MOTOR,0);
        analogWrite(RF_MOTOR,r);
    } else {
        analogWrite(RB_MOTOR,-r);
        analogWrite(RF_MOTOR,0);
    }
}

void MotorController::read() {
    this->encLTicks = this->encL.read();
    this->encRTicks = this->encR.read();
}

double MotorController::getEncL() { // in cm
    return ((double)(this->encLTicks-this->encLStart))/ticks_per_rev*wheel_circ;
}
double MotorController::getEncR() { // in cm
    return ((double)(this->encRTicks-this->encRStart))/ticks_per_rev*wheel_circ;
}
void MotorController::resetEncs() {
    this->read();
    this->encLStart = this->encLTicks;
    this->encRStart = this->encRTicks;
}
