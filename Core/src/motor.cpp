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

void MotorController::setMotion(bool a) {
    this->inMotion = a;
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

void MotorController::driveStraight(int dist, int bSpeed) {
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

void MotorController::control() {
    if (this->inMotion) {
        // only do things when inMotion
        if (this->isTurn) {
            // turning motion
            Serial.println("TargYaw: " + String(this->targYaw, 2) + "Yaw: " + String(this->yaw, 2));
            if (fabs(this->targYaw - this->yaw) > 5) {
                // if still out of tolerance range
                double diff = fmod((this->targYaw - this->yaw + 720), 360);
                long int ct = millis();
                int dt = ct - this->lastRun + 1;
                double op;
                if (diff < 180) {
                    // closer to turn counter clockwise
                    op = this->turnPID.feedback(diff, dt);
                    op = max(op, this->turnMinSpeed);
                    this->setSpeed(-op, op);
                    Serial.println("counter clockwise, diff: " + String(diff, 2) + "dt: " + String(dt) + "op: " + String(op, 2));
                } else {
                    // closer to turn clockwise
                    op = this->turnPID.feedback(360-diff, dt);
                    op = max(op, this->turnMinSpeed);
                    this->setSpeed(op, -op);
                    Serial.println("clockwise, diff: " + String(diff, 2) + "dt: " + String(dt) + "op: " + String(op, 2));
                }
                this->lastRun = ct;
            } else {
                this->inMotion = false;
                this->setSpeed(0, 0);
            }
        } else {
            // driving motion
            this->read();

            double l = this->getEncL(), r = this->getEncR();
            if (l < this->targL) {
                double diff = l - r;
                long int ct = millis();
                int dt = ct - this->lastRun + 1;
                double op = this->wheelPID.feedback(diff, dt);
                this->setSpeed(this->baseSpeed, this->baseSpeed + op);
                this->lastRun = ct;
                Serial.println(String(ct) + "\t" + String(dt) + "\t" + String(l, 2) + "\t" + String(r, 2) + "\t" + String(diff, 2) + "\t" + String(op, 2));
            } else {
                this->inMotion = false;
                this->setSpeed(0, 0);
            }
        }
    }
}

double MotorController::PIDfeedback(double diff, double diff_t) {
    return this->wheelPID.feedback(diff, diff_t);
}
