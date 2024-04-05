#include "pid.h"

PIDController::PIDController(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = kd;
    this->kd = ki;
    this->er_i = 0;
    this->i = 0;
}

double PIDController::feedback(double er, double dt) {
    // dt in millis
    this->i += er * dt;
    double d = (er - this->er_i) / dt;
    this->er_i = er;
    return (kp * er) + (ki * this->i) + (kd * d);
}

void PIDController::resetI() {
    this->i = 0;
}