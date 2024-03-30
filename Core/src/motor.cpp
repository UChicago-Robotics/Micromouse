#include <stdlib.h>
#include <math.h>

#include "motor.h"
#include "Arduino.h"
#include "const.h"
#include "Encoder.h"

// constants measured from bot

Encoder encR(RR_ENC, RL_ENC);
Encoder encL(LR_ENC, LL_ENC);

int encLTicks = 0;
int encRTicks = 0;
int encLStart = 0;
int encRStart = 0;

void motor::setSpeed(double l, double r) {
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

void motor::read() {
    encLTicks = encL.read();
    encRTicks = encR.read();
}

double motor::get_LEnc() { // in cm
    return ((double)(encLTicks-encLStart))/ticks_per_rev*wheel_circ;
}
double motor::get_REnc() { // in cm
    return ((double)(encRTicks-encRStart))/ticks_per_rev*wheel_circ;
}
void motor::resetEncs() {
    motor::read();
    encLStart = encLTicks;
    encRStart = encRTicks;
}
