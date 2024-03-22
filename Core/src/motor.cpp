#include <stdlib.h>
#include <math.h>

#include "motor.h"
#include "Arduino.h"
#include "const.h"
 #include "Encoder.h"

// constants measured from bot
const double ticks_per_rev = 360;
const double wheel_diam = 3.2;
const double wheel_circ = M_PI * 3.2;
const double dist_between_treads = 9.25;

const double turn_diam = dist_between_treads * 2;
const double turn_circ = M_PI * turn_diam;

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

int motor::get_LEnc() {
    return encLTicks-encLStart;
}
int motor::get_REnc() {
    return encRTicks-encRStart;
}
void motor::resetEncs() {
    motor::read();
    encLStart = encLTicks;
    encRStart = encRTicks;
}
