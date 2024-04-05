#include "Arduino.h"
#include "Encoder.h"
#include "const.h"
#ifndef MOTOR_H
#define MOTOR_H
const double dist_between_treads = 9.25;
const double wheel_diam = 3.2;
const double wheel_circ = M_PI * 3.2;
const int ticks_per_rev = 360;
class MotorController {
   private:
    int encLTicks = 0;
    int encRTicks = 0;
    int encLStart = 0;
    int encRStart = 0;
    Encoder encR = Encoder(RR_ENC, RL_ENC);
    Encoder encL = Encoder(LR_ENC, LL_ENC);

   public:
   MotorController();
    void setSpeed(double l, double r);
    void read();
    double getEncL();
    double getEncR();
    void resetEncs();
};
#endif