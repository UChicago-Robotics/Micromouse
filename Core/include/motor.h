#include "Arduino.h"
#include "Encoder.h"
#include "const.h"
#include "pid.h"
#ifndef MOTOR_H
#define MOTOR_H
const double BASE_WIDTH = 9.25;
const double WHEEL_DIAM = 3.2;
const double WHEEL_CIRC = M_PI * 3.2;
const int TICKS_PER_REV = 360;
const double EPSILON = 1;
const int MIN_SPEED = 35;
class MotorController {
   private:
    int encLTicks = 0;
    int encRTicks = 0;
    int encLStart = 0;
    int encRStart = 0;
    double targL = 0;
    double targR = 0;
    Encoder encR = Encoder(RR_ENC, RL_ENC);
    Encoder encL = Encoder(LR_ENC, LL_ENC);
    int lastRun = 0;
    PIDController wheelPID = PIDController(0.05, 0, 0);
    int baseSpeed = 0;
    bool inMotion = false;
    int counter = 0;

   public:
    MotorController();
    void init();
    void setSpeed(double l, double r);
    void read();
    double getEncL();
    double getEncR();
    void resetEncs();
    void setTarget(double targetL, double targetR);
    bool isInMotion();
    void driveStraight(double dist, int bSpeed);
    void control();
};
#endif