#include "Arduino.h"
#include "Encoder.h"
#include "const.h"
#include "pid.h"
#ifndef MOTOR_H
#define MOTOR_H
const double BASE_WIDTH = 9.25;
const double WHEEL_DIAM = 3.3;
const double WHEEL_CIRC = M_PI * 3.3;
const int TICKS_PER_REV = 360;
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
    long int lastRun = 0;
    PIDController wheelPID = PIDController(20, 0, 0);  // stupid but works - may require tuning later at high speeds etc, testing at 50
    int baseSpeed = 0;
    int cutoffSpeed = 30; // stall speed
    bool inMotion = false;

   public:
    MotorController();
    void init();
    void setSpeed(double l, double r);
    void read();
    double getEncL();
    double getEncR();
    void resetEncs();
    void setTargetL(int targetL);
    double getTargetL();
    void setBaseSpeed(double bs);
    double getBaseSpeed();
    void setLastRun(long int lr);
    long int getLastRun();
    bool isInMotion();
    void setMotion(bool a);
    void driveStraight(int dist, int bSpeed);
    void control();
    double getMinSpeed();
    double PIDfeedback(double diff, double diff_t);
};
#endif